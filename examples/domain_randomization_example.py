#!/usr/bin/env python3
"""
域随机化示例: 演示如何使用MuJoCo MCP进行域随机化，帮助从仿真到现实的迁移
"""

import sys
import time
import threading
import logging
import numpy as np
import random
import json
from model_context_protocol import MCPClient

# 导入mujoco_mcp模块
try:
    import mujoco_mcp
except ImportError:
    print("未找到mujoco_mcp模块。请确保已经安装该模块。")
    print("尝试执行: pip install -e .")
    sys.exit(1)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("domain_randomization")

# MuJoCo模型XML模板 - 简单的机械手臂（可注入随机参数）
ARM_XML_TEMPLATE = """
<mujoco model="arm">
  <compiler angle="degree" inertiafromgeom="true"/>
  <option timestep="{timestep}" gravity="{gravity}" />
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="matplane" texture="texplane" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>
  <worldbody>
    <light diffuse=".8 .8 .8" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="5 5 0.1" type="plane" material="matplane" condim="3"/>
    <body name="base" pos="0 0 0.1">
      <joint name="base_rot" type="hinge" axis="0 0 1" damping="{base_joint_damping}" />
      <geom name="base" type="cylinder" size="0.1 0.05" rgba="0.5 0.5 0.5 1"/>
      <body name="arm1" pos="0 0.15 0">
        <joint name="joint1" type="hinge" axis="1 0 0" damping="{joint1_damping}" />
        <geom name="arm1" type="capsule" size="0.05" fromto="0 0 0 0 0.3 0" rgba="0.7 0.7 0 1"/>
        <body name="arm2" pos="0 0.3 0">
          <joint name="joint2" type="hinge" axis="1 0 0" damping="{joint2_damping}" />
          <geom name="arm2" type="capsule" size="0.04" fromto="0 0 0 0 0.3 0" rgba="0 0.7 0.7 1"/>
          <body name="end_effector" pos="0 0.3 0">
            <joint name="wrist" type="hinge" axis="0 1 0" damping="{wrist_damping}" />
            <geom name="end_effector" type="sphere" size="0.05" rgba="0.7 0 0.7 1"/>
            <site name="target_check" pos="0 0 0" size="0.01"/>
          </body>
        </body>
      </body>
    </body>
    <body name="target" pos="{target_x} {target_y} {target_z}">
      <geom name="target" type="sphere" size="0.05" rgba="1 0 0 0.5"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="base_rot_motor" joint="base_rot" gear="{base_motor_gear}" />
    <motor name="joint1_motor" joint="joint1" gear="{joint1_motor_gear}" />
    <motor name="joint2_motor" joint="joint2" gear="{joint2_motor_gear}" />
    <motor name="wrist_motor" joint="wrist" gear="{wrist_motor_gear}" />
  </actuator>
  <sensor>
    <jointpos name="base_rot_pos" joint="base_rot"/>
    <jointpos name="joint1_pos" joint="joint1"/>
    <jointpos name="joint2_pos" joint="joint2"/>
    <jointpos name="wrist_pos" joint="wrist"/>
    <sitepos name="end_effector_pos" site="target_check"/>
    <framepos name="target_pos" objtype="body" objname="target"/>
  </sensor>
</mujoco>
"""

class RandomizationParams:
    """包含随机化参数的类"""
    
    def __init__(self):
        """初始化默认参数范围"""
        # 物理属性范围
        self.timestep_range = (0.005, 0.015)  # 模拟时间步长
        self.gravity_range = (-10.5, -9.5)  # 重力加速度
        
        # 阻尼系数范围
        self.damping_range = (0.05, 0.5)
        
        # 执行器齿轮比范围
        self.gear_range = (20, 40)
        
        # 目标位置范围
        self.target_x_range = (-0.3, 0.3)
        self.target_y_range = (0.4, 0.8)
        self.target_z_range = (0.1, 0.4)

    def generate_random_params(self):
        """生成一组随机参数"""
        # 随机物理参数
        timestep = random.uniform(*self.timestep_range)
        gravity_z = random.uniform(*self.gravity_range)
        gravity = f"0 0 {gravity_z}"
        
        # 随机阻尼系数
        base_joint_damping = random.uniform(*self.damping_range)
        joint1_damping = random.uniform(*self.damping_range)
        joint2_damping = random.uniform(*self.damping_range)
        wrist_damping = random.uniform(*self.damping_range)
        
        # 随机执行器齿轮比
        base_motor_gear = random.uniform(*self.gear_range)
        joint1_motor_gear = random.uniform(*self.gear_range)
        joint2_motor_gear = random.uniform(*self.gear_range)
        wrist_motor_gear = random.uniform(*self.gear_range)
        
        # 随机目标位置
        target_x = random.uniform(*self.target_x_range)
        target_y = random.uniform(*self.target_y_range)
        target_z = random.uniform(*self.target_z_range)
        
        # 返回参数字典
        return {
            "timestep": timestep,
            "gravity": gravity,
            "base_joint_damping": base_joint_damping,
            "joint1_damping": joint1_damping,
            "joint2_damping": joint2_damping,
            "wrist_damping": wrist_damping,
            "base_motor_gear": base_motor_gear,
            "joint1_motor_gear": joint1_motor_gear,
            "joint2_motor_gear": joint2_motor_gear,
            "wrist_motor_gear": wrist_motor_gear,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z
        }

class SimpleController:
    """简单的机械臂控制器"""
    
    def __init__(self):
        """初始化控制器"""
        # 控制器增益
        self.p_gain = 5.0  # 比例增益
        self.d_gain = 0.5  # 微分增益
        
        # 上一次误差
        self.prev_error = np.zeros(3)
    
    def compute_action(self, current_pos, target_pos):
        """计算控制动作"""
        # 计算位置误差
        error = np.array(target_pos) - np.array(current_pos)
        
        # 计算误差变化率（简化的微分）
        error_rate = error - self.prev_error
        self.prev_error = error.copy()
        
        # 简单的PD控制
        p_term = self.p_gain * error
        d_term = self.d_gain * error_rate
        
        # 合并控制信号
        control = p_term + d_term
        
        # 简单的映射到四个关节（这只是一个示例，不是真正的逆运动学）
        # 实际应用中应使用适当的逆运动学算法
        joint_control = np.zeros(4)
        
        # base旋转（水平方向）影响x和y
        joint_control[0] = 0.5 * (control[0] + control[1])
        
        # 其他关节影响高度和前进距离
        joint_control[1] = 0.7 * control[2] - 0.3 * control[1]
        joint_control[2] = 0.5 * control[2] + 0.3 * control[1]
        joint_control[3] = 0.3 * (control[0] - control[1])
        
        # 限制控制范围
        joint_control = np.clip(joint_control, -1.0, 1.0)
        
        return joint_control.tolist()

class DomainRandomizationTrainer:
    """域随机化训练器"""
    
    def __init__(self):
        """初始化训练器"""
        self.randomizer = RandomizationParams()
        self.controller = SimpleController()
        self.server_thread = None
        self.client = None
        self.success_count = 0
        self.total_episodes = 0
        
        self.statistics = {
            "episode_lengths": [],
            "success_rate": 0.0,
            "final_distances": []
        }
    
    def start_server(self):
        """启动MCP服务器"""
        logger.info("正在启动MuJoCo MCP服务器...")
        self.server_thread = threading.Thread(
            target=mujoco_mcp.start,
            kwargs={"host": "localhost", "port": 8000, "blocking": True},
            daemon=True
        )
        self.server_thread.start()
        time.sleep(1)  # 给服务器一些启动时间
    
    def connect(self):
        """连接到MCP服务器"""
        logger.info("正在连接到MuJoCo MCP服务器...")
        self.client = MCPClient("http://localhost:8000")
    
    def generate_model_xml(self):
        """生成随机化的模型XML"""
        params = self.randomizer.generate_random_params()
        return ARM_XML_TEMPLATE.format(**params), params
    
    def run_episode(self, max_steps=200, success_threshold=0.1):
        """运行一个训练回合"""
        # 生成随机化的模型
        model_xml, params = self.generate_model_xml()
        
        # 启动新的模拟
        result = self.client.call_tool("start_simulation", {"model_xml": model_xml})
        sim_id = result["simulation_id"]
        logger.info(f"启动模拟 ID: {sim_id}")
        logger.info(f"随机参数: {json.dumps(params, indent=2)}")
        
        try:
            # 重置模拟
            self.client.call_tool("reset_simulation", {"simulation_id": sim_id})
            
            # 获取目标位置
            sensors = self.client.get_resource("sensor_data", {"simulation_id": sim_id})
            target_pos = sensors["target_pos"]
            logger.info(f"目标位置: {target_pos}")
            
            # 训练循环
            success = False
            min_distance = float('inf')
            
            for step in range(max_steps):
                # 获取当前状态
                sensors = self.client.get_resource("sensor_data", {"simulation_id": sim_id})
                end_effector_pos = sensors["end_effector_pos"]
                
                # 计算到目标的距离
                distance = np.linalg.norm(np.array(end_effector_pos) - np.array(target_pos))
                min_distance = min(min_distance, distance)
                
                # 检查是否到达目标
                if distance < success_threshold:
                    success = True
                    logger.info(f"成功到达目标! 步数: {step+1}, 距离: {distance:.4f}")
                    break
                
                # 使用控制器计算动作
                action = self.controller.compute_action(end_effector_pos, target_pos)
                
                # 应用控制
                self.client.call_tool("apply_control", {
                    "simulation_id": sim_id,
                    "control": action
                })
                
                # 步进模拟
                self.client.call_tool("step_simulation", {
                    "simulation_id": sim_id,
                    "num_steps": 5
                })
                
                # 每隔一段时间打印状态
                if step % 20 == 0:
                    logger.info(f"步骤 {step}: 距离={distance:.4f}, 动作={[f'{a:.2f}' for a in action]}")
            
            # 更新统计信息
            self.total_episodes += 1
            if success:
                self.success_count += 1
            
            self.statistics["episode_lengths"].append(step + 1)
            self.statistics["final_distances"].append(min_distance)
            self.statistics["success_rate"] = self.success_count / self.total_episodes
            
            # 打印回合总结
            logger.info(f"回合 {self.total_episodes} 结束: " +
                       f"{'成功' if success else '失败'}, " +
                       f"最小距离: {min_distance:.4f}, " +
                       f"步数: {step+1}")
            
            return success, min_distance, step + 1
            
        finally:
            # 删除模拟
            self.client.call_tool("delete_simulation", {"simulation_id": sim_id})
    
    def run_training(self, num_episodes=10):
        """运行多个训练回合"""
        logger.info(f"开始域随机化训练，共{num_episodes}个回合...")
        
        for episode in range(num_episodes):
            logger.info(f"====== 开始回合 {episode+1}/{num_episodes} ======")
            self.run_episode()
            
            # 简单的进度报告
            if (episode + 1) % 5 == 0 or episode == num_episodes - 1:
                logger.info("\n=== 训练统计 ===")
                logger.info(f"成功率: {self.statistics['success_rate']:.2f}")
                logger.info(f"平均步数: {np.mean(self.statistics['episode_lengths']):.1f}")
                logger.info(f"平均最小距离: {np.mean(self.statistics['final_distances']):.4f}")
                logger.info("=================\n")
    
    def stop(self):
        """停止训练器和服务器"""
        logger.info("正在停止训练器...")
        
        # 停止服务器
        mujoco_mcp.stop()
        
        # 等待服务器线程结束
        if self.server_thread is not None:
            self.server_thread.join(timeout=2)
        
        logger.info("训练器已停止")

def main():
    """主程序"""
    logger.info("启动MuJoCo MCP域随机化示例")
    
    trainer = DomainRandomizationTrainer()
    
    try:
        # 启动MCP服务器
        trainer.start_server()
        
        # 连接到服务器
        trainer.connect()
        
        # 运行训练
        trainer.run_training(num_episodes=15)
        
    except KeyboardInterrupt:
        logger.info("用户中断，正在关闭...")
    except Exception as e:
        logger.error(f"发生错误: {str(e)}")
    finally:
        # 停止训练器
        trainer.stop()
        logger.info("程序退出")

if __name__ == "__main__":
    main() 