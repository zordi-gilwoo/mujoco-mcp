#!/usr/bin/env python3
"""
可视化示例: 演示如何在使用MuJoCo MCP服务器的同时进行可视化
"""

import sys
import time
import threading
import logging
import numpy as np
import mujoco
import mujoco.viewer
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
logger = logging.getLogger("viz_example")

# MuJoCo模型XML - 简单的人形机器人
HUMANOID_XML = """
<mujoco model="humanoid">
  <compiler angle="degree" inertiafromgeom="true"/>
  <option timestep="0.01" />
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="matplane" texture="texplane" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>
  <worldbody>
    <light diffuse=".8 .8 .8" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="10 10 0.1" type="plane" material="matplane" condim="3"/>
    <body name="torso" pos="0 0 1.5">
      <joint name="root" type="free" damping="0" armature="0" limited="false"/>
      <geom name="torso" type="sphere" size="0.1" pos="0 0 0" rgba="0.8 0.3 0.3 1"/>
      <body name="head" pos="0 0 0.2">
        <geom name="head" type="sphere" size="0.08" rgba="0.8 0.3 0.3 1"/>
      </body>
      <body name="right_upper_arm" pos="0 -0.15 0.05">
        <joint name="right_shoulder" type="ball"/>
        <geom name="right_upper_arm" type="capsule" size="0.025" fromto="0 0 0 0 -0.15 0" rgba="0.8 0.3 0.3 1"/>
        <body name="right_lower_arm" pos="0 -0.15 0">
          <joint name="right_elbow" axis="0 1 0" range="-160 0"/>
          <geom name="right_lower_arm" type="capsule" size="0.02" fromto="0 0 0 0 -0.15 0" rgba="0.8 0.3 0.3 1"/>
        </body>
      </body>
      <body name="left_upper_arm" pos="0 0.15 0.05">
        <joint name="left_shoulder" type="ball"/>
        <geom name="left_upper_arm" type="capsule" size="0.025" fromto="0 0 0 0 0.15 0" rgba="0.8 0.3 0.3 1"/>
        <body name="left_lower_arm" pos="0 0.15 0">
          <joint name="left_elbow" axis="0 1 0" range="-160 0"/>
          <geom name="left_lower_arm" type="capsule" size="0.02" fromto="0 0 0 0 0.15 0" rgba="0.8 0.3 0.3 1"/>
        </body>
      </body>
      <body name="right_thigh" pos="-0.05 -0.075 -0.15">
        <joint name="right_hip" type="ball"/>
        <geom name="right_thigh" type="capsule" size="0.035" fromto="0 0 0 0 0 -0.25" rgba="0.8 0.3 0.3 1"/>
        <body name="right_leg" pos="0 0 -0.25">
          <joint name="right_knee" axis="0 -1 0" range="-160 0"/>
          <geom name="right_leg" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.25" rgba="0.8 0.3 0.3 1"/>
        </body>
      </body>
      <body name="left_thigh" pos="-0.05 0.075 -0.15">
        <joint name="left_hip" type="ball"/>
        <geom name="left_thigh" type="capsule" size="0.035" fromto="0 0 0 0 0 -0.25" rgba="0.8 0.3 0.3 1"/>
        <body name="left_leg" pos="0 0 -0.25">
          <joint name="left_knee" axis="0 -1 0" range="-160 0"/>
          <geom name="left_leg" type="capsule" size="0.025" fromto="0 0 0 0 0 -0.25" rgba="0.8 0.3 0.3 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="right_shoulder_x" joint="right_shoulder" gear="20 0 0"/>
    <motor name="right_shoulder_y" joint="right_shoulder" gear="0 20 0"/>
    <motor name="right_shoulder_z" joint="right_shoulder" gear="0 0 20"/>
    <motor name="right_elbow" joint="right_elbow" gear="20"/>
    <motor name="left_shoulder_x" joint="left_shoulder" gear="20 0 0"/>
    <motor name="left_shoulder_y" joint="left_shoulder" gear="0 20 0"/>
    <motor name="left_shoulder_z" joint="left_shoulder" gear="0 0 20"/>
    <motor name="left_elbow" joint="left_elbow" gear="20"/>
    <motor name="right_hip_x" joint="right_hip" gear="20 0 0"/>
    <motor name="right_hip_y" joint="right_hip" gear="0 20 0"/>
    <motor name="right_hip_z" joint="right_hip" gear="0 0 20"/>
    <motor name="right_knee" joint="right_knee" gear="30"/>
    <motor name="left_hip_x" joint="left_hip" gear="20 0 0"/>
    <motor name="left_hip_y" joint="left_hip" gear="0 20 0"/>
    <motor name="left_hip_z" joint="left_hip" gear="0 0 20"/>
    <motor name="left_knee" joint="left_knee" gear="30"/>
  </actuator>
  <sensor>
    <accelerometer name="torso_accel" site="torso"/>
  </sensor>
</mujoco>
"""

class VisualizationController:
    """用于控制和可视化MuJoCo模拟的类"""
    
    def __init__(self):
        """初始化控制器"""
        self.server_thread = None
        self.client = None
        self.sim_id = None
        self.model = None
        self.data = None
        self.viewer = None
        self.running = False
        self.sync_mode = True  # 是否同步MCP服务器和本地可视化
    
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
        """连接到MCP服务器并初始化模拟"""
        logger.info("正在连接到MuJoCo MCP服务器...")
        self.client = MCPClient("http://localhost:8000")
        
        # 启动模拟
        logger.info("正在启动新的模拟...")
        result = self.client.call_tool("start_simulation", {"model_xml": HUMANOID_XML})
        self.sim_id = result["simulation_id"]
        logger.info(f"模拟ID: {self.sim_id}")
        
        # 获取模拟信息
        sim_info = self.client.get_resource("simulation_info", {"simulation_id": self.sim_id})
        logger.info(f"模拟信息: {sim_info}")
        
        # 重置模拟
        self.client.call_tool("reset_simulation", {"simulation_id": self.sim_id})
        
        # 创建本地模型用于可视化
        self.model = mujoco.MjModel.from_xml_string(HUMANOID_XML)
        self.data = mujoco.MjData(self.model)
    
    def start_visualization(self):
        """启动可视化"""
        logger.info("启动可视化...")
        
        self.running = True
        
        # 在新线程中启动可视化，这样不会阻塞主线程
        viz_thread = threading.Thread(target=self._run_visualization)
        viz_thread.daemon = True
        viz_thread.start()
    
    def _run_visualization(self):
        """运行可视化循环"""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            
            # 保持运行直到停止
            while self.running:
                if self.sync_mode:
                    # 从MCP服务器获取当前状态并同步到本地数据
                    self._sync_from_server()
                
                # 更新可视化
                viewer.sync()
                
                time.sleep(0.01)  # 限制更新速率
    
    def _sync_from_server(self):
        """从MCP服务器同步状态到本地"""
        try:
            # 获取关节位置
            positions = self.client.get_resource("joint_positions", {"simulation_id": self.sim_id})
            self.data.qpos[:] = positions
            
            # 获取关节速度
            velocities = self.client.get_resource("joint_velocities", {"simulation_id": self.sim_id})
            self.data.qvel[:] = velocities
            
            # 更新状态
            mujoco.mj_forward(self.model, self.data)
        except Exception as e:
            logger.error(f"同步数据时发生错误: {str(e)}")
    
    def apply_random_actions(self, num_actions=100, interval=0.1):
        """应用随机动作并步进模拟"""
        logger.info(f"开始应用{num_actions}个随机动作...")
        
        # 获取执行器数量
        actuator_count = self.model.nu
        
        for i in range(num_actions):
            # 生成随机动作
            actions = np.random.uniform(-0.5, 0.5, actuator_count)
            
            # 应用到MCP服务器
            self.client.call_tool("apply_control", {
                "simulation_id": self.sim_id,
                "control": actions.tolist()
            })
            
            # 步进模拟
            self.client.call_tool("step_simulation", {
                "simulation_id": self.sim_id,
                "num_steps": 5
            })
            
            if i % 10 == 0:
                logger.info(f"执行动作 {i}/{num_actions}")
            
            time.sleep(interval)
    
    def stop(self):
        """停止控制器"""
        logger.info("正在停止控制器...")
        
        self.running = False
        
        if self.sim_id is not None:
            try:
                # 删除模拟
                self.client.call_tool("delete_simulation", {"simulation_id": self.sim_id})
                logger.info("已删除模拟")
            except Exception as e:
                logger.error(f"删除模拟时发生错误: {str(e)}")
        
        if self.viewer is not None:
            self.viewer.close()
        
        # 停止服务器
        mujoco_mcp.stop()
        
        # 等待服务器线程结束
        if self.server_thread is not None:
            self.server_thread.join(timeout=2)
        
        logger.info("控制器已停止")

def main():
    """主程序"""
    logger.info("启动MuJoCo MCP可视化示例")
    
    controller = VisualizationController()
    
    try:
        # 启动MCP服务器
        controller.start_server()
        
        # 连接并初始化
        controller.connect()
        
        # 启动可视化
        controller.start_visualization()
        
        # 等待用户按下回车键
        input("按回车键开始随机动作序列...")
        
        # 应用随机动作
        controller.apply_random_actions(num_actions=200, interval=0.05)
        
        # 等待用户按下回车键结束
        input("演示完成，按回车键退出...")
        
    except KeyboardInterrupt:
        logger.info("用户中断，正在关闭...")
    except Exception as e:
        logger.error(f"发生错误: {str(e)}")
    finally:
        # 停止控制器
        controller.stop()
        logger.info("程序退出")

if __name__ == "__main__":
    main() 