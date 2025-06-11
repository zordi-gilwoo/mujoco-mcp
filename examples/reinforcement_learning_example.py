#!/usr/bin/env python3
"""
强化学习示例: 演示如何使用MuJoCo MCP服务器创建一个兼容强化学习的环境
"""

import sys
import time
import threading
import logging
import numpy as np
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
logger = logging.getLogger("rl_example")

# MuJoCo模型XML - 简单的inverted pendulum
INVERTED_PENDULUM_XML = """
<mujoco model="inverted_pendulum">
  <option timestep="0.01" />
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba=".9 .9 .9 1"/>
    <body name="cart" pos="0 0 0.1">
      <joint name="slide" type="slide" axis="1 0 0" damping="0.5"/>
      <geom name="cart" type="box" size="0.2 0.1 0.05" rgba="0.5 0.5 0.5 1"/>
      <body name="pole" pos="0 0 0.1">
        <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
        <geom name="pole" type="capsule" size="0.02 0.3" rgba="0 0.7 0.7 1" fromto="0 0 0 0 0 0.6"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="slide_motor" joint="slide" gear="50" ctrllimited="true" ctrlrange="-1.0 1.0"/>
  </actuator>
  <sensor>
    <jointpos name="cart_pos" joint="slide"/>
    <jointvel name="cart_vel" joint="slide"/>
    <jointpos name="pole_pos" joint="hinge"/>
    <jointvel name="pole_vel" joint="hinge"/>
  </sensor>
</mujoco>
"""

class MCPRLEnvironment:
    """使用MCP服务器的强化学习环境"""
    
    def __init__(self, client, model_xml):
        """初始化环境"""
        self.client = client
        self.model_xml = model_xml
        self.sim_id = None
        self.reset()
    
    def reset(self):
        """重置环境"""
        # 如果已经有模拟在运行，删除它
        if self.sim_id is not None:
            try:
                self.client.call_tool("delete_simulation", {"simulation_id": self.sim_id})
            except Exception:
                pass
        
        # 启动新的模拟
        result = self.client.call_tool("start_simulation", {"model_xml": self.model_xml})
        self.sim_id = result["simulation_id"]
        
        # 获取初始状态
        return self._get_state()
    
    def step(self, action):
        """执行一个动作并向前推进环境"""
        # 应用动作
        self.client.call_tool("apply_control", {
            "simulation_id": self.sim_id,
            "control": [float(action)]
        })
        
        # 向前推进模拟
        self.client.call_tool("step_simulation", {
            "simulation_id": self.sim_id, 
            "num_steps": 5  # 每个RL步骤执行5个物理步骤
        })
        
        # 获取新的状态
        state = self._get_state()
        
        # 计算奖励和完成标志
        reward = self._compute_reward(state)
        done = self._is_done(state)
        
        return state, reward, done, {}
    
    def close(self):
        """关闭环境"""
        if self.sim_id is not None:
            try:
                self.client.call_tool("delete_simulation", {"simulation_id": self.sim_id})
                self.sim_id = None
            except Exception as e:
                logger.error(f"关闭环境时发生错误: {str(e)}")
    
    def _get_state(self):
        """获取环境状态"""
        # 获取传感器数据
        sensors = self.client.get_resource("sensor_data", {"simulation_id": self.sim_id})
        
        # 提取相关状态变量
        cart_pos = sensors["cart_pos"][0]
        cart_vel = sensors["cart_vel"][0]
        pole_pos = sensors["pole_pos"][0]
        pole_vel = sensors["pole_vel"][0]
        
        # 组合成状态向量
        return np.array([cart_pos, cart_vel, np.sin(pole_pos), np.cos(pole_pos), pole_vel])
    
    def _compute_reward(self, state):
        """计算奖励"""
        cart_pos, cart_vel, sin_pole, cos_pole, pole_vel = state
        
        # 倒立摆竖直（cos_pole接近1）时奖励高
        upright_reward = cos_pole
        
        # 对cart位置进行惩罚（保持在中心附近）
        position_penalty = -0.1 * (cart_pos ** 2)
        
        # 对速度进行轻微惩罚（保持运动平滑）
        velocity_penalty = -0.1 * (cart_vel ** 2 + pole_vel ** 2)
        
        return upright_reward + position_penalty + velocity_penalty
    
    def _is_done(self, state):
        """检查是否完成（倒立摆倒下或小车移动过远）"""
        cart_pos, _, sin_pole, cos_pole, _ = state
        
        # 如果倒立摆倒下（cos_pole接近-1）或小车移动太远
        return cos_pole < -0.8 or abs(cart_pos) > 5.0

def start_server():
    """在单独的线程中启动MCP服务器"""
    logger.info("正在启动MuJoCo MCP服务器...")
    server_thread = threading.Thread(
        target=mujoco_mcp.start,
        kwargs={"host": "localhost", "port": 8000, "blocking": True},
        daemon=True
    )
    server_thread.start()
    time.sleep(1)  # 给服务器一些启动时间
    return server_thread

def run_simple_policy(env, episodes=3):
    """运行一个简单的控制策略"""
    for episode in range(episodes):
        state = env.reset()
        total_reward = 0
        steps = 0
        done = False
        
        logger.info(f"开始第 {episode + 1} 个回合")
        
        while not done and steps < 200:  # 最多运行200步
            # 简单的线性策略: 基于杆子的角度和角速度来控制小车
            cart_pos, cart_vel, sin_pole, cos_pole, pole_vel = state
            pole_angle = np.arctan2(sin_pole, cos_pole)
            
            # 简单的PD控制器
            kp, kd = 1.0, 0.3  # 比例和微分增益
            action = kp * pole_angle + kd * pole_vel
            
            # 执行动作
            state, reward, done, _ = env.step(action)
            total_reward += reward
            steps += 1
            
            if steps % 20 == 0:  # 每20步打印一次
                logger.info(f"  步骤 {steps}: 角度={np.degrees(pole_angle):.1f}°, 奖励={reward:.3f}, 总奖励={total_reward:.3f}")
        
        logger.info(f"回合 {episode + 1} 结束，总步数: {steps}, 总奖励: {total_reward:.3f}")

def main():
    """主程序"""
    logger.info("启动MuJoCo MCP强化学习示例")
    
    server_thread = start_server()
    
    try:
        # 连接到MCP服务器
        client = MCPClient("http://localhost:8000")
        
        # 创建环境
        env = MCPRLEnvironment(client, INVERTED_PENDULUM_XML)
        logger.info("环境已创建，开始运行示例策略...")
        
        # 运行示例策略
        run_simple_policy(env)
        
    except KeyboardInterrupt:
        logger.info("用户中断，正在关闭...")
    except Exception as e:
        logger.error(f"发生错误: {str(e)}")
    finally:
        # 停止服务器
        logger.info("正在停止服务器...")
        mujoco_mcp.stop()
        # 等待服务器线程结束
        server_thread.join(timeout=2)
        logger.info("程序退出")

if __name__ == "__main__":
    main() 