#!/usr/bin/env python3
"""
AI代理示例: 演示如何使用MuJoCo MCP服务器与AI代理集成
"""

import sys
import time
import threading
import logging
import json
from model_context_protocol import MCPClient, MCPServer, Tool, Resource, MCPRequest, MCPResponse

# 导入mujoco_mcp模块
try:
    import mujoco_mcp
except ImportError:
    print("未找到mujoco_mcp模块。请确保已经安装该模块。")
    print("尝试执行: pip install -e .")
    sys.exit(1)

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ai_agent_example")

# MuJoCo模型XML - 简单的机械臂
ROBOT_ARM_XML = """
<mujoco model="robot_arm">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="texplane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2" width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="matplane" texture="texplane" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>
  <worldbody>
    <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 5" dir="0 0 -1"/>
    <geom name="floor" pos="0 0 0" size="5 5 0.1" type="plane" material="matplane"/>
    <body name="base" pos="0 0 0.1">
      <joint name="base_rot" type="hinge" axis="0 0 1" limited="true" range="-180 180"/>
      <geom name="base" type="cylinder" size="0.1 0.05" rgba="0.5 0.5 0.5 1"/>
      <body name="upper_arm" pos="0 0 0.15">
        <joint name="shoulder" type="hinge" axis="0 1 0" limited="true" range="-90 90"/>
        <geom name="upper_arm" type="capsule" fromto="0 0 0 0.4 0 0" size="0.04" rgba="0.7 0.7 0 1"/>
        <body name="forearm" pos="0.4 0 0">
          <joint name="elbow" type="hinge" axis="0 1 0" limited="true" range="-120 0"/>
          <geom name="forearm" type="capsule" fromto="0 0 0 0.4 0 0" size="0.03" rgba="0 0.7 0.7 1"/>
          <body name="hand" pos="0.4 0 0">
            <joint name="wrist" type="hinge" axis="0 1 0" limited="true" range="-90 90"/>
            <geom name="hand" type="box" pos="0.1 0 0" size="0.1 0.02 0.05" rgba="0.7 0 0.7 1"/>
            <site name="tool_center" pos="0.2 0 0" size="0.01"/>
          </body>
        </body>
      </body>
    </body>
    <body name="target" pos="0.7 0.5 0.5">
      <geom name="target" type="sphere" size="0.05" rgba="1 0 0 0.5"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="base_rot_motor" joint="base_rot" gear="30"/>
    <motor name="shoulder_motor" joint="shoulder" gear="30"/>
    <motor name="elbow_motor" joint="elbow" gear="20"/>
    <motor name="wrist_motor" joint="wrist" gear="10"/>
  </actuator>
  <sensor>
    <jointpos name="base_rot_pos" joint="base_rot"/>
    <jointpos name="shoulder_pos" joint="shoulder"/>
    <jointpos name="elbow_pos" joint="elbow"/>
    <jointpos name="wrist_pos" joint="wrist"/>
    <site name="tool_pos" site="tool_center" />
  </sensor>
</mujoco>
"""

class AIAgentServer(MCPServer):
    """模拟AI代理的MCP服务器"""
    
    def __init__(self, host="localhost", port=8001):
        super().__init__(service_name="ai-agent", host=host, port=port)
        self.logger = logging.getLogger("ai_agent")
        self.mcp_client = None
        self.target_pos = [0.7, 0.5, 0.5]  # 目标位置，与XML中定义的匹配
        self._setup_resources_and_tools()
    
    def connect_to_mujoco(self, mcp_url):
        """连接到MuJoCo MCP服务器"""
        self.mcp_client = MCPClient(mcp_url)
        self.logger.info(f"已连接到MuJoCo MCP服务器: {mcp_url}")
    
    def _setup_resources_and_tools(self):
        """设置MCP资源和工具"""
        self.register_resource("robot_state", self._handle_robot_state)
        self.register_resource("target_position", self._handle_target_position)
        
        self.register_tool("move_to_target", self._move_to_target)
        self.register_tool("reset_robot", self._reset_robot)
    
    def _handle_robot_state(self, request: MCPRequest) -> Resource:
        """处理机器人状态资源请求"""
        if not self.mcp_client:
            return Resource(error="未连接到MuJoCo服务器")
        
        sim_id = request.params.get("simulation_id")
        if not sim_id:
            return Resource(error="缺少simulation_id参数")
        
        try:
            # 获取关节位置
            joints = self.mcp_client.get_resource("joint_positions", {"simulation_id": sim_id})
            
            # 获取传感器数据
            sensors = self.mcp_client.get_resource("sensor_data", {"simulation_id": sim_id})
            
            # 获取末端执行器位置
            tool_pos = sensors.get("tool_pos", [0, 0, 0])
            
            return Resource(data={
                "joints": joints,
                "tool_position": tool_pos
            })
        except Exception as e:
            self.logger.error(f"获取机器人状态时发生错误: {str(e)}")
            return Resource(error=f"获取机器人状态时发生错误: {str(e)}")
    
    def _handle_target_position(self, request: MCPRequest) -> Resource:
        """处理目标位置资源请求"""
        return Resource(data=self.target_pos)
    
    def _move_to_target(self, request: MCPRequest) -> MCPResponse:
        """将机器人移动到目标位置"""
        if not self.mcp_client:
            return MCPResponse(error="未连接到MuJoCo服务器")
        
        sim_id = request.params.get("simulation_id")
        if not sim_id:
            return MCPResponse(error="缺少simulation_id参数")
        
        max_steps = request.params.get("max_steps", 100)
        
        try:
            # 简单的迭代IK求解方法
            self.logger.info(f"开始将机器人移动到目标位置，最大步数: {max_steps}")
            
            for step in range(max_steps):
                # 获取当前状态
                state = self._handle_robot_state(MCPRequest(params={"simulation_id": sim_id}))
                if state.error:
                    return MCPResponse(error=state.error)
                
                tool_pos = state.data["tool_position"]
                joints = state.data["joints"]
                
                # 计算到目标的距离
                distance = sum([(a - b) ** 2 for a, b in zip(tool_pos, self.target_pos)]) ** 0.5
                
                if distance < 0.05:  # 如果足够接近目标，就停止
                    self.logger.info(f"已到达目标附近，距离: {distance:.3f}")
                    break
                
                # 简单的雅可比矩阵近似（这只是一个示例，实际应用中使用更复杂的IK）
                # 调整每个关节一点点，看末端执行器如何移动
                control = [0, 0, 0, 0]  # 四个关节的控制
                
                # 根据当前位置与目标位置的差异，创建一个简单的控制信号
                # 注意: 这是非常简化的，不是真正的IK
                for i, (current, target) in enumerate(zip(tool_pos, self.target_pos)):
                    error = target - current
                    # 为不同关节分配不同的增益
                    gains = [0.5, 0.3, 0.2, 0.1]
                    for j in range(4):
                        control[j] += error * gains[j] * (0.8 ** i)  # 影响随着维度增加而减少
                
                # 应用控制
                self.mcp_client.call_tool("apply_control", {
                    "simulation_id": sim_id,
                    "control": control
                })
                
                # 向前推进模拟
                self.mcp_client.call_tool("step_simulation", {
                    "simulation_id": sim_id,
                    "num_steps": 5
                })
                
                if step % 10 == 0:  # 每10步打印一次
                    self.logger.info(f"步骤 {step}: 距离={distance:.3f}, 控制={[f'{c:.2f}' for c in control]}")
            
            return MCPResponse(data={
                "status": "success",
                "steps_taken": step + 1,
                "final_distance": distance
            })
            
        except Exception as e:
            self.logger.error(f"移动机器人时发生错误: {str(e)}")
            return MCPResponse(error=f"移动机器人时发生错误: {str(e)}")
    
    def _reset_robot(self, request: MCPRequest) -> MCPResponse:
        """重置机器人到初始状态"""
        if not self.mcp_client:
            return MCPResponse(error="未连接到MuJoCo服务器")
        
        sim_id = request.params.get("simulation_id")
        if not sim_id:
            return MCPResponse(error="缺少simulation_id参数")
        
        try:
            # 重置模拟
            self.mcp_client.call_tool("reset_simulation", {"simulation_id": sim_id})
            return MCPResponse(data={"status": "success"})
        except Exception as e:
            self.logger.error(f"重置机器人时发生错误: {str(e)}")
            return MCPResponse(error=f"重置机器人时发生错误: {str(e)}")

def start_mujoco_server():
    """在单独的线程中启动MuJoCo MCP服务器"""
    logger.info("正在启动MuJoCo MCP服务器...")
    server_thread = threading.Thread(
        target=mujoco_mcp.start,
        kwargs={"host": "localhost", "port": 8000, "blocking": True},
        daemon=True
    )
    server_thread.start()
    time.sleep(1)  # 给服务器一些启动时间
    return server_thread

def start_ai_agent_server():
    """在单独的线程中启动AI代理服务器"""
    logger.info("正在启动AI代理服务器...")
    ai_server = AIAgentServer(host="localhost", port=8001)
    server_thread = threading.Thread(
        target=ai_server.start,
        daemon=True
    )
    server_thread.start()
    time.sleep(1)  # 给服务器一些启动时间
    return ai_server, server_thread

def run_demo():
    """运行演示"""
    # 连接到服务器
    mujoco_client = MCPClient("http://localhost:8000")
    ai_client = MCPClient("http://localhost:8001")
    
    # 让AI代理连接到MuJoCo服务器
    ai_agent_server = AIAgentServer.get_server_instance()
    ai_agent_server.connect_to_mujoco("http://localhost:8000")
    
    # 启动模拟
    logger.info("正在启动机器人手臂模拟...")
    result = mujoco_client.call_tool("start_simulation", {"model_xml": ROBOT_ARM_XML})
    sim_id = result["simulation_id"]
    logger.info(f"模拟ID: {sim_id}")
    
    # 先重置机器人
    logger.info("重置机器人到初始状态...")
    ai_client.call_tool("reset_robot", {"simulation_id": sim_id})
    
    # 获取机器人当前状态
    robot_state = ai_client.get_resource("robot_state", {"simulation_id": sim_id})
    logger.info(f"机器人初始状态: {json.dumps(robot_state, indent=2)}")
    
    # 获取目标位置
    target_pos = ai_client.get_resource("target_position")
    logger.info(f"目标位置: {target_pos}")
    
    # 让AI代理控制机器人移动到目标
    logger.info("开始AI代理控制机器人移动到目标...")
    result = ai_client.call_tool("move_to_target", {"simulation_id": sim_id, "max_steps": 150})
    logger.info(f"移动结果: {json.dumps(result, indent=2)}")
    
    # 再次获取机器人状态
    robot_state = ai_client.get_resource("robot_state", {"simulation_id": sim_id})
    logger.info(f"最终机器人状态: {json.dumps(robot_state, indent=2)}")
    
    # 清理
    logger.info("正在删除模拟...")
    mujoco_client.call_tool("delete_simulation", {"simulation_id": sim_id})
    logger.info("演示完成!")

def main():
    """主程序"""
    logger.info("启动MuJoCo MCP与AI代理集成示例")
    
    mujoco_thread = start_mujoco_server()
    ai_server, ai_thread = start_ai_agent_server()
    
    try:
        time.sleep(2)  # 给服务器一些启动时间
        run_demo()
    except KeyboardInterrupt:
        logger.info("用户中断，正在关闭...")
    except Exception as e:
        logger.error(f"发生错误: {str(e)}")
    finally:
        # 停止服务器
        logger.info("正在停止服务器...")
        mujoco_mcp.stop()
        # 等待服务器线程结束
        mujoco_thread.join(timeout=2)
        ai_thread.join(timeout=2)
        logger.info("程序退出")

if __name__ == "__main__":
    main() 