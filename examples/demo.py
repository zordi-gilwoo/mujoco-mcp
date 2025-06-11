#!/usr/bin/env python3
"""
MuJoCo-MCP 演示脚本
展示如何启动和使用增强的MuJoCo MCP系统

使用方法:
python demo.py

此脚本将:
1. 启动MuJoCo MCP服务器
2. 创建一个简单的模拟场景
3. 演示高级对象交互和轨迹规划功能
"""

import sys
import os
import time
import argparse
import asyncio
import logging
from typing import Dict, Any, List, Optional

# 添加源代码目录到Python路径
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(os.path.dirname(script_dir))
sys.path.insert(0, os.path.join(repo_root))

# 导入MuJoCo MCP模块
from mujoco_mcp.server import start_server
from mujoco_mcp.enhanced_simulation import EnhancedMuJoCoSimulation
from mujoco_mcp.enhanced_auth_manager import EnhancedAuthManager

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('mujoco_mcp_demo')

# 示例MuJoCo模型XML (简单的机械臂和几个物体)
EXAMPLE_MODEL_XML = """
<mujoco>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>
        
        <!-- 简单机器人 -->
        <body name="robot1" pos="0 0 0.5">
            <joint name="robot1_base_rot" type="ball"/>
            <geom name="robot1_base" type="cylinder" size="0.1 0.1" rgba="0.7 0.7 0.7 1"/>
            <body name="robot1_arm" pos="0 0 0.1">
                <joint name="robot1_shoulder" axis="0 1 0" range="-180 180"/>
                <geom name="robot1_arm_geom" type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" rgba="0.7 0.7 0.7 1"/>
                <body name="robot1_forearm" pos="0.5 0 0">
                    <joint name="robot1_elbow" axis="0 1 0" range="-90 90"/>
                    <geom name="robot1_forearm_geom" type="capsule" size="0.04" fromto="0 0 0 0.5 0 0" rgba="0.7 0.7 0.7 1"/>
                    <body name="robot1_wrist" pos="0.5 0 0">
                        <joint name="robot1_wrist_rot" axis="1 0 0" range="-180 180"/>
                        <geom name="robot1_wrist_geom" type="sphere" size="0.05" rgba="0.5 0.5 0.5 1"/>
                        <site name="robot1_grasp_site" pos="0 0 0.1" size="0.02"/>
                    </body>
                </body>
            </body>
        </body>
        
        <!-- 场景中的物体 -->
        <body name="red_cube" pos="1 0 0.1">
            <joint type="free"/>
            <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
        </body>
        
        <body name="blue_cube" pos="0 1 0.1">
            <joint type="free"/>
            <geom type="box" size="0.1 0.1 0.1" rgba="0 0 1 1"/>
        </body>
        
        <body name="green_sphere" pos="-1 0 0.1">
            <joint type="free"/>
            <geom type="sphere" size="0.1" rgba="0 1 0 1"/>
        </body>
    </worldbody>
    
    <actuator>
        <motor name="robot1_base_rot_motor" joint="robot1_base_rot"/>
        <motor name="robot1_shoulder_motor" joint="robot1_shoulder"/>
        <motor name="robot1_elbow_motor" joint="robot1_elbow"/>
        <motor name="robot1_wrist_motor" joint="robot1_wrist_rot"/>
    </actuator>
</mujoco>
"""

class DemoClient:
    """演示客户端，展示MuJoCo MCP的高级功能"""
    
    def __init__(self, server_port: int = 7777):
        """初始化演示客户端"""
        self.server_port = server_port
        self.server_process = None
        self.sim_id = None
        
    async def run_demo(self):
        """运行完整演示"""
        try:
            # 启动服务器
            logger.info("正在启动MuJoCo MCP服务器...")
            self.server_process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m", "mujoco_mcp.server",
                "--port", str(self.server_port),
                "--use-enhanced-auth",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            # 等待服务器启动
            logger.info("等待服务器启动...")
            await asyncio.sleep(2)
            
            # 导入MCP客户端
            # 注意：我们直接导入而不是作为模块导入，以避免循环依赖
            from mcp import ClientSession, WebSocketServerParameters
            
            # 连接到服务器
            logger.info("连接到MuJoCo MCP服务器...")
            ws_params = WebSocketServerParameters(
                host="localhost",
                port=self.server_port
            )
            
            session = None
            try:
                # 连接到WebSocket服务器
                ws_transport = await ws_params.connect()
                recv, send = ws_transport
                session = ClientSession(recv, send)
                await session.initialize()
                
                # 列出可用工具
                tools_response = await session.list_tools()
                logger.info(f"可用工具: {', '.join(tool.name for tool in tools_response.tools)}")
                
                # 启动模拟
                logger.info("启动MuJoCo模拟...")
                start_response = await session.execute_tool("start_simulation", {"model_xml": EXAMPLE_MODEL_XML})
                if start_response.result["success"]:
                    self.sim_id = start_response.result["data"]["sim_id"]
                    logger.info(f"模拟已启动，ID: {self.sim_id}")
                    
                    # 运行演示步骤
                    await self.run_demo_steps(session)
                else:
                    logger.error(f"启动模拟失败: {start_response.result.get('error', '未知错误')}")
                
            except Exception as e:
                logger.error(f"演示客户端错误: {str(e)}")
            finally:
                # 关闭会话
                if session:
                    try:
                        await session.close()
                    except:
                        pass
                        
        finally:
            # 终止服务器进程
            if self.server_process:
                try:
                    self.server_process.terminate()
                    await self.server_process.wait()
                except:
                    pass
    
    async def run_demo_steps(self, session):
        """运行演示步骤，展示各种功能"""
        logger.info("开始演示MuJoCo MCP高级功能...")
        
        # 获取场景对象
        logger.info("获取场景对象...")
        objects_response = await session.execute_tool("get_scene_objects", {"sim_id": self.sim_id})
        if objects_response.result["success"]:
            objects = objects_response.result["data"]["objects"]
            logger.info(f"场景中的对象: {', '.join(obj['id'] for obj in objects)}")
        else:
            logger.error(f"获取场景对象失败: {objects_response.result.get('error', '未知错误')}")
            return
        
        # 1. 获取机器人状态
        logger.info("获取机器人状态...")
        state_response = await session.execute_tool("get_robot_state", {
            "sim_id": self.sim_id,
            "robot_id": "robot1"
        })
        if state_response.result["success"]:
            robot_state = state_response.result["data"]
            logger.info(f"机器人状态: {robot_state}")
        
        # 暂停以便观察
        await asyncio.sleep(1)
        
        # 2. 移动到红色立方体
        logger.info("移动到红色立方体...")
        move_response = await session.execute_tool("move_to_object", {
            "sim_id": self.sim_id,
            "robot_id": "robot1",
            "object_id": "red_cube",
            "offset": [0, 0, 0.2],
            "speed": 0.5
        })
        
        if move_response.result["success"]:
            logger.info("已成功移动到红色立方体")
        else:
            logger.error(f"移动失败: {move_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(2)
        
        # 3. 抓取红色立方体
        logger.info("抓取红色立方体...")
        grasp_response = await session.execute_tool("grasp_object", {
            "sim_id": self.sim_id,
            "robot_id": "robot1",
            "object_id": "red_cube",
            "force": 50.0
        })
        
        if grasp_response.result["success"]:
            logger.info("已成功抓取红色立方体")
        else:
            logger.error(f"抓取失败: {grasp_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(2)
        
        # 4. 移动到新位置
        logger.info("移动到新位置...")
        move_pos_response = await session.execute_tool("move_to_position", {
            "sim_id": self.sim_id,
            "robot_id": "robot1",
            "position": [0.5, 0.5, 1.0],
            "speed": 0.5
        })
        
        if move_pos_response.result["success"]:
            logger.info("已成功移动到新位置")
        else:
            logger.error(f"移动失败: {move_pos_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(2)
        
        # 5. 释放物体
        logger.info("释放物体...")
        release_response = await session.execute_tool("release_object", {
            "sim_id": self.sim_id,
            "robot_id": "robot1"
        })
        
        if release_response.result["success"]:
            logger.info("已成功释放物体")
        else:
            logger.error(f"释放失败: {release_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(2)
        
        # 6. 规划轨迹并执行
        logger.info("规划并执行轨迹...")
        trajectory_response = await session.execute_tool("plan_trajectory", {
            "sim_id": self.sim_id,
            "robot_id": "robot1",
            "waypoints": [
                {"position": [0, 0, 1.0]},
                {"position": [-0.5, 0.5, 0.8]},
                {"position": [-1.0, 0, 0.6]}
            ],
            "avoid_obstacles": True,
            "speed": 0.3
        })
        
        if trajectory_response.result["success"]:
            logger.info("已成功规划并执行轨迹")
        else:
            logger.error(f"轨迹规划失败: {trajectory_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(3)
        
        # 7. 对绿色球施加力
        logger.info("对绿色球施加力...")
        force_response = await session.execute_tool("apply_force", {
            "sim_id": self.sim_id,
            "object_id": "green_sphere",
            "force": [10.0, 0.0, 5.0]  # 向右上方施加力
        })
        
        if force_response.result["success"]:
            logger.info("已成功对绿色球施加力")
        else:
            logger.error(f"施加力失败: {force_response.result.get('error', '未知错误')}")
        
        # 暂停以便观察
        await asyncio.sleep(2)
        
        # 8. 与蓝色立方体交互
        logger.info("与蓝色立方体交互...")
        interact_response = await session.execute_tool("interact_with_object", {
            "sim_id": self.sim_id,
            "robot_id": "robot1",
            "object_id": "blue_cube",
            "interaction_type": "push",
            "direction": [1.0, 0.0, 0.0]  # 向x轴正方向推
        })
        
        if interact_response.result["success"]:
            logger.info("已成功与蓝色立方体交互")
        else:
            logger.error(f"交互失败: {interact_response.result.get('error', '未知错误')}")
        
        # 演示完成
        logger.info("演示完成！MuJoCo MCP高级功能展示结束。")


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="MuJoCo-MCP 演示")
    parser.add_argument("--port", type=int, default=7777, help="服务器端口")
    args = parser.parse_args()
    
    # 运行演示
    client = DemoClient(server_port=args.port)
    await client.run_demo()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n演示已终止")
    except Exception as e:
        print(f"错误: {str(e)}")
    finally:
        print("演示已结束") 