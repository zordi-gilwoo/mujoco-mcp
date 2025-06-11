#!/usr/bin/env python3
"""
综合示例：展示如何使用MCP标准将大型语言模型(LLM)与MuJoCo物理引擎集成
"""

import asyncio
import os
import logging
import argparse
import json
import sys
import time
from typing import Dict, List, Any, Optional, Tuple, Union

# MCP客户端导入
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client
from contextlib import AsyncExitStack

# 尝试导入Anthropic API (如果不可用，将使用模拟的LLM响应)
try:
    from anthropic import Anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    print("未找到anthropic模块，将使用模拟的LLM响应。")
    print("要启用真实LLM，请安装: pip install anthropic")
    ANTHROPIC_AVAILABLE = False

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("mujoco_mcp_llm")


class SimulatedLLM:
    """模拟的LLM响应生成器，用于演示和测试"""
    
    def __init__(self):
        """初始化模拟LLM"""
        self.tool_calls_count = 0
        self.conversation_context = []
    
    async def generate_response(self, query: str, available_tools: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        生成模拟的LLM响应
        
        Args:
            query: 用户查询
            available_tools: 可用工具列表
            
        Returns:
            模拟的LLM响应
        """
        logger.info(f"模拟LLM处理查询: {query}")
        
        # 记录上下文
        self.conversation_context.append({"role": "user", "content": query})
        
        # 简单的关键词匹配来决定使用哪个工具
        tool_call = None
        content = None
        
        # 查询机器人状态
        if "状态" in query or "位置" in query or "查看" in query or "信息" in query:
            tool_name = "get_robot_state"
            for tool in available_tools:
                if tool["name"] == tool_name:
                    tool_call = {
                        "name": tool_name,
                        "input": {
                            "sim_id": "current",
                            "robot_id": "robot1"
                        }
                    }
                    break
        
        # 移动到位置
        elif "移动到" in query and "位置" in query:
            # 提取坐标，简单处理
            import re
            coords = re.findall(r'[-+]?\d*\.\d+|\d+', query)
            if len(coords) >= 3:
                x, y, z = map(float, coords[:3])
                tool_call = {
                    "name": "move_to_position",
                    "input": {
                        "sim_id": "current",
                        "robot_id": "robot1",
                        "position": [x, y, z],
                        "speed": 1.0
                    }
                }
        
        # 移动到物体
        elif "移动到" in query and ("物体" in query or "对象" in query or "立方体" in query or "球" in query):
            # 提取对象名称，简单处理
            obj_names = {
                "红色立方体": "red_cube", 
                "蓝色立方体": "blue_cube",
                "绿色球": "green_sphere"
            }
            
            object_id = None
            for cn_name, en_name in obj_names.items():
                if cn_name in query:
                    object_id = en_name
                    break
            
            if object_id:
                tool_call = {
                    "name": "move_to_object",
                    "input": {
                        "sim_id": "current",
                        "robot_id": "robot1",
                        "object_id": object_id,
                        "offset": [0, 0, 0.1],
                        "speed": 1.0
                    }
                }
        
        # 抓取物体
        elif "抓取" in query or "拿起" in query or "抓住" in query:
            # 提取对象名称，简单处理
            obj_names = {
                "红色立方体": "red_cube", 
                "蓝色立方体": "blue_cube",
                "绿色球": "green_sphere"
            }
            
            object_id = None
            for cn_name, en_name in obj_names.items():
                if cn_name in query:
                    object_id = en_name
                    break
            
            if object_id:
                tool_call = {
                    "name": "grasp_object",
                    "input": {
                        "sim_id": "current",
                        "robot_id": "robot1",
                        "object_id": object_id,
                        "force": 50.0
                    }
                }
        
        # 释放物体
        elif "释放" in query or "放下" in query or "放开" in query:
            tool_call = {
                "name": "release_object",
                "input": {
                    "sim_id": "current",
                    "robot_id": "robot1"
                }
            }
            
        # 获取场景对象
        elif "场景" in query or "环境" in query or "对象" in query:
            tool_call = {
                "name": "get_scene_objects",
                "input": {
                    "sim_id": "current"
                }
            }
            
        # 应用力
        elif "推" in query or "力" in query:
            # 提取对象名称和力的方向，简单处理
            obj_names = {
                "红色立方体": "red_cube", 
                "蓝色立方体": "blue_cube",
                "绿色球": "green_sphere"
            }
            
            object_id = None
            for cn_name, en_name in obj_names.items():
                if cn_name in query:
                    object_id = en_name
                    break
            
            if object_id:
                # 简单处理力的方向
                force = [10.0, 0.0, 0.0]  # 默认向x轴正方向
                if "向右" in query:
                    force = [10.0, 0.0, 0.0]
                elif "向左" in query:
                    force = [-10.0, 0.0, 0.0]
                elif "向前" in query:
                    force = [0.0, 10.0, 0.0]
                elif "向后" in query:
                    force = [0.0, -10.0, 0.0]
                elif "向上" in query:
                    force = [0.0, 0.0, 10.0]
                elif "向下" in query:
                    force = [0.0, 0.0, -10.0]
                    
                tool_call = {
                    "name": "apply_force",
                    "input": {
                        "sim_id": "current",
                        "object_id": object_id,
                        "force": force
                    }
                }
                
        # 没有匹配到工具调用
        if not tool_call:
            content = "抱歉，我不理解您的指令。请尝试使用诸如'查看机器人状态'、'移动到位置x,y,z'、'抓取红色立方体'等命令。"
        
        self.tool_calls_count += 1
        
        # 构建响应
        response = {
            "id": f"resp_{self.tool_calls_count}",
            "role": "assistant"
        }
        
        if tool_call:
            response["tool_calls"] = [tool_call]
        else:
            response["content"] = content
            
        # 记录上下文
        self.conversation_context.append(response)
        
        return response
    
    async def process_tool_results(self, tool_results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        处理工具调用结果
        
        Args:
            tool_results: 工具调用结果列表
            
        Returns:
            最终的LLM响应
        """
        logger.info(f"模拟LLM处理工具调用结果: {json.dumps(tool_results, indent=2)}")
        
        # 记录上下文
        for result in tool_results:
            self.conversation_context.append({
                "role": "tool",
                "name": result.get("name", "unknown_tool"),
                "content": json.dumps(result.get("result", {}))
            })
            
        # 根据工具结果生成自然语言回复
        content = None
        
        if tool_results:
            result = tool_results[0]["result"]
            tool_name = tool_results[0]["name"]
            
            if tool_name == "get_robot_state":
                if "success" in result and result["success"]:
                    robot_state = result["data"]
                    position = robot_state.get("end_effector_position", [0, 0, 0])
                    grasped = robot_state.get("grasped_object", None)
                    
                    content = f"机器人当前位于位置 [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]"
                    
                    if grasped:
                        content += f"，正在抓取物体 {grasped}"
                    else:
                        content += "，没有抓取任何物体"
                else:
                    content = "获取机器人状态失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "move_to_position":
                if "success" in result and result["success"]:
                    content = "已成功将机器人移动到指定位置"
                else:
                    content = "移动机器人失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "move_to_object":
                if "success" in result and result["success"]:
                    object_id = tool_results[0]["input"].get("object_id", "未知对象")
                    content = f"已成功将机器人移动到 {object_id} 附近"
                else:
                    content = "移动到对象失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "grasp_object":
                if "success" in result and result["success"]:
                    object_id = tool_results[0]["input"].get("object_id", "未知对象")
                    content = f"已成功抓取 {object_id}"
                else:
                    content = "抓取对象失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "release_object":
                if "success" in result and result["success"]:
                    if "released_object" in result["data"]:
                        object_id = result["data"]["released_object"]
                        content = f"已成功释放 {object_id}"
                    else:
                        content = "已成功释放物体"
                else:
                    content = "释放对象失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "get_scene_objects":
                if "success" in result and result["success"]:
                    objects = result["data"].get("objects", [])
                    if objects:
                        content = "场景中的对象:\n"
                        for obj in objects:
                            pos = obj.get("position", [0, 0, 0])
                            content += f"- {obj['id']}: 位置 [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]\n"
                    else:
                        content = "场景中没有对象"
                else:
                    content = "获取场景对象失败: " + result.get("error", "未知错误")
                    
            elif tool_name == "apply_force":
                if "success" in result and result["success"]:
                    object_id = tool_results[0]["input"].get("object_id", "未知对象")
                    force = tool_results[0]["input"].get("force", [0, 0, 0])
                    content = f"已成功对 {object_id} 施加力 [{force[0]:.1f}, {force[1]:.1f}, {force[2]:.1f}]"
                else:
                    content = "施加力失败: " + result.get("error", "未知错误")
        
        if not content:
            content = "已完成操作"
            
        # 构建最终响应
        response = {
            "id": f"final_{self.tool_calls_count}",
            "role": "assistant",
            "content": content
        }
        
        # 记录上下文
        self.conversation_context.append(response)
        
        return response


class MCPClient:
    """MCP客户端实现，集成LLM控制MuJoCo模拟"""
    
    def __init__(self):
        """初始化MCP客户端"""
        self.session = None
        self.exit_stack = AsyncExitStack()
        
        # 初始化LLM
        if ANTHROPIC_AVAILABLE and os.environ.get("ANTHROPIC_API_KEY"):
            self.llm = Anthropic()
            self.use_real_llm = True
            logger.info("使用Anthropic Claude LLM")
        else:
            self.llm = SimulatedLLM()
            self.use_real_llm = False
            logger.info("使用模拟LLM")
            
        # 存储MuJoCo模拟ID
        self.current_sim_id = None
    
    async def connect_to_server(self, server_script_path: str) -> bool:
        """
        连接到MCP服务器
        
        Args:
            server_script_path: 服务器脚本路径
            
        Returns:
            连接是否成功
        """
        try:
            # 检查脚本扩展名
            is_python = server_script_path.endswith('.py')
            is_js = server_script_path.endswith('.js')
            if not (is_python or is_js):
                logger.error("服务器脚本必须是.py或.js文件")
                return False
                
            # 设置命令和参数
            command = "python" if is_python else "node"
            server_params = StdioServerParameters(
                command=command,
                args=[server_script_path],
                env=None
            )
            
            # 连接服务器
            logger.info(f"正在连接到服务器: {command} {server_script_path}")
            stdio_transport = await self.exit_stack.enter_async_context(stdio_client(server_params))
            self.stdio, self.write = stdio_transport
            self.session = await self.exit_stack.enter_async_context(ClientSession(self.stdio, self.write))
            
            # 初始化会话
            await self.session.initialize()
            
            # 获取可用工具
            response = await self.session.list_tools()
            self.available_tools = response.tools
            logger.info(f"已连接到服务器，可用工具: {[tool.name for tool in self.available_tools]}")
            
            return True
            
        except Exception as e:
            logger.error(f"连接服务器失败: {str(e)}")
            return False
    
    async def close(self):
        """关闭连接"""
        await self.exit_stack.aclose()
        logger.info("已关闭连接")
    
    async def start_simulation(self, model_xml: str) -> Optional[str]:
        """
        启动MuJoCo模拟
        
        Args:
            model_xml: MuJoCo模型XML字符串
            
        Returns:
            模拟ID或None(如果失败)
        """
        try:
            # 查找启动模拟工具
            start_tool = None
            for tool in self.available_tools:
                if tool.name == "start_simulation":
                    start_tool = tool
                    break
                    
            if not start_tool:
                logger.error("服务器不支持start_simulation工具")
                return None
                
            # 调用工具启动模拟
            response = await self.session.execute_tool("start_simulation", {"model_xml": model_xml})
            
            if response.result["success"]:
                sim_id = response.result["data"]["sim_id"]
                self.current_sim_id = sim_id
                logger.info(f"已启动模拟，ID: {sim_id}")
                return sim_id
            else:
                logger.error(f"启动模拟失败: {response.result.get('error', '未知错误')}")
                return None
                
        except Exception as e:
            logger.error(f"启动模拟时出错: {str(e)}")
            return None
    
    async def process_query(self, query: str) -> str:
        """
        处理用户查询
        
        Args:
            query: 用户查询
            
        Returns:
            LLM响应内容
        """
        try:
            # 准备工具描述
            tool_descriptions = [{
                "name": tool.name,
                "description": tool.description,
                "input_schema": tool.inputSchema
            } for tool in self.available_tools]
            
            # 初始LLM调用
            if self.use_real_llm:
                # 使用真实的Anthropic API
                response = await self.llm.messages.create(
                    model="claude-3-opus-20240229",
                    max_tokens=1000,
                    messages=[{"role": "user", "content": query}],
                    tools=tool_descriptions
                )
                
                # 处理工具调用
                if hasattr(response, "content") and not response.content:
                    # 有工具调用
                    tool_calls = response.tool_calls
                    tool_results = []
                    
                    for tool_call in tool_calls:
                        tool_name = tool_call.name
                        tool_input = json.loads(tool_call.input)
                        
                        # 替换"current"为实际的模拟ID
                        if "sim_id" in tool_input and tool_input["sim_id"] == "current":
                            tool_input["sim_id"] = self.current_sim_id
                            
                        logger.info(f"调用工具: {tool_name}，参数: {json.dumps(tool_input, indent=2)}")
                        
                        # 执行工具调用
                        tool_response = await self.session.execute_tool(tool_name, tool_input)
                        
                        # 记录结果
                        tool_results.append({
                            "name": tool_name,
                            "input": tool_input,
                            "result": tool_response.result
                        })
                    
                    # 再次调用LLM处理工具结果
                    final_response = await self.llm.messages.create(
                        model="claude-3-opus-20240229",
                        max_tokens=1000,
                        messages=[
                            {"role": "user", "content": query},
                            {"role": "assistant", "tool_calls": tool_calls},
                            {"role": "user", "content": json.dumps({
                                "tool_results": tool_results
                            })}
                        ]
                    )
                    
                    return final_response.content[0].text
                    
                else:
                    # 无工具调用，直接返回内容
                    return response.content[0].text
                    
            else:
                # 使用模拟LLM
                response = await self.llm.generate_response(query, tool_descriptions)
                
                if "tool_calls" in response:
                    # 有工具调用
                    tool_results = []
                    
                    for tool_call in response["tool_calls"]:
                        tool_name = tool_call["name"]
                        tool_input = tool_call["input"]
                        
                        # 替换"current"为实际的模拟ID
                        if "sim_id" in tool_input and tool_input["sim_id"] == "current":
                            tool_input["sim_id"] = self.current_sim_id
                            
                        logger.info(f"调用工具: {tool_name}，参数: {json.dumps(tool_input, indent=2)}")
                        
                        # 执行工具调用
                        tool_response = await self.session.execute_tool(tool_name, tool_input)
                        
                        # 记录结果
                        tool_results.append({
                            "name": tool_name,
                            "input": tool_input,
                            "result": tool_response.result
                        })
                    
                    # 再次调用LLM处理工具结果
                    final_response = await self.llm.process_tool_results(tool_results)
                    return final_response["content"]
                    
                else:
                    # 无工具调用，直接返回内容
                    return response["content"]
                    
        except Exception as e:
            logger.error(f"处理查询时出错: {str(e)}")
            return f"处理查询时出错: {str(e)}"


async def interactive_session(client: MCPClient):
    """
    启动交互式会话
    
    Args:
        client: MCP客户端实例
    """
    print("\n====== MuJoCo-MCP LLM控制交互式会话 ======\n")
    print("输入命令控制MuJoCo模拟，输入'exit'退出\n")
    
    # 示例模型XML (简单场景，包含机器人和几个物体)
    model_xml = """
    <mujoco>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="floor" type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>
            
            <!-- 简单机器人 -->
            <body name="robot1_base" pos="0 0 0.5">
                <joint name="robot1_base_rot" type="ball"/>
                <geom name="robot1_base_geom" type="cylinder" size="0.1 0.1" rgba="0.7 0.7 0.7 1"/>
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
    
    # 启动模拟
    sim_id = await client.start_simulation(model_xml)
    if not sim_id:
        print("启动模拟失败，无法继续")
        return
    
    # 交互循环
    while True:
        # 获取用户输入
        try:
            query = input("\n> ")
        except EOFError:
            break
            
        # 检查退出命令
        if query.lower() in ['exit', 'quit', 'q']:
            break
            
        # 跳过空命令
        if not query.strip():
            continue
            
        # 处理查询
        print("\n处理中...")
        response = await client.process_query(query)
        print(f"\n{response}")


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="MuJoCo-MCP LLM控制示例")
    parser.add_argument('--server', type=str, help='MCP服务器路径 (例如: path/to/server.py)', required=False)
    parser.add_argument('--port', type=int, default=7777, help='MCP服务器端口')
    
    args = parser.parse_args()
    
    # 创建客户端
    client = MCPClient()
    
    # 如果提供了服务器路径，则连接到该服务器
    if args.server:
        success = await client.connect_to_server(args.server)
        if not success:
            print(f"无法连接到服务器: {args.server}")
            return
    else:
        # 否则尝试连接到本地服务器
        try:
            # 尝试在当前目录启动服务器
            server_script_path = "server.py"
            if not os.path.exists(server_script_path):
                # 查找源代码目录中的服务器
                repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                server_script_path = os.path.join(repo_root, "src", "mujoco_mcp", "server.py")
                
            logger.info(f"尝试连接到本地服务器: {server_script_path}")
            success = await client.connect_to_server(server_script_path)
            if not success:
                print("无法连接到本地服务器")
                return
        except Exception as e:
            print(f"连接服务器时出错: {str(e)}")
            return
    
    try:
        # 启动交互式会话
        await interactive_session(client)
    finally:
        # 关闭客户端
        await client.close()


if __name__ == "__main__":
    asyncio.run(main()) 