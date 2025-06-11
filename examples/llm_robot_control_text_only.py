#!/usr/bin/env python3
"""
LLM机器人控制示例 (纯文本版): 展示如何使用大型语言模型(LLM)通过自然语言控制机器人
此版本不需要MuJoCo和可视化，适合在任何环境中运行
"""

import sys
import os
import time
import re
import random
import logging
from typing import Dict, List, Any, Optional, Tuple, Union

# 尝试导入OpenAI API (如果不可用，将使用模拟的LLM响应)
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    print("未找到openai模块，将使用模拟的LLM响应。")
    print("要启用真实LLM，请安装: pip install openai")
    OPENAI_AVAILABLE = False

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger("llm_robot_control")

class VirtualRobot:
    """虚拟机器人类，用于模拟机器人行为"""
    
    def __init__(self):
        """初始化虚拟机器人"""
        # 机器人初始状态
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # base_rot, shoulder, elbow, wrist
        self.hand_position = [0.8, 0.0, 0.3]  # x, y, z
        
        # 物体位置
        self.objects = {
            "red_cube": [0.6, 0.2, 0.15],
            "blue_ball": [0.5, -0.3, 0.15],
            "green_cylinder": [-0.2, 0.4, 0.15]
        }
        
        # 模拟参数
        self.time = 0.0
        self.timestep = 0.01
    
    def get_joint_positions(self):
        """获取关节位置"""
        return self.joint_positions
    
    def get_state_description(self):
        """获取机器人状态的文本描述"""
        # 格式化为文本描述
        description = [
            "机器人当前状态:",
            f"- 手臂末端位置: x={self.hand_position[0]:.2f}, y={self.hand_position[1]:.2f}, z={self.hand_position[2]:.2f}",
            f"- 关节角度: base={self.joint_positions[0]:.2f}°, shoulder={self.joint_positions[1]:.2f}°, elbow={self.joint_positions[2]:.2f}°, wrist={self.joint_positions[3]:.2f}°",
            f"- 模拟时间: {self.time:.2f}秒",
            "",
            "物体位置:",
            f"- 红色立方体: x={self.objects['red_cube'][0]:.2f}, y={self.objects['red_cube'][1]:.2f}, z={self.objects['red_cube'][2]:.2f}",
            f"- 蓝色球体: x={self.objects['blue_ball'][0]:.2f}, y={self.objects['blue_ball'][1]:.2f}, z={self.objects['blue_ball'][2]:.2f}",
            f"- 绿色圆柱体: x={self.objects['green_cylinder'][0]:.2f}, y={self.objects['green_cylinder'][1]:.2f}, z={self.objects['green_cylinder'][2]:.2f}"
        ]
        
        return "\n".join(description)
    
    def move_to_position(self, target_pos, max_steps=100):
        """移动手臂到指定位置"""
        # 初始距离
        initial_distance = self._calculate_distance(self.hand_position, target_pos)
        
        # 模拟移动过程
        step_count = random.randint(10, min(30, max_steps))
        
        # 最终距离 (0.0到0.05之间的随机值，有80%概率成功)
        success = random.random() < 0.8
        final_distance = random.uniform(0.0, 0.05) if success else random.uniform(0.05, 0.1)
        
        # 设置手臂位置为目标位置附近
        for i in range(3):
            self.hand_position[i] = target_pos[i] + (random.uniform(-0.05, 0.05) if success else random.uniform(-0.1, 0.1))
        
        # 更新关节角度 (简单的随机变化)
        for i in range(4):
            self.joint_positions[i] += random.uniform(-20, 20)
        
        # 更新时间
        self.time += self.timestep * step_count
        
        return success, step_count, final_distance
    
    def move_to_object(self, object_name, max_steps=100):
        """移动到指定物体"""
        if object_name not in self.objects:
            return False, "未知物体名称"
        
        # 获取物体位置
        object_pos = self.objects[object_name]
        
        # 稍微抬高目标点以便抓取
        target_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.1]
        
        # 移动到物体上方
        success, steps, distance = self.move_to_position(target_pos, max_steps)
        
        if success:
            return True, f"成功移动到{object_name}上方，距离: {distance:.3f}"
        else:
            return False, f"未能到达{object_name}上方，最终距离: {distance:.3f}"
    
    def reset(self):
        """重置机器人到初始状态"""
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.hand_position = [0.8, 0.0, 0.3]
        self.time = 0.0
        return True
    
    def step_simulation(self, num_steps=10):
        """推进模拟"""
        # 小幅度随机移动手臂
        for i in range(3):
            self.hand_position[i] += random.uniform(-0.05, 0.05)
        
        # 小幅度随机变化关节角度
        for i in range(4):
            self.joint_positions[i] += random.uniform(-5, 5)
        
        # 更新时间
        self.time += self.timestep * num_steps
        
        return True
    
    def _calculate_distance(self, pos1, pos2):
        """计算两点之间的距离"""
        return sum([(a - b) ** 2 for a, b in zip(pos1, pos2)]) ** 0.5

class LLMRobotInterface:
    """使用LLM控制机器人的接口"""
    
    def __init__(self, api_key=None):
        """初始化接口"""
        self.robot = VirtualRobot()
        self.conversation_history = []
        
        # 设置OpenAI API密钥（如果提供）
        if OPENAI_AVAILABLE and api_key:
            openai.api_key = api_key
        
        # 命令映射
        self.commands = {
            "move_to": self._move_to_position,
            "move_to_object": self._move_to_object,
            "reset": self._reset_robot,
            "step": self._step_simulation,
            "get_state": self._get_robot_state
        }
    
    def process_command(self, command):
        """根据LLM输出处理命令"""
        # 尝试识别命令类型和参数
        try:
            # 解析简单的指令格式 command(arg1, arg2, ...)
            match = re.match(r'(\w+)\s*\(([^)]*)\)', command)
            if match:
                cmd_name = match.group(1).lower()
                args_str = match.group(2)
                args = [arg.strip() for arg in args_str.split(',')] if args_str else []
                
                # 如果是已知命令，执行它
                if cmd_name in self.commands:
                    return self.commands[cmd_name](*args)
                else:
                    return False, f"未知命令: {cmd_name}"
            else:
                return False, "无法解析命令格式。请使用command(arg1, arg2, ...)的格式。"
        except Exception as e:
            return False, f"处理命令时发生错误: {str(e)}"
    
    def _move_to_position(self, x, y, z):
        """移动到指定位置"""
        try:
            x, y, z = float(x), float(y), float(z)
            success, steps, distance = self.robot.move_to_position([x, y, z])
            if success:
                return True, f"成功移动到位置 [{x:.2f}, {y:.2f}, {z:.2f}]，用了{steps}步，最终距离: {distance:.3f}"
            else:
                return False, f"未能到达位置 [{x:.2f}, {y:.2f}, {z:.2f}]，最终距离: {distance:.3f}"
        except ValueError:
            return False, "坐标必须是数字"
    
    def _move_to_object(self, object_name):
        """移动到指定物体"""
        return self.robot.move_to_object(object_name)
    
    def _reset_robot(self):
        """重置机器人"""
        self.robot.reset()
        return True, "机器人已重置"
    
    def _step_simulation(self, steps=10):
        """步进模拟"""
        try:
            steps = int(steps)
            self.robot.step_simulation(steps)
            return True, f"模拟已推进{steps}步"
        except ValueError:
            return False, "步数必须是整数"
    
    def _get_robot_state(self):
        """获取机器人状态"""
        state = self.robot.get_state_description()
        return True, state
    
    def chat_with_llm(self, user_message):
        """与LLM交互获取控制命令"""
        # 添加用户消息到历史
        self.conversation_history.append({"role": "user", "content": user_message})
        
        # 使用真实的OpenAI API
        if OPENAI_AVAILABLE and openai.api_key:
            try:
                # 构建系统提示，让LLM了解可用的命令
                system_prompt = """
                你是一个机器人控制助手，能够通过自然语言控制一个机械臂。
                可用命令:
                - move_to(x, y, z): 移动机械臂末端到指定的坐标
                - move_to_object(object_name): 移动到指定物体("red_cube", "blue_ball", "green_cylinder")
                - reset(): 重置机器人到初始状态
                - step(n): 步进模拟n步
                - get_state(): 获取机器人当前状态
                
                用户会用自然语言描述他们想要机器人执行的操作。请根据用户的描述，返回相应的命令。
                只返回一个命令，格式为command(arg1, arg2, ...)。不要在命令前后加其他文本。
                """
                
                messages = [
                    {"role": "system", "content": system_prompt}
                ]
                
                # 添加最近的对话历史，最多10条
                messages.extend(self.conversation_history[-10:])
                
                # 获取LLM回复
                response = openai.chat.completions.create(
                    model="gpt-3.5-turbo",  # 可以根据需要更换模型
                    messages=messages,
                    temperature=0.7,
                    max_tokens=150
                )
                
                llm_response = response.choices[0].message.content.strip()
                
                # 添加助手回复到历史
                self.conversation_history.append({"role": "assistant", "content": llm_response})
                
                return llm_response
                
            except Exception as e:
                logger.error(f"调用OpenAI API时出错: {str(e)}")
                # 出错时返回模拟响应
                return self._get_mock_response(user_message)
        else:
            # 没有API密钥或OpenAI模块时，使用模拟响应
            return self._get_mock_response(user_message)
    
    def _get_mock_response(self, user_message):
        """生成模拟的LLM响应"""
        user_message = user_message.lower()
        
        # 简单的规则匹配来生成响应
        if "移动" in user_message or "去" in user_message:
            if "红色" in user_message or "立方体" in user_message:
                return "move_to_object(red_cube)"
            elif "蓝色" in user_message or "球" in user_message:
                return "move_to_object(blue_ball)"
            elif "绿色" in user_message or "圆柱" in user_message:
                return "move_to_object(green_cylinder)"
            else:
                # 随机位置
                x, y, z = 0.5 + 0.2 * random.random(), 0.3 * random.random(), 0.3 + 0.2 * random.random()
                return f"move_to({x:.2f}, {y:.2f}, {z:.2f})"
        elif "重置" in user_message:
            return "reset()"
        elif "状态" in user_message or "位置" in user_message:
            return "get_state()"
        elif "步进" in user_message or "运行" in user_message:
            steps = 10
            if "步" in user_message:
                # 尝试提取步数
                try:
                    num = re.search(r'(\d+)', user_message)
                    if num:
                        steps = int(num.group(1))
                except:
                    pass
            return f"step({steps})"
        else:
            return "get_state()"
    
    def get_initial_state(self):
        """获取初始状态"""
        return self._get_robot_state()

def run_interactive_demo():
    """运行交互式演示"""
    print("=" * 60)
    print("    LLM机器人控制演示 (纯文本版)")
    print("=" * 60)
    print("\n输入自然语言指令来控制虚拟机器人，输入'退出'结束演示。")
    print("例如: '移动到红色立方体', '告诉我机器人的状态', '重置机器人'\n")
    
    # 可以从环境变量获取API密钥
    api_key = os.environ.get('OPENAI_API_KEY')
    if OPENAI_AVAILABLE and not api_key:
        print("警告: 未设置OpenAI API密钥，将使用模拟的LLM响应。")
        print("设置环境变量OPENAI_API_KEY来使用真实的LLM。\n")
    
    interface = LLMRobotInterface(api_key)
    
    # 获取初始状态
    try:
        success, state = interface.get_initial_state()
        print("\n初始状态:")
        print(state)
    except Exception as e:
        print(f"获取初始状态时出错: {str(e)}")
    
    # 交互循环
    try:
        while True:
            print("\n" + "-" * 40)
            user_input = input("请输入指令: ")
            
            if user_input.lower() in ['退出', 'quit', 'exit']:
                break
            
            print("处理中...")
            
            # 从LLM获取命令
            llm_command = interface.chat_with_llm(user_input)
            print(f"LLM解析的命令: {llm_command}")
            
            try:
                # 执行命令
                success, result = interface.process_command(llm_command)
                
                # 显示结果
                if success:
                    print("执行成功!")
                    print(result)
                else:
                    print(f"执行失败: {result}")
            except Exception as e:
                print(f"执行命令时出错: {str(e)}")
    
    except KeyboardInterrupt:
        print("\n用户中断，正在退出...")
    finally:
        print("演示已结束。")

def main():
    """主程序"""
    run_interactive_demo()

if __name__ == "__main__":
    main() 