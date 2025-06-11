#!/usr/bin/env python3
"""
LLM机器人控制示例: 展示如何使用大型语言模型(LLM)通过自然语言控制MuJoCo中的机器人
"""

import sys
import os
import time
import threading
import logging
import json
import re
import numpy as np
from typing import Dict, List, Any, Optional, Tuple, Union

# 尝试导入mujoco和可视化工具
try:
    import mujoco
    import mujoco.viewer
    MUJOCO_AVAILABLE = True
except ImportError:
    print("未找到mujoco模块，将禁用可视化功能。")
    print("要启用可视化，请安装: pip install mujoco")
    MUJOCO_AVAILABLE = False

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

# MuJoCo模型XML - 可操控的机械臂
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
            <site name="tool_tip" pos="0.2 0 0" size="0.01"/>
          </body>
        </body>
      </body>
    </body>
    <!-- 可移动的物体 -->
    <body name="red_cube" pos="0.6 0.2 0.15">
      <joint type="free"/>
      <geom name="red_cube" type="box" size="0.05 0.05 0.05" rgba="1 0 0 1"/>
    </body>
    <body name="blue_ball" pos="0.5 -0.3 0.15">
      <joint type="free"/>
      <geom name="blue_ball" type="sphere" size="0.05" rgba="0 0 1 1"/>
    </body>
    <body name="green_cylinder" pos="-0.2 0.4 0.15">
      <joint type="free"/>
      <geom name="green_cylinder" type="cylinder" size="0.05 0.05" rgba="0 1 0 1"/>
    </body>
    <!-- 目标标记点 -->
    <site name="target" pos="0.7 0.5 0.5" size="0.02" rgba="1 1 0 0.5"/>
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
    <framepos name="hand_pos" objtype="site" objname="tool_tip"/>
    <framepos name="red_cube_pos" objtype="body" objname="red_cube"/>
    <framepos name="blue_ball_pos" objtype="body" objname="blue_ball"/>
    <framepos name="green_cylinder_pos" objtype="body" objname="green_cylinder"/>
    <framepos name="target_pos" objtype="site" objname="target"/>
  </sensor>
</mujoco>
"""

class MuJoCoSimulation:
    """简化版的MuJoCo模拟类"""
    
    def __init__(self):
        """初始化模拟"""
        self.model = None
        self.data = None
        self.xml = None
    
    def load_from_xml_string(self, xml_string):
        """从XML字符串加载模型"""
        if not MUJOCO_AVAILABLE:
            raise ImportError("未安装mujoco模块")
        
        self.xml = xml_string
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)
        
        # 初始化前向动力学
        mujoco.mj_forward(self.model, self.data)
    
    def step(self):
        """步进模拟"""
        mujoco.mj_step(self.model, self.data)
    
    def get_joint_positions(self):
        """获取关节位置"""
        return np.array(self.data.qpos)
    
    def get_joint_velocities(self):
        """获取关节速度"""
        return np.array(self.data.qvel)
    
    def get_sensor_data(self):
        """获取传感器数据"""
        # 读取传感器数据
        sensor_data = {}
        
        # 更新传感器
        mujoco.mj_forward(self.model, self.data)
        
        # 更新传感器数据 (注意：不使用mj_sensor，而是直接读取sensordata)
        
        # 复制传感器值
        for i in range(self.model.nsensor):
            name = self.model.sensor(i).name
            address = self.model.sensor(i).address
            dims = self.model.sensor(i).dim
            sensor_data[name] = np.array(self.data.sensordata[address:address+dims])
        
        return sensor_data
    
    def set_joint_positions(self, positions):
        """设置关节位置"""
        self.data.qpos[:] = positions
        mujoco.mj_forward(self.model, self.data)
    
    def apply_control(self, control):
        """应用控制输入"""
        self.data.ctrl[:] = control
    
    def reset(self):
        """重置模拟"""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

class RobotController:
    """控制MuJoCo中机器人的类"""
    
    def __init__(self):
        """初始化控制器"""
        self.simulation = None
        self.model = None
        self.data = None
        self.viewer = None
        self.running = False
        self.visualization_thread = None
        self.step_size = 0.01
        self.joint_names = ["base_rot", "shoulder", "elbow", "wrist"]
    
    def load_robot(self, xml_string=ROBOT_ARM_XML):
        """加载机器人模型"""
        logger.info("正在加载机器人模型...")
        self.simulation = MuJoCoSimulation()
        self.simulation.load_from_xml_string(xml_string)
        
        self.model = self.simulation.model
        self.data = self.simulation.data
    
    def start_visualization(self):
        """启动可视化"""
        if not MUJOCO_AVAILABLE:
            logger.warning("未安装mujoco模块，无法启动可视化")
            return False
        
        if not self.model or not self.data:
            logger.warning("未加载机器人模型，无法启动可视化")
            return False
        
        logger.info("启动可视化...")
        self.running = True
        
        # 在新线程中启动可视化，这样不会阻塞主线程
        self.visualization_thread = threading.Thread(target=self._run_visualization)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()
        return True
    
    def _run_visualization(self):
        """运行可视化循环"""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            
            # 保持运行直到停止
            while self.running:
                # 更新可视化
                viewer.sync()
                
                time.sleep(0.01)  # 限制更新速率
    
    def get_joint_positions(self):
        """获取当前关节位置"""
        return self.simulation.get_joint_positions().tolist()
    
    def get_joint_names(self):
        """获取关节名称"""
        return self.joint_names
    
    def get_sensor_data(self):
        """获取传感器数据"""
        return self.simulation.get_sensor_data()
    
    def get_robot_state(self):
        """获取机器人状态的文本描述"""
        sensor_data = self.get_sensor_data()
        joint_positions = self.get_joint_positions()
        
        # 提取关键位置
        hand_pos = sensor_data["hand_pos"]
        red_cube_pos = sensor_data["red_cube_pos"]
        blue_ball_pos = sensor_data["blue_ball_pos"]
        green_cylinder_pos = sensor_data["green_cylinder_pos"]
        
        # 格式化为文本描述
        description = [
            "机器人当前状态:",
            f"- 手臂末端位置: x={hand_pos[0]:.2f}, y={hand_pos[1]:.2f}, z={hand_pos[2]:.2f}",
            f"- 关节角度: base={joint_positions[0]:.2f}°, shoulder={joint_positions[1]:.2f}°, elbow={joint_positions[2]:.2f}°, wrist={joint_positions[3]:.2f}°",
            "",
            "物体位置:",
            f"- 红色立方体: x={red_cube_pos[0]:.2f}, y={red_cube_pos[1]:.2f}, z={red_cube_pos[2]:.2f}",
            f"- 蓝色球体: x={blue_ball_pos[0]:.2f}, y={blue_ball_pos[1]:.2f}, z={blue_ball_pos[2]:.2f}",
            f"- 绿色圆柱体: x={green_cylinder_pos[0]:.2f}, y={green_cylinder_pos[1]:.2f}, z={green_cylinder_pos[2]:.2f}"
        ]
        
        return "\n".join(description)
    
    def set_joint_positions(self, positions):
        """设置关节位置"""
        self.simulation.set_joint_positions(positions)
        return True
    
    def apply_control(self, control):
        """应用控制输入"""
        self.simulation.apply_control(control)
        return True
    
    def step_simulation(self, num_steps=1):
        """推进模拟"""
        for _ in range(num_steps):
            self.simulation.step()
        return True
    
    def reset_simulation(self):
        """重置模拟"""
        self.simulation.reset()
        return True
    
    def move_to_position(self, target_pos, max_steps=100, step_size=5):
        """移动手臂到指定位置"""
        logger.info(f"开始移动到位置: {target_pos}, 最大步数: {max_steps}")
        
        for step in range(max_steps):
            # 获取当前状态
            sensor_data = self.get_sensor_data()
            hand_pos = sensor_data["hand_pos"]
            
            # 计算到目标的距离
            distance = sum([(a - b) ** 2 for a, b in zip(hand_pos, target_pos)]) ** 0.5
            
            if distance < 0.05:  # 如果足够接近目标，就停止
                logger.info(f"已到达目标附近，距离: {distance:.3f}")
                return True, step + 1, distance
            
            # 简单的控制策略 (这是一个极度简化的IK)
            control = [0, 0, 0, 0]  # 四个关节的控制
            
            # 基于当前位置和目标位置的差异计算控制信号
            for i, (current, target) in enumerate(zip(hand_pos, target_pos)):
                error = target - current
                # 为不同关节分配不同的增益
                gains = [0.5, 0.3, 0.2, 0.1]
                for j in range(4):
                    control[j] += error * gains[j] * (0.8 ** i)
            
            # 应用控制
            self.apply_control(control)
            
            # 步进模拟
            self.step_simulation(step_size)
            
            if step % 10 == 0:  # 每10步打印一次状态
                logger.info(f"步骤 {step}: 距离={distance:.3f}, 控制={[f'{c:.2f}' for c in control]}")
        
        # 如果达到最大步数仍未到达目标
        sensor_data = self.get_sensor_data()
        hand_pos = sensor_data["hand_pos"]
        final_distance = sum([(a - b) ** 2 for a, b in zip(hand_pos, target_pos)]) ** 0.5
        logger.info(f"达到最大步数，最终距离: {final_distance:.3f}")
        return False, max_steps, final_distance
    
    def move_to_object(self, object_name, max_steps=100):
        """移动到指定物体"""
        if object_name not in ["red_cube", "blue_ball", "green_cylinder"]:
            return False, "未知物体名称"
        
        # 获取物体位置
        sensor_data = self.get_sensor_data()
        object_pos = sensor_data[f"{object_name}_pos"]
        
        # 稍微抬高目标点以便抓取
        target_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.1]
        
        # 移动到物体上方
        success, steps, distance = self.move_to_position(target_pos, max_steps)
        
        if success:
            return True, f"成功移动到{object_name}上方，距离: {distance:.3f}"
        else:
            return False, f"未能到达{object_name}上方，最终距离: {distance:.3f}"
    
    def stop(self):
        """停止控制器"""
        logger.info("正在停止控制器...")
        self.running = False
        
        if self.visualization_thread:
            self.visualization_thread.join(timeout=2)
        
        if self.viewer:
            self.viewer.close()
        
        logger.info("控制器已停止")

class LLMRobotInterface:
    """使用LLM控制机器人的接口"""
    
    def __init__(self, api_key=None):
        """初始化接口"""
        self.robot = RobotController()
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
    
    def initialize(self):
        """初始化机器人和可视化"""
        self.robot.load_robot()
        visualization_available = self.robot.start_visualization()
        return visualization_available
    
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
        self.robot.reset_simulation()
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
        state = self.robot.get_robot_state()
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
                你是一个机器人控制助手，能够通过自然语言控制MuJoCo中的一个机械臂。
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
                x, y, z = 0.5 + 0.2 * np.random.random(), 0.3 * np.random.random(), 0.3 + 0.2 * np.random.random()
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
    
    def stop(self):
        """停止接口"""
        self.robot.stop()

def run_interactive_demo():
    """运行交互式演示"""
    print("=" * 60)
    print("    LLM机器人控制演示")
    print("=" * 60)
    print("\n输入自然语言指令来控制机器人，输入'退出'结束演示。")
    print("例如: '移动到红色立方体', '告诉我机器人的状态', '重置机器人'\n")
    
    # 检查是否在macOS下通过mjpython运行
    if sys.platform == 'darwin' and not os.environ.get('MUJOCO_GL'):
        print("警告: 在macOS上，必须通过mjpython运行此脚本才能启用可视化。")
        print("请使用以下命令运行: mjpython examples/llm_robot_control.py\n")
    
    # 可以从环境变量获取API密钥
    api_key = os.environ.get('OPENAI_API_KEY')
    if OPENAI_AVAILABLE and not api_key:
        print("警告: 未设置OpenAI API密钥，将使用模拟的LLM响应。")
        print("设置环境变量OPENAI_API_KEY来使用真实的LLM。\n")
    
    interface = LLMRobotInterface(api_key)
    
    # 初始化
    print("正在初始化机器人和可视化...")
    viz_available = False
    
    try:
        # 尝试初始化机器人和可视化
        viz_available = interface.initialize()
    except RuntimeError as e:
        if "mjpython" in str(e) and sys.platform == 'darwin':
            print("错误: 在macOS上，必须通过mjpython运行此脚本。")
            print("请使用以下命令运行: mjpython examples/llm_robot_control.py")
            sys.exit(1)
        else:
            raise
    
    if not viz_available:
        print("警告: 可视化不可用，将只在命令行显示结果。")
    
    # 获取初始状态
    try:
        success, state = interface._get_robot_state()
        print("\n初始状态:")
        print(state)
    except Exception as e:
        print(f"获取初始状态时出错: {str(e)}")
        print("将继续执行，但某些功能可能不可用。")
    
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
        # 清理
        interface.stop()
        print("演示已结束。")

def main():
    """主程序"""
    run_interactive_demo()

if __name__ == "__main__":
    main() 