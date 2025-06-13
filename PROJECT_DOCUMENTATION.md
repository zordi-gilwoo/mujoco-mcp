# MuJoCo MCP - 项目文档

<div align="center">

![MuJoCo MCP](https://img.shields.io/badge/MuJoCo-MCP-blue)
![Version](https://img.shields.io/badge/version-0.7.4-green)
![License](https://img.shields.io/badge/license-MIT-blue)
![Python](https://img.shields.io/badge/python-3.10+-blue)
![MCP](https://img.shields.io/badge/MCP-2024--11--05-orange)

**用自然语言控制物理世界的机器人**

[English](./README.md) | 简体中文

</div>

---

## 📋 目录

- [项目概述](#项目概述)
- [核心特性](#核心特性)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [使用指南](#使用指南)
- [API参考](#api参考)
- [机器人模型库](#机器人模型库)
- [开发指南](#开发指南)
- [故障排除](#故障排除)
- [性能优化](#性能优化)
- [版本历史](#版本历史)
- [路线图](#路线图)
- [贡献指南](#贡献指南)
- [许可证](#许可证)

---

## 🎯 项目概述

### 什么是 MuJoCo MCP？

MuJoCo MCP 是一个革命性的开源项目，它通过实现 Model Context Protocol (MCP) 标准，让任何人都可以使用自然语言来控制 MuJoCo 物理引擎中的机器人。

### 核心价值

- **零门槛**：不需要编程经验，用中文对话即可控制机器人
- **即时响应**：从想法到执行小于100毫秒
- **全面覆盖**：支持57种主流机器人模型
- **标准协议**：遵循Anthropic官方MCP规范
- **开源免费**：MIT许可证，商用无忧

### 应用场景

| 领域 | 应用 | 价值 |
|------|------|------|
| 🎓 **教育** | 机器人编程教学 | 降低学习门槛，提高教学效率 |
| 🔬 **研究** | AI与机器人交互研究 | 加速实验迭代，专注算法创新 |
| 🏭 **工业** | 快速原型验证 | 缩短开发周期，降低成本 |
| 🎮 **娱乐** | 机器人游戏开发 | 激发创意，丰富交互体验 |

---

## ✨ 核心特性

### 1. 自然语言控制

```python
# 传统方式：需要编写复杂代码
robot = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(robot)
data.qpos[0] = 0.5
data.qpos[1] = -0.3
# ... 数百行代码

# MuJoCo MCP：一句话搞定
"创建一个机械臂并让它抓取红色方块"
```

### 2. 丰富的机器人库

- **机械臂** (17种)：Franka、UR系列、KUKA、ABB等
- **四足机器人** (12种)：Unitree Go2、ANYmal、Spot等
- **人形机器人** (8种)：Unitree H1/G1、Atlas、Cassie等
- **灵巧手** (6种)：Shadow Hand、Leap Hand、Allegro等
- **移动机器人** (7种)：Stretch、Fetch、PR2等
- **其他** (7种)：无人机、并联机器人等

### 3. 实时可视化

- 原生MuJoCo GUI界面
- 实时3D渲染
- 关节状态显示
- 力/力矩可视化
- 轨迹跟踪

### 4. 高级功能

- **视频录制**：MP4/GIF格式导出
- **截图捕获**：PNG格式高清截图
- **批量控制**：同时控制多个机器人
- **状态监控**：实时获取位置、速度、力反馈
- **场景定制**：自定义环境和物体

---

## 🏗️ 系统架构

### 整体架构

```
┌─────────────────┐     ┌─────────────────┐     ┌──────────────────┐
│  Claude Desktop │────▶│   MCP Server    │────▶│  MuJoCo Viewer   │
│  (自然语言输入) │◀────│  (协议转换层)   │◀────│  (物理仿真引擎)  │
└─────────────────┘     └─────────────────┘     └──────────────────┘
         ▲                       │                        │
         │                       ▼                        ▼
         │              ┌─────────────────┐     ┌──────────────────┐
         └──────────────│  业务逻辑层     │     │   渲染输出层     │
                        │  (场景/控制)    │     │  (图像/视频)    │
                        └─────────────────┘     └──────────────────┘
```

### 核心组件

#### 1. MCP Server (`mcp_server_remote.py`)
- **功能**：实现MCP协议，处理自然语言请求
- **通信**：标准输入输出（stdio）
- **协议**：JSON-RPC 2.0

#### 2. MuJoCo Viewer Server (`mujoco_viewer_server.py`)
- **功能**：管理物理仿真和GUI渲染
- **API**：使用官方`mujoco.viewer.launch_passive()`
- **通信**：Socket服务器（默认端口8888）

#### 3. Remote Server (`src/mujoco_mcp/remote_server.py`)
- **功能**：业务逻辑处理，场景管理
- **特性**：多模型支持，状态管理

#### 4. Menagerie Loader (`src/mujoco_mcp/menagerie_loader.py`)
- **功能**：加载和管理机器人模型
- **来源**：Google DeepMind Mujoco Menagerie

---

## 🚀 快速开始

### 环境要求

- Python 3.10+
- macOS、Linux 或 Windows
- MuJoCo 3.1.0+
- Claude Desktop App

### 安装步骤

#### 1. 克隆项目

```bash
git clone https://github.com/robotlearning123/mujoco-mcp.git
cd mujoco-mcp
```

#### 2. 创建虚拟环境

```bash
# 使用 conda
conda create -n mujoco-mcp python=3.10
conda activate mujoco-mcp

# 或使用 venv
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
```

#### 3. 安装依赖

```bash
# 安装项目
pip install -e .

# 验证安装
python -c "import mujoco; print(f'MuJoCo {mujoco.__version__}')"
```

#### 4. 克隆机器人模型库

```bash
# 克隆 Menagerie（如果尚未克隆）
cd ..
git clone https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco-mcp
```

#### 5. 配置 Claude Desktop

编辑 Claude Desktop 配置文件：

**macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
**Windows**: `%APPDATA%\Claude\claude_desktop_config.json`

```json
{
  "mcpServers": {
    "mujoco-mcp-remote": {
      "command": "/path/to/python",
      "args": ["/path/to/mujoco-mcp/mcp_server_remote.py"],
      "cwd": "/path/to/mujoco-mcp",
      "env": {
        "PYTHONUNBUFFERED": "1",
        "MENAGERIE_ROOT": "/path/to/mujoco_menagerie"
      }
    }
  }
}
```

### 启动系统

```bash
# 步骤1：启动 MuJoCo Viewer Server
python mujoco_viewer_server.py

# 步骤2：重启 Claude Desktop
# 系统会自动连接

# 步骤3：在 Claude 中测试
# 输入："创建一个四足机器人"
```

---

## 📖 使用指南

### 基础命令

#### 1. 创建机器人

```
"创建一个机械臂"
"加载 Shadow Hand 灵巧手"
"显示人形机器人 Unitree H1"
```

#### 2. 控制运动

```
"让机械臂抬起来"
"设置第一个关节角度为45度"
"让机器人做T-Pose姿势"
"控制四足机器人向前走"
```

#### 3. 场景操作

```
"重置机器人到初始位置"
"暂停仿真"
"步进100步"
"关闭当前机器人"
```

#### 4. 捕获输出

```
"截图当前画面"
"录制3秒视频"
"保存机器人状态"
```

### 高级用法

#### 1. 复杂动作序列

```python
# 在 Claude 中输入
"创建 Franka 机械臂，先移动到预抓取位置，然后闭合夹爪，最后抬起物体"
```

#### 2. 多机器人协作

```python
# 依次创建多个机器人
"创建一个机械臂"
"再创建一个四足机器人"
"让它们同时运动"
```

#### 3. 自然语言编程

```python
# 描述复杂任务
"创建一个会跳舞的人形机器人，每个动作持续1秒，包括挥手、转身和鞠躬"
```

---

## 🔧 API 参考

### MCP Tools

#### 1. `create_scene`
创建新的机器人场景

**参数**：
- `scene_type` (str): 机器人名称或预设场景

**返回**：
- `model_id` (str): 模型唯一标识
- `model_info` (dict): 模型信息（关节数、刚体数等）

**示例**：
```python
{
  "scene_type": "franka_emika_panda"
}
```

#### 2. `set_joint_positions`
设置关节位置

**参数**：
- `model_id` (str): 模型标识
- `positions` (list): 关节角度列表（弧度）

**示例**：
```python
{
  "model_id": "uuid-xxx",
  "positions": [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.0]
}
```

#### 3. `step_simulation`
步进物理仿真

**参数**：
- `model_id` (str): 模型标识
- `steps` (int): 步进数，默认1

#### 4. `get_state`
获取机器人状态

**返回**：
- `positions` (list): 关节位置
- `velocities` (list): 关节速度
- `forces` (list): 关节力/力矩

#### 5. `capture_render`
截图当前画面

**参数**：
- `model_id` (str): 模型标识
- `width` (int): 图像宽度，默认640
- `height` (int): 图像高度，默认480

**返回**：
- `image_data` (str): Base64编码的PNG图像

#### 6. `record_video`
录制视频

**参数**：
- `model_id` (str): 模型标识
- `duration` (float): 录制时长（秒）
- `fps` (int): 帧率，默认30
- `output_path` (str): 输出文件路径

#### 7. `execute_command`
执行自然语言命令

**参数**：
- `command` (str): 自然语言指令

#### 8. `close_viewer`
关闭当前查看器

---

## 🤖 机器人模型库

### 完整列表（57个模型）

#### 机械臂 (17)
| 模型名称 | 自由度 | 特点 | 应用场景 |
|---------|--------|------|----------|
| franka_emika_panda | 7+2 | 协作机器人 | 科研、轻工业 |
| franka_fr3 | 7+2 | 新一代Franka | 高精度操作 |
| universal_robots_ur5e | 6 | 工业标准 | 自动化产线 |
| kuka_iiwa_14 | 7 | 力控机器人 | 精密装配 |
| kinova_gen3 | 7 | 轻量化设计 | 服务机器人 |
| ufactory_xarm7 | 7 | 高性价比 | 教育科研 |

#### 四足机器人 (12)
| 模型名称 | 自由度 | 特点 | 应用场景 |
|---------|--------|------|----------|
| unitree_go2 | 12 | 消费级产品 | 陪伴、巡检 |
| unitree_a1 | 12 | 研究平台 | 算法开发 |
| anymal_c | 12 | 全地形 | 野外作业 |
| anymal_d | 12 | 增强版 | 工业巡检 |
| google_barkour_vb | 12 | 敏捷运动 | 研究平台 |

#### 人形机器人 (8)
| 模型名称 | 自由度 | 特点 | 应用场景 |
|---------|--------|------|----------|
| unitree_h1 | 19 | 全尺寸人形 | 通用服务 |
| unitree_g1 | 23 | 灵巧型 | 复杂操作 |
| robotis_op3 | 20 | 小型人形 | 教育研究 |
| berkeley_humanoid | 12 | 简化设计 | 快速原型 |

#### 灵巧手 (6)
| 模型名称 | 自由度 | 特点 | 应用场景 |
|---------|--------|------|----------|
| shadow_hand | 24 | 仿人设计 | 灵巧操作 |
| leap_hand | 16 | 低成本 | 快速部署 |
| robotiq_2f85 | 1 | 工业夹爪 | 抓取操作 |
| wonik_allegro | 16 | 高速运动 | 动态操作 |

---

## 👨‍💻 开发指南

### 项目结构

```
mujoco-mcp/
├── src/
│   └── mujoco_mcp/
│       ├── __init__.py          # 包初始化
│       ├── remote_server.py     # MCP服务器实现
│       ├── viewer_client.py     # 查看器客户端
│       ├── menagerie_loader.py  # 模型加载器
│       └── video_recorder.py    # 视频录制
├── mcp_server_remote.py         # MCP入口点
├── mujoco_viewer_server.py      # 查看器服务器
├── pyproject.toml               # 项目配置
├── examples/                    # 示例代码
├── tests/                       # 测试用例
└── docs/                        # 文档

```

### 添加新功能

#### 1. 添加新的MCP工具

```python
# 在 remote_server.py 中
@server.list_tools()
async def handle_list_tools() -> List[types.Tool]:
    tools.append(types.Tool(
        name="your_new_tool",
        description="工具描述",
        inputSchema={
            "type": "object",
            "properties": {
                "param1": {"type": "string"}
            }
        }
    ))

@server.call_tool()
async def handle_call_tool(name: str, arguments: Dict):
    if name == "your_new_tool":
        return your_tool_implementation(arguments)
```

#### 2. 添加新的机器人模型

```python
# 在 menagerie_loader.py 中
ROBOT_DESCRIPTIONS["your_robot"] = {
    "name": "Your Robot",
    "type": "arm",  # arm/quadruped/humanoid/hand
    "description": "机器人描述"
}
```

### 测试

```bash
# 运行所有测试
pytest tests/

# 运行特定测试
pytest tests/test_remote_server.py

# 测试覆盖率
pytest --cov=mujoco_mcp tests/
```

### 调试

#### 1. 启用详细日志

```python
# 设置环境变量
export MUJOCO_MCP_DEBUG=1
```

#### 2. 使用MCP Inspector

```bash
npx @modelcontextprotocol/inspector python mcp_server_remote.py
```

#### 3. 查看日志文件

```bash
# macOS
tail -f ~/Library/Logs/Claude/mcp-server-mujoco-mcp-remote.log

# 查看器服务器日志
tail -f viewer_server.log
```

---

## 🔍 故障排除

### 常见问题

#### 1. 连接错误

**问题**：`Failed to connect to MuJoCo Viewer`

**解决方案**：
```bash
# 1. 检查端口占用
lsof -i :8888

# 2. 杀死旧进程
pkill -f mujoco_viewer_server

# 3. 重新启动
python mujoco_viewer_server.py
```

#### 2. 模型加载失败

**问题**：`Model not found`

**解决方案**：
```bash
# 1. 检查环境变量
echo $MENAGERIE_ROOT

# 2. 确认模型存在
ls $MENAGERIE_ROOT/your_robot/

# 3. 更新模型库
cd $MENAGERIE_ROOT && git pull
```

#### 3. 渲染问题

**问题**：GUI窗口不显示

**解决方案**：
- macOS: 确保已授予屏幕录制权限
- Linux: 安装 `libglfw3`
- Windows: 更新显卡驱动

### 性能优化

#### 1. 降低仿真频率

```python
# 减少物理步进频率
server.call_tool("step_simulation", {
    "model_id": model_id,
    "steps": 10  # 批量步进
})
```

#### 2. 优化渲染

```python
# 降低渲染分辨率
server.call_tool("capture_render", {
    "width": 320,
    "height": 240
})
```

#### 3. 使用缓存

```python
# 启用模型缓存
export MUJOCO_MCP_CACHE=1
```

---

## 📈 版本历史

### v0.7.4 (2024-01-13) - 当前版本
- ✅ 完整的Menagerie集成（57个模型）
- ✅ 视频录制功能（MP4/GIF）
- ✅ 场景文件自动检测
- ✅ 改进的错误处理

### v0.7.3 (2024-01-12)
- ✅ 渲染捕获功能
- ✅ Base64图像传输
- ✅ 修复手部模型加载

### v0.7.0 (2024-01-10)
- ✅ 远程模式架构
- ✅ Socket通信层
- ✅ 多模型支持

### v0.6.0 (2024-01-05)
- ✅ MCP协议实现
- ✅ Claude Desktop集成
- ✅ 基础机器人控制

[查看完整更新日志](./CHANGELOG.md)

---

## 🗺️ 路线图

### 短期目标 (v0.8.0)
- [ ] 批量机器人控制
- [ ] 自定义场景编辑器
- [ ] 物理参数调节
- [ ] 碰撞检测可视化

### 中期目标 (v0.9.0)
- [ ] 强化学习集成
- [ ] 轨迹优化工具
- [ ] 多机协作框架
- [ ] Web界面

### 长期目标 (v1.0.0)
- [ ] 云端部署方案
- [ ] 真机控制桥接
- [ ] VR/AR支持
- [ ] 插件系统

---

## 🤝 贡献指南

我们欢迎所有形式的贡献！

### 如何贡献

1. Fork 项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

### 贡献准则

- 遵循 [PEP 8](https://pep8.org/) 代码风格
- 添加单元测试
- 更新相关文档
- 保持提交信息清晰

### 报告问题

请通过 [GitHub Issues](https://github.com/robotlearning123/mujoco-mcp/issues) 报告问题，包含：
- 问题描述
- 复现步骤
- 期望行为
- 系统信息

---

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

---

## 🙏 致谢

- [MuJoCo](https://mujoco.org/) - DeepMind 的物理引擎
- [Model Context Protocol](https://modelcontextprotocol.io/) - Anthropic 的 MCP 标准
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) - 机器人模型库
- 所有贡献者和用户

---

<div align="center">

**用 AI 的力量，让机器人控制变得简单！**

[⭐ Star](https://github.com/robotlearning123/mujoco-mcp) | [🐛 Issues](https://github.com/robotlearning123/mujoco-mcp/issues) | [💬 Discussions](https://github.com/robotlearning123/mujoco-mcp/discussions)

</div>