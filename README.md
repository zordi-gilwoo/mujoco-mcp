# MuJoCo MCP

[![Version](https://img.shields.io/badge/version-0.8.2-blue.svg)](CHANGELOG.md)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3.0%2B-green.svg)](https://github.com/google-deepmind/mujoco)
[![MCP](https://img.shields.io/badge/MCP-2024--11--05-purple.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Control MuJoCo physics simulations through natural language using the Model Context Protocol. Integrates with Claude Desktop and other MCP clients for seamless AI-powered robotics control.

📚 **[Documentation](DOCUMENTATION_INDEX.md)** | 🏗️ **[Architecture](ARCHITECTURE.md)** | 🔧 **[API Reference](API_REFERENCE.md)**

---

## Quick Start

### 1. Install
```bash
pip install mujoco mcp numpy
pip install -e .
```

### 2. Configure Claude Desktop
Add to your Claude Desktop config:
```json
{
  "mcpServers": {
    "mujoco-mcp": {
      "command": "python",
      "args": ["-m", "mujoco_mcp"],
      "env": {"PYTHONPATH": "./src"}
    }
  }
}
```

### 3. Use Natural Language
In Claude Desktop:
```
"Create a pendulum simulation"
"Set the pendulum angle to 45 degrees"
"Step the simulation 100 times"
```

---

## Features

### Core Capabilities
- 🗣️ **Natural Language Control** - Control robots with plain English
- 🎮 **Real-time Visualization** - Native MuJoCo viewer GUI
- 🌐 **Browser-Based Viewer** - WebRTC streaming with 60 FPS
- 🤖 **56+ Robot Models** - Full MuJoCo Menagerie integration
- 🎯 **Scene Generation** - LLM-powered scene creation from text

### Advanced Features
- **Advanced Controllers** - PID, trajectory planning, MPC
- **Multi-Robot Coordination** - Formation control, cooperative tasks
- **RL Integration** - Gymnasium-compatible environments
- **GPU Acceleration** - EGL headless rendering, H.264 encoding
- **Multi-Client Support** - Process-based isolation, auto port allocation

---

## MCP Tools

| Tool | Description |
|------|-------------|
| `get_server_info` | Get server status and capabilities |
| `create_scene` | Create physics simulation |
| `step_simulation` | Advance simulation time |
| `get_state` | Get current simulation state |
| `set_joint_positions` | Control joint angles |
| `reset_simulation` | Reset to initial state |
| `execute_command` | Natural language commands |
| `close_viewer` | Close visualization window |

See [API Reference](API_REFERENCE.md) for complete documentation.

---

## Browser Visualization

### WebRTC Viewer (Recommended)
Real-time browser-based visualization with multi-client support.

```bash
# Start viewer
./scripts/run_py_viewer.sh

# Or using Python module
python -m py_remote_viewer

# Open browser
open http://localhost:8000
```

**Features:**
- 60 FPS video streaming
- Multi-client collaboration
- Interactive camera controls
- LLM-powered scene creation
- GPU acceleration

See [WebRTC Viewer Guide](docs/guides/WEBRTC_VIEWER_GUIDE.md) for details.

---

## Scene Generation

Create MuJoCo scenes from natural language:

```bash
# Generate scene from text
PYTHONPATH=./src python text_llm.py "create a cart pole with a 2m long pole"
```

**Example Prompts:**
- "Create a cluttered workbench with a table and three boxes"
- "Place a table, then line up three cylinders on top"
- "Build a cart-pole rig with a 1.8m pole tilted 15 degrees"
- "Create a double pendulum with two 1.5m cylinders"
- "Stack three boxes and balance a sphere on top"

Requires `OPENAI_API_KEY` environment variable.

---

## Robot Control Examples

### Basic Physics
```
"Create a pendulum simulation"
"Create a double pendulum"
"Create a cart pole simulation"
```

### Robot Control
```
"Load a Franka Panda robot"
"Move the robot arm in a circle"
"Set all joints to home position"
```

### Multi-Robot
```
"Create two robot arms side by side"
"Make them work together to lift a box"
```

### RL Training
```python
from mujoco_mcp.rl_integration import create_reaching_env

env = create_reaching_env("franka_panda")
obs, info = env.reset()
for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, done, truncated, info = env.step(action)
```

---

## Advanced Setup

### MuJoCo Menagerie (Robot Models)
```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
export MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie
```

### Process Pool Architecture
Each client gets an isolated viewer process with automatic port allocation and cleanup. See [Process Pool Architecture](docs/architecture/PROCESS_POOL_ARCHITECTURE.md).

---

## Documentation

- **[Documentation Index](DOCUMENTATION_INDEX.md)** - Complete documentation hub
- **[Architecture](ARCHITECTURE.md)** - System design
- **[API Reference](API_REFERENCE.md)** - Complete API
- **[WebRTC Guide](docs/guides/WEBRTC_VIEWER_GUIDE.md)** - Browser viewer
- **[Claude Guide](docs/guides/CLAUDE_GUIDE.md)** - Claude Desktop setup
- **[Advanced Features](docs/features/ADVANCED_FEATURES_GUIDE.md)** - Advanced capabilities
- **[Testing](docs/testing/TESTING.md)** - How to run tests
- **[Changelog](CHANGELOG.md)** - Version history


---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## License

MIT License - see [LICENSE](LICENSE) for details.

---

## Acknowledgments

- [MuJoCo](https://mujoco.org/) by Google DeepMind
- [Model Context Protocol](https://modelcontextprotocol.io/) by Anthropic
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) for robot models

---

Built with ❤️ for the robotics and AI community