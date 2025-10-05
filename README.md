# MuJoCo MCP

[![Version](https://img.shields.io/badge/version-0.8.2-blue.svg)](CHANGELOG.md)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3.0%2B-green.svg)](https://github.com/google-deepmind/mujoco)
[![MCP](https://img.shields.io/badge/MCP-2024--11--05-purple.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

Control MuJoCo physics simulations through natural language using the Model Context Protocol. Integrates with Claude Desktop and other MCP clients for seamless AI-powered robotics control.

📚 **[Documentation](DOCUMENTATION_INDEX.md)** | 🏗️ **[Architecture](ARCHITECTURE.md)** | 🔧 **[API Reference](API_REFERENCE.md)**

---

## Installation

### Prerequisites
- Python 3.10 or higher
- conda package manager ([Install Miniconda](https://docs.conda.io/en/latest/miniconda.html))
- Git
- (Optional) GPU with CUDA for hardware acceleration

### Step 1: Clone the Repository
```bash
git clone https://github.com/zordi-gilwoo/mujoco-mcp.git
cd mujoco-mcp
```

### Step 2: Set Up MuJoCo Menagerie
Clone the robot models library (required for 56+ robot models including Franka Panda, UR5e, Spot, etc.):
```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

### Step 3: Configure LLM API Key
Choose one LLM provider for AI-powered scene generation (required). Get your API key from:
- **OpenAI**: https://platform.openai.com/api-keys
- **Anthropic (Claude)**: https://console.anthropic.com/
- **Google (Gemini)**: https://ai.google.dev/

### Step 4: Create Conda Environment
```bash
# Create conda environment with dependencies
conda env create -f environment.yml
conda activate mujoco-mcp

# Set environment variables in conda environment
# For OpenAI:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie
conda env config vars set OPENAI_API_KEY="sk-your-key-here"
conda env config vars set LLM_PROVIDER="openai"

# OR for Claude:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie
conda env config vars set CLAUDE_API_KEY="sk-ant-your-key-here"
conda env config vars set LLM_PROVIDER="claude"

# OR for Gemini:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie
conda env config vars set GEMINI_API_KEY="your-key-here"
conda env config vars set LLM_PROVIDER="gemini"

# Reactivate environment to load variables
conda deactivate
conda activate mujoco-mcp

# Install the package in development mode
pip install -e .
```

### Step 5: Verify Installation
```bash
# Test core functionality
python scripts/quick_internal_test.py

# Verify environment variables
python -c "import os; print('Menagerie:', os.environ.get('MUJOCO_MENAGERIE_PATH')); print('LLM Provider:', os.environ.get('LLM_PROVIDER'))"
```

---

## Quick Start: Browser Viewer

The easiest way to get started is with the WebRTC browser viewer. It provides real-time visualization, interactive controls, and LLM-powered scene generation—all in your browser.

### 1. Start the Viewer
```bash
./scripts/run_py_viewer.sh
```

### 2. Open Your Browser
Navigate to:
```
http://localhost:8000
```

### 3. Create Scenes with Natural Language
In the browser interface:
- **"Create a cart pole with a 2m long pole"**
- **"Place a table with three cylinders lined up on top"**
- **"Load a Franka Panda robot"**
- **"Create a double pendulum"**

### 4. Control the Simulation
Use the browser controls to:
- ▶️ Play/pause simulation
- 🎥 Rotate camera with mouse drag
- 🔍 Zoom with scroll wheel
- ⚙️ Adjust simulation speed
- 📊 View real-time state data


See [WebRTC Viewer Guide](docs/guides/WEBRTC_VIEWER_GUIDE.md) for advanced usage.

---

## Alternative: Claude Desktop Integration

You can also use MuJoCo MCP directly in Claude Desktop for conversational control.

### 1. Configure Claude Desktop
Add to your Claude Desktop config (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS):
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

### 2. Restart Claude Desktop

### 3. Use Natural Language
In Claude Desktop:
```
"Create a pendulum simulation"
"Set the pendulum angle to 45 degrees"
"Step the simulation 100 times"
```

See [Claude Desktop Guide](docs/guides/CLAUDE_GUIDE.md) for detailed setup.

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

## Scene Generation

Create MuJoCo scenes from natural language:

```bash
# Generate scene from text
PYTHONPATH=./src python scripts/text_llm.py "create a cart pole with a 2m long pole"
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

## Advanced Topics

### Process Pool Architecture
Each WebRTC client gets an isolated viewer process with automatic port allocation and cleanup. This enables true multi-user collaboration without conflicts. See [Process Pool Architecture](docs/architecture/PROCESS_POOL_ARCHITECTURE.md) for details.

### GPU Acceleration
Enable hardware-accelerated rendering and H.264 encoding for improved performance. See [EGL & H.264 Features](docs/features/EGL_H264_FEATURES.md) for configuration.

### RL Integration
Use MuJoCo MCP as a Gymnasium-compatible environment for reinforcement learning. See [Advanced Features Guide](docs/features/ADVANCED_FEATURES_GUIDE.md) for examples.

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