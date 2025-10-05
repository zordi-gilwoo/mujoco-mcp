# MuJoCo MCP

[![Version](https://img.shields.io/badge/version-0.8.2-blue.svg)](CHANGELOG.md)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3.0%2B-green.svg)](https://github.com/google-deepmind/mujoco)
[![MCP](https://img.shields.io/badge/MCP-2024--11--05-purple.svg)](https://modelcontextprotocol.io/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

**Control robots and physics simulations using plain English.**

Talk to your simulations like you would talk to a colleague: *"Create a cart pole"*, *"Move the robot arm in a circle"*, *"What's the current position?"*

---

## ✨ What is This?

MuJoCo MCP lets you control physics simulations through natural language. Instead of writing complex code, just describe what you want:

- **"Create a Franka Panda robot"** - Instantly loads a realistic robot model
- **"Make it pick up a box"** - Generates the motion automatically  
- **"Show me the simulation in my browser"** - Real-time 3D visualization

Perfect for robotics research, prototyping, teaching, or just exploring physics simulations.

📚 **[Full Documentation](DOCUMENTATION_INDEX.md)** | 🏗️ **[Architecture](ARCHITECTURE.md)** | 🔧 **[API Reference](API_REFERENCE.md)**

---

## 🚀 Quick Start (5 minutes)

### What You'll Need
- **Python 3.10+** - [Download here](https://www.python.org/downloads/)
- **conda** - Makes installation easy ([Get Miniconda](https://docs.conda.io/en/latest/miniconda.html))
- **An LLM API key** - Choose one:
  - [OpenAI](https://platform.openai.com/api-keys) (recommended, GPT-4)
  - [Anthropic](https://console.anthropic.com/) (Claude)
  - [Google AI](https://ai.google.dev/) (Gemini)

### Installation

**1. Get the code:**
```bash
git clone https://github.com/zordi-gilwoo/mujoco-mcp.git
cd mujoco-mcp
```

**2. Get robot models** (56+ robots including Franka Panda, UR5e, Spot):
```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
```

**3. Set up your environment:**
```bash
# Create environment with all dependencies
conda env create -f environment.yml
conda activate mujoco-mcp

# Configure with your LLM API key (choose one)
# For OpenAI:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie \
  OPENAI_API_KEY="sk-your-key-here" \
  LLM_PROVIDER="openai"

# OR for Claude:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie \
  CLAUDE_API_KEY="sk-ant-your-key-here" \
  LLM_PROVIDER="claude"

# OR for Gemini:
conda env config vars set MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie \
  GEMINI_API_KEY="your-key-here" \
  LLM_PROVIDER="gemini"

# Reload environment and install
conda deactivate
conda activate mujoco-mcp
pip install -e .
```

**4. Verify it works:**
```bash
python scripts/quick_internal_test.py
```

✅ **Done!** Now let's create your first simulation.

---

## 🎮 Your First Simulation

### Start the Browser Viewer

This is the easiest way to get started—everything runs in your browser!

```bash
./scripts/run_py_viewer.sh
```

Then open your browser to: **http://localhost:8000**

### Try These Commands

Type these into the browser interface:

1. **"Create a cart pole with a 2m long pole"**  
   → Customize simulations with natural language

2. **"Place three boxes on a table, put a Franka Panda in front of the table."**  
   → Build complex scenes automatically

---

## 🎨 Scene Generation

MuJoCo MCP can generate complete physics scenes from natural language descriptions. Use the browser viewer (above) or run the generator directly:

```bash
# Generate and save scene XML
PYTHONPATH=./src python scripts/text_llm.py "create a cart pole with a 2m long pole"
```

**More Example Prompts:**
- "Create a cluttered workbench with a table and three boxes"
- "Place a table, then line up three cylinders on top"
- "Build a cart-pole rig with a 1.8m pole tilted 15 degrees"
- "Create a double pendulum with two 1.5m cylinders"
- "Stack three boxes and balance a sphere on top"

The generator uses your configured LLM provider (OpenAI, Claude, or Gemini) to understand your description and create physically accurate scenes.

---

## 💬 Alternative: Use with Claude Desktop

Prefer a conversational interface? Use MuJoCo MCP directly in Claude Desktop.

### Setup

**1. Add this to your Claude config** (`~/Library/Application Support/Claude/claude_desktop_config.json` on macOS):
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

**2. Restart Claude Desktop**

**3. Start chatting:**
```
You: "Create a pendulum simulation"
Claude: ✓ Created pendulum with realistic physics

You: "Set the angle to 45 degrees"  
Claude: ✓ Updated pendulum position

You: "Run it for 100 steps"
Claude: ✓ Simulation advanced, current angle: 38.2°
```

**Learn more:** [Claude Desktop Guide](docs/guides/CLAUDE_GUIDE.md)

---

## ✨ Features

- 🗣️ **Natural Language Control** - Control robots with plain English commands
- 🎮 **Real-time Visualization** - Browser-based viewer with 60 FPS WebRTC streaming
- 🤖 **56+ Robot Models** - Full MuJoCo Menagerie integration (Franka Panda, UR5e, Spot, and more)
- 🎯 **LLM Scene Generation** - Create simulations from text descriptions
- 🔌 **MCP Integration** - Works with Claude Desktop and other MCP clients
- 🧠 **RL Ready** - Gymnasium-compatible environments for training
- 🚀 **Production Ready** - Multi-client support, GPU acceleration, process isolation

---

## 🔧 Available MCP Tools

| Tool | Description |
|------|-------------|
| `get_server_info` | Get server status and capabilities |
| `create_scene` | Create physics simulation from XML or description |
| `step_simulation` | Advance simulation time |
| `get_state` | Get current simulation state (positions, velocities) |
| `set_joint_positions` | Control robot joint angles |
| `reset_simulation` | Reset to initial state |
| `execute_command` | Execute natural language commands |
| `close_viewer` | Close visualization window |

**Full API documentation:** [API Reference](API_REFERENCE.md)

---

## 💡 Usage Examples

### Natural Language Commands

**Basic Physics:**
```
"Create a pendulum simulation"
"Create a double pendulum"
"Create a cart pole simulation"
```

**Robot Control:**
```
"Load a Franka Panda robot"
"Move the robot arm in a circle"
"Set all joints to home position"
```

**Multi-Robot Coordination:**
```
"Create two robot arms side by side"
"Make them work together to lift a box"
```

### Python API

**Reinforcement Learning:**
```python
from mujoco_mcp.rl_integration import create_reaching_env

env = create_reaching_env("franka_panda")
obs, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, done, truncated, info = env.step(action)
```

---

## 🚀 Advanced Topics

Want to go deeper? MuJoCo MCP includes powerful features for research and production:

- **RL Training** - Gymnasium-compatible environments for reinforcement learning
- **Process Pool Architecture** - Isolated viewer processes for true multi-user collaboration
- **GPU Acceleration** - Hardware-accelerated rendering and H.264 encoding
- **Advanced Controllers** - PID, trajectory planning, and MPC implementations
- **Multi-Robot Coordination** - Formation control and cooperative task execution

See the [Advanced Features Guide](docs/features/ADVANCED_FEATURES_GUIDE.md) for detailed examples and configuration.

---

## 📚 Documentation

- **[Documentation Index](DOCUMENTATION_INDEX.md)** - Complete documentation hub
- **[Claude Desktop Guide](docs/guides/CLAUDE_GUIDE.md)** - Claude Desktop integration
- **[API Reference](API_REFERENCE.md)** - Complete MCP tools documentation
- **[Advanced Features](docs/features/ADVANCED_FEATURES_GUIDE.md)** - RL, GPU acceleration, multi-client
- **[Architecture](ARCHITECTURE.md)** - System design and technical details
- **[Testing Guide](docs/testing/TESTING.md)** - How to run tests
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