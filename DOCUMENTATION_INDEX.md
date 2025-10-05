# MuJoCo MCP Documentation Index

## 📚 Complete Documentation Guide

Welcome to the MuJoCo MCP documentation. This index provides a comprehensive overview of all available documentation and guides you to the right resources.

### Current Version: **v0.8.2**
### MCP Protocol: **2024-11-05 Standard**
### Last Updated: October 2025

---

## 🚀 Getting Started

### For New Users
1. **[README.md](README.md)** - Project overview and quick start
2. **[Installation Guide](#installation)** - Detailed setup instructions
3. **[Examples](examples/)** - Demo applications and tutorials
4. **[WebRTC Viewer](py_remote_viewer/)** - Browser-based remote viewer

### For Developers
1. **[Architecture Overview](#architecture)** - System design and components
2. **[API Reference](#api-reference)** - Complete API documentation
3. **[Scene Generation](src/mujoco_mcp/scene_gen/)** - Structured scene creation
4. **[CONTRIBUTING.md](CONTRIBUTING.md)** - How to contribute

---

## ✨ Key Features

### 🌐 WebRTC Remote Viewer
A fully Python-based, headless, browser-accessible MuJoCo viewer:
- **Real-time Streaming**: WebRTC-based video streaming to browser
- **Interactive Controls**: Mouse, keyboard, and touch input
- **Multi-Client Support**: Multiple users can view simultaneously
- **Scene Creation**: Natural language scene generation with LLM
- **Built-in Scenes**: Pendulum, cart-pole, Franka Panda demos
- **Production Ready**: FastAPI server with async architecture

**Quick Start:**
```bash
./scripts/run_py_viewer.sh
# Open browser: http://localhost:8000
```

### 🎨 Parametric Scene Generation
Create physics scenes from natural language with custom dimensions:
- **Parametric Primitives**: Box, sphere, cylinder, capsule, ellipsoid
- **Custom Dimensions**: Specify exact sizes in meters
- **Color Control**: Optional RGBA values for visual customization
- **Smart Constraints**: Axis-aware constraint composition
- **Multi-Provider LLM**: OpenAI, Claude (Anthropic), Gemini

**Example:**
```python
"Create a cart pole with a 2m long pole"
# Generates cart + 2-meter cylinder automatically
```

### 🧩 Enhanced Constraint System
Sophisticated spatial constraint solving:
- **8 Constraint Types**: on_top_of, beside, in_front_of, inside, aligned_with_axis, within_reach, no_collision, oriented_towards
- **Multi-Constraint Composition**: Combine horizontal + vertical constraints
- **Collision Detection**: Rotation-aware AABB with global optimization
- **Stable Poses**: Automatic orientation for physical stability
- **Robot Reachability**: Validate and optimize robot-object layouts

### 🤖 Advanced Reasoning
Comprehensive enhancement system:
- **Enhanced Collision Detection**: Rotation-aware AABB with global optimization
- **Advanced Spatial Reasoning**: Stable poses and robot reachability
- **Symbolic Plan Interface**: Explicit NL→Plan→Scene separation for auditability
- **Enhanced Asset Semantics**: Grasp affordances and support surfaces
- **Robust Constraint Solver**: Backtracking and global optimization

---

## 📖 Core Documentation

### System Overview
- **[README.md](README.md)**
  - Project introduction
  - Quick start guide
  - Basic features
  - Example usage

### Architecture & Design
- **[ARCHITECTURE.md](ARCHITECTURE.md)**
  - System architecture
  - Component interactions
  - Design patterns
  - Technical decisions

### User Guides
- **[WebRTC Remote Viewer](py_remote_viewer/README.md)**
  - Browser-based real-time simulation viewer
  - LLM-powered scene creation with multi-provider support
  - Multi-client collaboration
  - Complete setup and deployment
  - Scene creation interface
  - API reference

- **[Claude Desktop Guide](docs/guides/CLAUDE_GUIDE.md)**
  - Claude Desktop MCP setup
  - Robot control integration
  - Step-by-step tutorials

### Feature Documentation
- **[Advanced Features](docs/features/ADVANCED_FEATURES_GUIDE.md)**
  - Advanced control algorithms
  - Multi-robot coordination
  - Sensor feedback and RL integration

- **[EGL H264 Features](docs/features/EGL_H264_FEATURES.md)** 
  - GPU-accelerated headless rendering with EGL
  - Hardware video encoding (H.264)
  - Multiple hardware encoder support (NVENC, QuickSync, VideoToolbox)
  - Software fallback for compatibility

- **[LLM Integration](docs/features/LLM_INTEGRATION.md)**
  - Natural language scene generation
  - Multiple LLM providers (OpenAI, Claude, Gemini)
  - Structured scene descriptions
  - Constraint-based placement

- **[Menagerie Integration](docs/features/MENAGERIE_INTEGRATION.md)**
  - Robot model library
  - 56+ robot models
  - Dynamic model loading

- **[Physics Scenes](docs/features/SCENES.md)**
  - Built-in physics scenes
  - Scene parameters
  - Custom scene creation

### Scene Generation System
- **[Scene Schema](src/mujoco_mcp/scene_gen/scene_schema.py)**
  - Structured scene descriptions
  - Spatial constraints
  - Object and robot placement
  - Pydantic validation

- **[Constraint Solver](src/mujoco_mcp/scene_gen/constraint_solver.py)**
  - Enhanced collision detection
  - Advanced spatial reasoning
  - Multi-constraint composition
  - Axis-aware constraint application

- **[LLM Scene Generator](src/mujoco_mcp/scene_gen/llm_scene_generator.py)**
  - Multi-provider LLM support
  - Natural language to scene conversion
  - Symbolic plan interface
  - Constraint reference validation

- **[Metadata Extractor](src/mujoco_mcp/scene_gen/metadata_extractor.py)**
  - Asset metadata management
  - Dimension computation
  - Bounding box calculation
  - Dynamic XML parsing

- **[XML Builder](src/mujoco_mcp/scene_gen/scene_xml_builder.py)**
  - MuJoCo XML generation
  - Parametric primitives
  - Robot integration
  - Scene composition

- **[Enhanced Features](src/mujoco_mcp/scene_gen/)**
  - Enhanced collision detection with rotation-aware AABB
  - Advanced spatial reasoning with stable poses
  - Symbolic plan interface for NL→Plan→Scene separation
  - Enhanced asset semantics with grasp affordances
  - Robust constraint solver with backtracking

### WebRTC Remote Viewer
- **[Remote Viewer Overview](py_remote_viewer/README.md)**
  - Python-based headless viewer
  - WebRTC video streaming
  - Real-time browser interaction
  - Multi-client support

- **[Scene Creation Interface](py_remote_viewer/README.md#scene-creation-interface)**
  - Text prompt-based scene generation
  - Preset scenes (pendulum, cart-pole, franka_panda)
  - XML editor with syntax highlighting
  - Live scene loading

- **[MuJoCo Simulation](py_remote_viewer/mujoco_simulation.py)**
  - Real physics simulation
  - Default and custom models
  - State management
  - Camera integration

- **[WebRTC Architecture](py_remote_viewer/README.md#architecture)**
  - FastAPI server
  - aiortc integration
  - Event protocol
  - Multi-client session management

### Architecture Documentation
- **[Multi-Client Architecture](docs/architecture/MULTI_CLIENT_ARCHITECTURE.md)**
  - Multi-client design with session management
  - Process pool architecture for isolation
  - Automatic port allocation
  - Resource management and cleanup
  - Scalability and monitoring

- **[WebRTC Integration Guide](docs/architecture/WEBRTC_INTEGRATION_GUIDE.md)**
  - WebRTC architecture
  - Integration patterns
  - Future roadmap

### Motion Control
- **[examples/README_MOTION_CONTROL.md](examples/README_MOTION_CONTROL.md)**
  - Motion control demos
  - MuJoCo Menagerie integration
  - Robot configurations
  - Control strategies

---

## 🔧 Technical Reference

### MCP Tools API
- **Core Simulation Tools** (9 total)
  - `get_server_info` - Server status and capabilities
  - `create_scene` - Create physics simulation from XML or natural language
  - `step_simulation` - Advance simulation time
  - `get_state` - Current simulation data
  - `set_joint_positions` - Control joint angles
  - `reset_simulation` - Reset to initial state
  - `execute_command` - Natural language control
  - `get_loaded_models` - List active simulations
  - `close_viewer` - Close MuJoCo GUI window

### WebRTC Viewer API
- **REST Endpoints**
  - `GET /api/config` - Server configuration
  - `GET /api/stats` - Server statistics and simulation state
  - `GET /api/health` - Health check
  - `POST /api/execute-command` - Execute text commands
  - `GET /api/scene/current` - Get current scene XML
  - `POST /api/scene/load` - Load scene from XML
  - `POST /api/config/api-key` - Configure LLM provider
  - `POST /api/scene/generate` - Generate scene from natural language
  
- **WebSocket Endpoints**
  - `WS /ws/signaling` - WebRTC signaling and event handling

### Scene Generation API
- **Available Primitive Types**
  - `primitive:box` - Parametric box (width, depth, height)
  - `primitive:sphere` - Parametric sphere (radius)
  - `primitive:cylinder` - Parametric cylinder (radius, height)
  - `primitive:capsule` - Parametric capsule (radius, length)
  - `primitive:ellipsoid` - Parametric ellipsoid (radius_x, radius_y, radius_z)

- **Available Constraint Types**
  - `on_top_of` - Place subject on top of reference
  - `in_front_of` - Place subject in front of reference
  - `beside` - Place subject beside reference
  - `no_collision` - Ensure minimum clearance
  - `within_reach` - Place within robot's workspace
  - `inside` - Place subject inside container
  - `aligned_with_axis` - Align subject along axis
  - `oriented_towards` - Orient subject towards reference

- **LLM Providers**
  - OpenAI (GPT-4, GPT-3.5)
  - Claude (Anthropic)
  - Gemini (Google)

### Change History
- **[CHANGELOG.md](CHANGELOG.md)**
  - Version history
  - Feature additions
  - Bug fixes
  - Breaking changes

### Testing & Quality
- **[Testing Guide](docs/testing/TESTING.md)**
  - How to run tests
  - Test categories
  - Quick commands
  - CI/CD integration

---

## 🛠️ Development Resources

### Installation & Setup

#### Basic Setup
```bash
# Clone repository
git clone https://github.com/zordi-gilwoo/mujoco-mcp.git
cd mujoco-mcp

# Install in development mode
pip install -e .

# Configure Claude Desktop
cp cursor_mcp_config.json ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

#### WebRTC Viewer Setup
```bash
# Install WebRTC viewer dependencies
pip install fastapi uvicorn aiortc av numpy pydantic websockets

# Start the WebRTC viewer
./scripts/run_py_viewer.sh

# Open browser
open http://localhost:8000

# Optional: Configure LLM integration
export OPENAI_API_KEY="sk-..."
export STRUCTURED_SCENE_LLM="enabled"
```

#### Scene Generation Setup
```bash
# Install LLM provider packages (choose one or more)
pip install openai              # For OpenAI/GPT-4
pip install anthropic           # For Claude
pip install google-generativeai # For Gemini

# Set API key
export OPENAI_API_KEY="sk-..."
# or
export CLAUDE_API_KEY="sk-ant-..."
# or
export GEMINI_API_KEY="..."

# Enable LLM integration
export STRUCTURED_SCENE_LLM="enabled"
export LLM_PROVIDER="openai"  # or "claude" or "gemini"

# Test scene generation
python -c "
from mujoco_mcp.scene_gen import LLMSceneGenerator, MetadataExtractor
generator = LLMSceneGenerator(MetadataExtractor())
scene = generator.generate_scene_description('Create a simple pendulum')
print(scene.to_xml())
"
```

#### Advanced Setup
```bash
# Install MuJoCo Menagerie (for robot models)
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/mujoco_menagerie
export MUJOCO_MENAGERIE_PATH=~/mujoco_menagerie

# Start enhanced viewer server (legacy)
/opt/miniconda3/bin/mjpython mujoco_viewer_server_enhanced.py

# Run benchmarks
python benchmarks/physics_benchmarks.py

# Test advanced features
python test_advanced_features.py

# Run scene generation tests
python -m pytest tests/test_scene_gen*.py -v
```

### Architecture Overview

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ Claude Desktop  │────▶│   MCP Server    │────▶│ Viewer Server   │
│  (MCP Client)   │◀────│  (Python SDK)   │◀────│   (MuJoCo)      │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        JSON-RPC          Socket (8888)           launch_passive()
             │                    │
             │                    ▼
             │            ┌─────────────────┐
             │            │ Scene Generator │
             │            │  • LLM Integration
             │            │  • Constraint Solver
             │            │  • XML Builder
             │            └─────────────────┘
             │
             ▼
    ┌─────────────────┐
    │ WebRTC Viewer   │
    │  • Browser UI    │
    │  • Multi-Client  │
    │  • Real-time     │
    └─────────────────┘
```

### Module Structure
```
mujoco-mcp/
├── src/mujoco_mcp/
│   ├── __init__.py                    # Package initialization
│   ├── __main__.py                    # Entry point
│   ├── server.py                      # Main MCP server
│   ├── viewer_client.py               # Socket client
│   ├── mcp_server.py                  # MCP protocol handler
│   ├── advanced_controllers.py        # Control algorithms
│   ├── multi_robot_coordinator.py     # Multi-robot systems
│   ├── sensor_feedback.py             # Sensor processing
│   ├── rl_integration.py              # RL environments
│   ├── visualization_tools.py         # Real-time plotting
│   └── scene_gen/                     # Scene generation system
│       ├── scene_schema.py            # Pydantic models
│       ├── llm_scene_generator.py     # LLM integration
│       ├── constraint_solver.py       # Spatial constraint solving
│       ├── scene_xml_builder.py       # MuJoCo XML generation
│       ├── metadata_extractor.py      # Asset metadata
│       ├── enhanced_collision.py      # Rotation-aware collision detection
│       ├── spatial_reasoning.py       # Stable poses & reachability
│       ├── symbolic_plan.py           # NL→Plan→Scene interface
│       ├── enhanced_semantics.py      # Grasp affordances & semantics
│       ├── robust_solver.py           # Backtracking constraint solver
│       ├── shared_types.py            # Common data structures
│       └── assets_db.json             # Asset database
├── py_remote_viewer/                  # WebRTC remote viewer
│   ├── __init__.py                    # Package initialization
│   ├── __main__.py                    # CLI entry point
│   ├── server.py                      # FastAPI server
│   ├── signaling.py                   # WebRTC signaling
│   ├── webrtc_track.py                # Video track
│   ├── mujoco_simulation.py           # MuJoCo integration
│   ├── builtin_scenes.py              # Built-in scenes
│   ├── camera_state.py                # Camera management
│   ├── events.py                      # Event protocol
│   ├── config.py                      # Configuration
│   └── logging_utils.py               # Logging utilities
├── client/                            # Web client
│   ├── index.html                     # Main UI
│   ├── app.js                         # Client JavaScript
│   └── styles.css                     # Styling
├── examples/
│   ├── motion_control_demo.py         # Direct control demo
│   └── mcp_motion_control.py          # MCP interface demo
├── benchmarks/
│   └── physics_benchmarks.py          # Performance tests
└── mujoco_viewer_server.py            # Legacy viewer server
```

---

## 📋 Common Tasks

### Creating a Simple Simulation
```python
# In Claude Desktop (MCP)
"Create a pendulum simulation"
"Set the pendulum angle to 45 degrees"
"Step the simulation 100 times"
```

### Using the WebRTC Remote Viewer
```bash
# Start the remote viewer server
./scripts/run_py_viewer.sh

# Open browser and navigate to http://localhost:8000
# Click "Connect" to start WebRTC streaming

# Use text commands:
# - "play" or "start" - Begin simulation
# - "pause" - Pause simulation  
# - "reset" - Reset to initial state
# - "create cart pole" - Load cart-pole scene
# - "create pendulum" - Load pendulum scene
```

### Generating Scenes with Natural Language
```bash
# Via WebRTC viewer API
curl -X POST "http://localhost:8000/api/scene/generate" \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Create a cart pole with a 2m long pole"}'

# Configure LLM provider first
curl -X POST "http://localhost:8000/api/config/api-key" \
  -H "Content-Type: application/json" \
  -d '{"provider": "openai", "api_key": "sk-..."}'
```

### Scene Generation with Parametric Primitives
```python
# Using Python API
from mujoco_mcp.scene_gen import LLMSceneGenerator, MetadataExtractor

metadata_extractor = MetadataExtractor()
generator = LLMSceneGenerator(metadata_extractor)

# Generate from natural language
scene = generator.generate_scene_description(
    "Place a table, then line up three cylinders on top: "
    "a 0.1m green post, a 0.2m orange post, and a 0.3m blue post"
    "and put a Franka Panda in front of the table"
)

# Convert to MuJoCo XML
xml = scene.to_xml()
```

### Creating Custom Scenes with Primitives
```python
# Using scene schema directly
from mujoco_mcp.scene_gen import SceneDescription, ObjectPlacement, SpatialConstraint

scene = SceneDescription(
    objects=[
        ObjectPlacement(
            object_id="table",
            object_type="table_standard",
            constraints=[]
        ),
        ObjectPlacement(
            object_id="red_ball",
            object_type="primitive:sphere",
            dimensions={"radius": 0.1},
            color=(1.0, 0.0, 0.0, 1.0),
            constraints=[
                SpatialConstraint(
                    type="on_top_of",
                    subject="red_ball",
                    reference="table",
                    clearance=0.001
                )
            ]
        )
    ]
)

# Convert to XML
xml = scene.to_xml()
```

### Loading Robot Models
```python
# Load from MuJoCo Menagerie
"Load Franka Panda robot"
"Create a Unitree Go2 scene"
"Show me the Shadow Hand model"
```

### Advanced Control
```python
# Using Python API
from mujoco_mcp.advanced_controllers import create_arm_controller

controller = create_arm_controller("franka_panda")
controller.set_trajectory(waypoints, times)
```

### Multi-Robot Coordination
```python
from mujoco_mcp.multi_robot_coordinator import MultiRobotCoordinator

coordinator = MultiRobotCoordinator()
coordinator.add_robot("arm1", "franka_panda", {"manipulation": True})
coordinator.add_robot("arm2", "ur5e", {"manipulation": True})
coordinator.start_coordination()
```

---

## 🐛 Troubleshooting


### Getting Help
- Check [TESTING_SUMMARY.md](TESTING_SUMMARY.md) for known issues
- Review examples in [examples/](examples/) directory
- Consult [ADVANCED_FEATURES_GUIDE.md](ADVANCED_FEATURES_GUIDE.md) for complex use cases
- Check [py_remote_viewer/README.md](py_remote_viewer/README.md) for viewer troubleshooting
- Review [AGENTS.md](AGENTS.md) for AI agent integration

---

## 🚀 Quick Reference

### WebRTC Viewer Commands
```bash
# Start viewer
./scripts/run_py_viewer.sh

# With custom configuration
VIEWER_PORT=8080 DEBUG_MODE=1 ./scripts/run_py_viewer.sh

# Health check
curl http://localhost:8000/api/health

# Load scene
curl -X POST "http://localhost:8000/api/scene/load" \
  -H "Content-Type: application/json" \
  -d '{"xml": "<mujoco>...</mujoco>"}'
```

### Scene Generation Examples
```python
# Natural language prompts
"Create a cart pole with a 2m long pole"
"Place a table with three red balls on top"
"Put a Franka Panda in front of a table with a cup"

# Direct Python API
from mujoco_mcp.scene_gen import SceneDescription, ObjectPlacement

scene = SceneDescription(
    objects=[
        ObjectPlacement(
            object_id="pole",
            object_type="primitive:cylinder",
            dimensions={"radius": 0.02, "height": 2.0},
            color=(0.8, 0.2, 0.2, 1.0),
            constraints=[]
        )
    ]
)
```

### LLM Provider Configuration
```bash
# OpenAI
export OPENAI_API_KEY="sk-..."
export OPENAI_MODEL="gpt-4"

# Claude (Anthropic)
export CLAUDE_API_KEY="sk-ant-..."
export CLAUDE_MODEL="claude-3-sonnet-20241022"

# Gemini (Google)
export GEMINI_API_KEY="..."
export GEMINI_MODEL="gemini-1.5-pro"

# Enable LLM integration
export STRUCTURED_SCENE_LLM="enabled"
```

### Primitive Types Quick Reference
| Primitive | Required Dimensions | Example |
|-----------|-------------------|---------|
| `primitive:box` | width, depth, height | `{"width": 0.3, "depth": 0.3, "height": 0.2}` |
| `primitive:sphere` | radius | `{"radius": 0.1}` |
| `primitive:cylinder` | radius, height | `{"radius": 0.02, "height": 2.0}` |
| `primitive:capsule` | radius, length | `{"radius": 0.03, "length": 0.5}` |
| `primitive:ellipsoid` | radius_x, radius_y, radius_z | `{"radius_x": 0.1, "radius_y": 0.05, "radius_z": 0.05}` |

### Constraint Types Quick Reference
| Constraint | Controls | Example Use Case |
|-----------|----------|-----------------|
| `on_top_of` | Z position | "Place cup on table" |
| `beside` | XY position (Y direction) | "Line up objects in a row" |
| `in_front_of` | XY position (X direction) | "Robot in front of table" |
| `inside` | Full position | "Object inside container" |
| `no_collision` | Avoidance | "Ensure objects don't overlap" |
| `within_reach` | Robot workspace | "Object reachable by robot" |
| `aligned_with_axis` | Alignment | "Align along reference axis" |
| `oriented_towards` | Orientation | "Face object towards target" |

---

## 📚 Additional Resources

### Project Documentation
- [API Reference](API_REFERENCE.md) - Complete API documentation
- [Architecture](ARCHITECTURE.md) - System architecture details
- [Scene Generation Status](AGENTS.md) - Implementation status
- [Code Quality Guide](CODE_QUALITY.md) - Development standards

### External Documentation
- [MCP Specification](https://modelcontextprotocol.io/specification/2024-11-05)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- [Gymnasium Documentation](https://gymnasium.farama.org/)
- [WebRTC Standards](https://webrtc.org/)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [aiortc Documentation](https://aiortc.readthedocs.io/)

### Related Projects
- [Blender MCP](https://github.com/ahujasid/blender-mcp) - Similar architecture
- [MuJoCo MPC](https://github.com/google-deepmind/mujoco_mpc) - Model predictive control
- [dm_control](https://github.com/google-deepmind/dm_control) - DeepMind control suite

### Community
- Report issues: [GitHub Issues](https://github.com/robotlearning123/mujoco-mcp/issues)
- Discussions: Use GitHub Discussions for questions
- Contributing: See [CONTRIBUTING.md](CONTRIBUTING.md)

---
