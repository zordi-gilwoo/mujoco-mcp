# WebRTC Viewer Integration Guide

## Overview

The Python WebRTC viewer provides browser-based access to MuJoCo simulations through real-time video streaming. It complements the main MCP server by providing visualization capabilities.

---

## Architecture

### System Integration

```
┌─────────────────┐     MCP      ┌──────────────────┐
│   Claude AI     │◄────────────►│  MCP Server      │
│   Assistant     │   Protocol   │  (Session Mgmt)  │
└─────────────────┘              └─────────┬────────┘
                                           │ Socket IPC
┌─────────────────┐    WebRTC     ┌───────▼────────┐
│   Web Browser   │◄─────────────►│ WebRTC Viewer  │
│   Multi-Client  │   Streaming   │ (Physics Viz)  │
└─────────────────┘              └─────────────────┘
```

### Components

**MCP Server**
- AI-driven simulation control
- Scene creation and model management
- Physics simulation orchestration

**WebRTC Viewer**
- Real-time visualization in browser
- Multi-user collaboration
- Interactive camera controls
- LLM-powered scene creation

**Integration Benefits**
- AI scene generation + real-time visualization
- Multi-user collaboration on AI-created scenes
- Remote simulation access from any browser
- No local MuJoCo installation required for viewers

---

## Usage

### Basic Workflow

```bash
# 1. Start WebRTC viewer
./scripts/run_py_viewer.sh

# 2. Open browser
open http://localhost:8000

# 3. Use Claude Desktop with MCP
# Claude creates scenes → WebRTC viewer displays them
```

### Creating Scenes

**Via WebRTC UI:**
- Use text commands: "create pendulum", "create cart pole"
- LLM generation with natural language prompts
- XML editor for direct model editing

**Via MCP (Claude):**
- Claude creates scenes using MCP tools
- Scenes can be viewed in WebRTC browser interface
- Supports all Menagerie robot models

---

## Multi-Client Support

### Capabilities

- **Concurrent Connections**: Multiple browsers viewing same simulation
- **Shared State**: All clients see synchronized simulation
- **Independent Cameras**: Each client can control their own view
- **Scene Broadcasting**: Scene updates propagate to all clients

### Example Use Cases

- Teaching: Instructor controls, students watch
- Collaboration: Team members viewing simulation together
- Development: Developer + stakeholder review
- Debugging: Multiple viewpoints of same simulation

---

## Configuration

### Server Configuration

```bash
# Port and host
export VIEWER_HOST="0.0.0.0"
export VIEWER_PORT="8000"

# Video settings
export FRAME_WIDTH="640"
export FRAME_HEIGHT="480"
export FRAME_RATE="30"

# Development
export DEBUG_MODE="1"
export LOG_LEVEL="DEBUG"
```

### LLM Integration

```bash
# Configure LLM provider
curl -X POST "http://localhost:8000/api/config/api-key" \
  -H "Content-Type: application/json" \
  -d '{"provider": "openai", "api_key": "sk-..."}'
```

---

## Development

### Running Locally

```bash
# Development mode
DEBUG_MODE=1 ./scripts/run_py_viewer.sh

# Custom port
VIEWER_PORT=8080 ./scripts/run_py_viewer.sh
```

### Testing

```bash
# Unit tests
pytest tests/py_remote_viewer/ -v

# Development check
python -m py_remote_viewer.dev_check
```

---

## Technical Details

### WebRTC Stack
- **Server**: FastAPI + aiortc
- **Video**: H.264 encoding with av
- **Protocol**: WebRTC with STUN/TURN support
- **Events**: JSON-based event protocol

### MuJoCo Integration
- Real physics simulation with MuJoCo
- Default scene: Pendulum with gravity
- Custom model loading via XML
- Scene generation with LLM

For detailed API documentation, see [py_remote_viewer/README.md](../../py_remote_viewer/README.md).

---

**Last Updated**: October 2025