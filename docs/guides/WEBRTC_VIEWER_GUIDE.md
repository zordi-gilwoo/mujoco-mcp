# WebRTC Viewer Guide

**Last Updated:** October 2025

## 🌐 Real-time Browser-based MuJoCo Simulation

The WebRTC Viewer provides browser-based access to MuJoCo simulations through real-time video streaming.

---

## 🚀 Quick Start

### Installation

```bash
# Install dependencies
pip install aiortc fastapi uvicorn numpy mujoco

# Start viewer
./scripts/run_py_viewer.sh

# Open browser
open http://localhost:8000
```

### Development Check

```bash
# Validate installation
python -m py_remote_viewer.dev_check
```

---

## 🎮 Features

### Real-time Physics Simulation
- 60 FPS MuJoCo physics simulation
- WebRTC video streaming to browser
- Interactive mouse/keyboard/touch controls
- Headless server operation (no display required)

### Multi-Client Support
- Multiple browsers can connect simultaneously
- Shared simulation state across all clients
- Independent camera controls per client
- Automatic connection cleanup

### Scene Creation
- **Built-in Scenes**: Pendulum, cart-pole, Franka Panda
- **Text Commands**: "create pendulum", "play", "pause", "reset"
- **LLM Integration**: Natural language scene generation
- **XML Editor**: Direct model editing

### LLM Integration
- **Supported Providers**: OpenAI (GPT-4), Claude (Anthropic), Gemini (Google)
- **API Key Configuration**: Secure browser-based setup
- **Natural Language**: "Create a cart pole with a 2m long pole"
- **Dynamic Loading**: Hot-swap models during simulation

---

## 🎯 Usage

### Web Interface Controls

**Camera:**
- Left-click + drag: Rotate camera
- Right-click + drag: Pan camera
- Scroll wheel: Zoom
- Arrow keys: Precise rotation
- Page Up/Down: Zoom

**Simulation:**
- Start/Pause buttons
- Reset button
- Step-by-step execution

### Text Commands

```bash
# Simulation control
"play" or "start"    - Begin simulation
"pause"              - Pause simulation
"reset"              - Reset to initial state
"step 10"            - Step 10 frames

# Scene loading
"create pendulum"    - Load pendulum scene
"create cart pole"   - Load cart-pole scene
"create franka panda" - Load Franka robot
```

### API Usage

```bash
# Load scene
curl -X POST "http://localhost:8000/api/scene/load" \
  -H "Content-Type: application/json" \
  -d '{"xml": "<mujoco>...</mujoco>"}'

# Configure LLM
curl -X POST "http://localhost:8000/api/config/api-key" \
  -H "Content-Type: application/json" \
  -d '{"provider": "openai", "api_key": "sk-..."}'

# Generate scene
curl -X POST "http://localhost:8000/api/scene/generate" \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Create a double pendulum"}'
```

---

## ⚙️ Configuration

### Environment Variables

```bash
# Server settings
export VIEWER_HOST="localhost"
export VIEWER_PORT="8000"

# Video settings
export FRAME_WIDTH="640"
export FRAME_HEIGHT="480"
export FRAME_RATE="30"

# Development
export DEBUG_MODE="1"
export LOG_LEVEL="DEBUG"

# WebRTC
export STUN_SERVER="stun:stun.l.google.com:19302"
```

### LLM Provider Setup

```bash
# OpenAI
export OPENAI_API_KEY="sk-..."
export OPENAI_MODEL="gpt-4"

# Claude
export CLAUDE_API_KEY="sk-ant-..."
export CLAUDE_MODEL="claude-3-sonnet-20241022"

# Gemini
export GEMINI_API_KEY="..."
export GEMINI_MODEL="gemini-1.5-pro"
```

---

## 📡 API Endpoints

### Health & Status
- `GET /api/health` - Server health check
- `GET /api/config` - Server configuration
- `GET /api/stats` - Server statistics

### Scene Management
- `POST /api/scene/load` - Load scene from XML
- `GET /api/scene/current` - Get current scene
- `POST /api/scene/generate` - Generate scene with LLM

### LLM Configuration
- `POST /api/config/api-key` - Configure LLM provider

### Commands
- `POST /api/execute-command` - Execute text commands

### WebSocket
- `WS /ws/signaling` - WebRTC signaling and events

---

## 🧪 Testing

### Unit Tests
```bash
# Run all tests (16 tests)
pytest tests/py_remote_viewer/test_py_remote_viewer_comprehensive.py -v

# Development validation
python -m py_remote_viewer.dev_check
```

### End-to-End Tests
```bash
# Browser automation tests
pytest tests/py_remote_viewer/test_py_remote_viewer_e2e.py -v
```

---

## 🐛 Troubleshooting

### WebRTC Connection Fails
- Check STUN server: `export STUN_SERVER="stun:stun.l.google.com:19302"`
- Ensure modern browser with WebRTC support
- Try different browser

### Black Screen / No Rendering
- Check MuJoCo initialization in logs
- Verify OpenGL/EGL context available
- System falls back to error frames in headless environments

### Port Already in Use
- Use different port: `VIEWER_PORT=8001 ./scripts/run_py_viewer.sh`
- Kill existing process: `pkill -f py_remote_viewer`

### Scene Generation Not Working
- Configure LLM API key via `/api/config/api-key`
- Check server logs for LLM errors
- Install provider package: `pip install openai anthropic google-generativeai`

---

## 📊 Performance

### Metrics
- **Frame Rate**: 30-60 FPS
- **Latency**: <100ms (local network)
- **Concurrent Clients**: 10+ tested
- **Resource Usage**: ~512MB base + per-client overhead

### Optimization
- Uses efficient H.264 video encoding
- Shared simulation state across clients
- Automatic resource cleanup
- Connection pooling

---

## 📚 Additional Resources

- [py_remote_viewer/README.md](../../py_remote_viewer/README.md) - Package documentation
- [DOCUMENTATION_INDEX.md](../../DOCUMENTATION_INDEX.md) - Complete doc index
- [AGENTS.md](../../AGENTS.md) - Scene generation system

---

**Status**: Production Ready (v0.1.0)  
**Last Updated**: October 2025