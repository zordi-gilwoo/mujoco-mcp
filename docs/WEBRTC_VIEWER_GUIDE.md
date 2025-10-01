# WebRTC Viewer Guide

## 🌐 **Real-time Browser-based MuJoCo Simulation**

The WebRTC Viewer provides a complete browser-based interface for interacting with MuJoCo physics simulations in real-time. This guide covers installation, usage, and advanced features.

### Quick Links
- [Installation & Setup](#installation--setup)
- [API Reference](#api-reference) 
- [LLM Integration](#llm-integration)
- [Multi-client Support](#multi-client-support)
- [Deployment Guide](#deployment-guide)

---

## 🚀 **Installation & Setup**

### Prerequisites
```bash
pip install aiortc fastapi uvicorn numpy mujoco
```

### Quick Start
```bash
# Launch WebRTC viewer
./scripts/run_py_viewer.sh

# Or using Python module
python -m py_remote_viewer --port 8000

# Open browser
open http://localhost:8000
```

### Development Check
```bash
python -m py_remote_viewer.dev_check
```

---

## 🎮 **Core Features**

### Real-time Physics Simulation
- **60 FPS MuJoCo Integration**: Native physics simulation with real-time stepping
- **WebRTC Streaming**: Low-latency video streaming to browser
- **Interactive Controls**: Mouse/keyboard/touch interaction with simulation
- **Headless Operation**: Server deployment without display requirements

### Scene Creation System
- **Natural Language Generation**: Create scenes using text prompts
- **LLM-Powered**: Integration with OpenAI, Claude, and Gemini
- **Live Editing**: Real-time XML editing with syntax highlighting
- **Dynamic Loading**: Hot-swap models during simulation

### Multi-client Architecture
- **Concurrent Connections**: Multiple users can connect simultaneously
- **Shared Simulation State**: All clients see the same physics simulation
- **Real-time Collaboration**: Scene updates broadcast to all clients
- **Automatic Cleanup**: Connection management and resource cleanup

---

## 🤖 **LLM Integration**

### Supported Providers
- **OpenAI**: GPT-4, GPT-3.5-turbo
- **Anthropic**: Claude 3.5 Sonnet
- **Google**: Gemini Pro

### Setup API Keys
1. Open the WebRTC viewer in browser
2. Click the "API Keys" button in the interface
3. Select your preferred LLM provider
4. Enter your API key
5. Test the connection

### Scene Generation
```javascript
// Example: Generate a scene using natural language
executeFreestyleCommand("Create a double pendulum with friction and gravity");
```

### API Key Management
```bash
# Configure via API
curl -X POST http://localhost:8000/api/config/api-key \
  -H "Content-Type: application/json" \
  -d '{"provider": "openai", "api_key": "sk-..."}'
```

---

## 🌐 **API Reference**

### REST Endpoints

#### Health & Status
- `GET /api/health` - Server health check
- `GET /api/config` - Server configuration
- `GET /api/stats` - Real-time statistics

#### Scene Management
- `POST /api/scene/load` - Load scene from XML
- `POST /api/scene/generate` - Generate scene using LLM
- `GET /api/scene/presets` - Available scene templates

#### LLM Configuration
- `POST /api/config/api-key` - Configure LLM API keys
- `GET /api/config/api-key` - Get current LLM configuration

### WebSocket Signaling
```javascript
// Connect to signaling server
const ws = new WebSocket('ws://localhost:8000/ws/signaling');

// Send events
ws.send(JSON.stringify({
  type: 'mouse_move',
  x: 120, y: 240,
  buttons: 1
}));
```

### Event Protocol
```json
// Mouse events
{"type": "mouse_move", "x": 120, "y": 240, "buttons": 1}
{"type": "mouse_down", "x": 120, "y": 240, "button": 0}

// Keyboard events
{"type": "key_down", "code": "Space", "alt": false, "ctrl": false}

// Command events
{"type": "command", "cmd": "pause", "params": {}}
```

---

## 🔧 **Multi-client Support**

### Architecture
```
┌─────────────────┐    WebRTC     ┌──────────────────┐
│   Browser 1     │◄─────────────►│                  │
├─────────────────┤               │  Python Server  │
│   Browser 2     │◄─────────────►│                  │
├─────────────────┤               │  • FastAPI       │
│   Browser N     │◄─────────────►│  • aiortc        │
└─────────────────┘               │  • Multi-Client  │
                                  └──────────────────┘
                                            │
                                            ▼
                                  ┌──────────────────┐
                                  │ MuJoCo Physics   │
                                  │ • Shared State   │
                                  │ • Real Simulation│
                                  └──────────────────┘
```

### Features
- **Concurrent Connections**: Support for 10+ simultaneous browser connections
- **Scene Broadcasting**: Scene updates automatically distributed to all clients
- **Independent Interaction**: Each client can control camera and interact with simulation
- **Connection Management**: Automatic tracking and cleanup of client connections

---

## 🐳 **Deployment Guide**

### Docker Deployment
```dockerfile
# Use provided Dockerfile
docker build -t mujoco-webrtc-viewer .
docker run -p 8000:8000 mujoco-webrtc-viewer
```

### Kubernetes Deployment
```bash
# Deploy using provided manifests
make k8s-deploy

# Auto-scaling with 2-10 replicas
kubectl get hpa mujoco-viewer
```

### Production Configuration
```bash
# Environment variables
export HOST=0.0.0.0
export PORT=8000
export LOG_LEVEL=INFO
export FRAME_RATE=30
```

### Load Balancing
- Nginx configuration provided for WebSocket/WebRTC optimization
- SSL/HTTPS support with certificate management
- Health check routing with automatic failover

---

## 🧪 **Testing**

### Unit Tests
```bash
cd tests/py_remote_viewer
python -m pytest test_py_remote_viewer_comprehensive.py -v
```

### End-to-End Tests
```bash
cd tests/py_remote_viewer  
python -m pytest test_py_remote_viewer_e2e.py -v
```

### Development Validation
```bash
python -m py_remote_viewer.dev_check
```

---

## 🔗 **Integration**

### MCP Server Integration
The WebRTC viewer integrates seamlessly with the main MCP server:
- **Session Isolation**: Each MCP session can have dedicated WebRTC viewers
- **Shared Models**: Access to the same MuJoCo models and scenes
- **API Compatibility**: Compatible with existing MCP tools and workflows

### Programmatic Control
```python
from py_remote_viewer import create_app
from py_remote_viewer.config import ViewerConfig

# Create server programmatically
config = ViewerConfig(host="localhost", port=8000)
app = create_app(config)

# Run with uvicorn
import uvicorn
uvicorn.run(app, host=config.host, port=config.port)
```

---

## 📊 **Performance**

### Benchmarks
- **Frame Rate**: Consistent 30-60 FPS video streaming
- **Latency**: <100ms end-to-end for local networks
- **Multi-client**: 10+ concurrent connections tested
- **Resource Usage**: Optimized for server deployment

### Optimization
- **GPU Acceleration**: EGL headless rendering support
- **Hardware Encoding**: H.264 encoding with NVENC/QuickSync fallbacks
- **Connection Pooling**: Efficient resource sharing across clients

---

## 🛠️ **Development**

### Project Structure
```
py_remote_viewer/
├── __init__.py              # Package initialization
├── config.py               # Configuration management
├── server.py               # FastAPI server
├── signaling.py            # WebRTC signaling
├── webrtc_track.py         # Video track implementation
├── mujoco_simulation.py    # MuJoCo physics integration
├── events.py               # Event protocol definitions
├── camera_state.py         # Camera control logic
└── logging_utils.py        # Logging utilities

client/
├── index.html              # Web interface
├── app.js                  # WebRTC client logic
└── styles.css              # Interface styling
```

### Contributing
See the main [CONTRIBUTING.md](../CONTRIBUTING.md) for development guidelines and contribution process.

### Issues & Support
For issues specific to the WebRTC viewer, please file issues with the "webrtc-viewer" label in the main repository.

---

## 📚 **Additional Resources**

- [Main MuJoCo MCP Documentation](../DOCUMENTATION_INDEX.md)
- [API Reference](API_REFERENCE.md) 
- [Architecture Guide](../ARCHITECTURE.md)
- [Advanced Features](../ADVANCED_FEATURES_GUIDE.md)
- [Examples](../examples/)