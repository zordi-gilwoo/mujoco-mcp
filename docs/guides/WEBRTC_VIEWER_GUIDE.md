# WebRTC Viewer Guide

**Last Updated:** October 5, 2025

## 🌐 **Real-time Browser-based MuJoCo Simulation**

The WebRTC Viewer is a Python-based headless viewer that enables browser-based access to MuJoCo simulations through real-time video streaming. This system provides a modern alternative for accessing physics simulations remotely without requiring local MuJoCo installation.

### Quick Links
- [Installation & Setup](#installation--setup)
- [Core Features](#core-features)
- [LLM Integration](#llm-integration)
- [API Reference](#api-reference)
- [Multi-client Support](#multi-client-support)
- [Deployment Guide](#deployment-guide)
- [Testing](#testing)

---

## 🚀 **Installation & Setup**

### Prerequisites
```bash
pip install aiortc fastapi uvicorn numpy mujoco
```

### Quick Start
```bash
# Install dependencies
pip install -r requirements.txt

# Launch WebRTC viewer (recommended)
./scripts/run_py_viewer.sh

# Or using Python module directly
python -m py_remote_viewer --port 8000

# Open browser
open http://localhost:8000
```

### Development Check
```bash
# Validate installation and dependencies
python -m py_remote_viewer.dev_check
```

### Configuration

#### Environment Variables
- `HOST` - Server bind address (default: localhost)
- `PORT` - Server port (default: 8000)
- `LOG_LEVEL` - Logging level (INFO, DEBUG, WARNING, ERROR)
- `FRAME_WIDTH` / `FRAME_HEIGHT` - Video resolution
- `FRAME_RATE` - Video frame rate (default: 30)

---

## 🎮 **Core Features**

### Real-time Physics Simulation
- **60 FPS MuJoCo Integration**: Native physics simulation with real-time stepping
- **WebRTC Streaming**: Real-time video with hardware acceleration support
- **Interactive Controls**: Mouse/keyboard/touch interaction with simulation
- **Headless Operation**: Server deployment without X11/display requirements
- **Multi-Client Support**: Concurrent browser connections with shared simulation state

### LLM-Powered Scene Creation
- **Multi-Provider Support**: OpenAI GPT-4, Claude 3.5 Sonnet, Google Gemini
- **API Key Management**: Secure in-browser configuration with local storage
- **Natural Language Processing**: Text-to-scene generation ("Create a double pendulum")
- **Dynamic Loading**: Hot-swap models during simulation with client broadcasting
- **Live Editing**: Real-time XML editing with syntax highlighting

### Reinforcement Learning Integration
- **Gymnasium Integration**: Full reinforcement learning environment support
- **Multi-Robot Support**: Franka Panda, UR5e, ANYmal-C, cart-pole configurations
- **Task Templates**: Reaching, balancing, locomotion, manipulation scenarios
- **Script Generation**: Automated training pipeline creation

### GPU Acceleration & Performance
- **EGL Headless Rendering**: GPU acceleration without X11 displays
- **Hardware H.264 Encoding**: NVENC, QuickSync, AMF with software fallbacks
- **Multi-client Optimization**: Efficient resource sharing and cleanup
- **Performance Monitoring**: Real-time metrics collection and reporting

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
- `GET /api/health` - Health check with system status
- `GET /api/config` - Server configuration and capabilities
- `GET /api/stats` - Real-time statistics and performance metrics

#### Scene Management
- `POST /api/scene/load` - Load scene from MuJoCo XML
- `POST /api/scene/generate` - Generate scene using LLM from text prompt
- `GET /api/scene/presets` - Available scene templates

#### RL Integration
- `POST /api/rl/create` - Create RL environment
- `GET /api/rl/robots` - Available robot configurations

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
├─────────────────┤               │  Python Server   │
│   Browser 2     │◄─────────────►│                  │
├─────────────────┤               │  • FastAPI       │
│   Browser N     │◄─────────────►│  • aiortc        │
└─────────────────┘               │  • Multi-Client  │
                                  │  • Broadcasting  │
                                  │  • EGL Rendering │
                                  └──────────────────┘
                                            │
                                            ▼
                                  ┌──────────────────┐
                                  │ MuJoCo Physics   │
                                  │ • Shared State   │
                                  │ • Real Simulation│
                                  │ • RL Environments│
                                  │ • Scene Loading  │
                                  │ • GPU Rendering  │
                                  └──────────────────┘
```

### Features
- **Concurrent Connections**: Support for 10+ simultaneous browser connections
- **Shared Simulation State**: All clients see the same physics simulation
- **Real-time Collaboration**: Scene updates broadcast to all clients
- **Independent Interaction**: Each client can control camera and interact with simulation
- **Connection Management**: Automatic tracking and cleanup of client connections
- **Automatic Cleanup**: Connection management and resource cleanup

---

## 🐳 **Deployment Guide**

### Docker Deployment
```dockerfile
FROM python:3.9-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY py_remote_viewer/ ./py_remote_viewer/
COPY client/ ./client/
COPY scripts/ ./scripts/

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:8000/api/health || exit 1

# Run server
CMD ["python", "-m", "py_remote_viewer", "--host", "0.0.0.0", "--port", "8000"]
```

```bash
# Build and run
docker build -t mujoco-webrtc-viewer .
docker run -p 8000:8000 mujoco-webrtc-viewer
```

### Docker Compose Deployment
```yaml
version: '3.8'

services:
  mujoco-viewer:
    build: .
    ports:
      - "8000:8000"
    environment:
      - LOG_LEVEL=INFO
      - HOST=0.0.0.0
      - PORT=8000
    volumes:
      - ./logs:/app/logs
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/api/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - mujoco-viewer
    restart: unless-stopped
```

```bash
# Deploy full stack
docker-compose up -d
```

### Kubernetes Deployment
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: mujoco-viewer
spec:
  replicas: 3
  selector:
    matchLabels:
      app: mujoco-viewer
  template:
    metadata:
      labels:
        app: mujoco-viewer
    spec:
      containers:
      - name: mujoco-viewer
        image: mujoco-viewer:latest
        ports:
        - containerPort: 8000
        env:
        - name: HOST
          value: "0.0.0.0"
        - name: PORT
          value: "8000"
        resources:
          requests:
            memory: "512Mi"
            cpu: "250m"
          limits:
            memory: "2Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /api/health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /api/health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5

---
apiVersion: v1
kind: Service
metadata:
  name: mujoco-viewer-service
spec:
  selector:
    app: mujoco-viewer
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8000
  type: LoadBalancer
```

```bash
# Deploy to Kubernetes cluster
kubectl apply -f k8s-deployment.yaml

# Check deployment status
kubectl get pods,services,ingress

# Monitor deployment
kubectl get hpa mujoco-viewer

# Auto-scaling with 2-10 replicas
kubectl autoscale deployment mujoco-viewer --cpu-percent=70 --min=2 --max=10
```

### Production Configuration
```bash
# Environment variables
export HOST=0.0.0.0
export PORT=8000
export LOG_LEVEL=INFO
export FRAME_RATE=30
export FRAME_WIDTH=640
export FRAME_HEIGHT=480
```

### Load Balancing
- Nginx configuration provided for WebSocket/WebRTC optimization
- SSL/HTTPS support with certificate management
- Health check routing with automatic failover
- Connection upgrade handling for WebSocket/WebRTC

---

## 🧪 **Testing**

### Comprehensive Test Suite

#### Unit Tests
```bash
# Run all unit tests
python -m pytest tests/py_remote_viewer/test_py_remote_viewer_comprehensive.py -v
```

**Coverage:**
- **16/16 unit tests passing** ✅
- Configuration management
- Event protocol serialization
- Camera state controls
- Simulation lifecycle
- WebRTC track functionality

#### Integration Tests
- **22/22 modules operational** ✅
- Complete import validation
- API endpoint functionality
- MuJoCo integration
- Server startup/shutdown

#### End-to-End Tests
```bash
# Run browser automation tests
python -m pytest tests/py_remote_viewer/test_py_remote_viewer_e2e.py -v
```

**Coverage:**
- Browser interface loading
- WebRTC connection establishment
- Event capture and processing
- Scene loading and management
- Multi-client connection testing

#### Development Validation
```bash
python -m py_remote_viewer.dev_check
```

**Validates:**
- All module imports (22 modules)
- Basic functionality
- MuJoCo integration
- Configuration loading
- Headless environment compatibility

### Test Results
- **Unit Tests**: 16/16 passing ✅
- **Integration Tests**: 22/22 modules operational ✅
- **End-to-End Tests**: Playwright browser automation ✅
- **Production Validation**: Headless environment compatibility ✅

---

## 🔗 **Integration**

### MCP Server Integration
The WebRTC viewer integrates seamlessly with the main MCP server:

```
Claude AI → MCP Server → Session Management
                    ↓
               WebRTC Viewer → Multi-client Visualization
```

**Benefits:**
- **Visual Interface**: Browser-based access to simulations
- **Real-time Collaboration**: Multiple users viewing same simulation
- **Remote Access**: Run simulations on servers, view from browsers
- **Enhanced Interaction**: Mouse/keyboard controls and scene creation
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
- **Resource Usage**: 512MB-2GB base + per-client overhead

### Optimization
- **GPU Acceleration**: EGL headless rendering support
- **Hardware Encoding**: H.264 encoding with NVENC/QuickSync/AMF fallbacks
- **Connection Pooling**: Efficient resource sharing across clients
- **CPU Usage**: 10-30% per client (varies with scene complexity)
- **Network**: Efficient H.264 encoding reduces bandwidth requirements

---

## 🛠️ **Development**

### Project Structure
```
py_remote_viewer/
├── __init__.py              # Package initialization
├── __main__.py              # CLI entry point
├── config.py                # Configuration management
├── server.py                # FastAPI server
├── signaling.py             # WebRTC signaling
├── webrtc_track.py          # Video track implementation
├── mujoco_simulation.py     # MuJoCo physics integration
├── events.py                # Event protocol definitions
├── camera_state.py          # Camera control logic
├── app_factory.py           # FastAPI application factory
├── logging_utils.py         # Logging utilities
├── dev_check.py             # Development validation
└── README.md                # Package documentation

client/
├── index.html               # Web interface
├── app.js                   # WebRTC client logic
└── styles.css               # Interface styling
```

### Contributing
See the main [CONTRIBUTING.md](../CONTRIBUTING.md) for development guidelines and contribution process.

### Issues & Support
For issues specific to the WebRTC viewer, please file issues with the "webrtc-viewer" label in the main repository.

---

## 📚 **Additional Resources**

- [Main MuJoCo MCP Documentation](../DOCUMENTATION_INDEX.md)
- [WebRTC API Reference](../WEBRTC_API_REFERENCE.md)
- [Architecture Guide](../ARCHITECTURE.md)
- [Advanced Features](../ADVANCED_FEATURES_GUIDE.md)
- [Testing Results](../TEST_RESULTS.md)
- [Examples](../examples/)

---

**Document Status:** ✅ Consolidated from WEBRTC_VIEWER.md and WEBRTC_VIEWER_GUIDE.md  
**Last Updated:** October 5, 2025