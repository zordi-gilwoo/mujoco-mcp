# WebRTC Remote Viewer

## Overview

The WebRTC Remote Viewer is a Python-based headless viewer that enables browser-based access to MuJoCo simulations through real-time video streaming. This system provides a modern alternative for accessing physics simulations remotely without requiring local MuJoCo installation.

## Features

### Core WebRTC Architecture
- **Real MuJoCo Integration**: Native physics simulation with 60 FPS stepping
- **WebRTC Streaming**: Real-time video with hardware acceleration support
- **Multi-Client Support**: Concurrent browser connections with shared simulation state
- **Professional Web Interface**: Responsive HTML5 design with real-time collaboration

### LLM-Powered Scene Creation
- **Multi-Provider Support**: OpenAI GPT-4, Claude 3.5 Sonnet, Google Gemini
- **API Key Management**: Secure in-browser configuration with local storage
- **Natural Language Processing**: Text-to-scene generation ("Create a double pendulum")
- **Dynamic Loading**: Hot-swap models during simulation with client broadcasting

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

## Quick Start

### Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Launch server
./scripts/run_py_viewer.sh

# Open browser
open http://localhost:8000
```

### Docker Deployment

```bash
# Build and run with Docker
make build && make docker

# Or use docker-compose for full stack
docker-compose up
```

### Kubernetes Deployment

```bash
# Deploy to Kubernetes cluster
make k8s-deploy

# Monitor deployment
kubectl get pods,services,ingress
```

## Architecture

```
┌─────────────────┐    WebRTC     ┌──────────────────┐
│   Web Browser   │◄─────────────►│  Python Server  │
│                 │               │                  │
│ • HTML Client   │   WebSocket   │ • FastAPI        │
│ • WebRTC Peer   │   Signaling   │ • aiortc         │
│ • Event Capture │               │ • Multi-Client   │
│ • Scene Creator │               │ • Broadcasting   │
│ • RL Interface  │               │ • EGL Rendering  │
└─────────────────┘               └──────────────────┘
                                            │
                                            ▼
                                  ┌──────────────────┐
                                  │ MuJoCo Physics   │
                                  │ • Real Simulation│
                                  │ • RL Environments│
                                  │ • Scene Loading  │
                                  │ • GPU Rendering  │
                                  └──────────────────┘
```

## Configuration

### Environment Variables

- `HOST` - Server bind address (default: localhost)
- `PORT` - Server port (default: 8000)
- `LOG_LEVEL` - Logging level (INFO, DEBUG, WARNING, ERROR)
- `FRAME_WIDTH` / `FRAME_HEIGHT` - Video resolution
- `FRAME_RATE` - Video frame rate (default: 30)

### API Endpoints

- `GET /api/health` - Health check with system status
- `GET /api/config` - Server configuration
- `GET /api/stats` - Performance metrics and connection stats
- `POST /api/scene/load` - Load new scene from XML
- `POST /api/scene/generate` - Generate scene from text prompt
- `WS /ws/signaling` - WebRTC signaling and events

## Integration with Main MCP Server

The WebRTC viewer complements the main MCP server by providing:

- **Visual Interface**: Browser-based access to simulations
- **Real-time Collaboration**: Multiple users viewing same simulation
- **Remote Access**: Run simulations on servers, view from browsers
- **Enhanced Interaction**: Mouse/keyboard controls and scene creation

### Combined Architecture

```
Claude AI → MCP Server → Session Management
                    ↓
               WebRTC Viewer → Multi-client Visualization
```

## Testing

### Running Tests

```bash
# Run comprehensive test suite
python -m pytest tests/py_remote_viewer/ -v

# Run development validation
python -m py_remote_viewer.dev_check

# Run browser automation tests
python -m pytest tests/py_remote_viewer/test_py_remote_viewer_e2e.py
```

### Test Coverage

- **Unit Tests**: 16/16 tests passing
- **Integration Tests**: 22/22 modules operational
- **End-to-End Tests**: Playwright browser automation
- **Production Validation**: Headless environment compatibility

## Production Deployment

### Docker

```dockerfile
FROM python:3.9-slim
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY . /app
WORKDIR /app
EXPOSE 8000
CMD ["python", "-m", "py_remote_viewer", "--host", "0.0.0.0"]
```

### Kubernetes

See `k8s-deployment.yaml` for complete manifests including:
- Deployment with auto-scaling (2-10 replicas)
- Service with load balancing
- Ingress with SSL termination
- Health checks and resource limits

### Load Balancing

The included nginx configuration provides:
- WebSocket/WebRTC proxy support
- SSL termination
- Connection upgrade handling
- Health check routing

## Related Documentation

- [Main Project README](../README.md) - Overall project overview
- [MCP API Reference](../API_REFERENCE.md) - MCP protocol documentation
- [Architecture Guide](../ARCHITECTURE.md) - System design overview
- [Advanced Features](../ADVANCED_FEATURES_GUIDE.md) - Advanced capabilities