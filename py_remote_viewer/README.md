# MuJoCo Remote Viewer - Python Scaffold

A fully Python-based, headless, browser-accessible MuJoCo viewer scaffold that enables running simulations on a headless server and interacting through a WebRTC pipeline from a web browser.

## 🎯 Overview

This is a fully Python-based, headless, browser-accessible MuJoCo viewer that provides a complete WebRTC-based remote viewing solution. The system enables running simulations on headless servers with real-time browser interaction and full MuJoCo physics simulation capabilities.

## ✨ Features

- **WebRTC Video Streaming**: Real-time video streaming using aiortc
- **Interactive Controls**: Mouse, keyboard, and touch input handling
- **Camera Management**: Orbit/pan/zoom camera controls with presets
- **Simulation Interface**: Start/pause/reset simulation controls
- **Scene Creation**: Text prompt-based scene generation with XML editor
- **Event Protocol**: JSON-based event system for all interactions
- **MuJoCo Physics**: Real physics simulation with graceful error handling
- **Web Interface**: Modern, responsive HTML/CSS/JS client
- **FastAPI Server**: High-performance async web server
- **Configuration**: Environment-based configuration system
- **Logging**: Comprehensive logging with configurable levels

## 🔬 **MuJoCo Integration**

The remote viewer now includes full MuJoCo physics simulation:

### Real Physics Simulation
- **Default Model**: Pendulum with gravity and damping
- **Custom Models**: Load any MuJoCo XML model via API
- **Physics Stepping**: Real-time simulation at 60 FPS
- **State Management**: Full access to joint positions, velocities, forces

### Scene Creation Interface
- **Text Prompts**: Enter natural language descriptions to generate scenes
- **Preset Scenes**: Quick access to common simulation types (pendulum, cart-pole, etc.)
- **XML Editor**: View and edit generated MuJoCo XML with syntax highlighting
- **Live Loading**: Dynamically load new scenes into running simulation
- **Validation**: Real-time XML validation and error reporting

### Rendering Pipeline
- **Headless Support**: Works in server environments without displays
- **Error Handling**: Graceful fallback when OpenGL unavailable
- **Frame Generation**: Real MuJoCo frames when possible, error frames when not
- **Camera Integration**: MuJoCo camera system with user controls

### Configuration
```bash
# Start MuJoCo remote viewer (default)
./scripts/run_py_viewer.sh
```

## 🏗 Architecture

```
┌─────────────────┐    WebRTC     ┌──────────────────┐
│   Web Browser   │◄─────────────►│  Python Server  │
│                 │               │                  │
│ • HTML Client   │   WebSocket   │ • FastAPI        │
│ • WebRTC Peer   │   Signaling   │ • aiortc         │
│ • Event Capture │               │ • Video Track    │
└─────────────────┘               └──────────────────┘
                                            │
                                            ▼
                                  ┌──────────────────┐
                                  │ Simulation Stub  │
                                  │ • Camera State   │
                                  │ • Event Handler  │
                                  │ • Future: MuJoCo │
                                  └──────────────────┘
```

### Core Components

- **`server.py`**: FastAPI application with WebRTC signaling
- **`signaling.py`**: WebSocket-based WebRTC signaling server
- **`webrtc_track.py`**: Video track implementation
- **`events.py`**: Event protocol definitions and parsing
- **`camera_state.py`**: Camera state management and controls
- **`simulation_stub.py`**: Placeholder simulation interface
- **`config.py`**: Configuration management
- **Client**: HTML/CSS/JS web interface

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- Modern web browser with WebRTC support

### Installation

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Start the server:**
   ```bash
   ./scripts/run_py_viewer.sh
   ```

3. **Open your browser:**
   Navigate to `http://localhost:8000`

### Alternative Start Methods

```bash
# Using Python module
python -m py_remote_viewer

# Using server directly
python -m py_remote_viewer.server

# With custom configuration
VIEWER_PORT=8080 DEBUG_MODE=1 ./scripts/run_py_viewer.sh
```

## 🎮 Usage

### Web Interface

1. **Connect**: Click "Connect" to establish WebRTC connection
2. **View**: Watch the MuJoCo physics simulation
3. **Interact**: 
   - Left-click + drag to rotate camera
   - Right-click + drag to pan camera
   - Scroll wheel to zoom
   - Arrow keys for precise rotation
   - Page Up/Down for zoom
   - Space bar to pause/resume

### Camera Controls

- **Preset Views**: Front, Side, Top, Reset buttons
- **Mouse Controls**: Left-drag (rotate), Right-drag (pan), Scroll (zoom)
- **Keyboard**: Arrow keys, Page Up/Down, Space

### Simulation Controls

- **Start**: Begin simulation
- **Pause**: Pause/resume simulation
- **Reset**: Reset to initial state

## ⚙️ Configuration

Configure via environment variables:

```bash
# Server settings
export VIEWER_HOST="localhost"
export VIEWER_PORT="8000"

# Frame settings
export FRAME_WIDTH="640"
export FRAME_HEIGHT="480"
export FRAME_RATE="30"

# Development settings
export DEBUG_MODE="0"
export DEBUG_MODE="0"
export LOG_LEVEL="INFO"

# WebRTC settings
export STUN_SERVER="stun:stun.l.google.com:19302"
```

### ViewerConfig Options

```python
config = ViewerConfig(
    host="localhost",           # Server host
    port=8000,                 # Server port
    frame_width=640,           # Video frame width
    frame_height=480,          # Video frame height
    frame_rate=30,             # Target frame rate
    debug_mode=False,          # Enable debug logging
    log_level="INFO"           # Logging level
)
```

## 🔧 Development

### Running Tests

```bash
# Basic import and functionality check
python -m py_remote_viewer.dev_check

# Check specific components
python -c "import py_remote_viewer; print('ok')"
```

### Development Server

```bash
# Debug mode with detailed logging
DEBUG_MODE=1 LOG_LEVEL=DEBUG python -m py_remote_viewer
```

### Project Structure

```
py_remote_viewer/
├── __init__.py              # Package initialization
├── __main__.py              # CLI entry point
├── config.py                # Configuration management
├── server.py                # FastAPI server
├── signaling.py             # WebRTC signaling
├── webrtc_track.py          # Video track implementation
├── events.py                # Event protocol
├── camera_state.py          # Camera state management
├── simulation_stub.py       # Simulation interface (legacy)
├── mujoco_simulation.py     # Real MuJoCo integration
├── app_factory.py           # App factory patterns
├── logging_utils.py         # Logging utilities
├── dev_check.py             # Development checks
└── README.md               # This file

client/
├── index.html              # Main web interface
├── app.js                  # Client-side JavaScript
└── styles.css              # Styling
```

## 📡 Event Protocol

The system uses a JSON-based event protocol for all interactions:

### Mouse Events
```json
{
  "type": "mouse_move",
  "x": 120,
  "y": 240,
  "buttons": 1
}
```

### Scroll Events
```json
{
  "type": "scroll",
  "x": 120,
  "y": 240,
  "dx": 0,
  "dy": -1
}
```

### Keyboard Events
```json
{
  "type": "key_down",
  "code": "Space",
  "key": " ",
  "alt": false,
  "ctrl": false,
  "shift": false
}
```

### Command Events
```json
{
  "type": "command",
  "cmd": "pause",
  "params": {}
}
```

## 🔮 Integration Status & Future Steps

### ✅ **Phase 1: MuJoCo Integration (COMPLETED)**
1. ✅ Replace `SimulationStub` with real MuJoCo simulation
2. ✅ Implement MuJoCo model loading and physics simulation
3. ✅ Add frame capture interface (with headless fallback)
4. ✅ Replace `SyntheticVideoTrack` with `MuJoCoVideoTrack` (COMPLETED)

**Current Status:** The remote viewer now supports real MuJoCo physics simulation! The system can:
- Load and run MuJoCo models with full physics simulation
- Handle camera controls that affect MuJoCo rendering
- Stream real simulation data via the API
- Gracefully fall back to error frames in headless environments (no EGL/OpenGL)

**Usage:**
```bash
# Use real MuJoCo simulation (default)
./scripts/run_py_viewer.sh
```

### 🔄 **Phase 2: Performance Optimization (Next)**
1. Implement EGL headless rendering context for server environments
2. Add H.264 encoding via PyAV for better performance
3. Optimize frame capture and encoding pipeline
4. Add performance metrics and monitoring
5. Hardware acceleration support

### 🚀 **Phase 3: Advanced Features**
1. Multi-client support with session management
2. Authentication and authorization
3. Advanced camera controls and modes
4. Scene loading and model switching
5. Physics debugging visualization

### 🏭 **Phase 4: Production Deployment**
1. Docker containerization
2. Kubernetes deployment configs
3. Load balancing and scaling
4. Monitoring and alerting

## 📊 Performance Metrics

The server reports metrics every 5 seconds:

- Connected clients count
- Total connection attempts
- Events received count
- Frame generation rate (MuJoCo)
- Memory usage (future)

Access metrics via:
- Server logs
- `/api/stats` endpoint
- Web interface statistics panel

## 🌐 **API Reference**

The server provides several REST API endpoints:

### Configuration & Status
- **`GET /api/config`** - Get current server configuration
- **`GET /api/stats`** - Get server statistics and simulation state
- **`GET /api/health`** - Health check endpoint

### Scene Management
- **`POST /api/scene/load`** - Load a new scene from MuJoCo XML
  ```json
  {
    "xml": "<mujoco>...</mujoco>"
  }
  ```
  Response:
  ```json
  {
    "success": true,
    "message": "Scene loaded successfully"
  }
  ```

### WebSocket Endpoints
- **`WS /ws/signaling`** - WebRTC signaling and event handling

## 🐛 Troubleshooting

### Common Issues

**1. WebRTC Connection Fails**
```bash
# Check STUN server connectivity
export STUN_SERVER="stun:stun.l.google.com:19302"
```

**2. Import Errors**
```bash
# Install missing dependencies
pip install -r requirements.txt

# Check Python version
python --version  # Should be 3.10+
```

**3. Port Already in Use**
```bash
# Use different port
export VIEWER_PORT="8001"
```

**4. Browser Compatibility**
- Ensure modern browser with WebRTC support
- Enable camera/microphone permissions if prompted
- Try different browser if issues persist

### Debug Mode

Enable detailed logging:
```bash
DEBUG_MODE=1 LOG_LEVEL=DEBUG python -m py_remote_viewer
```

### Health Check

```bash
# Check server health
curl http://localhost:8000/api/health

# Check configuration
curl http://localhost:8000/api/config
```

## 📄 License

This project follows the same license as the main mujoco-mcp repository.

## 🤝 Contributing

1. Follow existing code style and patterns
2. Add tests for new functionality
3. Update documentation for changes
4. Use type hints and docstrings
5. Test with multiple browsers

## 📞 Support

For issues specific to this Python scaffold:
1. Check the troubleshooting section
2. Run the development check: `python -m py_remote_viewer.dev_check`
3. Enable debug logging
4. Open an issue with detailed logs and reproduction steps