# WebRTC Viewer API Reference

## 🌐 **Complete API Documentation**

This document provides comprehensive API documentation for the Python WebRTC MuJoCo viewer, covering all endpoints, WebSocket messages, and JavaScript client APIs.

---

## 📋 **Table of Contents**

1. [HTTP REST API](#http-rest-api)
2. [WebSocket Signaling API](#websocket-signaling-api)  
3. [JavaScript Client API](#javascript-client-api)
4. [Python Server API](#python-server-api)
5. [Event Protocol](#event-protocol)
6. [Configuration API](#configuration-api)

---

## 🔧 **HTTP REST API**

### **Base URL**: `http://localhost:8000`

### **Health & Status Endpoints**

#### `GET /api/health`
Health check endpoint for monitoring server status.

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2024-01-15T10:30:00Z",
  "version": "1.0.0",
  "uptime": 3600
}
```

#### `GET /api/config`
Get current server configuration.

**Response:**
```json
{
  "frame_width": 640,
  "frame_height": 480,
  "frame_rate": 30,
  "port": 8000,
  "host": "localhost",
  "debug": false,
  "gpu_enabled": true,
  "h264_enabled": true
}
```

#### `GET /api/stats`
Get detailed server statistics and metrics.

**Response:**
```json
{
  "clients_connected": 3,
  "total_connections": 15,
  "events_received": 1250,
  "frames_sent": 45000,
  "uptime": 3600,
  "cpu_usage": 15.2,
  "memory_usage": 256.7,
  "gpu_usage": 85.3,
  "mujoco": {
    "initialized": true,
    "initialization_error": null,
    "model_name": "default_pendulum",
    "nbody": 3,
    "nq": 1,
    "nu": 1,
    "renderer_available": true,
    "simulation_time": 125.6,
    "simulation_steps": 7536
  }
}
```

### **Scene Management Endpoints**

#### `POST /api/scene/load`
Load a new scene from MuJoCo XML.

**Request Body:**
```json
{
  "xml": "<mujoco model=\"my_scene\">...</mujoco>",
  "reset_simulation": true
}
```

**Response:**
```json
{
  "success": true,
  "message": "Scene loaded successfully",
  "model_info": {
    "name": "my_scene",
    "nbody": 5,
    "nq": 3,
    "nu": 2
  }
}
```

#### `GET /api/scene/current`
Get current scene information.

**Response:**
```json
{
  "model_name": "default_pendulum",
  "xml": "<mujoco>...</mujoco>",
  "bodies": ["world", "pendulum", "mass"],
  "joints": ["pivot"],
  "actuators": ["motor"]
}
```

#### `POST /api/scene/create`
Create a scene from natural language description.

**Request Body:**
```json
{
  "prompt": "Create a double pendulum with friction",
  "parameters": {
    "length1": 0.6,
    "length2": 0.4,
    "mass1": 0.5,
    "mass2": 0.3,
    "damping": 0.1
  }
}
```

**Response:**
```json
{
  "success": true,
  "xml": "<mujoco>...</mujoco>",
  "message": "Double pendulum scene created successfully"
}
```

### **RL Integration Endpoints**

#### `POST /api/rl/create-environment`
Create a reinforcement learning environment.

**Request Body:**
```json
{
  "robot_type": "franka_panda",
  "task_type": "reaching",
  "parameters": {
    "workspace_size": [0.5, 0.5, 0.3],
    "target_position": [0.3, 0.2, 0.1],
    "reward_type": "dense"
  }
}
```

**Response:**
```json
{
  "success": true,
  "environment_id": "franka_reaching_001",
  "xml": "<mujoco>...</mujoco>",
  "training_script": "# Complete RL training script\nimport gymnasium as gym\n..."
}
```

#### `GET /api/rl/environments`
List available RL environment types.

**Response:**
```json
{
  "robots": ["franka_panda", "ur5e", "anymal_c", "cart_pole"],
  "tasks": ["reaching", "balancing", "locomotion", "manipulation"],
  "action_spaces": ["continuous", "discrete"],
  "observation_types": ["state", "image", "hybrid"]
}
```

### **Recording & Export Endpoints**

#### `POST /api/record/start`
Start recording simulation video.

**Request Body:**
```json
{
  "format": "mp4",
  "quality": "high",
  "duration": 30,
  "fps": 30
}
```

**Response:**
```json
{
  "success": true,
  "recording_id": "rec_001",
  "message": "Recording started"
}
```

#### `GET /api/record/{recording_id}/status`
Get recording status.

**Response:**
```json
{
  "status": "recording",
  "elapsed_time": 15.2,
  "frames_captured": 456,
  "file_size": "2.3 MB"
}
```

#### `GET /api/record/{recording_id}/download`
Download completed recording.

**Response:** Binary video file with appropriate headers.

---

## 🔌 **WebSocket Signaling API**

### **Connection**: `ws://localhost:8000/ws/signaling`

### **Message Format**
All WebSocket messages use JSON format:
```json
{
  "type": "message_type",
  "data": { /* message-specific data */ },
  "client_id": "client_uuid",
  "timestamp": "2024-01-15T10:30:00Z"
}
```

### **WebRTC Signaling Messages**

#### **Client → Server: WebRTC Offer**
```json
{
  "type": "offer",
  "data": {
    "sdp": "v=0\r\no=- 123456789 2 IN IP4 127.0.0.1\r\n...",
    "type": "offer"
  }
}
```

#### **Server → Client: WebRTC Answer**
```json
{
  "type": "answer",
  "data": {
    "sdp": "v=0\r\no=- 987654321 2 IN IP4 127.0.0.1\r\n...",
    "type": "answer"
  }
}
```

#### **ICE Candidate Exchange**
```json
{
  "type": "ice_candidate",
  "data": {
    "candidate": "candidate:1 1 UDP 2130706431 192.168.1.100 54400 typ host",
    "sdpMid": "0",
    "sdpMLineIndex": 0
  }
}
```

### **Event Messages**

#### **User Input Events**
```json
{
  "type": "event",
  "data": {
    "event_type": "mouse_move",
    "x": 320,
    "y": 240,
    "buttons": 1,
    "timestamp": 1642248600000
  }
}
```

#### **Simulation Control Events**
```json
{
  "type": "command",
  "data": {
    "command": "pause_simulation",
    "params": {}
  }
}
```

### **Scene Update Broadcasts**

#### **Server → All Clients: Scene Updated**
```json
{
  "type": "scene_updated",
  "data": {
    "xml": "<mujoco>...</mujoco>",
    "model_info": {
      "name": "new_scene",
      "nbody": 5,
      "nq": 3
    }
  }
}
```

#### **Server → All Clients: Statistics Update**
```json
{
  "type": "stats_update",
  "data": {
    "clients_connected": 4,
    "simulation_time": 125.6,
    "fps": 59.8
  }
}
```

---

## 📱 **JavaScript Client API**

### **MuJoCoWebRTCClient Class**

#### **Constructor**
```javascript
const client = new MuJoCoWebRTCClient({
  serverUrl: 'ws://localhost:8000/ws/signaling',
  videoElement: document.getElementById('remote-video'),
  debug: false
});
```

#### **Connection Management**
```javascript
// Connect to server
await client.connect();

// Disconnect from server
await client.disconnect();

// Check connection status
const isConnected = client.isConnected();
```

#### **Event Handling**
```javascript
// Register event handlers
client.on('connected', () => {
  console.log('Connected to server');
});

client.on('disconnected', () => {
  console.log('Disconnected from server');
});

client.on('video_started', (stream) => {
  console.log('Video stream started');
});

client.on('scene_updated', (sceneData) => {
  console.log('Scene updated:', sceneData);
});

client.on('stats_update', (stats) => {
  console.log('Server stats:', stats);
});
```

#### **Scene Management**
```javascript
// Load scene from XML
await client.loadScene(xmlString);

// Create scene from prompt
const sceneData = await client.createScene({
  prompt: "Create a cart pole system",
  parameters: { cart_mass: 1.0, pole_length: 0.5 }
});

// Get current scene info
const currentScene = await client.getCurrentScene();
```

#### **Input Events**
```javascript
// Send mouse event
client.sendMouseEvent({
  type: 'mouse_move',
  x: 320,
  y: 240,
  buttons: 1
});

// Send keyboard event
client.sendKeyboardEvent({
  type: 'key_down',
  code: 'Space',
  ctrlKey: false,
  altKey: false
});

// Send camera preset command
client.sendCommand({
  command: 'camera_preset',
  preset: 'front'
});
```

#### **Recording Control**
```javascript
// Start recording
const recordingId = await client.startRecording({
  format: 'mp4',
  quality: 'high',
  duration: 60
});

// Stop recording
await client.stopRecording(recordingId);

// Download recording
const blob = await client.downloadRecording(recordingId);
```

### **Utility Functions**

#### **Scene Validation**
```javascript
// Validate MuJoCo XML
const isValid = await MuJoCoUtils.validateXML(xmlString);

// Get scene statistics
const stats = await MuJoCoUtils.getSceneStats(xmlString);
```

#### **Event Helpers**
```javascript
// Set up automatic mouse/keyboard capture
MuJoCoUtils.setupInputCapture(videoElement, client);

// Enable touch support for mobile
MuJoCoUtils.enableTouchControls(videoElement, client);
```

---

## 🐍 **Python Server API**

### **SignalingServer Class**

#### **Initialization**
```python
from py_remote_viewer.signaling import SignalingServer
from py_remote_viewer.config import ViewerConfig

config = ViewerConfig(
    frame_width=1280,
    frame_height=720,
    frame_rate=60,
    port=8000
)

server = SignalingServer(config)
```

#### **Client Management**
```python
# Get connected clients
clients = server.get_connected_clients()

# Send message to specific client
await server.send_to_client(client_id, message)

# Broadcast to all clients
await server.broadcast_message(message)

# Get client statistics
stats = server.get_client_stats(client_id)
```

#### **Scene Management**
```python
# Load scene XML
success = await server.load_scene(xml_string)

# Broadcast scene update
await server.broadcast_scene_update(xml_string)

# Get current scene
scene_info = server.get_current_scene()
```

### **MuJoCoSimulation Class**

#### **Physics Control**
```python
from py_remote_viewer.mujoco_simulation import MuJoCoSimulation

sim = MuJoCoSimulation()

# Load model from XML
success = sim.load_model(xml_string)

# Step simulation
sim.step(num_steps=10)

# Reset simulation
sim.reset()

# Get current state
state = sim.get_state()
```

#### **Rendering**
```python
# Render frame
frame = sim.render_frame(width=640, height=480)

# Set camera
sim.set_camera_position(position=[1, 1, 1], target=[0, 0, 0])

# Get camera info
camera_info = sim.get_camera_info()
```

### **WebRTC Video Track**

#### **Custom Video Track**
```python
from py_remote_viewer.webrtc_track import MuJoCoVideoTrack

track = MuJoCoVideoTrack(config, mujoco_simulation)

# Get next frame
frame = await track.recv()

# Update simulation reference
track.update_simulation(new_simulation)
```

---

## 📨 **Event Protocol**

### **Mouse Events**
```json
{
  "type": "mouse_move",
  "x": 320,
  "y": 240,
  "buttons": 1,
  "timestamp": 1642248600000
}

{
  "type": "mouse_down",
  "x": 100,
  "y": 150,
  "button": 0,
  "timestamp": 1642248600001
}

{
  "type": "mouse_up",
  "x": 100,
  "y": 150,
  "button": 0,
  "timestamp": 1642248600002
}

{
  "type": "scroll",
  "x": 200,
  "y": 200,
  "dx": 0,
  "dy": -1,
  "timestamp": 1642248600003
}
```

### **Keyboard Events**
```json
{
  "type": "key_down",
  "code": "Space",
  "key": " ",
  "ctrlKey": false,
  "altKey": false,
  "shiftKey": false,
  "timestamp": 1642248600004
}

{
  "type": "key_up",
  "code": "Space",
  "key": " ",
  "ctrlKey": false,
  "altKey": false,
  "shiftKey": false,
  "timestamp": 1642248600005
}
```

### **Command Events**
```json
{
  "type": "command",
  "cmd": "camera_preset",
  "params": {
    "preset": "front"
  },
  "timestamp": 1642248600006
}

{
  "type": "command",
  "cmd": "pause_simulation",
  "params": {},
  "timestamp": 1642248600007
}

{
  "type": "command",
  "cmd": "reset_simulation",
  "params": {},
  "timestamp": 1642248600008
}
```

---

## ⚙️ **Configuration API**

### **ViewerConfig Class**
```python
from py_remote_viewer.config import ViewerConfig

config = ViewerConfig(
    # Video settings
    frame_width=1280,
    frame_height=720,
    frame_rate=60,
    
    # Server settings
    host="0.0.0.0",
    port=8000,
    debug=False,
    
    # Performance settings
    max_clients=50,
    gpu_enabled=True,
    h264_enabled=True,
    
    # Feature flags
    scene_creation_enabled=True,
    rl_integration_enabled=True,
    recording_enabled=True
)
```

### **Environment Variables**
```bash
# Basic configuration
WEBRTC_HOST=0.0.0.0
WEBRTC_PORT=8000
WEBRTC_DEBUG=false

# Video settings
WEBRTC_FRAME_WIDTH=1280
WEBRTC_FRAME_HEIGHT=720
WEBRTC_FRAME_RATE=60

# Performance settings
WEBRTC_MAX_CLIENTS=50
WEBRTC_GPU_ENABLED=true
WEBRTC_H264_ENABLED=true

# Feature toggles
WEBRTC_SCENE_CREATION=true
WEBRTC_RL_INTEGRATION=true
WEBRTC_RECORDING=true
```

### **Runtime Configuration Updates**
```python
# Update configuration at runtime
server.update_config({
    "frame_rate": 30,
    "max_clients": 25
})

# Get current configuration
current_config = server.get_config()

# Reset to defaults
server.reset_config()
```

---

This comprehensive API reference provides complete documentation for all aspects of the WebRTC MuJoCo viewer, enabling developers to integrate, extend, and customize the system for their specific needs.