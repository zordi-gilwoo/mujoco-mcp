# EGL Headless Rendering and H.264 Encoding

## Overview

MuJoCo MCP includes EGL headless rendering and H.264 video encoding for server deployments.

### EGL Headless Rendering
- GPU-accelerated rendering without display
- Works in headless server environments
- Automatic fallback to software rendering

### H.264 Video Encoding
- Hardware-accelerated video compression
- Multiple encoder support (NVENC, QuickSync, VideoToolbox, VAAPI, AMF)
- Software fallback with libx264

---

## MCP Tools

### `enable_egl_rendering`

Enable EGL-based GPU rendering.

**Parameters:**
- `model_id` (string, required): Simulation model ID
- `width` (integer, optional): Render width (default: 640)
- `height` (integer, optional): Render height (default: 480)

**Example:**
```json
{
  "name": "enable_egl_rendering",
  "arguments": {
    "model_id": "pendulum",
    "width": 1280,
    "height": 720
  }
}
```

### `render_egl_frame`

Render frame using EGL GPU acceleration.

**Parameters:**
- `model_id` (string, required): Model ID
- `width` (integer, optional): Width (default: 640)
- `height` (integer, optional): Height (default: 480)
- `camera_id` (integer, optional): Camera ID (default: -1)

**Returns:** Base64-encoded PNG image

### `setup_h264_encoder`

Setup H.264 encoder for video recording.

**Parameters:**
- `model_id` (string, required): Model ID
- `width` (integer, optional): Width (default: 640)
- `height` (integer, optional): Height (default: 480)
- `fps` (integer, optional): Frames per second (default: 30)
- `bitrate` (string, optional): Bitrate (default: '2M')
- `streaming` (boolean, optional): Enable streaming mode (default: false)

### `record_h264_video`

Record H.264 video from simulation.

**Parameters:**
- `model_id` (string, required): Model ID
- `duration` (number, optional): Duration in seconds (default: 5.0)
- `camera_id` (integer, optional): Camera ID (default: -1)
- `return_bytes` (boolean, optional): Return video bytes (default: true)

**Returns:** Base64-encoded MP4 video

### `check_gpu_support`

Check EGL and H.264 capabilities.

**Returns:** Support information including:
- EGL availability and GPU info
- H.264 encoder capabilities
- Hardware/software encoding status

---

## Implementation

### EGL Renderer

```python
from mujoco_mcp.egl_renderer import EGLRenderer, check_egl_support

# Check support
support_info = check_egl_support()
print(f"EGL Available: {support_info['egl_available']}")

# Use EGL renderer
with EGLRenderer(model, width=640, height=480) as renderer:
    frame = renderer.render(data, camera_id=-1)
```

### H.264 Encoder

```python
from mujoco_mcp.h264_encoder import H264Encoder, check_h264_support

# Check support
support_info = check_h264_support()
print(f"Hardware Encoders: {support_info['hardware_encoders']}")

# Use encoder
encoder = H264Encoder(width=640, height=480, fps=30, bitrate='2M')
encoder.add_frame(frame)
video_bytes = encoder.encode_to_bytes()
```

---

## Hardware Support

### EGL Requirements
- OpenGL-capable GPU
- EGL library (included with GPU drivers)
- PyOpenGL Python package

### H.264 Requirements
- **FFmpeg binary** (required)
- **Hardware encoders** (optional, for best performance):
  - NVIDIA: NVENC (GeForce GTX 600+)
  - Intel: QuickSync (3rd gen Core+)
  - AMD: AMF (GCN architecture+)
  - Apple: VideoToolbox (macOS)

---

## Performance

### EGL Rendering
- **GPU vs CPU**: 5-10x faster for complex scenes
- **Memory**: Reduced CPU memory usage
- **Parallel Processing**: GPU capabilities

### H.264 Encoding
- **Hardware vs Software**: 10-50x faster encoding
- **Quality**: Better quality at lower bitrates
- **Real-time**: Enables streaming applications

---

## Usage Examples

### Basic EGL Rendering

```python
# Create simulation
await handle_call_tool('create_scene', {'scene_type': 'pendulum'})

# Enable EGL
await handle_call_tool('enable_egl_rendering', {'model_id': 'pendulum'})

# Render frame
result = await handle_call_tool('render_egl_frame', {'model_id': 'pendulum'})
```

### Video Recording

```python
# Setup encoder
await handle_call_tool('setup_h264_encoder', {
    'model_id': 'pendulum',
    'width': 1280,
    'height': 720,
    'fps': 60,
    'bitrate': '5M'
})

# Record video
result = await handle_call_tool('record_h264_video', {
    'model_id': 'pendulum',
    'duration': 10.0
})
# Returns base64-encoded MP4
```

---

## Troubleshooting

### EGL Issues
- **"EGL not available"**: Install GPU drivers and PyOpenGL
- **"No EGL display"**: Verify headless environment setup
- **Context creation fails**: May need Xvfb

### H.264 Issues
- **"FFmpeg not found"**: Install FFmpeg (`apt-get install ffmpeg`)
- **Encoding fails**: Check FFmpeg codec support
- **Hardware unavailable**: Verify GPU supports hardware encoding

---

## Dependencies

### Required
- `mujoco>=2.3.0`
- `numpy>=1.22.0`

### Optional
- `pillow` - Image processing
- `ffmpeg-python` - Python FFmpeg wrapper
- `PyOpenGL` - EGL rendering support

### System
- `ffmpeg` - Video encoding binary
- GPU drivers with EGL support
- Hardware video encoders (GPU-specific)

---

**Status**: ✅ Implemented  
**Last Updated**: October 2025