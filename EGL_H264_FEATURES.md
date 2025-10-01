# EGL Headless Rendering and H.264 Encoding Features

This document describes the new EGL headless rendering and H.264 video encoding capabilities added to MuJoCo MCP.

## Overview

### EGL Headless Rendering
- **Purpose**: Enable GPU-accelerated rendering in server environments without display
- **Benefits**: Significantly faster rendering performance compared to software fallback
- **Fallback**: Automatically falls back to software rendering when EGL is unavailable

### H.264 Video Encoding
- **Purpose**: Optimize video streaming performance with hardware acceleration
- **Benefits**: Efficient video compression for real-time streaming applications
- **Formats**: Support for both batch encoding and streaming chunks

## New MCP Tools

### 1. `enable_egl_rendering`
Enable EGL-based GPU rendering for a simulation.

**Parameters:**
- `model_id` (string, required): ID of the simulation model
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

### 2. `render_egl_frame`
Render a frame using EGL GPU acceleration.

**Parameters:**
- `model_id` (string, required): ID of the simulation model
- `width` (integer, optional): Render width (default: 640)
- `height` (integer, optional): Render height (default: 480)
- `camera_id` (integer, optional): Camera ID (default: -1)

**Returns:** Base64-encoded PNG image

### 3. `setup_h264_encoder`
Setup H.264 encoder for video recording/streaming.

**Parameters:**
- `model_id` (string, required): ID of the simulation model
- `width` (integer, optional): Video width (default: 640)
- `height` (integer, optional): Video height (default: 480)
- `fps` (integer, optional): Frames per second (default: 30)
- `bitrate` (string, optional): Bitrate (e.g. '2M', default: '2M')
- `streaming` (boolean, optional): Enable streaming mode (default: false)

### 4. `record_h264_video`
Record H.264 video from simulation frames.

**Parameters:**
- `model_id` (string, required): ID of the simulation model
- `duration` (number, optional): Recording duration in seconds (default: 5.0)
- `camera_id` (integer, optional): Camera ID (default: -1)
- `return_bytes` (boolean, optional): Return video as base64 bytes (default: true)

**Returns:** Base64-encoded MP4 video or confirmation message

### 5. `check_gpu_support`
Check EGL and H.264 hardware support capabilities.

**Returns:** Detailed support information including:
- EGL availability and GPU info
- H.264 encoder capabilities (hardware/software)
- Performance recommendations

## Technical Implementation

### EGL Renderer (`egl_renderer.py`)

#### Key Classes:
- **`EGLContext`**: Manages EGL context lifecycle
- **`EGLRenderer`**: High-level EGL rendering interface

#### Features:
- Automatic hardware detection
- Graceful fallback to software rendering
- Context management with proper cleanup
- Support for various GPU vendors (NVIDIA, Intel, AMD, etc.)

### H.264 Encoder (`h264_encoder.py`)

#### Key Classes:
- **`H264Encoder`**: Batch video encoding
- **`StreamingEncoder`**: Real-time streaming encoder

#### Features:
- Hardware encoder detection (NVENC, QuickSync, VideoToolbox, VAAPI, AMF)
- Automatic fallback to software encoding
- Configurable quality/performance settings
- Real-time streaming with chunk-based output

## Hardware Support

### EGL Requirements
- OpenGL-capable GPU
- EGL library (usually included with GPU drivers)
- PyOpenGL Python package

### H.264 Requirements
- FFmpeg binary (for encoding)
- Hardware encoders (optional, for best performance):
  - NVIDIA: NVENC (GeForce GTX 600+ / Quadro K600+)
  - Intel: QuickSync (3rd gen Core processors+)
  - AMD: AMF (GCN architecture+)
  - Apple: VideoToolbox (macOS only)

## Performance Benefits

### EGL Rendering
- **GPU vs CPU**: 5-10x faster rendering for complex scenes
- **Memory**: Reduced CPU memory usage
- **Parallel Processing**: GPU parallel processing capabilities

### H.264 Encoding
- **Hardware vs Software**: 10-50x faster encoding
- **Quality**: Better quality at lower bitrates
- **Real-time**: Enables real-time streaming applications

## Usage Examples

### Basic EGL Rendering
```python
# Create simulation
await handle_call_tool('create_scene', {'scene_type': 'pendulum'})

# Enable EGL rendering
await handle_call_tool('enable_egl_rendering', {'model_id': 'pendulum'})

# Render frame
result = await handle_call_tool('render_egl_frame', {'model_id': 'pendulum'})
# Returns base64-encoded PNG image
```

### Video Recording
```python
# Setup H.264 encoder
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
# Returns base64-encoded MP4 video
```

### Streaming Mode
```python
# Setup streaming encoder
await handle_call_tool('setup_h264_encoder', {
    'model_id': 'pendulum',
    'streaming': True,
    'bitrate': '1M'
})

# The encoder will return chunks as they're ready for streaming
```

## Troubleshooting

### EGL Issues
- **"EGL not available"**: Install GPU drivers and PyOpenGL
- **"No EGL display"**: Check if running in truly headless environment
- **Context creation fails**: May need to set `DISPLAY=:99` with Xvfb

### H.264 Issues
- **"FFmpeg not found"**: Install FFmpeg binary (`apt-get install ffmpeg`)
- **Encoding fails**: Check FFmpeg supports your desired codec/settings
- **Hardware encoding unavailable**: Verify GPU supports hardware encoding

### Performance Tips
- Use EGL rendering for better performance when available
- Choose appropriate bitrates for your use case (1M for streaming, 5M+ for recording)
- Use hardware encoders when available (check `check_gpu_support` output)
- Lower resolution/FPS for real-time applications if needed

## Dependencies

### Required
- `mujoco>=2.3.0`
- `numpy>=1.22.0`

### Optional (for full functionality)
- `pillow` - Image processing and PNG encoding
- `ffmpeg-python` - Python FFmpeg wrapper
- `PyOpenGL` - EGL rendering support

### System Dependencies
- `ffmpeg` - Video encoding binary
- GPU drivers with EGL support
- Hardware video encoders (GPU-specific)