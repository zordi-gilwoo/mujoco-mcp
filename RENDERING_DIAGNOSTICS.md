# Rendering Diagnostics - Enhanced Logging

## Changes Made

I've added comprehensive diagnostic logging to help identify why scenes aren't rendering in the browser after model changes.

### New Diagnostic Logs

1. **Model Loading (`mujoco_simulation.py`)**:
   - Now logs `"Renderer ready: True/False"` after model loads
   - Shows when renderer needs reinitialization
   - Shows when renderer dimensions change
   - Full stack traces for rendering failures

2. **Video Track (`webrtc_track.py`)**:
   - Detects and logs when the MuJoCo model changes
   - Tracks model changes by object ID
   - Logs when rendering returns None

3. **Renderer Recovery (`mujoco_simulation.py`)**:
   - Automatically attempts to reinitialize renderer if it becomes None
   - Better error messages for renderer failures

## Testing Steps

1. **Restart the server:**
```bash
Ctrl+C  # Stop the current server
./scripts/run_py_viewer.sh
```

2. **Open browser to http://localhost:8000**

3. **Watch the terminal output while testing:**
   - Click "Connect" - should see initial pendulum
   - Click "Pendulum" button - watch for these logs:
     - `[MuJoCoSimulation] Model loaded successfully...`
     - `- Renderer ready: True`
     - `[MuJoCoVideoTrack] Model changed detected at frame XXXX`
   - Click "Franka Panda" button - watch for same logs
   - Click "Cart Pole" button - watch for same logs

## What to Look For

### If Rendering Works:
You should see in the terminal:
```
[MuJoCoSimulation] Model loaded successfully from xml[...]
  - Bodies: 3
  - Joints: 1
  - Degrees of freedom: 1
  - Actuators: 1
  - Renderer ready: True
[MuJoCoVideoTrack] Model changed detected at frame 234, continuing rendering
```

And the browser should show the new scene rendering.

### If Rendering Fails:
You might see:
```
[MuJoCoSimulation] render_frame called but renderer is None, reinitializing...
[MuJoCoSimulation] Failed to reinitialize renderer: <error message>
```
Or:
```
[MuJoCoVideoTrack] Render returned None at frame XXXX
```
Or:
```
[MuJoCoSimulation] Rendering failed: <error message>
<full traceback>
```

## Common Issues to Check

### Issue 1: Renderer Becomes None
If you see `"renderer is None"` messages, the renderer is failing to initialize or being destroyed during model reload.

**Solution**: The code now attempts automatic reinitialization. If this keeps failing, there may be an OpenGL/display issue.

### Issue 2: Render Returns None
If frames are consistently None, check:
- OpenGL support in the environment
- Display availability (EGL for headless)
- MuJoCo installation

### Issue 3: Model Changes Not Detected
If you don't see `"Model changed detected"` messages after loading new scenes, the model isn't actually being updated in the simulation object.

## Terminal Commands for Diagnosis

While the server is running, you can grep for specific issues:

```bash
# In another terminal, watch for rendering errors
tail -f <server_log> | grep -i "rendering\|renderer\|error"

# Watch for model changes
tail -f <server_log> | grep -i "model.*loaded\|model changed"

# Watch for scene updates
tail -f <server_log> | grep -i "scene.*updated\|scene.*loaded"
```

## Next Steps Based on Logs

Please restart the server and try loading different scenes, then share:
1. The terminal output showing the model load and any error messages
2. Whether the video in the browser updates or stays frozen
3. Any error messages in the browser console (F12 → Console tab)

This will help identify the exact point where rendering breaks down.
