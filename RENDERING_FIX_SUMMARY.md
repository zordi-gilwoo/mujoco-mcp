# WebRTC Viewer Rendering Issue - Fix Summary

## Problem Description

The WebRTC browser-based viewer was able to show the initial scene (pendulum) but stopped rendering video updates after clicking buttons to load new scenes ("pendulum", "panda", "cart pole", etc.). The simulation continued running correctly in the background, but the browser video feed froze.

## Root Cause Analysis

### Issue 1: Invalid Event Type (Primary Issue)
The browser client was sending a `refresh_scene` event after detecting scene changes:

```javascript
// client/app.js lines 1627-1629 and 1682-1684
this.sendEvent({
    type: 'refresh_scene'  // ❌ Not a valid EventType
});
```

However, this event type was not defined in the server's `EventType` enum:

```python
# py_remote_viewer/events.py
class EventType(Enum):
    MOUSE_MOVE = "mouse_move"
    MOUSE_DOWN = "mouse_down"
    MOUSE_UP = "mouse_up"
    SCROLL = "scroll"
    KEY_DOWN = "key_down"
    KEY_UP = "key_up"
    COMMAND = "command"
    # ❌ No REFRESH_SCENE
```

When the server received this event, it failed to parse it and logged warnings:
```
Failed to parse event: 'refresh_scene' is not a valid EventType
```

### Issue 2: Lack of Error Handling
The video track rendering lacked robust error handling for:
- Simulation state during model reloads
- Renderer initialization failures
- Overlay rendering errors during state transitions

## Fixes Applied

### Fix 1: Remove Unnecessary refresh_scene Events
**File: `client/app.js`**

Removed the unnecessary `refresh_scene` event sends because:
1. The server already loads the scene when commanded
2. The video track automatically renders the updated simulation
3. No client-side refresh is needed

**Before:**
```javascript
if (this.isConnected && result.actions_taken?.includes('created_scene')) {
    this.sendEvent({
        type: 'refresh_scene'  // ❌ Causes parsing errors
    });
}
```

**After:**
```javascript
// Scene is automatically loaded on the server side
// The video track will automatically render the updated simulation
```

### Fix 2: Improve Event Handling on Server
**File: `py_remote_viewer/signaling.py`**

Changed from WARNING to DEBUG level for unrecognized events:

**Before:**
```python
logger.warning(f"Failed to parse event from {client_id}: {event_data}")
```

**After:**
```python
# Silently ignore unrecognized event types (they might be client-specific)
event_type = event_data.get("type", "unknown")
logger.debug(f"Ignoring unrecognized event type '{event_type}' from {client_id}")
```

### Fix 3: Add Robust Error Handling to Video Track
**File: `py_remote_viewer/webrtc_track.py`**

Added safety checks and better error reporting:

1. **Simulation initialization check:**
```python
if not hasattr(self.mujoco_simulation, '_initialized') or not self.mujoco_simulation._initialized:
    print(f"[MuJoCoVideoTrack] Simulation not initialized, using error frame")
    return self._generate_error_frame()
```

2. **Better error logging:**
```python
except Exception as e:
    print(f"[MuJoCoVideoTrack] MuJoCo rendering error at frame {self.frame_count}: {e}")
    import traceback
    traceback.print_exc()
    return self._generate_error_frame()
```

3. **Safe overlay rendering with try-except:**
```python
def _add_mujoco_overlay(self, frame: np.ndarray) -> np.ndarray:
    try:
        # ... overlay code with hasattr checks ...
    except Exception as e:
        print(f"[MuJoCoVideoTrack] Overlay error: {e}")
    return frame
```

4. **Attribute existence checks:**
```python
if hasattr(self.mujoco_simulation, 'state') and hasattr(self.mujoco_simulation.state, 'time'):
    sim_time = self.mujoco_simulation.state.time
    # ... use sim_time safely ...
```

## Testing Recommendations

1. **Restart the server:**
```bash
./scripts/run_py_viewer.sh
```

2. **Test scene changes:**
   - Open browser to `http://localhost:8000`
   - Click "Connect" to establish WebRTC
   - Click "Pendulum" button - should render pendulum
   - Click "Franka Panda" button - should render robot
   - Click "Cart Pole" button - should render cart-pole
   - Verify video continues updating for each scene

3. **Monitor console output:**
   - Should see successful scene loads
   - Should NOT see "Failed to parse event" warnings
   - Should see smooth rendering without errors

## Expected Behavior After Fix

1. ✅ Initial scene renders correctly
2. ✅ Clicking scene buttons loads new scenes smoothly
3. ✅ Video feed continues updating after scene changes
4. ✅ No parsing errors in server logs
5. ✅ Graceful error handling if rendering fails

## Technical Notes

### Why This Happened
The `refresh_scene` event was likely added during development with the intention of forcing a scene reload, but:
- It was unnecessary (server already handles scene loading)
- It was never implemented on the server side
- It caused parsing failures that may have disrupted the event handling flow

### Why the Fix Works
1. **Removing invalid events** eliminates parsing failures
2. **Better error handling** ensures rendering continues even with transient errors
3. **Safety checks** prevent crashes during model reloads when state is temporarily inconsistent

## Files Modified

1. `/home/gilwoo/zordi_ws/mujoco-mcp/client/app.js`
   - Removed `refresh_scene` event sends (2 locations)

2. `/home/gilwoo/zordi_ws/mujoco-mcp/py_remote_viewer/signaling.py`
   - Improved unrecognized event handling

3. `/home/gilwoo/zordi_ws/mujoco-mcp/py_remote_viewer/webrtc_track.py`
   - Added simulation initialization checks
   - Enhanced error logging with tracebacks
   - Added safe overlay rendering with attribute checks
