# Multi-Client MuJoCo MCP Architecture

## Overview

This document describes the multi-client architecture implementation for MuJoCo MCP, enabling multiple AI agents or users to run independent MuJoCo simulations simultaneously without interfering with each other.

## Problem Statement

The original MuJoCo MCP server used a global `viewer_client` variable, which meant:
- All MCP clients shared the same simulation session
- When one client loaded a scene, all other clients would see the same simulation
- No isolation between different users or AI agents
- Resource conflicts and state pollution between clients

## Solution Architecture

### 1. Session Management (`session_manager.py`)

**SessionManager Class**: Manages multiple client sessions with isolation
- Creates unique session IDs for each MCP client connection
- Tracks session metadata (creation time, last activity, viewer port, active models)
- Allocates unique viewer ports to prevent conflicts
- Provides automatic cleanup of stale sessions

**SessionInfo Dataclass**: Stores per-session information
- `session_id`: Unique identifier for the session
- `client_id`: Human-readable client identifier  
- `viewer_port`: Dedicated port for this client's viewer server
- `active_models`: Dictionary tracking loaded models in this session
- Activity tracking for cleanup purposes

### 2. Updated MCP Server (`mcp_server_menagerie.py`)

**Key Changes**:
- Removed global `viewer_client` variable
- All tool handlers now use `session_manager.get_viewer_client()` 
- Model IDs are prefixed with session ID to prevent conflicts
- Added `get_session_info` tool for debugging and monitoring

**Session-Aware Operations**:
- `create_scene`: Creates models with session-specific IDs
- `step_simulation`: Routes commands to correct client's viewer
- `get_state`: Retrieves state from session-specific models
- `reset_simulation`: Resets only the current session's models
- `close_viewer`: Can close specific models or entire sessions

### 3. Enhanced Viewer Client Integration

**Leveraged Existing ViewerManager**: 
- Uses the existing `ViewerManager` class for multi-client support
- Each session gets its own viewer client instance
- Proper resource cleanup when sessions end

## Technical Implementation

### Session ID Generation
```python
def _extract_session_id(self, context: Any = None) -> str:
    # Uses thread ID as session identifier
    # In production MCP, would use connection context
    thread_id = threading.current_thread().ident
    return f"session_{thread_id}"
```

### Model ID Namespacing
```python
# Session-specific model IDs prevent conflicts
session_model_id = f"{session.session_id}_{scene_name}"

# Example: "session_12345_pendulum" vs "session_67890_pendulum"
```

### Port Allocation
```python
def _allocate_port(self) -> int:
    self._port_counter += 1
    return self._base_port + self._port_counter

# Results in: 8889, 8890, 8891, etc.
```

## Benefits

### 🔒 Complete Session Isolation
- Each client operates in its own simulation environment
- No cross-contamination between different users or AI agents
- Independent model loading, simulation state, and physics parameters

### 🔌 Resource Management
- Unique viewer ports prevent conflicts
- Automatic cleanup of inactive sessions
- Memory-efficient resource tracking

### 🏷️ Model Namespacing
- Session-prefixed model IDs prevent naming conflicts
- Multiple clients can load models with the same name
- Clear separation of models between sessions

### 📊 Monitoring & Debugging
- New `get_session_info` tool provides visibility into sessions
- Track active models, connection status, and resource usage
- Easy identification of which client owns which resources

### 🧹 Automatic Cleanup
- Sessions automatically clean up when clients disconnect
- Stale session detection and removal
- Proper resource deallocation

## Usage Examples

### Multiple AI Agents
```python
# Agent 1 creates a pendulum simulation
await handle_call_tool("create_scene", {"scene_type": "pendulum"})

# Agent 2 creates a cartpole simulation (simultaneously)  
await handle_call_tool("create_scene", {"scene_type": "cart_pole"})

# Both run independently without interference
```

### Session Information
```python
# Get current session details
result = await handle_call_tool("get_session_info", {})
# Returns session ID, client ID, active models, etc.
```

### Proper Cleanup
```python
# Close specific model
await handle_call_tool("close_viewer", {"model_id": "pendulum"})

# Close entire session
await handle_call_tool("close_viewer", {})
```

## Testing & Validation

### Multi-Client Tests (`tests/test_multi_client.py`)
- Session isolation verification
- Concurrent client testing
- Model ID namespacing validation
- Resource cleanup testing

### Demo Script (`demo_multi_client.py`)
- Sequential client demonstration
- Concurrent multi-threading test
- Session monitoring examples

## Backward Compatibility

The implementation maintains full backward compatibility:
- Existing single-client usage continues to work
- All original tools maintain the same interface
- No breaking changes to the MCP protocol
- Previous demos and examples still function correctly

## Future Enhancements

### Real MCP Connection Context
Currently uses thread IDs for session identification. Production deployment should:
- Extract session IDs from MCP connection context
- Use proper client authentication/identification
- Implement connection lifecycle management

### Enhanced Resource Management
- Configurable session timeouts
- Memory usage monitoring per session
- Automatic resource limits per client

### Advanced Features
- Session sharing/collaboration between clients
- Cross-session model sharing with permissions
- Real-time session monitoring dashboard

## Architecture Benefits Summary

✅ **Session Isolation**: Each client has independent simulation environment
✅ **Concurrent Support**: Multiple clients can operate simultaneously  
✅ **Resource Safety**: No conflicts between client resources
✅ **Scalability**: Architecture supports many concurrent clients
✅ **Maintainability**: Clean separation of concerns
✅ **Backward Compatible**: Existing code continues to work
✅ **Observable**: Full visibility into multi-client operations

This architecture enables MuJoCo MCP to support enterprise and multi-user scenarios where multiple AI agents or human users need to run independent physics simulations simultaneously.