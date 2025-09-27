# MuJoCo MCP Architecture Evolution

## Overview

This document outlines the evolution of multi-client support in MuJoCo MCP, from the original session-based approach to the enhanced process-based isolation architecture.

## Stage 1: Original Multi-Client Architecture (Pre-Enhancement)

### Key Components
- **SessionManager** (`src/mujoco_mcp/session_manager.py`)
- **ViewerManager** (`src/mujoco_mcp/viewer_client.py`)
- **Demo & Tests** (`demo_multi_client.py`, `tests/test_multi_client.py`)

### Capabilities
✅ **Session Isolation**: Each client gets unique session ID and port  
✅ **Client Management**: ViewerManager handles multiple client connections  
✅ **Port Allocation**: Sequential port assignment (8889, 8890, etc.)  
✅ **Resource Cleanup**: Session cleanup and stale session management  
✅ **Model Namespacing**: Session-prefixed model IDs prevent conflicts  

### Architecture
```
Client A → SessionManager → ViewerClient (port 8889) → Single Viewer Server Process
Client B → SessionManager → ViewerClient (port 8890) → Single Viewer Server Process  
Client C → SessionManager → ViewerClient (port 8891) → Single Viewer Server Process
```

**Limitation**: All clients share the same viewer server process, just with different ports and session isolation.

## Stage 2: Enhanced Process Pool Architecture (Current)

### New Components Added
- **ProcessManager** (`src/mujoco_mcp/process_manager.py`)
- **MockViewerServer** (`src/mujoco_mcp/mock_viewer_server.py`)
- **Enhanced Tests** (`test_process_manager.py`, `test_simple_integration.py`)

### New Capabilities
✅ **True Process Isolation**: Each client gets dedicated Python process  
✅ **Advanced Port Management**: Dynamic availability checking (8001-9000 range)  
✅ **Process Health Monitoring**: Background threads monitor process health  
✅ **Automatic Process Cleanup**: Graceful termination and resource recovery  
✅ **MCP Integration**: New tools for process pool management  

### Enhanced Architecture
```
Client A → SessionManager → ProcessManager → Dedicated Viewer Process (PID 1234, port 8001)
Client B → SessionManager → ProcessManager → Dedicated Viewer Process (PID 1235, port 8002)
Client C → SessionManager → ProcessManager → Dedicated Viewer Process (PID 1236, port 8003)
```

**Benefit**: Complete process isolation - memory, CPU, crash isolation between clients.

## What Changed vs What's New

### Modified Files

#### `src/mujoco_mcp/session_manager.py`
**Original Features Preserved:**
- Session creation and management
- Port allocation logic
- Session cleanup
- Statistics gathering

**Enhancements Added:**
- `use_isolated_process` flag in SessionInfo
- Integration with ProcessManager
- Process information in session statistics
- Process cleanup in session cleanup

#### `src/mujoco_mcp/viewer_server.py`
**Original Features Preserved:**
- Viewer server class proxy
- Script path resolution

**Enhancements Added:**
- Command-line argument parsing for isolated mode
- Support for --session-id and --isolated flags

#### `src/mujoco_mcp/mcp_server_menagerie.py`
**Original Features Preserved:**
- All existing MCP tools
- Session management integration

**Enhancements Added:**
- ProcessManager import
- New MCP tools: `get_process_pool_stats`, `list_active_processes`, `terminate_process`

### New Files Added

#### `src/mujoco_mcp/process_manager.py`
**Completely New**: Process spawning, health monitoring, port management

#### `src/mujoco_mcp/mock_viewer_server.py`
**Completely New**: Testing infrastructure without MuJoCo dependencies

#### Test Files
- `test_process_manager.py` - New process manager tests
- `test_simple_integration.py` - Integration tests
- `demo_multi_client_enhanced.py` - Enhanced demo

#### Documentation
- `PROCESS_POOL_ARCHITECTURE.md` - Technical architecture guide
- `ARCHITECTURE_EVOLUTION.md` - This evolution document

## Backward Compatibility

The enhancement maintains **100% backward compatibility**:

1. **Existing Code**: All existing MCP tools and APIs work unchanged
2. **Configuration**: Can disable process isolation with `use_isolated_processes=False`
3. **Tests**: Original multi-client tests continue to pass
4. **Demos**: Original demo continues to work

## Performance Impact

### Memory Usage
- **Original**: ~50MB shared across all clients
- **Enhanced**: ~50MB per client process (isolated)

### Startup Time  
- **Original**: Instant client connections
- **Enhanced**: ~2 seconds per process spawn

### Isolation Benefits
- **Crash Isolation**: One client crash doesn't affect others
- **Memory Isolation**: No memory leaks between clients  
- **Resource Isolation**: CPU and file handle isolation
- **Security**: Process-level security boundaries

## Configuration Options

### Session-Based Isolation (Original)
```python
session_manager = SessionManager(use_isolated_processes=False)
```

### Process-Based Isolation (Enhanced) 
```python
session_manager = SessionManager(use_isolated_processes=True)  # Default
```

### Process Manager Configuration
```python
ProcessManager(
    port_range_start=8001,
    port_range_end=9000,
    use_mock_server=False  # For testing
)
```

## Use Cases

### When to Use Session-Based (Original)
- Development and testing
- Resource-constrained environments
- When process isolation isn't critical
- Single-user scenarios

### When to Use Process-Based (Enhanced)
- Production multi-tenant deployments
- Enterprise environments with multiple AI agents
- When crash isolation is critical
- High-security requirements
- Scalable cloud deployments

## Summary

The enhancement **builds upon** rather than **replaces** the original multi-client architecture. It adds an optional layer of true process isolation while preserving all existing functionality. The result is a more robust, scalable, and secure multi-client platform suitable for enterprise deployments.

**Key Principle**: Existing session-based isolation for compatibility, enhanced process-based isolation for production scalability.

### Latest Improvements (v0.8.2+)
- **Unified Demonstration**: Single demo showcasing both architectural approaches
- **Enhanced Documentation**: Clear mode distinctions and usage recommendations  
- **Streamlined Integration**: Cleaner code organization and reduced redundancy
- **Production Ready**: Both approaches fully tested and documented for different deployment scenarios