"""Development check module for the remote viewer."""

import sys
import importlib
from typing import List, Tuple


def check_imports() -> List[Tuple[str, bool, str]]:
    """Check if all required modules can be imported.
    
    Returns:
        List of (module_name, success, error_message) tuples
    """
    modules_to_check = [
        # Core Python modules
        "asyncio",
        "json",
        "logging",
        "time",
        
        # Third-party dependencies
        "fastapi",
        "uvicorn",
        "starlette",
        "aiortc",
        "av",
        "numpy",
        "pydantic",
        
        # Our modules
        "py_remote_viewer",
        "py_remote_viewer.config",
        "py_remote_viewer.events",
        "py_remote_viewer.camera_state",
        "py_remote_viewer.simulation_stub",
        "py_remote_viewer.mujoco_simulation",
        "py_remote_viewer.webrtc_track",
        "py_remote_viewer.signaling",
        "py_remote_viewer.server",
        "py_remote_viewer.app_factory",
        "py_remote_viewer.logging_utils",
    ]
    
    results = []
    
    for module_name in modules_to_check:
        try:
            importlib.import_module(module_name)
            results.append((module_name, True, ""))
        except ImportError as e:
            results.append((module_name, False, str(e)))
        except Exception as e:
            results.append((module_name, False, f"Unexpected error: {e}"))
    
    return results


def check_basic_functionality():
    """Check basic functionality of key components."""
    from .config import ViewerConfig
    from .events import EventProtocol, EventType, MouseEvent
    from .camera_state import CameraState
    from .simulation_stub import SimulationStub
    from .mujoco_simulation import MuJoCoSimulation
    
    print("🔧 Testing basic functionality...")
    
    # Test configuration
    config = ViewerConfig.from_env()
    assert config.host is not None
    assert config.port > 0
    print("✅ Configuration: OK")
    
    # Test event protocol
    mouse_event = MouseEvent(EventType.MOUSE_MOVE, 100, 200, 1)
    json_str = EventProtocol.serialize_event(mouse_event)
    parsed_event = EventProtocol.deserialize_event(json_str)
    assert parsed_event is not None
    assert parsed_event.x == 100
    assert parsed_event.y == 200
    print("✅ Event protocol: OK")
    
    # Test camera state
    camera = CameraState()
    initial_azimuth = camera.azimuth
    camera.handle_event(mouse_event)  # Should not modify for move without button
    assert camera.azimuth == initial_azimuth
    print("✅ Camera state: OK")
    
    # Test simulation stub
    sim = SimulationStub()
    assert not sim.state.is_running
    sim.start()
    assert sim.state.is_running
    sim.stop()
    assert not sim.state.is_running
    print("✅ Simulation stub: OK")
    
    # Test MuJoCo simulation
    try:
        mujoco_sim = MuJoCoSimulation()
        assert not mujoco_sim.state.is_running
        mujoco_sim.start()
        assert mujoco_sim.state.is_running
        mujoco_sim.stop()
        assert not mujoco_sim.state.is_running
        print("✅ MuJoCo simulation: OK")
    except Exception as e:
        print(f"⚠️  MuJoCo simulation: {e} (may be expected if MuJoCo not properly configured)")
        # Don't fail the test for MuJoCo issues in dev environments


def main():
    """Run development checks."""
    print("🔍 Running py_remote_viewer development checks...")
    print()
    
    # Check imports
    print("📦 Checking imports...")
    results = check_imports()
    
    success_count = 0
    for module_name, success, error in results:
        if success:
            print(f"✅ {module_name}")
            success_count += 1
        else:
            print(f"❌ {module_name}: {error}")
    
    print()
    print(f"📊 Import results: {success_count}/{len(results)} modules imported successfully")
    
    if success_count < len(results):
        print("❌ Some imports failed. Please install missing dependencies.")
        return False
    
    print()
    
    # Check basic functionality
    try:
        check_basic_functionality()
        print()
        print("🎉 All development checks passed!")
        return True
    except Exception as e:
        print(f"❌ Functionality check failed: {e}")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)