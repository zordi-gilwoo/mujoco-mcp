"""Python-based headless WebRTC MuJoCo viewer scaffold."""

__version__ = "0.1.0"
__author__ = "MuJoCo MCP Team"

# Core components
from .camera_state import CameraState
from .events import EventProtocol, EventType
from .simulation_stub import SimulationStub
from .mujoco_simulation import MuJoCoSimulation

__all__ = ["CameraState", "EventProtocol", "EventType", "SimulationStub", "MuJoCoSimulation"]