"""Simulation management stub for remote viewer scaffold."""

import time
import asyncio
from typing import Dict, Any, Optional
from dataclasses import dataclass, field

from .camera_state import CameraState
from .events import EventData


@dataclass
class SimulationState:
    """Current state of the simulation."""
    
    time: float = 0.0
    step_count: int = 0
    is_running: bool = False
    is_paused: bool = False
    
    # Placeholder simulation data
    objects: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert simulation state to dictionary."""
        return {
            "time": self.time,
            "step_count": self.step_count,
            "is_running": self.is_running,
            "is_paused": self.is_paused,
            "objects": self.objects,
        }


class SimulationStub:
    """Placeholder simulation management for the remote viewer scaffold.
    
    This provides a simulation interface that will later be replaced with
    real MuJoCo integration.
    """
    
    def __init__(self):
        self.state = SimulationState()
        self.camera = CameraState()
        self._step_size = 0.01  # 10ms per step
        self._last_step_time = 0.0
        
        # Initialize some placeholder objects
        self._initialize_placeholder_scene()
    
    def _initialize_placeholder_scene(self):
        """Initialize a placeholder scene with some objects."""
        self.state.objects = {
            "ground": {
                "type": "plane",
                "position": [0.0, 0.0, 0.0],
                "color": [0.5, 0.5, 0.5],
            },
            "box": {
                "type": "box",
                "position": [0.0, 0.0, 1.0],
                "size": [0.5, 0.5, 0.5],
                "color": [1.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
            },
            "sphere": {
                "type": "sphere", 
                "position": [1.0, 1.0, 0.5],
                "radius": 0.3,
                "color": [0.0, 1.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
            },
        }
    
    def load_model(self, model_source: str) -> bool:
        """Load a simulation model (placeholder implementation).
        
        Args:
            model_source: Model file path or XML string
            
        Returns:
            bool: True if model loaded successfully
        """
        print(f"[SimulationStub] Loading model: {model_source[:100]}...")
        
        # Simulate loading time
        time.sleep(0.1)
        
        # Reset state
        self.state = SimulationState()
        self.camera = CameraState()
        self._initialize_placeholder_scene()
        
        print("[SimulationStub] Model loaded successfully (placeholder)")
        return True
    
    def start(self):
        """Start the simulation."""
        if not self.state.is_running:
            self.state.is_running = True
            self.state.is_paused = False
            self._last_step_time = time.time()
            print("[SimulationStub] Simulation started")
    
    def pause(self):
        """Pause the simulation."""
        if self.state.is_running:
            self.state.is_paused = True
            print("[SimulationStub] Simulation paused")
    
    def resume(self):
        """Resume the simulation."""
        if self.state.is_running and self.state.is_paused:
            self.state.is_paused = False
            self._last_step_time = time.time()
            print("[SimulationStub] Simulation resumed")
    
    def stop(self):
        """Stop the simulation."""
        self.state.is_running = False
        self.state.is_paused = False
        print("[SimulationStub] Simulation stopped")
    
    def reset(self):
        """Reset the simulation to initial state."""
        self.state.time = 0.0
        self.state.step_count = 0
        self._initialize_placeholder_scene()
        print("[SimulationStub] Simulation reset")
    
    def step(self, num_steps: int = 1):
        """Step the simulation forward.
        
        Args:
            num_steps: Number of simulation steps to execute
        """
        if not self.state.is_running or self.state.is_paused:
            return
        
        current_time = time.time()
        dt = current_time - self._last_step_time
        
        for _ in range(num_steps):
            self._update_physics(self._step_size)
            self.state.step_count += 1
            self.state.time += self._step_size
        
        self._last_step_time = current_time
    
    def _update_physics(self, dt: float):
        """Update placeholder physics simulation.
        
        Args:
            dt: Time step size
        """
        # Simple physics simulation for demonstration
        
        # Animate the box with a simple oscillation
        if "box" in self.state.objects:
            box = self.state.objects["box"]
            t = self.state.time
            box["position"][2] = 1.0 + 0.5 * abs(t % 2.0 - 1.0)  # Bounce between 0.5 and 1.5
        
        # Rotate the sphere around the origin
        if "sphere" in self.state.objects:
            sphere = self.state.objects["sphere"]
            t = self.state.time
            radius = 1.5
            sphere["position"][0] = radius * np.cos(t * 0.5)
            sphere["position"][1] = radius * np.sin(t * 0.5)
    
    def handle_event(self, event: EventData) -> bool:
        """Handle input event.
        
        Args:
            event: Input event data
            
        Returns:
            bool: True if event was handled successfully
        """
        print(f"[SimulationStub] Received event: {event}")
        
        # Let camera handle the event first
        camera_modified = self.camera.handle_event(event)
        
        # Handle simulation-specific commands
        if hasattr(event, 'type') and hasattr(event, 'cmd'):
            cmd = getattr(event, 'cmd', None)
            if cmd == "start":
                self.start()
                return True
            elif cmd == "pause":
                self.pause()
                return True
            elif cmd == "resume":
                self.resume()
                return True
            elif cmd == "stop":
                self.stop()
                return True
            elif cmd == "reset":
                self.reset()
                return True
        
        return camera_modified
    
    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state.
        
        Returns:
            Dict containing simulation and camera state
        """
        return {
            "simulation": self.state.to_dict(),
            "camera": self.camera.to_dict(),
        }
    
    async def run_async(self):
        """Run simulation loop asynchronously."""
        print("[SimulationStub] Starting async simulation loop")
        
        while self.state.is_running:
            if not self.state.is_paused:
                self.step()
            
            # Run at approximately 60 FPS
            await asyncio.sleep(1.0 / 60.0)
        
        print("[SimulationStub] Async simulation loop stopped")


# Add numpy import that was missing
import numpy as np