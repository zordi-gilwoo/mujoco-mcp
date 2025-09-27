"""Real MuJoCo simulation integration for remote viewer."""

import time
import asyncio
import numpy as np
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
import mujoco

from .camera_state import CameraState
from .events import EventData
from .simulation_stub import SimulationState  # Reuse the state structure


class MuJoCoSimulation:
    """Real MuJoCo simulation management for the remote viewer.
    
    This replaces SimulationStub with actual MuJoCo physics simulation.
    """
    
    def __init__(self, model_xml: str = None):
        self.state = SimulationState()
        self.camera = CameraState()
        self._step_size = 0.01  # 10ms per step
        self._last_step_time = 0.0
        
        # MuJoCo components
        self.model = None
        self.data = None
        self.renderer = None
        self._initialized = False
        
        # Default to a simple pendulum if no model provided
        if model_xml is None:
            model_xml = self._get_default_model()
        
        self.load_model(model_xml)
    
    def _get_default_model(self) -> str:
        """Get a default MuJoCo model for demonstration."""
        return """
        <mujoco model="pendulum">
            <compiler angle="radian"/>
            
            <asset>
                <material name="MatPlane" reflectance="0.5" texture="TexPlane" texrepeat="1 1" texuniform="true"/>
                <texture name="TexPlane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 .2 .3" width="512" height="512"/>
            </asset>
            
            <default>
                <joint damping="0.05"/>
                <geom contype="0" friction="1 0.1 0.1" rgba="0.7 0.7 0 1"/>
            </default>
            
            <worldbody>
                <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
                <camera name="track" mode="trackcom" pos="0 -2 1" xyaxes="1 0 0 0 1 2"/>
                
                <geom name="floor" pos="0 0 -0.5" size="2 2 0.1" type="plane" material="MatPlane"/>
                
                <body name="pole" pos="0 0 1">
                    <joint name="hinge" type="hinge" axis="1 0 0" pos="0 0 0"/>
                    <geom name="pole" type="capsule" size="0.045" fromto="0 0 0 0 0 -1" rgba="0 0.7 0.7 1"/>
                    
                    <body name="mass" pos="0 0 -1">
                        <geom name="mass" type="sphere" size="0.15" rgba="1 0 0 1"/>
                    </body>
                </body>
            </worldbody>
            
            <actuator>
                <motor joint="hinge" gear="1" ctrllimited="true" ctrlrange="-3 3"/>
            </actuator>
        </mujoco>
        """
    
    def load_model(self, model_xml: str) -> bool:
        """Load a MuJoCo model from XML string.
        
        Args:
            model_xml: MuJoCo model XML string
            
        Returns:
            bool: True if model loaded successfully
        """
        try:
            print(f"[MuJoCoSimulation] Loading MuJoCo model...")
            
            # Load model and create data
            self.model = mujoco.MjModel.from_xml_string(model_xml)
            self.data = mujoco.MjData(self.model)
            
            # Initialize renderer for frame capture
            self._initialize_renderer()
            
            # Reset state
            self.state = SimulationState()
            self.camera = CameraState()
            self._update_simulation_objects()
            
            self._initialized = True
            print(f"[MuJoCoSimulation] Model loaded successfully")
            print(f"  - Bodies: {self.model.nbody}")
            print(f"  - Joints: {self.model.njnt}")
            print(f"  - Degrees of freedom: {self.model.nq}")
            print(f"  - Actuators: {self.model.nu}")
            
            return True
            
        except Exception as e:
            print(f"[MuJoCoSimulation] Failed to load model: {e}")
            return False
    
    def _initialize_renderer(self):
        """Initialize MuJoCo renderer for frame capture."""
        try:
            # Create renderer with default resolution
            # In headless environments, this may fail but simulation will still work
            self.renderer = mujoco.Renderer(self.model, width=640, height=480)
            print("[MuJoCoSimulation] Renderer initialized")
        except Exception as e:
            print(f"[MuJoCoSimulation] Renderer initialization failed (headless environment): {e}")
            self.renderer = None
            print("[MuJoCoSimulation] Continuing without renderer - frames will show error pattern")
    
    def _update_simulation_objects(self):
        """Update simulation state with current MuJoCo state."""
        if not self._initialized:
            return
        
        # Extract body positions and create objects dictionary
        objects = {}
        
        for i in range(self.model.nbody):
            body_name = self.model.body(i).name or f"body_{i}"
            if body_name == "world":
                continue
                
            # Get body position
            body_pos = self.data.xpos[i].copy()
            
            # Create object entry
            objects[body_name] = {
                "type": "body",
                "position": body_pos.tolist(),
                "id": i,
            }
        
        self.state.objects = objects
    
    def start(self):
        """Start the simulation."""
        if not self._initialized:
            print("[MuJoCoSimulation] Cannot start - model not loaded")
            return
            
        if not self.state.is_running:
            self.state.is_running = True
            self.state.is_paused = False
            self._last_step_time = time.time()
            print("[MuJoCoSimulation] Simulation started")
    
    def pause(self):
        """Pause the simulation."""
        if self.state.is_running:
            self.state.is_paused = True
            print("[MuJoCoSimulation] Simulation paused")
    
    def resume(self):
        """Resume the simulation."""
        if self.state.is_running and self.state.is_paused:
            self.state.is_paused = False
            self._last_step_time = time.time()
            print("[MuJoCoSimulation] Simulation resumed")
    
    def stop(self):
        """Stop the simulation."""
        self.state.is_running = False
        self.state.is_paused = False
        print("[MuJoCoSimulation] Simulation stopped")
    
    def reset(self):
        """Reset the simulation to initial state."""
        if not self._initialized:
            return
            
        # Reset MuJoCo data
        mujoco.mj_resetData(self.model, self.data)
        
        # Reset state counters
        self.state.time = 0.0
        self.state.step_count = 0
        
        # Update objects
        self._update_simulation_objects()
        
        print("[MuJoCoSimulation] Simulation reset")
    
    def step(self, num_steps: int = 1):
        """Step the simulation forward.
        
        Args:
            num_steps: Number of simulation steps to execute
        """
        if not self._initialized or not self.state.is_running or self.state.is_paused:
            return
        
        current_time = time.time()
        
        for _ in range(num_steps):
            # Step MuJoCo simulation
            mujoco.mj_step(self.model, self.data)
            
            self.state.step_count += 1
            self.state.time = self.data.time
        
        # Update simulation objects
        self._update_simulation_objects()
        
        self._last_step_time = current_time
    
    def render_frame(self, width: int = 640, height: int = 480, camera_id: int = -1) -> Optional[np.ndarray]:
        """Render a frame from the current simulation state.
        
        Args:
            width: Frame width
            height: Frame height
            camera_id: Camera ID to use (-1 for free camera)
            
        Returns:
            RGB frame as numpy array or None if rendering fails
        """
        if not self._initialized or self.renderer is None:
            return None
        
        try:
            # Update renderer size if needed
            if self.renderer.width != width or self.renderer.height != height:
                self.renderer = mujoco.Renderer(self.model, width=width, height=height)
            
            # Apply camera settings
            self._apply_camera_settings(camera_id)
            
            # Update scene and render
            self.renderer.update_scene(self.data)
            frame = self.renderer.render()
            
            return frame
            
        except Exception as e:
            print(f"[MuJoCoSimulation] Rendering failed: {e}")
            return None
    
    def _apply_camera_settings(self, camera_id: int = -1):
        """Apply camera state to the renderer.
        
        Args:
            camera_id: Camera ID to use (-1 for free camera)
        """
        if camera_id >= 0 and camera_id < self.model.ncam:
            # Use fixed camera
            cam = self.renderer.camera
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam.fixedcamid = camera_id
        else:
            # Use free camera with current camera state
            cam = self.renderer.camera
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            
            # Convert camera state to MuJoCo camera parameters
            cam.distance = self.camera.distance
            cam.azimuth = np.radians(self.camera.azimuth)
            cam.elevation = np.radians(self.camera.elevation)
            cam.lookat[:] = self.camera.target
    
    def handle_event(self, event: EventData) -> bool:
        """Handle input event.
        
        Args:
            event: Input event data
            
        Returns:
            bool: True if event was handled successfully
        """
        print(f"[MuJoCoSimulation] Received event: {event}")
        
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
            "mujoco": {
                "initialized": self._initialized,
                "model_name": "default_pendulum" if self.model else None,
                "nbody": self.model.nbody if self.model else 0,
                "nq": self.model.nq if self.model else 0,
                "nu": self.model.nu if self.model else 0,
                "renderer_available": self.renderer is not None,
            }
        }
    
    async def run_async(self):
        """Run simulation loop asynchronously."""
        print("[MuJoCoSimulation] Starting async simulation loop")
        
        while self.state.is_running:
            if not self.state.is_paused:
                self.step()
            
            # Run at approximately 60 FPS
            await asyncio.sleep(1.0 / 60.0)
        
        print("[MuJoCoSimulation] Async simulation loop stopped")