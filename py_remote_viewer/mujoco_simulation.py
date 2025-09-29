"""Real MuJoCo simulation integration for remote viewer."""

import time
import asyncio
import os
import numpy as np
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
import mujoco

from .camera_state import CameraState
from .events import EventData
from .simulation_stub import SimulationState  # Reuse the state structure
from .builtin_scenes import get_builtin_scene, default_scene_xml


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
        self._free_camera = None
        self._initialized = False
        self._initialization_error = None
        
        # Default to a simple pendulum if no model provided
        if model_xml is None:
            model_xml = default_scene_xml()
        
        # Try to load model with graceful error handling
        try:
            self.load_model(model_xml)
        except Exception as e:
            self._initialization_error = str(e)
            print(f"[MuJoCoSimulation] Failed to initialize: {e}")
            print("[MuJoCoSimulation] Will continue with error frames")
    
    def load_builtin_scene(self, name: str) -> bool:
        """Load one of the bundled demo scenes by keyword."""
        xml = get_builtin_scene(name)
        if xml is None:
            print(f"[MuJoCoSimulation] Unknown builtin scene '{name}'")
            return False
        return self.load_model(xml)
    
    def load_model(self, model_xml: str) -> bool:
        """Load a MuJoCo model from XML string.

        Args:
            model_xml: MuJoCo model XML string
            
        Returns:
            bool: True if model loaded successfully
        """
        try:
            print(f"[MuJoCoSimulation] Loading MuJoCo model...")
            was_running = getattr(self.state, "is_running", False)

            # Load model and create data
            if isinstance(model_xml, str) and os.path.exists(model_xml):
                model_source_desc = f"path={model_xml}"
                mj_model = mujoco.MjModel.from_xml_path(model_xml)
            else:
                model_source_desc = f"xml[{len(model_xml)} chars]"
                mj_model = mujoco.MjModel.from_xml_string(model_xml)

            self.model = mj_model
            self.data = mujoco.MjData(self.model)

            # Initialize renderer for frame capture
            self._initialize_renderer()
            
            # Reset state
            self.state = SimulationState()
            self.camera = CameraState()
            self._update_simulation_objects()
            
            self._initialized = True
            print(f"[MuJoCoSimulation] Model loaded successfully from {model_source_desc}")
            print(f"  - Bodies: {self.model.nbody}")
            print(f"  - Joints: {self.model.njnt}")
            print(f"  - Degrees of freedom: {self.model.nq}")
            print(f"  - Actuators: {self.model.nu}")

            # Resume playback automatically so scene changes are visible immediately
            if was_running or not self.state.is_running:
                self.play()
            
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
            self._initialize_free_camera()
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
    
    def play(self):
        """Begin or resume continuous simulation playback."""
        if not self._initialized:
            print("[MuJoCoSimulation] Cannot play - model not loaded")
            return

        was_running = self.state.is_running
        was_paused = self.state.is_paused if was_running else False

        self._kick_from_rest()

        self.state.is_running = True
        self.state.is_paused = False
        self._last_step_time = time.time()

        if not was_running:
            print("[MuJoCoSimulation] Simulation playing")
        elif was_paused:
            print("[MuJoCoSimulation] Simulation resumed")

    def _kick_from_rest(self):
        """Apply a small perturbation so the model moves when playback starts."""
        if (
            self.data is None
            or self.model is None
            or self.model.nq <= 0
        ):
            return

        if np.allclose(self.data.qpos, self.model.qpos0) and np.allclose(self.data.qvel, 0):
            perturb = np.zeros(self.model.nq)
            perturb[0] = np.deg2rad(10.0)
            self.data.qpos[:] = self.model.qpos0 + perturb
            self.data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.data)
            self._log_state(prefix="kick")

    def start(self):
        """Backward compatible entry point for play()."""
        self.play()
    
    def pause(self):
        """Pause the simulation."""
        if self.state.is_running:
            self.state.is_paused = True
            print("[MuJoCoSimulation] Simulation paused")
    
    def resume(self):
        """Resume the simulation."""
        if self.state.is_running and self.state.is_paused:
            self.play()
    
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
        self.state.is_paused = False
        
        # Update objects
        self._update_simulation_objects()
        
        print("[MuJoCoSimulation] Simulation reset")
    
    def step(self, num_steps: int = 1, *, force: bool = False):
        """Step the simulation forward.

        Args:
            num_steps: Number of simulation steps to execute
            force: Execute steps even if the simulation is paused or stopped
        """
        if not self._initialized:
            return

        if num_steps <= 0:
            return

        if not force and (not self.state.is_running or self.state.is_paused):
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

        if force or self.state.step_count % 60 == 0:
            self._log_state(prefix="step")
    
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
                # Recreate renderer at the requested resolution
                self.renderer = mujoco.Renderer(self.model, width=width, height=height)
                self._initialize_free_camera()

            # Ensure derived quantities are up to date before rendering
            mujoco.mj_forward(self.model, self.data)

            # Apply camera settings and update scene with the requested view
            camera_handle = self._apply_camera_settings(camera_id)

            # Update scene and render
            self.renderer.update_scene(self.data, camera=camera_handle)
            frame = self.renderer.render()
            
            return frame
            
        except Exception as e:
            print(f"[MuJoCoSimulation] Rendering failed: {e}")
            return None
    
    def _initialize_free_camera(self):
        """Create and initialize the free camera used for custom views."""
        self._free_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(self.model, self._free_camera)

    def _apply_camera_settings(self, camera_id: int = -1):
        """Apply camera state to the renderer.

        Args:
            camera_id: Camera ID to use (-1 for free camera)
        """
        if camera_id >= 0 and camera_id < self.model.ncam:
            # Use fixed camera defined in the model
            return camera_id

        if self._free_camera is None:
            self._initialize_free_camera()

        cam = self._free_camera
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.fixedcamid = -1
        cam.trackbodyid = -1
        cam.distance = self.camera.distance
        cam.azimuth = self.camera.azimuth
        cam.elevation = self.camera.elevation
        cam.lookat[:] = self.camera.target

        return cam
    
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
            if cmd == "play" or cmd == "start":
                self.play()
                return True
            elif cmd == "pause":
                self.pause()
                return True
            elif cmd == "resume":
                self.play()
                return True
            elif cmd == "stop":
                self.stop()
                return True
            elif cmd == "reset":
                self.reset()
                return True
            elif cmd == "step":
                steps = 1
                if hasattr(event, "params") and event.params:
                    raw = event.params
                    if isinstance(raw, dict):
                        candidate = raw.get("num_steps") or raw.get("steps") or raw.get("count")
                        if isinstance(candidate, (int, float)):
                            steps = int(candidate)
                        elif isinstance(candidate, str) and candidate.strip().isdigit():
                            steps = int(candidate.strip())
                self.step(max(1, steps), force=True)
                return True

        return camera_modified

    def _log_state(self, prefix: str = "state"):
        if not self._initialized or self.data is None:
            return

        qpos = np.array(self.data.qpos[: min(4, self.model.nq)])
        qvel = np.array(self.data.qvel[: min(4, self.model.nv)])
        print(
            f"[MuJoCoSimulation] {prefix}: t={self.data.time:.3f} "
            f"qpos={np.array2string(qpos, precision=3, suppress_small=True)} "
            f"qvel={np.array2string(qvel, precision=3, suppress_small=True)}"
        )
    
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
                "initialization_error": self._initialization_error,
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
