"""WebRTC video track implementation for MuJoCo rendering."""

import asyncio
import time
from fractions import Fraction
import numpy as np
from av import VideoFrame
from aiortc import VideoStreamTrack

from .config import ViewerConfig


class MuJoCoVideoTrack(VideoStreamTrack):
    """WebRTC video track for real MuJoCo rendering.

    This track renders frames from an actual MuJoCo simulation and streams
    them via WebRTC to the browser client.
    """

    def __init__(self, config: ViewerConfig, mujoco_simulation):
        super().__init__()
        self.config = config
        self.mujoco_simulation = mujoco_simulation
        self.start_time = time.time()
        self.frame_count = 0
        self._sim_timestep = self._infer_sim_timestep()
        self._step_accumulator = 0.0
        self._last_model_id = None  # Track when model changes

        print(
            f"[MuJoCoVideoTrack] Initialized with {config.frame_width}x{config.frame_height} @ {config.frame_rate}fps"
        )
        print("[MuJoCoVideoTrack] Using real MuJoCo rendering")

    async def recv(self) -> VideoFrame:
        """Generate and return the next video frame from MuJoCo simulation."""
        try:
            # Log first few calls to verify recv() is being called
            if self.frame_count < 5:
                print(f"[MuJoCoVideoTrack] recv() called for frame {self.frame_count}")

            # Calculate timing for consistent frame rate
            target_time = self.start_time + (self.frame_count / self.config.frame_rate)
            current_time = time.time()

            if current_time < target_time:
                await asyncio.sleep(target_time - current_time)

            # Render frame from MuJoCo simulation
            try:
                self._advance_simulation()
            except Exception as exc:
                print(f"[MuJoCoVideoTrack] Simulation step error: {exc}")
                import traceback

                traceback.print_exc()

            frame_data = self._render_mujoco_frame()

            # Log every 100 frames to confirm frames are being generated
            if self.frame_count % 100 == 0 and self.frame_count > 0:
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed if elapsed > 0 else 0
                print(f"[MuJoCoVideoTrack] Generated {self.frame_count} frames ({fps:.1f} FPS)")

            # Create VideoFrame from numpy array
            frame = VideoFrame.from_ndarray(frame_data, format="rgb24")
            frame.pts = self.frame_count
            frame.time_base = Fraction(1, self.config.frame_rate)

            self.frame_count += 1

            return frame
        except Exception as e:
            print(f"[MuJoCoVideoTrack] CRITICAL ERROR in recv(): {e}")
            import traceback

            traceback.print_exc()
            # Return error frame to keep stream alive
            error_frame = self._generate_error_frame()
            frame = VideoFrame.from_ndarray(error_frame, format="rgb24")
            frame.pts = self.frame_count
            frame.time_base = Fraction(1, self.config.frame_rate)
            self.frame_count += 1
            return frame

    def _render_mujoco_frame(self) -> np.ndarray:
        """Render a frame from the MuJoCo simulation.

        Returns:
            np.ndarray: RGB frame data with shape (height, width, 3)
        """
        try:
            # Verify simulation is initialized before rendering
            if (
                not hasattr(self.mujoco_simulation, "_initialized")
                or not self.mujoco_simulation._initialized
            ):
                print("[MuJoCoVideoTrack] Simulation not initialized, using error frame")
                return self._generate_error_frame()

            # Check if model changed
            current_model_id = (
                id(self.mujoco_simulation.model)
                if hasattr(self.mujoco_simulation, "model") and self.mujoco_simulation.model
                else None
            )
            if current_model_id != self._last_model_id:
                if self._last_model_id is not None:  # Not first time
                    print(f"[MuJoCoVideoTrack] *** MODEL CHANGED at frame {self.frame_count} ***")
                    print(f"  Simulation initialized: {self.mujoco_simulation._initialized}")
                    print(f"  Renderer available: {self.mujoco_simulation.renderer is not None}")
                    if hasattr(self.mujoco_simulation, "model") and self.mujoco_simulation.model:
                        print(f"  Model bodies: {self.mujoco_simulation.model.nbody}")
                self._last_model_id = current_model_id

            # Get frame from MuJoCo simulation
            frame = self.mujoco_simulation.render_frame(
                width=self.config.frame_width, height=self.config.frame_height
            )

            if frame is not None:
                # Log occasionally to confirm rendering is happening
                if self.frame_count % 100 == 0:
                    mean_value = np.mean(frame)
                    print(
                        f"[MuJoCoVideoTrack] Frame {self.frame_count}: shape={frame.shape}, brightness={mean_value:.1f}"
                    )
                    # Warn if frames are consistently black
                    if mean_value < 1.0:
                        print("  ⚠️  Frame appears completely black - possible rendering issue")

                # Add overlay information
                frame = self._add_mujoco_overlay(frame)
                return frame
            else:
                # Fall back to error frame if rendering fails
                print(f"[MuJoCoVideoTrack] Render returned None at frame {self.frame_count}")
                return self._generate_error_frame()

        except Exception as e:
            print(f"[MuJoCoVideoTrack] MuJoCo rendering error at frame {self.frame_count}: {e}")
            import traceback

            traceback.print_exc()
            return self._generate_error_frame()

    def _add_mujoco_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Add information overlay to the MuJoCo frame.

        Args:
            frame: RGB frame data from MuJoCo

        Returns:
            np.ndarray: Frame with overlay information
        """
        try:
            height, width = frame.shape[:2]

            # Draw frame counter in top-left corner
            frame_text = f"Frame: {self.frame_count}"
            self._draw_simple_text(frame, frame_text, (10, 10))

            # Draw simulation time in top-right corner (with safety check)
            if hasattr(self.mujoco_simulation, "state") and hasattr(
                self.mujoco_simulation.state, "time"
            ):
                sim_time = self.mujoco_simulation.state.time
                time_text = f"Sim: {sim_time:.2f}s"
                self._draw_simple_text(frame, time_text, (width - 120, 10))

            # Draw simulation status in bottom-left corner (with safety check)
            if hasattr(self.mujoco_simulation, "state"):
                if self.mujoco_simulation.state.is_running:
                    if self.mujoco_simulation.state.is_paused:
                        status_text = "PAUSED"
                    else:
                        status_text = "RUNNING"
                else:
                    status_text = "STOPPED"

                self._draw_simple_text(frame, status_text, (10, height - 25))

            # Draw camera info in bottom-right corner (with safety check)
            if hasattr(self.mujoco_simulation, "camera") and hasattr(
                self.mujoco_simulation.camera, "distance"
            ):
                cam_text = f"Cam: {self.mujoco_simulation.camera.distance:.1f}m"
                self._draw_simple_text(frame, cam_text, (width - 120, height - 25))

        except Exception as e:
            # If overlay fails, just return the frame without overlay
            print(f"[MuJoCoVideoTrack] Overlay error: {e}")

        return frame

    def _generate_error_frame(self) -> np.ndarray:
        """Generate an error frame when MuJoCo rendering fails.

        Returns:
            np.ndarray: RGB error frame
        """
        width = self.config.frame_width
        height = self.config.frame_height

        # Create red error frame
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:, :, 0] = 128  # Red background

        # Add error message
        error_text = "MuJoCo Render Error"
        text_x = width // 2 - len(error_text) * 4
        text_y = height // 2 - 6

        self._draw_simple_text(frame, error_text, (text_x, text_y))

        # Add frame counter
        frame_text = f"Frame: {self.frame_count}"
        self._draw_simple_text(frame, frame_text, (10, 10))

        return frame

    def _draw_simple_text(self, frame: np.ndarray, text: str, pos: tuple):
        """Draw simple text on frame using basic rectangle patterns.

        Args:
            frame: RGB frame data to draw on
            text: Text string to draw
            pos: (x, y) position for text
        """
        x, y = pos
        char_width, char_height = 8, 12

        for i, char in enumerate(text):
            char_x = x + i * char_width
            if char_x + char_width >= frame.shape[1] or y + char_height >= frame.shape[0]:
                break

            # Draw a simple rectangle for each character (placeholder)
            if char.isalnum() or char in ".:- ()":
                # White background
                frame[y : y + char_height, char_x : char_x + char_width] = [255, 255, 255]

                # Black border for visibility
                if char != " ":
                    frame[y : y + 2, char_x : char_x + char_width] = [0, 0, 0]  # Top
                    frame[y + char_height - 2 : y + char_height, char_x : char_x + char_width] = [
                        0,
                        0,
                        0,
                    ]  # Bottom
                    frame[y : y + char_height, char_x : char_x + 2] = [0, 0, 0]  # Left
                    frame[y : y + char_height, char_x + char_width - 2 : char_x + char_width] = [
                        0,
                        0,
                        0,
                    ]  # Right

    def _advance_simulation(self):
        """Advance the MuJoCo simulation to keep pace with real time."""
        self._sync_sim_timestep()

        frame_duration = 1.0 / max(self.config.frame_rate, 1)
        self._step_accumulator += frame_duration

        steps_to_run = 0
        timestep = max(self._sim_timestep, 1e-4)

        while self._step_accumulator >= timestep:
            self._step_accumulator -= timestep
            steps_to_run += 1

        if steps_to_run == 0:
            steps_to_run = 1

        self.mujoco_simulation.step(steps_to_run)

    def _infer_sim_timestep(self) -> float:
        """Best effort estimate of the MuJoCo simulation timestep."""
        try:
            model = getattr(self.mujoco_simulation, "model", None)
            if model is not None:
                dt = float(getattr(model.opt, "timestep", 0.0))
                if dt > 0:
                    return dt
        except Exception:
            pass

        return getattr(self.mujoco_simulation, "_step_size", 0.01)

    def _sync_sim_timestep(self):
        """Refresh cached timestep in case the model was reloaded."""
        current_dt = self._infer_sim_timestep()
        if abs(current_dt - self._sim_timestep) > 1e-6:
            self._sim_timestep = current_dt
