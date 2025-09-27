"""WebRTC video track implementation with synthetic frame generation."""

import asyncio
import time
import numpy as np
from typing import Tuple
from av import VideoFrame
from aiortc import VideoStreamTrack

from .config import ViewerConfig


class SyntheticVideoTrack(VideoStreamTrack):
    """WebRTC video track that generates synthetic frames for development/testing.
    
    This track produces a colorful gradient pattern that changes over time,
    allowing verification of the WebRTC pipeline without requiring MuJoCo rendering.
    """
    
    def __init__(self, config: ViewerConfig):
        super().__init__()
        self.config = config
        self.start_time = time.time()
        self.frame_count = 0
        
        print(f"[SyntheticVideoTrack] Initialized with {config.frame_width}x{config.frame_height} @ {config.frame_rate}fps")
    
    async def recv(self) -> VideoFrame:
        """Generate and return the next video frame."""
        # Calculate timing for consistent frame rate
        target_time = self.start_time + (self.frame_count / self.config.frame_rate)
        current_time = time.time()
        
        if current_time < target_time:
            await asyncio.sleep(target_time - current_time)
        
        # Generate synthetic frame
        frame_data = self._generate_synthetic_frame()
        
        # Create VideoFrame from numpy array
        frame = VideoFrame.from_ndarray(frame_data, format="rgb24")
        frame.pts = self.frame_count
        frame.time_base = (1, self.config.frame_rate)
        
        self.frame_count += 1
        
        return frame
    
    def _generate_synthetic_frame(self) -> np.ndarray:
        """Generate a synthetic RGB frame with animated patterns.
        
        Returns:
            np.ndarray: RGB frame data with shape (height, width, 3)
        """
        width = self.config.frame_width
        height = self.config.frame_height
        
        # Create coordinate grids
        x = np.linspace(0, 1, width)
        y = np.linspace(0, 1, height)
        X, Y = np.meshgrid(x, y)
        
        # Time-based animation parameter
        t = (time.time() - self.start_time) * 0.5  # Slow animation
        
        # Generate colorful patterns
        # Red channel: radial gradient with time-based rotation
        red = (np.sin(2 * np.pi * (X + t)) + 1) * 0.5
        
        # Green channel: diagonal gradient with wave
        green = (np.sin(2 * np.pi * (Y + X + t * 0.7)) + 1) * 0.5
        
        # Blue channel: circular pattern
        center_x, center_y = 0.5, 0.5
        radius = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
        blue = (np.sin(2 * np.pi * (radius * 3 + t * 1.5)) + 1) * 0.5
        
        # Combine channels and convert to uint8
        frame = np.stack([red, green, blue], axis=2)
        frame = (frame * 255).astype(np.uint8)
        
        # Add some text overlay to show frame info
        frame = self._add_text_overlay(frame)
        
        return frame
    
    def _add_text_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Add simple text overlay to the frame.
        
        Args:
            frame: RGB frame data
            
        Returns:
            np.ndarray: Frame with text overlay
        """
        # Simple text rendering by drawing rectangles for digits
        # This is a placeholder - in a real implementation you might use PIL or OpenCV
        
        height, width = frame.shape[:2]
        
        # Draw frame counter in top-left corner
        frame_text = f"Frame: {self.frame_count}"
        self._draw_simple_text(frame, frame_text, (10, 10))
        
        # Draw timestamp in top-right corner
        elapsed = time.time() - self.start_time
        time_text = f"Time: {elapsed:.1f}s"
        self._draw_simple_text(frame, time_text, (width - 120, 10))
        
        # Draw resolution info in bottom-left corner
        res_text = f"{width}x{height}"
        self._draw_simple_text(frame, res_text, (10, height - 25))
        
        return frame
    
    def _draw_simple_text(self, frame: np.ndarray, text: str, pos: Tuple[int, int]):
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
            if char.isalnum() or char in ".:- ":
                # White background
                frame[y:y+char_height, char_x:char_x+char_width] = [255, 255, 255]
                
                # Black border for visibility
                if char != ' ':
                    frame[y:y+2, char_x:char_x+char_width] = [0, 0, 0]  # Top
                    frame[y+char_height-2:y+char_height, char_x:char_x+char_width] = [0, 0, 0]  # Bottom
                    frame[y:y+char_height, char_x:char_x+2] = [0, 0, 0]  # Left
                    frame[y:y+char_height, char_x+char_width-2:char_x+char_width] = [0, 0, 0]  # Right


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
        
        print(f"[MuJoCoVideoTrack] Initialized with {config.frame_width}x{config.frame_height} @ {config.frame_rate}fps")
        print(f"[MuJoCoVideoTrack] Using real MuJoCo rendering")
    
    async def recv(self) -> VideoFrame:
        """Generate and return the next video frame from MuJoCo simulation."""
        # Calculate timing for consistent frame rate
        target_time = self.start_time + (self.frame_count / self.config.frame_rate)
        current_time = time.time()
        
        if current_time < target_time:
            await asyncio.sleep(target_time - current_time)
        
        # Render frame from MuJoCo simulation
        frame_data = self._render_mujoco_frame()
        
        # Create VideoFrame from numpy array
        frame = VideoFrame.from_ndarray(frame_data, format="rgb24")
        frame.pts = self.frame_count
        frame.time_base = (1, self.config.frame_rate)
        
        self.frame_count += 1
        
        return frame
    
    def _render_mujoco_frame(self) -> np.ndarray:
        """Render a frame from the MuJoCo simulation.
        
        Returns:
            np.ndarray: RGB frame data with shape (height, width, 3)
        """
        try:
            # Get frame from MuJoCo simulation
            frame = self.mujoco_simulation.render_frame(
                width=self.config.frame_width,
                height=self.config.frame_height
            )
            
            if frame is not None:
                # Add overlay information
                frame = self._add_mujoco_overlay(frame)
                return frame
            else:
                # Fall back to error frame if rendering fails
                return self._generate_error_frame()
                
        except Exception as e:
            print(f"[MuJoCoVideoTrack] MuJoCo rendering error: {e}")
            return self._generate_error_frame()
    
    def _add_mujoco_overlay(self, frame: np.ndarray) -> np.ndarray:
        """Add information overlay to the MuJoCo frame.
        
        Args:
            frame: RGB frame data from MuJoCo
            
        Returns:
            np.ndarray: Frame with overlay information
        """
        height, width = frame.shape[:2]
        
        # Draw frame counter in top-left corner
        frame_text = f"Frame: {self.frame_count}"
        self._draw_simple_text(frame, frame_text, (10, 10))
        
        # Draw simulation time in top-right corner
        sim_time = self.mujoco_simulation.state.time
        time_text = f"Sim: {sim_time:.2f}s"
        self._draw_simple_text(frame, time_text, (width - 120, 10))
        
        # Draw simulation status in bottom-left corner
        if self.mujoco_simulation.state.is_running:
            if self.mujoco_simulation.state.is_paused:
                status_text = "PAUSED"
            else:
                status_text = "RUNNING"
        else:
            status_text = "STOPPED"
        
        self._draw_simple_text(frame, status_text, (10, height - 25))
        
        # Draw camera info in bottom-right corner
        cam_text = f"Cam: {self.mujoco_simulation.camera.distance:.1f}m"
        self._draw_simple_text(frame, cam_text, (width - 120, height - 25))
        
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
                frame[y:y+char_height, char_x:char_x+char_width] = [255, 255, 255]
                
                # Black border for visibility
                if char != ' ':
                    frame[y:y+2, char_x:char_x+char_width] = [0, 0, 0]  # Top
                    frame[y+char_height-2:y+char_height, char_x:char_x+char_width] = [0, 0, 0]  # Bottom
                    frame[y:y+char_height, char_x:char_x+2] = [0, 0, 0]  # Left
                    frame[y:y+char_height, char_x+char_width-2:char_x+char_width] = [0, 0, 0]  # Right