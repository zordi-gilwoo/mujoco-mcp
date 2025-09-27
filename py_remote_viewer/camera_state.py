"""Camera state management for the remote viewer."""

import numpy as np
from typing import Tuple, Dict, Any
from dataclasses import dataclass, field
from .events import EventData, MouseEvent, ScrollEvent, KeyEvent, CommandEvent, EventType


@dataclass
class CameraState:
    """Manages camera position, orientation, and viewing parameters."""
    
    # Camera position and orientation
    distance: float = 5.0  # Distance from target
    azimuth: float = 0.0   # Horizontal rotation (degrees)
    elevation: float = -20.0  # Vertical rotation (degrees)
    
    # Target point (what the camera is looking at)
    target: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
    
    # Field of view
    fov: float = 45.0  # Field of view in degrees
    
    # View bounds
    min_distance: float = 0.1
    max_distance: float = 50.0
    min_elevation: float = -90.0
    max_elevation: float = 90.0
    
    # Interaction sensitivity
    mouse_sensitivity: float = 0.5
    scroll_sensitivity: float = 0.1
    pan_sensitivity: float = 0.01
    
    # State tracking
    is_rotating: bool = False
    is_panning: bool = False
    last_mouse_pos: Tuple[int, int] = (0, 0)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert camera state to dictionary."""
        return {
            "distance": self.distance,
            "azimuth": self.azimuth,
            "elevation": self.elevation,
            "target": self.target.tolist(),
            "fov": self.fov,
            "position": self.get_position().tolist(),
        }
    
    def get_position(self) -> np.ndarray:
        """Calculate camera position from spherical coordinates."""
        # Convert to radians
        az_rad = np.radians(self.azimuth)
        el_rad = np.radians(self.elevation)
        
        # Spherical to cartesian conversion
        x = self.distance * np.cos(el_rad) * np.cos(az_rad)
        y = self.distance * np.cos(el_rad) * np.sin(az_rad)
        z = self.distance * np.sin(el_rad)
        
        return self.target + np.array([x, y, z])
    
    def handle_event(self, event: EventData) -> bool:
        """Handle input event and update camera state.
        
        Returns:
            bool: True if camera state was modified, False otherwise.
        """
        if isinstance(event, MouseEvent):
            return self._handle_mouse_event(event)
        elif isinstance(event, ScrollEvent):
            return self._handle_scroll_event(event)
        elif isinstance(event, KeyEvent):
            return self._handle_key_event(event)
        elif isinstance(event, CommandEvent):
            return self._handle_command_event(event)
        
        return False
    
    def _handle_mouse_event(self, event: MouseEvent) -> bool:
        """Handle mouse events for camera control."""
        modified = False
        
        if event.type == EventType.MOUSE_DOWN:
            if event.buttons & 1:  # Left button
                self.is_rotating = True
                self.last_mouse_pos = (event.x, event.y)
            elif event.buttons & 2:  # Right button
                self.is_panning = True
                self.last_mouse_pos = (event.x, event.y)
                
        elif event.type == EventType.MOUSE_UP:
            self.is_rotating = False
            self.is_panning = False
            
        elif event.type == EventType.MOUSE_MOVE:
            if self.is_rotating and (event.buttons & 1):
                dx = event.x - self.last_mouse_pos[0]
                dy = event.y - self.last_mouse_pos[1]
                
                # Update azimuth and elevation
                self.azimuth += dx * self.mouse_sensitivity
                self.elevation = np.clip(
                    self.elevation - dy * self.mouse_sensitivity,
                    self.min_elevation,
                    self.max_elevation
                )
                
                # Normalize azimuth to [0, 360)
                self.azimuth = self.azimuth % 360.0
                
                self.last_mouse_pos = (event.x, event.y)
                modified = True
                
            elif self.is_panning and (event.buttons & 2):
                dx = event.x - self.last_mouse_pos[0]
                dy = event.y - self.last_mouse_pos[1]
                
                # Calculate pan vectors relative to camera orientation
                az_rad = np.radians(self.azimuth)
                right = np.array([-np.sin(az_rad), np.cos(az_rad), 0.0])
                up = np.array([0.0, 0.0, 1.0])
                
                # Apply panning
                pan_amount = self.distance * self.pan_sensitivity
                self.target += right * dx * pan_amount
                self.target += up * dy * pan_amount
                
                self.last_mouse_pos = (event.x, event.y)
                modified = True
        
        return modified
    
    def _handle_scroll_event(self, event: ScrollEvent) -> bool:
        """Handle scroll events for zoom control."""
        # Zoom in/out by adjusting distance
        zoom_factor = 1.0 + event.dy * self.scroll_sensitivity
        new_distance = self.distance * zoom_factor
        
        # Clamp to valid range
        self.distance = np.clip(new_distance, self.min_distance, self.max_distance)
        
        return True
    
    def _handle_key_event(self, event: KeyEvent) -> bool:
        """Handle keyboard events for camera control."""
        if event.type != EventType.KEY_DOWN:
            return False
            
        modified = False
        
        # Arrow keys for rotation
        if event.code == "ArrowLeft":
            self.azimuth = (self.azimuth - 5.0) % 360.0
            modified = True
        elif event.code == "ArrowRight":
            self.azimuth = (self.azimuth + 5.0) % 360.0
            modified = True
        elif event.code == "ArrowUp":
            self.elevation = np.clip(self.elevation + 5.0, self.min_elevation, self.max_elevation)
            modified = True
        elif event.code == "ArrowDown":
            self.elevation = np.clip(self.elevation - 5.0, self.min_elevation, self.max_elevation)
            modified = True
        
        # Page Up/Down for zoom
        elif event.code == "PageUp":
            self.distance = np.clip(self.distance * 0.9, self.min_distance, self.max_distance)
            modified = True
        elif event.code == "PageDown":
            self.distance = np.clip(self.distance * 1.1, self.min_distance, self.max_distance)
            modified = True
        
        return modified
    
    def _handle_command_event(self, event: CommandEvent) -> bool:
        """Handle command events for camera control."""
        modified = False
        
        if event.cmd == "reset_camera":
            self.distance = 5.0
            self.azimuth = 0.0
            self.elevation = -20.0
            self.target = np.array([0.0, 0.0, 0.0])
            modified = True
            
        elif event.cmd == "set_camera_preset":
            preset = event.params.get("preset") if event.params else None
            if preset == "front":
                self.azimuth = 0.0
                self.elevation = 0.0
                modified = True
            elif preset == "side":
                self.azimuth = 90.0
                self.elevation = 0.0
                modified = True
            elif preset == "top":
                self.azimuth = 0.0
                self.elevation = 90.0
                modified = True
                
        return modified