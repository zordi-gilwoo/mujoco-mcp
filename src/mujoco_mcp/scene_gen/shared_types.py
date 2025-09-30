#!/usr/bin/env python3
"""
Shared data structures for scene generation

Contains common classes used across multiple modules to avoid circular imports.
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class Pose:
    """Represents a 3D pose with position and orientation."""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # quaternion [x, y, z, w]
    
    def __post_init__(self):
        """Ensure arrays are numpy arrays with correct shapes."""
        self.position = np.array(self.position, dtype=float)
        self.orientation = np.array(self.orientation, dtype=float)
        
        # Default orientation if not provided
        if len(self.orientation) == 0:
            self.orientation = np.array([0.0, 0.0, 0.0, 1.0])


class AABBBox:
    """Axis-aligned bounding box for collision detection."""
    
    def __init__(self, min_coords: np.ndarray, max_coords: np.ndarray):
        self.min = np.array(min_coords, dtype=float)
        self.max = np.array(max_coords, dtype=float)
    
    @classmethod
    def from_metadata(cls, metadata, pose: Pose) -> 'AABBBox':
        """Create AABB from asset metadata and pose."""
        bbox_min, bbox_max = metadata.get_bounding_box()
        
        # Transform bounding box to world coordinates
        # Note: This is a simplified transform that only considers translation
        # Full implementation would handle rotation as well
        world_min = np.array(bbox_min) + pose.position
        world_max = np.array(bbox_max) + pose.position
        
        return cls(world_min, world_max)
    
    def overlaps(self, other: 'AABBBox') -> bool:
        """Check if this AABB overlaps with another."""
        return np.all(self.min <= other.max) and np.all(self.max >= other.min)
    
    def separation_vector(self, other: 'AABBBox') -> np.ndarray:
        """Calculate minimum separation vector to resolve overlap."""
        if not self.overlaps(other):
            return np.zeros(3)
        
        # Calculate overlap in each axis
        overlap_x = min(self.max[0] - other.min[0], other.max[0] - self.min[0])
        overlap_y = min(self.max[1] - other.min[1], other.max[1] - self.min[1])
        overlap_z = min(self.max[2] - other.min[2], other.max[2] - self.min[2])
        
        # Find minimum overlap axis
        min_overlap = min(overlap_x, overlap_y, overlap_z)
        
        if min_overlap == overlap_x:
            # Separate along X axis
            direction = 1 if self.min[0] < other.min[0] else -1
            return np.array([direction * overlap_x, 0, 0])
        elif min_overlap == overlap_y:
            # Separate along Y axis
            direction = 1 if self.min[1] < other.min[1] else -1
            return np.array([0, direction * overlap_y, 0])
        else:
            # Separate along Z axis
            direction = 1 if self.min[2] < other.min[2] else -1
            return np.array([0, 0, direction * overlap_z])