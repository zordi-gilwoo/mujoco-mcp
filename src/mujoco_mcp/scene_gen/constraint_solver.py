#!/usr/bin/env python3
"""
Constraint Solver

Solves spatial constraints to determine object and robot poses in 3D space.
Implements greedy constraint satisfaction with collision avoidance.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Set
from dataclasses import dataclass

from .scene_schema import SceneDescription, SpatialConstraint
from .metadata_extractor import MetadataExtractor, AssetMetadata

logger = logging.getLogger("mujoco_mcp.scene_gen.constraint_solver")


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
    def from_metadata(cls, metadata: AssetMetadata, pose: Pose) -> 'AABBBox':
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


class ConstraintSolver:
    """
    Solves spatial constraints to determine object placements.
    
    Uses a greedy approach:
    1. Place unconstrained objects at origin/specified positions
    2. Iteratively place constrained objects when their references are placed
    3. Apply collision resolution for no_collision constraints
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.max_collision_iterations = 10
        self.collision_push_distance = 0.01  # meters
    
    def solve(self, scene: SceneDescription) -> Dict[str, Pose]:
        """
        Solve all constraints in the scene and return entity poses.
        
        Args:
            scene: Scene description with objects, robots, and constraints
            
        Returns:
            Dictionary mapping entity IDs to their solved poses
            
        Raises:
            ValueError: If constraints cannot be satisfied
        """
        poses: Dict[str, Pose] = {}
        placed_entities: Set[str] = set()
        all_entities = scene.get_all_entity_ids()
        
        logger.info(f"Solving constraints for {len(all_entities)} entities")
        
        # Phase 1: Place unconstrained entities (typically tables/anchor objects)
        self._place_unconstrained_entities(scene, poses, placed_entities)
        
        # Phase 2: Iteratively place constrained entities
        max_iterations = len(all_entities) * 2  # Safety limit
        iteration = 0
        
        while placed_entities != set(all_entities) and iteration < max_iterations:
            iteration += 1
            progress_made = False
            
            for entity_id in all_entities:
                if entity_id in placed_entities:
                    continue
                
                # Check if all constraint references are placed
                constraints = scene.get_constraints_for_entity(entity_id)
                if self._can_place_entity(constraints, placed_entities):
                    try:
                        pose = self._place_constrained_entity(
                            entity_id, constraints, poses, scene
                        )
                        poses[entity_id] = pose
                        placed_entities.add(entity_id)
                        progress_made = True
                        logger.debug(f"Placed entity {entity_id} at iteration {iteration}")
                    except Exception as e:
                        logger.warning(f"Failed to place {entity_id}: {e}")
            
            if not progress_made:
                unplaced = set(all_entities) - placed_entities
                raise ValueError(f"Cannot satisfy constraints for entities: {unplaced}")
        
        if placed_entities != set(all_entities):
            unplaced = set(all_entities) - placed_entities
            raise ValueError(f"Constraint solving incomplete. Unplaced entities: {unplaced}")
        
        # Phase 3: Resolve collisions
        self._resolve_collisions(scene, poses)
        
        logger.info(f"Successfully solved all constraints in {iteration} iterations")
        return poses
    
    def _place_unconstrained_entities(
        self, 
        scene: SceneDescription, 
        poses: Dict[str, Pose], 
        placed_entities: Set[str]
    ):
        """Place entities that have no constraints."""
        # Place unconstrained objects
        for obj in scene.objects:
            if not obj.constraints:
                # Place at origin with optional orientation
                orientation = np.array(obj.orientation) if obj.orientation else np.array([0, 0, 0, 1])
                pose = Pose(
                    position=np.array([0.0, 0.0, 0.0]),
                    orientation=orientation
                )
                poses[obj.object_id] = pose
                placed_entities.add(obj.object_id)
                logger.debug(f"Placed unconstrained object {obj.object_id} at origin")
        
        # Place unconstrained robots
        for robot in scene.robots:
            if not robot.constraints:
                # Use base_position if specified, otherwise origin
                position = np.array(robot.base_position) if robot.base_position else np.array([0.0, 0.0, 0.0])
                orientation = np.array(robot.base_orientation) if robot.base_orientation else np.array([0, 0, 0, 1])
                
                pose = Pose(position=position, orientation=orientation)
                poses[robot.robot_id] = pose
                placed_entities.add(robot.robot_id)
                logger.debug(f"Placed unconstrained robot {robot.robot_id} at {position}")
    
    def _can_place_entity(self, constraints: List[SpatialConstraint], placed_entities: Set[str]) -> bool:
        """Check if an entity can be placed (all its constraint references are placed)."""
        for constraint in constraints:
            if constraint.reference not in placed_entities:
                return False
        return True
    
    def _place_constrained_entity(
        self, 
        entity_id: str, 
        constraints: List[SpatialConstraint], 
        poses: Dict[str, Pose],
        scene: SceneDescription
    ) -> Pose:
        """Place an entity based on its constraints."""
        if not constraints:
            raise ValueError(f"Entity {entity_id} has no constraints but was called for constrained placement")
        
        # Get entity metadata
        entity_metadata = self._get_entity_metadata(entity_id, scene)
        
        # Start with default pose
        position = np.array([0.0, 0.0, 0.0])
        orientation = self._get_entity_orientation(entity_id, scene)
        
        # Apply each constraint in sequence
        for constraint in constraints:
            if constraint.type == "no_collision":
                continue  # Handle collision constraints in separate phase
            
            position = self._apply_constraint(
                constraint, position, entity_metadata, poses, scene
            )
        
        return Pose(position=position, orientation=orientation)
    
    def _get_entity_metadata(self, entity_id: str, scene: SceneDescription) -> AssetMetadata:
        """Get metadata for an entity (object or robot)."""
        # Check objects
        for obj in scene.objects:
            if obj.object_id == entity_id:
                metadata = self.metadata_extractor.get_metadata(obj.object_type)
                if metadata is None:
                    metadata = self.metadata_extractor._create_fallback_metadata(obj.object_type)
                return metadata
        
        # Check robots
        for robot in scene.robots:
            if robot.robot_id == entity_id:
                metadata = self.metadata_extractor.get_metadata(robot.robot_type)
                if metadata is None:
                    metadata = self.metadata_extractor._create_fallback_metadata(robot.robot_type)
                return metadata
        
        raise ValueError(f"Entity {entity_id} not found in scene")
    
    def _get_entity_orientation(self, entity_id: str, scene: SceneDescription) -> np.ndarray:
        """Get orientation for an entity."""
        # Check objects
        for obj in scene.objects:
            if obj.object_id == entity_id:
                return np.array(obj.orientation) if obj.orientation else np.array([0, 0, 0, 1])
        
        # Check robots
        for robot in scene.robots:
            if robot.robot_id == entity_id:
                return np.array(robot.base_orientation) if robot.base_orientation else np.array([0, 0, 0, 1])
        
        return np.array([0, 0, 0, 1])
    
    def _apply_constraint(
        self, 
        constraint: SpatialConstraint, 
        current_position: np.ndarray,
        entity_metadata: AssetMetadata,
        poses: Dict[str, Pose],
        scene: SceneDescription
    ) -> np.ndarray:
        """Apply a single constraint to determine new position."""
        reference_pose = poses[constraint.reference]
        reference_metadata = self._get_entity_metadata(constraint.reference, scene)
        
        if constraint.type == "on_top_of":
            return self._apply_on_top_of(
                constraint, entity_metadata, reference_pose, reference_metadata
            )
        elif constraint.type == "in_front_of":
            return self._apply_in_front_of(
                constraint, entity_metadata, reference_pose, reference_metadata
            )
        elif constraint.type == "beside":
            return self._apply_beside(
                constraint, entity_metadata, reference_pose, reference_metadata
            )
        else:
            logger.warning(f"Unknown constraint type: {constraint.type}")
            return current_position
    
    def _apply_on_top_of(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata
    ) -> np.ndarray:
        """Apply on_top_of constraint."""
        # Get reference top surface
        ref_top = reference_metadata.get_semantic_point("top_surface")
        if ref_top is None:
            # Fallback to bounding box top
            _, bbox_max = reference_metadata.get_bounding_box()
            ref_top = [reference_pose.position[0], reference_pose.position[1], bbox_max[2]]
        else:
            ref_top = reference_pose.position + np.array(ref_top)
        
        # Get entity bottom surface offset
        entity_bottom = entity_metadata.get_semantic_point("bottom_surface")
        if entity_bottom is None:
            entity_bottom = [0, 0, 0]
        
        # Calculate position to place entity bottom at reference top + clearance
        position = np.array([
            ref_top[0] - entity_bottom[0],
            ref_top[1] - entity_bottom[1], 
            ref_top[2] + constraint.clearance - entity_bottom[2]
        ])
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _apply_in_front_of(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata
    ) -> np.ndarray:
        """Apply in_front_of constraint (places entity in +X direction from reference)."""
        # Get reference dimensions
        ref_dims = reference_metadata.get_dimensions()
        ref_width = ref_dims.get('width', 0.1)
        
        # Get entity dimensions
        entity_dims = entity_metadata.get_dimensions()
        entity_width = entity_dims.get('width', 0.1)
        
        # Place entity in front (positive X direction)
        position = reference_pose.position.copy()
        position[0] += ref_width/2 + entity_width/2 + constraint.clearance
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _apply_beside(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata
    ) -> np.ndarray:
        """Apply beside constraint (places entity in +Y direction from reference)."""
        # Get reference dimensions
        ref_dims = reference_metadata.get_dimensions()
        ref_depth = ref_dims.get('depth', 0.1)
        
        # Get entity dimensions
        entity_dims = entity_metadata.get_dimensions()
        entity_depth = entity_dims.get('depth', 0.1)
        
        # Place entity beside (positive Y direction)
        position = reference_pose.position.copy()
        position[1] += ref_depth/2 + entity_depth/2 + constraint.clearance
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _resolve_collisions(self, scene: SceneDescription, poses: Dict[str, Pose]):
        """Resolve collision constraints using iterative AABB pushing."""
        # Collect all no_collision constraints
        collision_constraints = []
        for entity_id in scene.get_all_entity_ids():
            constraints = scene.get_constraints_for_entity(entity_id)
            for constraint in constraints:
                if constraint.type == "no_collision":
                    collision_constraints.append(constraint)
        
        if not collision_constraints:
            return
        
        logger.info(f"Resolving {len(collision_constraints)} collision constraints")
        
        for iteration in range(self.max_collision_iterations):
            collision_resolved = True
            
            for constraint in collision_constraints:
                subject_id = constraint.subject
                reference_id = constraint.reference
                
                if subject_id not in poses or reference_id not in poses:
                    continue
                
                # Get AABBs
                subject_metadata = self._get_entity_metadata(subject_id, scene)
                reference_metadata = self._get_entity_metadata(reference_id, scene)
                
                subject_aabb = AABBBox.from_metadata(subject_metadata, poses[subject_id])
                reference_aabb = AABBBox.from_metadata(reference_metadata, poses[reference_id])
                
                # Check for overlap
                if subject_aabb.overlaps(reference_aabb):
                    # Calculate separation vector
                    separation = subject_aabb.separation_vector(reference_aabb)
                    
                    # Apply minimum clearance
                    if np.linalg.norm(separation) > 0:
                        separation = separation / np.linalg.norm(separation) * max(
                            np.linalg.norm(separation), constraint.clearance + self.collision_push_distance
                        )
                    
                    # Move subject away from reference
                    poses[subject_id].position += separation
                    collision_resolved = False
                    
                    logger.debug(f"Moved {subject_id} by {separation} to resolve collision with {reference_id}")
            
            if collision_resolved:
                logger.info(f"All collisions resolved in {iteration + 1} iterations")
                break
        else:
            logger.warning(f"Collision resolution did not converge after {self.max_collision_iterations} iterations")