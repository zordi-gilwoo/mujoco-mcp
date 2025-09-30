#!/usr/bin/env python3
"""
Enhanced Collision Detection and Resolution

Addresses limitations in PR #12's collision system:
1. Rotation-aware AABB collision detection
2. Physics-based contact validation using MuJoCo
3. Global optimization for collision resolution

This implements Phase 2A of the enhanced scene generation system.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Set
from dataclasses import dataclass

from .shared_types import Pose, AABBBox
from .scene_schema import SceneDescription, SpatialConstraint
from .metadata_extractor import MetadataExtractor, AssetMetadata

# Try to import MuJoCo for physics-based validation
try:
    import mujoco
    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False

logger = logging.getLogger("mujoco_mcp.scene_gen.enhanced_collision")


def quaternion_to_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = quat
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])


def transform_bbox_with_rotation(bbox_min: np.ndarray, bbox_max: np.ndarray, 
                               pose: Pose) -> Tuple[np.ndarray, np.ndarray]:
    """
    Transform bounding box corners considering rotation.
    Returns the AABB of the rotated object.
    """
    # Get all 8 corners of the bounding box
    corners = np.array([
        [bbox_min[0], bbox_min[1], bbox_min[2]],
        [bbox_max[0], bbox_min[1], bbox_min[2]],
        [bbox_min[0], bbox_max[1], bbox_min[2]],
        [bbox_max[0], bbox_max[1], bbox_min[2]],
        [bbox_min[0], bbox_min[1], bbox_max[2]],
        [bbox_max[0], bbox_min[1], bbox_max[2]],
        [bbox_min[0], bbox_max[1], bbox_max[2]],
        [bbox_max[0], bbox_max[1], bbox_max[2]]
    ])
    
    # Apply rotation
    rotation_matrix = quaternion_to_matrix(pose.orientation)
    rotated_corners = corners @ rotation_matrix.T
    
    # Translate to world position
    world_corners = rotated_corners + pose.position
    
    # Find new AABB
    new_min = np.min(world_corners, axis=0)
    new_max = np.max(world_corners, axis=0)
    
    return new_min, new_max


@dataclass
class CollisionInfo:
    """Information about a collision between two entities."""
    entity1_id: str
    entity2_id: str
    penetration_depth: float
    separation_vector: np.ndarray
    contact_points: List[np.ndarray]
    is_physics_validated: bool = False


class RotationAwareAABB(AABBBox):
    """Enhanced AABB that considers object rotation."""
    
    @classmethod
    def from_metadata(cls, metadata: AssetMetadata, pose: Pose) -> 'RotationAwareAABB':
        """Create rotation-aware AABB from asset metadata and pose."""
        bbox_min, bbox_max = metadata.get_bounding_box()
        
        # Transform bounding box considering rotation
        world_min, world_max = transform_bbox_with_rotation(
            np.array(bbox_min), np.array(bbox_max), pose
        )
        
        return cls(world_min, world_max)
    
    def get_overlap_volume(self, other: 'RotationAwareAABB') -> float:
        """Calculate the volume of overlap between two AABBs."""
        if not self.overlaps(other):
            return 0.0
        
        overlap_min = np.maximum(self.min, other.min)
        overlap_max = np.minimum(self.max, other.max)
        overlap_dims = overlap_max - overlap_min
        
        return np.prod(overlap_dims)
    
    def get_surface_contact_area(self, other: 'RotationAwareAABB') -> float:
        """Estimate surface contact area between two AABBs."""
        if not self.overlaps(other):
            return 0.0
        
        overlap_min = np.maximum(self.min, other.min)
        overlap_max = np.minimum(self.max, other.max)
        overlap_dims = overlap_max - overlap_min
        
        # Find the minimum dimension (likely the contact interface)
        min_dim_idx = np.argmin(overlap_dims)
        contact_dims = np.delete(overlap_dims, min_dim_idx)
        
        return np.prod(contact_dims) if len(contact_dims) > 0 else 0.0


class PhysicsCollisionValidator:
    """Validates collisions using MuJoCo physics simulation."""
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.physics_available = MUJOCO_AVAILABLE
        
        if not self.physics_available:
            logger.warning("MuJoCo not available - physics validation disabled")
    
    def validate_scene_physics(self, scene: SceneDescription, poses: Dict[str, Pose]) -> List[CollisionInfo]:
        """
        Validate scene using MuJoCo physics and return detailed collision info.
        """
        if not self.physics_available:
            logger.debug("Physics validation skipped - MuJoCo not available")
            return []
        
        try:
            # Build minimal XML for collision checking
            xml_string = self._build_collision_check_xml(scene, poses)
            
            # Load into MuJoCo
            model = mujoco.MjModel.from_xml_string(xml_string)
            data = mujoco.MjData(model)
            
            # Forward step to detect contacts
            mujoco.mj_forward(model, data)
            
            # Extract contact information
            collisions = []
            for i in range(data.ncon):
                contact = data.contact[i]
                geom1_id = model.geom_names[contact.geom1]
                geom2_id = model.geom_names[contact.geom2]
                
                collision_info = CollisionInfo(
                    entity1_id=geom1_id,
                    entity2_id=geom2_id,
                    penetration_depth=contact.dist,
                    separation_vector=contact.frame[:3],  # Contact normal
                    contact_points=[contact.pos],
                    is_physics_validated=True
                )
                collisions.append(collision_info)
            
            logger.info(f"Physics validation found {len(collisions)} contacts")
            return collisions
            
        except Exception as e:
            logger.warning(f"Physics validation failed: {e}")
            return []
    
    def _build_collision_check_xml(self, scene: SceneDescription, poses: Dict[str, Pose]) -> str:
        """Build minimal XML for collision checking."""
        xml_parts = ['<mujoco><worldbody>']
        
        # Add objects
        for obj in scene.objects:
            if obj.object_id in poses:
                pose = poses[obj.object_id]
                metadata = self.metadata_extractor.get_metadata(obj.object_type)
                if metadata:
                    # Simple box geometry for collision checking
                    dims = metadata.get_dimensions()
                    size_str = f"{dims.get('width', 0.1)/2} {dims.get('depth', 0.1)/2} {dims.get('height', 0.1)/2}"
                    
                    xml_parts.append(f'''
                    <body name="{obj.object_id}" pos="{pose.position[0]} {pose.position[1]} {pose.position[2]}"
                          quat="{pose.orientation[3]} {pose.orientation[0]} {pose.orientation[1]} {pose.orientation[2]}">
                        <geom name="{obj.object_id}_geom" type="box" size="{size_str}" contype="1" conaffinity="1"/>
                    </body>
                    ''')
        
        # Add robots (simplified as boxes)
        for robot in scene.robots:
            if robot.robot_id in poses:
                pose = poses[robot.robot_id]
                metadata = self.metadata_extractor.get_metadata(robot.robot_type)
                if metadata:
                    dims = metadata.get_dimensions()
                    size_str = f"{dims.get('width', 0.3)/2} {dims.get('depth', 0.3)/2} {dims.get('height', 1.0)/2}"
                    
                    xml_parts.append(f'''
                    <body name="{robot.robot_id}" pos="{pose.position[0]} {pose.position[1]} {pose.position[2]}"
                          quat="{pose.orientation[3]} {pose.orientation[0]} {pose.orientation[1]} {pose.orientation[2]}">
                        <geom name="{robot.robot_id}_geom" type="box" size="{size_str}" contype="1" conaffinity="1"/>
                    </body>
                    ''')
        
        xml_parts.append('</worldbody></mujoco>')
        return ''.join(xml_parts)


class GlobalCollisionOptimizer:
    """
    Global optimization for collision resolution that considers all entities
    simultaneously to prevent drift and local minima.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.physics_validator = PhysicsCollisionValidator(metadata_extractor)
        self.max_iterations = 20
        self.convergence_threshold = 0.001  # meters
        
    def optimize_poses(self, scene: SceneDescription, initial_poses: Dict[str, Pose]) -> Dict[str, Pose]:
        """
        Globally optimize poses to resolve all collisions while maintaining constraints.
        """
        poses = {k: Pose(v.position.copy(), v.orientation.copy()) for k, v in initial_poses.items()}
        
        # Get collision constraints
        collision_constraints = self._get_collision_constraints(scene)
        if not collision_constraints:
            logger.debug("No collision constraints to optimize")
            return poses
        
        logger.info(f"Optimizing {len(collision_constraints)} collision constraints")
        
        for iteration in range(self.max_iterations):
            total_movement = 0.0
            collision_resolved = True
            
            # Detect all current collisions
            current_collisions = self._detect_all_collisions(scene, poses, collision_constraints)
            
            if not current_collisions:
                logger.info(f"All collisions resolved in {iteration} iterations")
                break
            
            # Resolve collisions using global energy minimization approach
            movement_vectors = self._compute_global_movement_vectors(
                scene, poses, current_collisions, collision_constraints
            )
            
            # Apply movements
            for entity_id, movement in movement_vectors.items():
                if np.linalg.norm(movement) > self.convergence_threshold:
                    poses[entity_id].position += movement
                    total_movement += np.linalg.norm(movement)
                    collision_resolved = False
            
            logger.debug(f"Iteration {iteration}: moved {len(movement_vectors)} entities, total movement: {total_movement:.4f}m")
            
            if collision_resolved or total_movement < self.convergence_threshold:
                break
        
        # Final physics validation if available
        if self.physics_validator.physics_available:
            physics_collisions = self.physics_validator.validate_scene_physics(scene, poses)
            if physics_collisions:
                logger.warning(f"Physics validation still shows {len(physics_collisions)} contacts after optimization")
        
        return poses
    
    def _get_collision_constraints(self, scene: SceneDescription) -> List[SpatialConstraint]:
        """Extract all no_collision constraints from the scene."""
        constraints = []
        for entity_id in scene.get_all_entity_ids():
            entity_constraints = scene.get_constraints_for_entity(entity_id)
            for constraint in entity_constraints:
                if constraint.type == "no_collision":
                    constraints.append(constraint)
        return constraints
    
    def _detect_all_collisions(self, scene: SceneDescription, poses: Dict[str, Pose], 
                             collision_constraints: List[SpatialConstraint]) -> List[CollisionInfo]:
        """Detect all current collisions using rotation-aware AABBs."""
        collisions = []
        
        for constraint in collision_constraints:
            subject_id = constraint.subject
            reference_id = constraint.reference
            
            if subject_id not in poses or reference_id not in poses:
                continue
            
            # Get enhanced AABBs
            subject_metadata = self._get_entity_metadata(subject_id, scene)
            reference_metadata = self._get_entity_metadata(reference_id, scene)
            
            subject_aabb = RotationAwareAABB.from_metadata(subject_metadata, poses[subject_id])
            reference_aabb = RotationAwareAABB.from_metadata(reference_metadata, poses[reference_id])
            
            if subject_aabb.overlaps(reference_aabb):
                penetration = subject_aabb.get_overlap_volume(reference_aabb)
                separation = subject_aabb.separation_vector(reference_aabb)
                
                collision_info = CollisionInfo(
                    entity1_id=subject_id,
                    entity2_id=reference_id,
                    penetration_depth=penetration,
                    separation_vector=separation,
                    contact_points=[]
                )
                collisions.append(collision_info)
        
        return collisions
    
    def _compute_global_movement_vectors(self, scene: SceneDescription, poses: Dict[str, Pose],
                                       collisions: List[CollisionInfo], 
                                       collision_constraints: List[SpatialConstraint]) -> Dict[str, np.ndarray]:
        """
        Compute movement vectors for all entities to minimize global collision energy.
        """
        movement_vectors = {entity_id: np.zeros(3) for entity_id in poses.keys()}
        
        # Weight movements by collision severity and constraint priorities
        for collision in collisions:
            # Find the corresponding constraint for clearance info
            clearance = 0.01  # default
            for constraint in collision_constraints:
                if ((constraint.subject == collision.entity1_id and constraint.reference == collision.entity2_id) or
                    (constraint.subject == collision.entity2_id and constraint.reference == collision.entity1_id)):
                    clearance = constraint.clearance
                    break
            
            # Calculate movement based on separation vector and desired clearance
            separation_magnitude = np.linalg.norm(collision.separation_vector)
            if separation_magnitude > 0:
                # Add additional clearance beyond just separation
                required_movement = collision.separation_vector / separation_magnitude * (separation_magnitude + clearance)
                
                # Distribute movement between both entities (weighted by "mobility")
                mobility1 = self._get_entity_mobility(collision.entity1_id, scene)
                mobility2 = self._get_entity_mobility(collision.entity2_id, scene)
                total_mobility = mobility1 + mobility2
                
                if total_mobility > 0:
                    # Entity1 moves in positive separation direction
                    movement_vectors[collision.entity1_id] += required_movement * (mobility1 / total_mobility) * 0.5
                    # Entity2 moves in negative separation direction  
                    movement_vectors[collision.entity2_id] -= required_movement * (mobility2 / total_mobility) * 0.5
        
        return movement_vectors
    
    def _get_entity_mobility(self, entity_id: str, scene: SceneDescription) -> float:
        """
        Get mobility weight for an entity (how easily it can be moved).
        Entities with fewer constraints are more mobile.
        """
        constraints = scene.get_constraints_for_entity(entity_id)
        non_collision_constraints = [c for c in constraints if c.type != "no_collision"]
        
        # Base mobility inversely related to number of constraints
        base_mobility = 1.0 / (len(non_collision_constraints) + 1)
        
        # Tables and large objects are less mobile
        for obj in scene.objects:
            if obj.object_id == entity_id and "table" in obj.object_type.lower():
                base_mobility *= 0.1
        
        return base_mobility
    
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