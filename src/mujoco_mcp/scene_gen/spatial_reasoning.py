#!/usr/bin/env python3
"""
Advanced Spatial Reasoning

Implements Phase 2B of the enhanced scene generation system:
1. Stable pose enumeration and sampling
2. Workspace volume validation  
3. Robot base placement rules and reachability checks
4. Orientation constraints and multi-anchor constraints

This addresses the limitations in PR #12 around spatial reasoning capabilities.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Set, Union
from dataclasses import dataclass
from enum import Enum

from .shared_types import Pose, AABBBox
from .scene_schema import SceneDescription, SpatialConstraint, ObjectPlacement, RobotConfiguration
from .metadata_extractor import MetadataExtractor, AssetMetadata

logger = logging.getLogger("mujoco_mcp.scene_gen.spatial_reasoning")


class SupportSurface(Enum):
    """Types of support surfaces for stable poses."""
    TOP = "top"
    BOTTOM = "bottom"
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"


@dataclass
class StablePose:
    """
    Represents a stable pose for an object on a support surface.
    """
    object_type: str
    support_surface: SupportSurface
    relative_position: np.ndarray  # [x, y, z] offset from support center
    relative_orientation: np.ndarray  # quaternion [x, y, z, w]
    stability_score: float  # 0.0 to 1.0, higher is more stable
    contact_area: float  # Contact area with support surface
    description: str = ""
    
    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.relative_position = np.array(self.relative_position, dtype=float)
        self.relative_orientation = np.array(self.relative_orientation, dtype=float)


@dataclass
class WorkspaceVolume:
    """
    Defines a workspace volume for robot operations.
    """
    name: str
    center: np.ndarray  # [x, y, z] center of workspace
    extents: np.ndarray  # [width, depth, height] extents 
    orientation: np.ndarray  # quaternion [x, y, z, w] workspace orientation
    accessibility_score: float  # 0.0 to 1.0, how easily robot can reach
    preferred_approach_directions: List[np.ndarray]  # List of unit vectors
    
    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.center = np.array(self.center, dtype=float)
        self.extents = np.array(self.extents, dtype=float)
        self.orientation = np.array(self.orientation, dtype=float)
        self.preferred_approach_directions = [
            np.array(direction, dtype=float) for direction in self.preferred_approach_directions
        ]
    
    def contains_point(self, point: np.ndarray) -> bool:
        """Check if a point is inside this workspace volume."""
        # Transform point to workspace-local coordinates
        # For simplicity, assume axis-aligned workspace for now
        relative_pos = point - self.center
        return np.all(np.abs(relative_pos) <= self.extents / 2)
    
    def get_volume(self) -> float:
        """Get the volume of this workspace."""
        return np.prod(self.extents)


class StablePoseDatabase:
    """
    Database of stable poses for different object types.
    Provides pose enumeration and sampling capabilities.
    """
    
    def __init__(self):
        self.stable_poses: Dict[str, List[StablePose]] = {}
        self._initialize_default_poses()
    
    def _initialize_default_poses(self):
        """Initialize default stable poses for common objects."""
        
        # Table - primarily supports objects on top
        self.stable_poses["table_standard"] = [
            StablePose(
                object_type="table_standard",
                support_surface=SupportSurface.BOTTOM,
                relative_position=np.array([0, 0, 0]),
                relative_orientation=np.array([0, 0, 0, 1]),
                stability_score=1.0,
                contact_area=1.0,
                description="Table resting on floor"
            )
        ]
        
        # Cup - can be upright or on its side
        self.stable_poses["cup_ceramic_small"] = [
            StablePose(
                object_type="cup_ceramic_small",
                support_surface=SupportSurface.BOTTOM,
                relative_position=np.array([0, 0, 0]),
                relative_orientation=np.array([0, 0, 0, 1]),
                stability_score=0.9,
                contact_area=0.05,
                description="Cup upright on base"
            ),
            StablePose(
                object_type="cup_ceramic_small",
                support_surface=SupportSurface.LEFT,
                relative_position=np.array([0, 0, 0.02]),
                relative_orientation=np.array([0, 0.707, 0, 0.707]),  # 90 degrees around Y
                stability_score=0.7,
                contact_area=0.08,
                description="Cup on its side"
            )
        ]
        
        # Box - can rest on any face
        self.stable_poses["box_small"] = [
            StablePose(
                object_type="box_small",
                support_surface=SupportSurface.BOTTOM,
                relative_position=np.array([0, 0, 0]),
                relative_orientation=np.array([0, 0, 0, 1]),
                stability_score=1.0,
                contact_area=0.01,
                description="Box on bottom face"
            ),
            StablePose(
                object_type="box_small",
                support_surface=SupportSurface.LEFT,
                relative_position=np.array([0, 0, 0.05]),
                relative_orientation=np.array([0, 0, 0.707, 0.707]),  # 90 degrees around Z
                stability_score=0.8,
                contact_area=0.01,
                description="Box on side face"
            )
        ]
        
        logger.info(f"Initialized stable poses for {len(self.stable_poses)} object types")
    
    def get_stable_poses(self, object_type: str, 
                        support_surface: Optional[SupportSurface] = None,
                        min_stability: float = 0.0) -> List[StablePose]:
        """
        Get stable poses for an object type, optionally filtered by support surface and stability.
        """
        poses = self.stable_poses.get(object_type, [])
        
        # Filter by support surface
        if support_surface is not None:
            poses = [p for p in poses if p.support_surface == support_surface]
        
        # Filter by minimum stability
        poses = [p for p in poses if p.stability_score >= min_stability]
        
        # Sort by stability score (highest first)
        poses.sort(key=lambda p: p.stability_score, reverse=True)
        
        return poses
    
    def sample_stable_pose(self, object_type: str, 
                          support_surface: Optional[SupportSurface] = None,
                          min_stability: float = 0.5) -> Optional[StablePose]:
        """
        Sample a stable pose weighted by stability score.
        """
        poses = self.get_stable_poses(object_type, support_surface, min_stability)
        if not poses:
            return None
        
        # Weight by stability score
        weights = [p.stability_score for p in poses]
        weights = np.array(weights) / np.sum(weights)
        
        # Sample
        chosen_idx = np.random.choice(len(poses), p=weights)
        return poses[chosen_idx]
    
    def add_stable_pose(self, stable_pose: StablePose):
        """Add a new stable pose to the database."""
        if stable_pose.object_type not in self.stable_poses:
            self.stable_poses[stable_pose.object_type] = []
        self.stable_poses[stable_pose.object_type].append(stable_pose)


class RobotReachabilityChecker:
    """
    Checks robot reachability and validates robot base placement.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.robot_specs = self._initialize_robot_specs()
    
    def _initialize_robot_specs(self) -> Dict[str, Dict]:
        """Initialize robot specifications for reachability checking."""
        return {
            "franka_panda": {
                "reach_radius": 0.855,  # Maximum reach in meters
                "workspace_height": (0.0, 1.2),  # Min/max Z workspace
                "base_clearance": 0.3,  # Minimum clearance from obstacles
                "preferred_base_height": 0.0,  # Preferred base Z position
                "workspace_volumes": [
                    WorkspaceVolume(
                        name="primary",
                        center=np.array([0.6, 0.0, 0.5]),
                        extents=np.array([1.0, 1.2, 0.8]),
                        orientation=np.array([0, 0, 0, 1]),
                        accessibility_score=1.0,
                        preferred_approach_directions=[
                            np.array([1, 0, 0]),  # Forward
                            np.array([0, 1, 0]),  # Right
                            np.array([0, -1, 0])  # Left
                        ]
                    )
                ]
            }
        }
    
    def check_reachability(self, robot_type: str, robot_pose: Pose, 
                          target_position: np.ndarray) -> Tuple[bool, float]:
        """
        Check if a target position is reachable by the robot.
        
        Returns:
            (is_reachable, reachability_score)
        """
        specs = self.robot_specs.get(robot_type)
        if not specs:
            logger.warning(f"No specifications found for robot type: {robot_type}")
            return False, 0.0
        
        # Calculate distance from robot base to target
        base_to_target = target_position - robot_pose.position
        distance = np.linalg.norm(base_to_target)
        
        # Check if within reach radius
        max_reach = specs["reach_radius"]
        if distance > max_reach:
            return False, 0.0
        
        # Check if within workspace height limits
        min_z, max_z = specs["workspace_height"]
        target_z = target_position[2]
        if target_z < min_z or target_z > max_z:
            return False, 0.0
        
        # Calculate reachability score (1.0 at optimal distance, lower at extremes)
        optimal_distance = max_reach * 0.7  # 70% of max reach is optimal
        distance_score = 1.0 - abs(distance - optimal_distance) / max_reach
        
        # Height score (1.0 at mid-workspace, lower at extremes)
        mid_height = (min_z + max_z) / 2
        height_range = max_z - min_z
        height_score = 1.0 - abs(target_z - mid_height) / (height_range / 2)
        
        reachability_score = min(distance_score, height_score)
        return True, max(0.0, reachability_score)
    
    def find_optimal_base_position(self, robot_type: str, target_positions: List[np.ndarray],
                                 scene: SceneDescription, existing_poses: Dict[str, Pose]) -> Tuple[np.ndarray, float]:
        """
        Find optimal base position for robot to reach all target positions.
        
        Returns:
            (optimal_position, overall_reachability_score)
        """
        specs = self.robot_specs.get(robot_type)
        if not specs:
            raise ValueError(f"No specifications found for robot type: {robot_type}")
        
        max_reach = specs["reach_radius"]
        base_clearance = specs["base_clearance"]
        
        # Search grid around targets
        search_positions = []
        for target in target_positions:
            # Create search grid around each target
            for dx in np.linspace(-max_reach, max_reach, 10):
                for dy in np.linspace(-max_reach, max_reach, 10):
                    candidate_pos = target + np.array([dx, dy, 0])
                    search_positions.append(candidate_pos)
        
        best_position = None
        best_score = 0.0
        
        for candidate_pos in search_positions:
            # Check if position collides with existing objects
            if self._check_base_collision(candidate_pos, robot_type, existing_poses, scene):
                continue
            
            # Calculate reachability scores for all targets
            robot_pose = Pose(position=candidate_pos, orientation=np.array([0, 0, 0, 1]))
            scores = []
            
            for target in target_positions:
                reachable, score = self.check_reachability(robot_type, robot_pose, target)
                if not reachable:
                    scores.append(0.0)
                    break
                scores.append(score)
            
            if len(scores) == len(target_positions) and min(scores) > 0:
                overall_score = np.mean(scores)
                if overall_score > best_score:
                    best_score = overall_score
                    best_position = candidate_pos
        
        if best_position is None:
            # Fallback to default position
            logger.warning(f"Could not find optimal base position for {robot_type}, using default")
            best_position = np.array([0.8, 0.0, 0.0])
            best_score = 0.1
        
        return best_position, best_score
    
    def _check_base_collision(self, base_position: np.ndarray, robot_type: str,
                            existing_poses: Dict[str, Pose], scene: SceneDescription) -> bool:
        """Check if robot base would collide with existing objects."""
        # Get robot metadata for collision checking
        robot_metadata = self.metadata_extractor.get_metadata(robot_type)
        if not robot_metadata:
            robot_metadata = self.metadata_extractor._create_fallback_metadata(robot_type)
        
        # Create AABB for robot base
        robot_pose = Pose(position=base_position, orientation=np.array([0, 0, 0, 1]))
        robot_aabb = AABBBox.from_metadata(robot_metadata, robot_pose)
        
        # Check against all existing objects
        for entity_id, pose in existing_poses.items():
            entity_metadata = self._get_entity_metadata(entity_id, scene)
            if entity_metadata:
                entity_aabb = AABBBox.from_metadata(entity_metadata, pose)
                if robot_aabb.overlaps(entity_aabb):
                    return True
        
        return False
    
    def _get_entity_metadata(self, entity_id: str, scene: SceneDescription) -> Optional[AssetMetadata]:
        """Get metadata for an entity in the scene."""
        # Check objects
        for obj in scene.objects:
            if obj.object_id == entity_id:
                return self.metadata_extractor.get_metadata(obj.object_type)
        
        # Check robots
        for robot in scene.robots:
            if robot.robot_id == entity_id:
                return self.metadata_extractor.get_metadata(robot.robot_type)
        
        return None


class AdvancedSpatialReasoner:
    """
    Main class for advanced spatial reasoning capabilities.
    Integrates stable pose sampling, workspace validation, and reachability checking.
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        self.stable_pose_db = StablePoseDatabase()
        self.reachability_checker = RobotReachabilityChecker(metadata_extractor)
    
    def enhance_object_placement(self, obj: ObjectPlacement, 
                                support_metadata: AssetMetadata,
                                support_pose: Pose) -> Tuple[Pose, float]:
        """
        Enhance object placement using stable pose reasoning.
        
        Returns:
            (enhanced_pose, stability_score)
        """
        # Sample stable pose for this object
        stable_pose = self.stable_pose_db.sample_stable_pose(
            obj.object_type,
            support_surface=SupportSurface.TOP,  # Assume placing on top for now
            min_stability=0.3
        )
        
        if stable_pose is None:
            # Fallback to basic placement
            logger.debug(f"No stable pose found for {obj.object_type}, using basic placement")
            basic_pose = Pose(
                position=support_pose.position + np.array([0, 0, 0.1]),
                orientation=np.array(obj.orientation) if obj.orientation else np.array([0, 0, 0, 1])
            )
            return basic_pose, 0.5
        
        # Calculate world pose from stable pose
        support_top = support_pose.position.copy()
        support_dims = support_metadata.get_dimensions()
        support_top[2] += support_dims.get('height', 0.1) / 2
        
        world_position = support_top + stable_pose.relative_position
        world_orientation = stable_pose.relative_orientation
        
        enhanced_pose = Pose(position=world_position, orientation=world_orientation)
        
        logger.debug(f"Enhanced placement for {obj.object_id}: stability={stable_pose.stability_score:.2f}")
        
        return enhanced_pose, stable_pose.stability_score
    
    def validate_robot_workspace(self, robot: RobotConfiguration,
                                scene: SceneDescription,
                                existing_poses: Dict[str, Pose]) -> Tuple[bool, float, List[str]]:
        """
        Validate robot workspace and suggest improvements.
        
        Returns:
            (is_valid, overall_score, suggestions)
        """
        suggestions = []
        
        if robot.robot_id not in existing_poses:
            return False, 0.0, ["Robot pose not found"]
        
        robot_pose = existing_poses[robot.robot_id]
        
        # Get all target positions (objects on tables that robot might need to manipulate)
        target_positions = []
        for obj in scene.objects:
            if obj.object_id in existing_poses:
                obj_pose = existing_poses[obj.object_id]
                # Only consider objects that are likely manipulation targets
                if "cup" in obj.object_type or "box" in obj.object_type:
                    target_positions.append(obj_pose.position)
        
        if not target_positions:
            return True, 1.0, ["No manipulation targets found"]
        
        # Check reachability for each target
        reachability_scores = []
        unreachable_targets = []
        
        for i, target in enumerate(target_positions):
            reachable, score = self.reachability_checker.check_reachability(
                robot.robot_type, robot_pose, target
            )
            if reachable:
                reachability_scores.append(score)
            else:
                unreachable_targets.append(f"target_{i}")
                reachability_scores.append(0.0)
        
        overall_score = np.mean(reachability_scores) if reachability_scores else 0.0
        
        # Generate suggestions
        if unreachable_targets:
            suggestions.append(f"Unreachable targets: {len(unreachable_targets)}")
            
            # Suggest better base position
            optimal_pos, optimal_score = self.reachability_checker.find_optimal_base_position(
                robot.robot_type, target_positions, scene, existing_poses
            )
            suggestions.append(f"Suggested base position: [{optimal_pos[0]:.2f}, {optimal_pos[1]:.2f}, {optimal_pos[2]:.2f}]")
        
        if overall_score < 0.7:
            suggestions.append("Consider repositioning robot for better reachability")
        
        is_valid = overall_score > 0.3 and len(unreachable_targets) == 0
        
        return is_valid, overall_score, suggestions
    
    def suggest_workspace_improvements(self, scene: SceneDescription,
                                     poses: Dict[str, Pose]) -> List[str]:
        """
        Suggest improvements to workspace layout for better manipulation.
        """
        suggestions = []
        
        # Analyze robot-object relationships
        for robot in scene.robots:
            if robot.robot_id in poses:
                is_valid, score, robot_suggestions = self.validate_robot_workspace(
                    robot, scene, poses
                )
                
                if not is_valid:
                    suggestions.append(f"Robot {robot.robot_id} workspace issues:")
                    suggestions.extend([f"  - {s}" for s in robot_suggestions])
        
        # Analyze object stability
        for obj in scene.objects:
            if obj.object_id in poses:
                stable_poses = self.stable_pose_db.get_stable_poses(obj.object_type)
                if stable_poses:
                    max_stability = max(p.stability_score for p in stable_poses)
                    if max_stability < 0.7:
                        suggestions.append(f"Object {obj.object_id} may be unstable (max stability: {max_stability:.2f})")
        
        if not suggestions:
            suggestions.append("Workspace layout appears optimal")
        
        return suggestions