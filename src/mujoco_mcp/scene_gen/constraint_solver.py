#!/usr/bin/env python3
"""
Constraint Solver

Solves spatial constraints to determine object and robot poses in 3D space.
Implements greedy constraint satisfaction with collision avoidance.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Set

from .scene_schema import SceneDescription, SpatialConstraint
from .metadata_extractor import MetadataExtractor, AssetMetadata
from .shared_types import Pose, AABBBox

logger = logging.getLogger("mujoco_mcp.scene_gen.constraint_solver")

# Import enhanced collision detection for Phase 2A
try:
    from .enhanced_collision import GlobalCollisionOptimizer, RotationAwareAABB
    ENHANCED_COLLISION_AVAILABLE = True
    logger.info("Enhanced collision detection enabled (Phase 2A)")
except ImportError:
    ENHANCED_COLLISION_AVAILABLE = False
    logger.warning("Enhanced collision detection not available - using basic collision")

# Import advanced spatial reasoning for Phase 2B
try:
    from .spatial_reasoning import AdvancedSpatialReasoner
    SPATIAL_REASONING_AVAILABLE = True
    logger.info("Advanced spatial reasoning enabled (Phase 2B)")
except ImportError:
    SPATIAL_REASONING_AVAILABLE = False
    logger.warning("Advanced spatial reasoning not available")

# Import enhanced semantics for Phase 2D
try:
    from .enhanced_semantics import EnhancedAssetDatabase
    ENHANCED_SEMANTICS_AVAILABLE = True
    logger.info("Enhanced asset semantics enabled (Phase 2D)")
except ImportError:
    ENHANCED_SEMANTICS_AVAILABLE = False
    logger.warning("Enhanced asset semantics not available")

# Import robust solver for Phase 2E
try:
    from .robust_solver import RobustConstraintSolver
    ROBUST_SOLVER_AVAILABLE = True
    logger.info("Robust constraint solver enabled (Phase 2E)")
except ImportError:
    ROBUST_SOLVER_AVAILABLE = False
    logger.warning("Robust constraint solver not available")



    
    @classmethod
    def from_metadata(cls, metadata: AssetMetadata, pose: Pose) -> 'AABBBox':
        """Create AABB from asset metadata and pose."""
        # Use enhanced collision detection if available (Phase 2A)
        if ENHANCED_COLLISION_AVAILABLE:
            return RotationAwareAABB.from_metadata(metadata, pose)
        
        # Original implementation for backward compatibility
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
        
        # Initialize enhanced collision system if available (Phase 2A)
        if ENHANCED_COLLISION_AVAILABLE:
            self.collision_optimizer = GlobalCollisionOptimizer(metadata_extractor)
            self.use_enhanced_collision = True
            logger.info("Using enhanced collision detection and global optimization")
        else:
            self.collision_optimizer = None
            self.use_enhanced_collision = False
            logger.info("Using basic collision detection")
        
        # Initialize advanced spatial reasoning if available (Phase 2B)
        if SPATIAL_REASONING_AVAILABLE:
            self.spatial_reasoner = AdvancedSpatialReasoner(metadata_extractor)
            self.use_spatial_reasoning = True
            logger.info("Using advanced spatial reasoning for stable poses and reachability")
        else:
            self.spatial_reasoner = None
            self.use_spatial_reasoning = False
            logger.info("Using basic spatial reasoning")
        
        # Initialize enhanced semantics if available (Phase 2D)
        if ENHANCED_SEMANTICS_AVAILABLE:
            self.enhanced_asset_db = EnhancedAssetDatabase()
            self.use_enhanced_semantics = True
            logger.info("Using enhanced asset semantics for grasp sites and support surfaces")
        else:
            self.enhanced_asset_db = None
            self.use_enhanced_semantics = False
            logger.info("Using basic asset semantics")
        
        # Initialize robust solver if available (Phase 2E)
        if ROBUST_SOLVER_AVAILABLE:
            self.robust_solver = RobustConstraintSolver(metadata_extractor)
            self.use_robust_solver = True
            logger.info("Using robust constraint solver with backtracking and global optimization")
        else:
            self.robust_solver = None
            self.use_robust_solver = False
            logger.info("Using basic constraint solver")
    
    def solve(self, scene: SceneDescription) -> Dict[str, Pose]:
        """
        Solve all constraints in the scene and return entity poses.
        
        Phase 2E Enhancement: Automatically uses robust solver for complex constraint scenarios.
        
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
        
        # Phase 2E: Decide whether to use robust solver
        if self.use_robust_solver and self._should_use_robust_solver(scene):
            logger.info("Complex constraints detected - using robust solver with backtracking")
            robust_poses, solution_info = self.robust_solver.solve_robust(scene)
            
            # Log solution details
            logger.info(f"Robust solver result: score={solution_info.get('final_score', 0):.3f}, "
                       f"nodes_explored={solution_info.get('search_nodes_explored', 0)}, "
                       f"backtracks={solution_info.get('backtrack_events', 0)}")
            
            if solution_info.get('conflicts_detected'):
                logger.warning(f"Detected {len(solution_info['conflicts_detected'])} constraint conflicts")
                for conflict in solution_info['conflicts_detected']:
                    logger.warning(f"  - {conflict['type']}: {conflict['description']}")
            
            if robust_poses:
                return robust_poses
            else:
                logger.warning("Robust solver failed, falling back to basic solver")
        
        # Use original solving approach (Phase 1 implementation)
        return self._solve_basic(scene)
    
    def _should_use_robust_solver(self, scene: SceneDescription) -> bool:
        """Determine if robust solver should be used based on constraint complexity."""
        if not self.robust_solver:
            return False
        
        # Count total constraints and detect complexity indicators
        total_constraints = 0
        spatial_constraints = 0
        collision_constraints = 0
        multi_constraint_entities = 0
        
        for entity_id in scene.get_all_entity_ids():
            constraints = scene.get_constraints_for_entity(entity_id)
            total_constraints += len(constraints)
            
            entity_spatial = 0
            entity_collision = 0
            
            for constraint in constraints:
                if constraint.type in ["on_top_of", "in_front_of", "beside", "inside", "aligned_with_axis", "within_reach"]:
                    spatial_constraints += 1
                    entity_spatial += 1
                elif constraint.type == "no_collision":
                    collision_constraints += 1
                    entity_collision += 1
            
            if entity_spatial + entity_collision > 2:
                multi_constraint_entities += 1
        
        # Use robust solver for complex scenarios
        complexity_indicators = [
            total_constraints > 8,  # Many constraints overall
            multi_constraint_entities > 2,  # Multiple entities with many constraints
            collision_constraints > 3,  # Many collision constraints
            len(scene.get_all_entity_ids()) > 5  # Many entities
        ]
        
        use_robust = sum(complexity_indicators) >= 2
        
        if use_robust:
            logger.info(f"Triggering robust solver: constraints={total_constraints}, "
                       f"multi_constrained_entities={multi_constraint_entities}, "
                       f"collision_constraints={collision_constraints}")
        
        return use_robust
    
    def _solve_basic(self, scene: SceneDescription) -> Dict[str, Pose]:
        """
        Basic constraint solving approach (original implementation).
        """
        poses: Dict[str, Pose] = {}
        placed_entities: Set[str] = set()
        all_entities = scene.get_all_entity_ids()
        
        logger.info(f"Using basic constraint solver for {len(all_entities)} entities")
        
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
        if self.use_enhanced_collision and self.collision_optimizer:
            # Use enhanced global collision optimization (Phase 2A)
            logger.info("Applying enhanced collision resolution with global optimization")
            poses = self.collision_optimizer.optimize_poses(scene, poses)
        else:
            # Use original collision resolution for backward compatibility
            logger.info("Applying basic collision resolution")
            self._resolve_collisions(scene, poses)
        
        # Phase 4: Enhance placement with spatial reasoning (Phase 2B)
        if self.use_spatial_reasoning and self.spatial_reasoner:
            logger.info("Applying advanced spatial reasoning enhancements")
            poses = self._enhance_poses_with_spatial_reasoning(scene, poses)
        
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
        # Phase 2B: New constraint types
        elif constraint.type == "inside":
            return self._apply_inside(
                constraint, entity_metadata, reference_pose, reference_metadata
            )
        elif constraint.type == "aligned_with_axis":
            return self._apply_aligned_with_axis(
                constraint, entity_metadata, reference_pose, reference_metadata
            )
        elif constraint.type == "within_reach":
            return self._apply_within_reach(
                constraint, entity_metadata, reference_pose, reference_metadata, scene
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
    
    # Phase 2B: New constraint type implementations
    def _apply_inside(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata
    ) -> np.ndarray:
        """Apply inside constraint (entity placed inside reference container)."""
        # Place entity at center of reference container
        position = reference_pose.position.copy()
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _apply_aligned_with_axis(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata
    ) -> np.ndarray:
        """Apply aligned_with_axis constraint (entity aligned along an axis of reference)."""
        # For now, align along X-axis at same Y and Z
        position = reference_pose.position.copy()
        
        # Get reference dimensions to place entity next to reference
        ref_dims = reference_metadata.get_dimensions()
        entity_dims = entity_metadata.get_dimensions()
        
        # Place along X-axis with clearance
        position[0] += ref_dims.get('width', 0.1)/2 + entity_dims.get('width', 0.1)/2 + constraint.clearance
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _apply_within_reach(
        self, 
        constraint: SpatialConstraint,
        entity_metadata: AssetMetadata,
        reference_pose: Pose,
        reference_metadata: AssetMetadata,
        scene: SceneDescription
    ) -> np.ndarray:
        """Apply within_reach constraint (entity positioned within robot's reach)."""
        # If spatial reasoning is available, use reachability checker
        if self.use_spatial_reasoning and self.spatial_reasoner:
            # Find robot in scene
            robot_type = None
            for robot in scene.robots:
                if robot.robot_id == constraint.reference:
                    robot_type = robot.robot_type
                    break
            
            if robot_type:
                # Position entity within optimal reach distance
                reach_specs = self.spatial_reasoner.reachability_checker.robot_specs.get(robot_type, {})
                optimal_distance = reach_specs.get('reach_radius', 0.8) * 0.7  # 70% of max reach
                
                # Place in front of robot at optimal distance
                direction = np.array([1, 0, 0])  # Default forward direction
                position = reference_pose.position + direction * optimal_distance
                
                # Ensure reasonable height
                position[2] = max(position[2], reference_pose.position[2] + 0.3)
                
                return position
        
        # Fallback: place in front at reasonable distance
        position = reference_pose.position.copy()
        position[0] += 0.6  # 60cm in front
        position[2] += 0.3  # 30cm up
        
        # Apply optional offset
        if constraint.offset:
            position += np.array(constraint.offset)
        
        return position
    
    def _enhance_poses_with_spatial_reasoning(self, scene: SceneDescription, poses: Dict[str, Pose]) -> Dict[str, Pose]:
        """Enhance poses using advanced spatial reasoning (Phase 2B)."""
        enhanced_poses = poses.copy()
        
        # Enhance object placements with stable pose reasoning
        for obj in scene.objects:
            if obj.object_id not in enhanced_poses:
                continue
                
            # Find support surface for this object
            support_entity_id = None
            support_constraint = None
            
            for constraint in obj.constraints:
                if constraint.type == "on_top_of":
                    support_entity_id = constraint.reference
                    support_constraint = constraint
                    break
            
            if support_entity_id and support_entity_id in enhanced_poses:
                support_metadata = self._get_entity_metadata(support_entity_id, scene)
                support_pose = enhanced_poses[support_entity_id]
                
                # Use spatial reasoner to enhance placement
                enhanced_pose, stability_score = self.spatial_reasoner.enhance_object_placement(
                    obj, support_metadata, support_pose
                )
                
                # Apply any additional constraints (like offset)
                if support_constraint and support_constraint.offset:
                    enhanced_pose.position += np.array(support_constraint.offset)
                
                enhanced_poses[obj.object_id] = enhanced_pose
                logger.debug(f"Enhanced {obj.object_id} placement with stability score: {stability_score:.2f}")
        
        # Optimize robot base positions for reachability
        for robot in scene.robots:
            if robot.robot_id not in enhanced_poses:
                continue
            
            # Get manipulation targets (objects the robot should reach)
            target_positions = []
            for obj in scene.objects:
                if obj.object_id in enhanced_poses:
                    # Only consider small objects as manipulation targets
                    if "cup" in obj.object_type or "box" in obj.object_type:
                        target_positions.append(enhanced_poses[obj.object_id].position)
            
            if target_positions:
                # Find optimal base position
                optimal_pos, reachability_score = self.spatial_reasoner.reachability_checker.find_optimal_base_position(
                    robot.robot_type, target_positions, scene, enhanced_poses
                )
                
                # Update robot pose if significant improvement
                current_pose = enhanced_poses[robot.robot_id]
                if reachability_score > 0.7:  # Only update if good reachability
                    enhanced_poses[robot.robot_id] = Pose(
                        position=optimal_pos,
                        orientation=current_pose.orientation
                    )
                    logger.debug(f"Optimized {robot.robot_id} base position for reachability: {reachability_score:.2f}")
        
        # Validate workspace and log suggestions
        workspace_suggestions = self.spatial_reasoner.suggest_workspace_improvements(scene, enhanced_poses)
        if workspace_suggestions:
            logger.info("Workspace improvement suggestions:")
            for suggestion in workspace_suggestions:
                logger.info(f"  - {suggestion}")
        
        return enhanced_poses