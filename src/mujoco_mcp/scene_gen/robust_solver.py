#!/usr/bin/env python3
"""
Robust Constraint Solver

Implements Phase 2E of the enhanced scene generation system:
1. Backtracking capability for conflict resolution
2. Multi-pass refinement and global scoring
3. Support for complex constraint combinations

This addresses the limitation in PR #12 of the greedy constraint solver
that can get stuck in local minima and doesn't handle conflicting constraints well.
"""

import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Set, Any
from dataclasses import dataclass, field
from enum import Enum
import copy

from .shared_types import Pose, AABBBox
from .scene_schema import SceneDescription, SpatialConstraint
from .metadata_extractor import MetadataExtractor

logger = logging.getLogger("mujoco_mcp.scene_gen.robust_solver")


class ConstraintConflictType(Enum):
    """Types of constraint conflicts."""
    SPATIAL_IMPOSSIBLE = "spatial_impossible"  # Physically impossible to satisfy
    OVERCONSTRAINED = "overconstrained"        # Too many conflicting constraints
    UNDER_CONSTRAINED = "under_constrained"    # Ambiguous placement
    CIRCULAR_DEPENDENCY = "circular_dependency" # Circular constraint dependencies


@dataclass
class ConstraintConflict:
    """Represents a constraint conflict."""
    conflict_type: ConstraintConflictType
    involved_entities: List[str]
    conflicting_constraints: List[SpatialConstraint]
    severity: float  # 0.0 to 1.0, higher is more severe
    description: str
    suggested_resolution: Optional[str] = None


@dataclass
class PlacementSolution:
    """Represents a potential solution for entity placement."""
    entity_id: str
    pose: Pose
    constraint_satisfaction_score: float  # 0.0 to 1.0
    conflict_cost: float  # 0.0 to 1.0, lower is better
    alternative_poses: List[Pose] = field(default_factory=list)
    
    @property
    def total_score(self) -> float:
        """Calculate total score (higher is better)."""
        return self.constraint_satisfaction_score - self.conflict_cost


@dataclass
class GlobalSceneState:
    """Represents the global state of scene placement."""
    poses: Dict[str, Pose]
    placed_entities: Set[str]
    global_score: float
    constraint_violations: List[ConstraintConflict]
    iteration: int


class BacktrackingSearchNode:
    """Node in the backtracking search tree."""
    
    def __init__(self, entity_id: str, pose: Pose, parent: Optional['BacktrackingSearchNode'] = None):
        self.entity_id = entity_id
        self.pose = pose
        self.parent = parent
        self.children: List['BacktrackingSearchNode'] = []
        self.score = 0.0
        self.depth = parent.depth + 1 if parent else 0
    
    def get_path_poses(self) -> Dict[str, Pose]:
        """Get all poses from root to this node."""
        poses = {}
        node = self
        while node:
            poses[node.entity_id] = node.pose
            node = node.parent
        return poses
    
    def add_child(self, child: 'BacktrackingSearchNode'):
        """Add a child node."""
        child.parent = self
        child.depth = self.depth + 1
        self.children.append(child)


class RobustConstraintSolver:
    """
    Enhanced constraint solver with backtracking, global optimization, and conflict resolution.
    
    This solver addresses the limitations of the greedy approach by:
    1. Using backtracking search to explore alternative solutions
    2. Global scoring to find optimal solutions rather than first-feasible
    3. Sophisticated conflict detection and resolution
    4. Multi-pass refinement for quality improvement
    """
    
    def __init__(self, metadata_extractor: MetadataExtractor):
        self.metadata_extractor = metadata_extractor
        
        # Search parameters
        self.max_search_depth = 10
        self.max_alternatives_per_entity = 5
        self.beam_search_width = 3
        self.max_backtrack_iterations = 50
        
        # Scoring weights
        self.constraint_satisfaction_weight = 0.4
        self.collision_avoidance_weight = 0.3
        self.spatial_quality_weight = 0.2
        self.efficiency_weight = 0.1
        
        # Convergence criteria
        self.min_score_improvement = 0.01
        self.max_refinement_passes = 5
    
    def solve_robust(self, scene: SceneDescription) -> Tuple[Dict[str, Pose], Dict[str, Any]]:
        """
        Solve scene constraints using robust backtracking and global optimization.
        
        Returns:
            (poses, solution_info) where solution_info contains detailed metrics
        """
        logger.info(f"Starting robust constraint solving for {len(scene.get_all_entity_ids())} entities")
        
        solution_info = {
            "algorithm": "robust_backtracking",
            "search_nodes_explored": 0,
            "backtrack_events": 0,
            "refinement_passes": 0,
            "conflicts_detected": [],
            "final_score": 0.0,
            "convergence_reason": "unknown"
        }
        
        # Phase 1: Analyze constraints and detect potential conflicts
        conflicts = self._analyze_constraint_conflicts(scene)
        solution_info["conflicts_detected"] = [
            {
                "type": conflict.conflict_type.value,
                "entities": conflict.involved_entities,
                "severity": conflict.severity,
                "description": conflict.description
            } for conflict in conflicts
        ]
        
        # Phase 2: Generate initial solution using backtracking search
        best_solution, search_stats = self._backtracking_search(scene)
        solution_info.update(search_stats)
        
        if best_solution is None:
            logger.error("Failed to find any valid solution")
            return {}, solution_info
        
        # Phase 3: Multi-pass refinement
        refined_solution, refinement_stats = self._multi_pass_refinement(scene, best_solution)
        solution_info.update(refinement_stats)
        
        final_poses = {entity_id: pose for entity_id, pose in refined_solution.poses.items()}
        solution_info["final_score"] = refined_solution.global_score
        
        logger.info(f"Robust solving completed with score: {refined_solution.global_score:.3f}")
        
        return final_poses, solution_info
    
    def _analyze_constraint_conflicts(self, scene: SceneDescription) -> List[ConstraintConflict]:
        """Analyze scene constraints to detect potential conflicts."""
        conflicts = []
        all_entities = scene.get_all_entity_ids()
        
        # Check for circular dependencies
        dependency_graph = {}
        for entity_id in all_entities:
            constraints = scene.get_constraints_for_entity(entity_id)
            dependencies = []
            for constraint in constraints:
                if constraint.type != "no_collision":  # no_collision is not a dependency
                    dependencies.append(constraint.reference)
            dependency_graph[entity_id] = dependencies
        
        circular_deps = self._detect_circular_dependencies(dependency_graph)
        for cycle in circular_deps:
            conflicts.append(ConstraintConflict(
                conflict_type=ConstraintConflictType.CIRCULAR_DEPENDENCY,
                involved_entities=cycle,
                conflicting_constraints=[],
                severity=1.0,
                description=f"Circular dependency detected: {' -> '.join(cycle)}",
                suggested_resolution="Break circular dependency by removing one constraint"
            ))
        
        # Check for overconstrained entities
        for entity_id in all_entities:
            constraints = scene.get_constraints_for_entity(entity_id)
            spatial_constraints = [c for c in constraints if c.type in ["on_top_of", "in_front_of", "beside"]]
            
            if len(spatial_constraints) > 2:
                conflicts.append(ConstraintConflict(
                    conflict_type=ConstraintConflictType.OVERCONSTRAINED,
                    involved_entities=[entity_id],
                    conflicting_constraints=spatial_constraints,
                    severity=0.7,
                    description=f"Entity {entity_id} has {len(spatial_constraints)} spatial constraints",
                    suggested_resolution="Consider reducing number of spatial constraints"
                ))
        
        # Check for spatial impossibilities (simplified heuristic)
        for entity_id in all_entities:
            constraints = scene.get_constraints_for_entity(entity_id)
            for i, constraint1 in enumerate(constraints):
                for constraint2 in constraints[i+1:]:
                    if self._constraints_potentially_conflict(constraint1, constraint2):
                        conflicts.append(ConstraintConflict(
                            conflict_type=ConstraintConflictType.SPATIAL_IMPOSSIBLE,
                            involved_entities=[entity_id, constraint1.reference, constraint2.reference],
                            conflicting_constraints=[constraint1, constraint2],
                            severity=0.8,
                            description=f"Potentially conflicting constraints for {entity_id}",
                            suggested_resolution="Review constraint combination for feasibility"
                        ))
        
        logger.info(f"Detected {len(conflicts)} potential constraint conflicts")
        return conflicts
    
    def _detect_circular_dependencies(self, graph: Dict[str, List[str]]) -> List[List[str]]:
        """Detect circular dependencies in constraint graph."""
        cycles = []
        visited = set()
        rec_stack = set()
        
        def dfs(node, path):
            if node in rec_stack:
                cycle_start = path.index(node)
                cycles.append(path[cycle_start:] + [node])
                return
            
            if node in visited:
                return
            
            visited.add(node)
            rec_stack.add(node)
            path.append(node)
            
            for neighbor in graph.get(node, []):
                if neighbor in graph:  # Only follow if neighbor has outgoing edges
                    dfs(neighbor, path)
            
            rec_stack.remove(node)
            path.pop()
        
        for node in graph:
            if node not in visited:
                dfs(node, [])
        
        return cycles
    
    def _constraints_potentially_conflict(self, c1: SpatialConstraint, c2: SpatialConstraint) -> bool:
        """Check if two constraints potentially conflict."""
        # Simplified conflict detection - could be enhanced
        if c1.reference == c2.reference:
            # Two different spatial relationships to same reference might conflict
            spatial_types = {"on_top_of", "in_front_of", "beside"}
            if c1.type in spatial_types and c2.type in spatial_types and c1.type != c2.type:
                return True
        return False
    
    def _backtracking_search(self, scene: SceneDescription) -> Tuple[Optional[GlobalSceneState], Dict[str, Any]]:
        """Perform backtracking search to find valid scene layout."""
        stats = {
            "search_nodes_explored": 0,
            "backtrack_events": 0,
            "max_depth_reached": 0
        }
        
        all_entities = scene.get_all_entity_ids()
        root_node = BacktrackingSearchNode("root", Pose(np.zeros(3), np.array([0, 0, 0, 1])))
        
        best_solution = None
        best_score = -float('inf')
        
        # Sort entities by constraint complexity (place simpler ones first)
        sorted_entities = self._sort_entities_by_complexity(scene, all_entities)
        
        def search(node: BacktrackingSearchNode, remaining_entities: List[str], 
                  current_poses: Dict[str, Pose]) -> bool:
            nonlocal best_solution, best_score
            stats["search_nodes_explored"] += 1
            stats["max_depth_reached"] = max(stats["max_depth_reached"], node.depth)
            
            if not remaining_entities:
                # Complete solution found
                scene_state = self._evaluate_scene_state(scene, current_poses)
                if scene_state.global_score > best_score:
                    best_score = scene_state.global_score
                    best_solution = scene_state
                return True
            
            if node.depth >= self.max_search_depth:
                return False
            
            entity_id = remaining_entities[0]
            remaining = remaining_entities[1:]
            
            # Generate alternative poses for this entity
            alternative_poses = self._generate_alternative_poses(entity_id, scene, current_poses)
            
            # Try each alternative
            for pose in alternative_poses[:self.max_alternatives_per_entity]:
                new_poses = current_poses.copy()
                new_poses[entity_id] = pose
                
                # Quick feasibility check
                if self._is_placement_feasible(entity_id, pose, scene, new_poses):
                    child_node = BacktrackingSearchNode(entity_id, pose, node)
                    node.add_child(child_node)
                    
                    # Recursive search
                    if search(child_node, remaining, new_poses):
                        # Continue searching for better solutions
                        pass
                    else:
                        stats["backtrack_events"] += 1
            
            return best_solution is not None
        
        # Start search
        search(root_node, sorted_entities, {})
        
        return best_solution, stats
    
    def _sort_entities_by_complexity(self, scene: SceneDescription, entities: List[str]) -> List[str]:
        """Sort entities by constraint complexity (simpler first)."""
        def complexity_score(entity_id):
            constraints = scene.get_constraints_for_entity(entity_id)
            # Entities with fewer constraints are simpler
            # Entities that others depend on should be placed first
            dependency_count = sum(1 for other_entity in entities 
                                 for other_constraint in scene.get_constraints_for_entity(other_entity)
                                 if other_constraint.reference == entity_id)
            return len(constraints) - dependency_count * 2  # Prioritize dependencies
        
        return sorted(entities, key=complexity_score)
    
    def _generate_alternative_poses(self, entity_id: str, scene: SceneDescription, 
                                   existing_poses: Dict[str, Pose]) -> List[Pose]:
        """Generate alternative poses for an entity."""
        alternatives = []
        
        # Get entity constraints
        constraints = scene.get_constraints_for_entity(entity_id)
        
        if not constraints:
            # Unconstrained entity - generate grid of poses
            for x in np.linspace(-1, 1, 3):
                for y in np.linspace(-1, 1, 3):
                    alternatives.append(Pose(
                        position=np.array([x, y, 0]),
                        orientation=np.array([0, 0, 0, 1])
                    ))
        else:
            # Constrained entity - generate poses based on constraints
            base_pose = self._compute_constraint_based_pose(entity_id, constraints, existing_poses, scene)
            if base_pose:
                alternatives.append(base_pose)
                
                # Generate variations around base pose
                for dx in [-0.1, 0, 0.1]:
                    for dy in [-0.1, 0, 0.1]:
                        if dx == 0 and dy == 0:
                            continue
                        variation = Pose(
                            position=base_pose.position + np.array([dx, dy, 0]),
                            orientation=base_pose.orientation
                        )
                        alternatives.append(variation)
        
        return alternatives
    
    def _compute_constraint_based_pose(self, entity_id: str, constraints: List[SpatialConstraint],
                                     existing_poses: Dict[str, Pose], scene: SceneDescription) -> Optional[Pose]:
        """Compute pose based on constraints (simplified version)."""
        # This is a simplified version - would use the existing constraint application logic
        # from the original constraint solver
        
        position = np.array([0.0, 0.0, 0.0])
        orientation = np.array([0.0, 0.0, 0.0, 1.0])
        
        for constraint in constraints:
            if constraint.reference in existing_poses:
                ref_pose = existing_poses[constraint.reference]
                
                if constraint.type == "on_top_of":
                    position = ref_pose.position + np.array([0, 0, 0.1])
                elif constraint.type == "in_front_of":
                    position = ref_pose.position + np.array([0.5, 0, 0])
                elif constraint.type == "beside":
                    position = ref_pose.position + np.array([0, 0.5, 0])
        
        return Pose(position=position, orientation=orientation)
    
    def _is_placement_feasible(self, entity_id: str, pose: Pose, scene: SceneDescription,
                             current_poses: Dict[str, Pose]) -> bool:
        """Quick feasibility check for a placement."""
        # Check basic spatial bounds
        if pose.position[2] < 0:  # Below ground
            return False
        
        if np.linalg.norm(pose.position) > 10:  # Too far from origin
            return False
        
        # Check for major collisions (simplified)
        entity_metadata = self._get_entity_metadata(entity_id, scene)
        if entity_metadata:
            entity_aabb = AABBBox.from_metadata(entity_metadata, pose)
            
            for other_id, other_pose in current_poses.items():
                if other_id != entity_id:
                    other_metadata = self._get_entity_metadata(other_id, scene)
                    if other_metadata:
                        other_aabb = AABBBox.from_metadata(other_metadata, other_pose)
                        if entity_aabb.overlaps(other_aabb):
                            return False
        
        return True
    
    def _evaluate_scene_state(self, scene: SceneDescription, poses: Dict[str, Pose]) -> GlobalSceneState:
        """Evaluate the quality of a complete scene state."""
        # Calculate component scores
        constraint_score = self._calculate_constraint_satisfaction_score(scene, poses)
        collision_score = self._calculate_collision_avoidance_score(scene, poses)
        spatial_score = self._calculate_spatial_quality_score(scene, poses)
        efficiency_score = self._calculate_efficiency_score(scene, poses)
        
        # Weighted global score
        global_score = (
            constraint_score * self.constraint_satisfaction_weight +
            collision_score * self.collision_avoidance_weight +
            spatial_score * self.spatial_quality_weight +
            efficiency_score * self.efficiency_weight
        )
        
        # Detect any remaining violations
        violations = self._detect_constraint_violations(scene, poses)
        
        return GlobalSceneState(
            poses=poses,
            placed_entities=set(poses.keys()),
            global_score=global_score,
            constraint_violations=violations,
            iteration=0
        )
    
    def _calculate_constraint_satisfaction_score(self, scene: SceneDescription, poses: Dict[str, Pose]) -> float:
        """Calculate how well constraints are satisfied."""
        total_constraints = 0
        satisfied_constraints = 0
        
        for entity_id in scene.get_all_entity_ids():
            constraints = scene.get_constraints_for_entity(entity_id)
            for constraint in constraints:
                total_constraints += 1
                
                if self._is_constraint_satisfied(constraint, poses, scene):
                    satisfied_constraints += 1
        
        return satisfied_constraints / max(total_constraints, 1)
    
    def _is_constraint_satisfied(self, constraint: SpatialConstraint, poses: Dict[str, Pose], 
                               scene: SceneDescription) -> bool:
        """Check if a specific constraint is satisfied."""
        if constraint.subject not in poses or constraint.reference not in poses:
            return False
        
        subject_pose = poses[constraint.subject]
        reference_pose = poses[constraint.reference]
        
        if constraint.type == "on_top_of":
            return subject_pose.position[2] > reference_pose.position[2]
        elif constraint.type == "in_front_of":
            return subject_pose.position[0] > reference_pose.position[0]
        elif constraint.type == "beside":
            return abs(subject_pose.position[1] - reference_pose.position[1]) > constraint.clearance
        elif constraint.type == "no_collision":
            # Check for collision using AABB
            subject_metadata = self._get_entity_metadata(constraint.subject, scene)
            reference_metadata = self._get_entity_metadata(constraint.reference, scene)
            if subject_metadata and reference_metadata:
                subject_aabb = AABBBox.from_metadata(subject_metadata, subject_pose)
                reference_aabb = AABBBox.from_metadata(reference_metadata, reference_pose)
                return not subject_aabb.overlaps(reference_aabb)
        
        return True
    
    def _calculate_collision_avoidance_score(self, scene: SceneDescription, poses: Dict[str, Pose]) -> float:
        """Calculate collision avoidance score."""
        total_pairs = 0
        collision_free_pairs = 0
        
        entities = list(poses.keys())
        for i, entity1 in enumerate(entities):
            for entity2 in entities[i+1:]:
                total_pairs += 1
                
                # Check collision using AABB
                metadata1 = self._get_entity_metadata(entity1, scene)
                metadata2 = self._get_entity_metadata(entity2, scene)
                
                if metadata1 and metadata2:
                    aabb1 = AABBBox.from_metadata(metadata1, poses[entity1])
                    aabb2 = AABBBox.from_metadata(metadata2, poses[entity2])
                    
                    if not aabb1.overlaps(aabb2):
                        collision_free_pairs += 1
                else:
                    collision_free_pairs += 1  # Assume no collision if metadata missing
        
        return collision_free_pairs / max(total_pairs, 1)
    
    def _calculate_spatial_quality_score(self, scene: SceneDescription, poses: Dict[str, Pose]) -> float:
        """Calculate spatial arrangement quality."""
        # Heuristics for good spatial arrangement
        total_score = 0.0
        count = 0
        
        for entity_id, pose in poses.items():
            # Preference for reasonable heights
            if 0 <= pose.position[2] <= 2.0:
                total_score += 1.0
            count += 1
            
            # Preference for entities being reasonably close to origin
            distance_from_origin = np.linalg.norm(pose.position[:2])
            if distance_from_origin <= 2.0:
                total_score += 1.0 - (distance_from_origin / 2.0)
            count += 1
        
        return total_score / max(count, 1)
    
    def _calculate_efficiency_score(self, scene: SceneDescription, poses: Dict[str, Pose]) -> float:
        """Calculate layout efficiency (compactness, accessibility, etc.)."""
        if len(poses) < 2:
            return 1.0
        
        # Calculate compactness (prefer tighter groupings)
        positions = np.array([pose.position for pose in poses.values()])
        centroid = np.mean(positions, axis=0)
        distances = [np.linalg.norm(pos - centroid) for pos in positions]
        avg_distance = np.mean(distances)
        
        # Normalize and invert (smaller distances = higher efficiency)
        compactness_score = max(0, 1.0 - avg_distance / 2.0)
        
        return compactness_score
    
    def _detect_constraint_violations(self, scene: SceneDescription, poses: Dict[str, Pose]) -> List[ConstraintConflict]:
        """Detect any remaining constraint violations."""
        violations = []
        
        for entity_id in scene.get_all_entity_ids():
            constraints = scene.get_constraints_for_entity(entity_id)
            for constraint in constraints:
                if not self._is_constraint_satisfied(constraint, poses, scene):
                    violations.append(ConstraintConflict(
                        conflict_type=ConstraintConflictType.SPATIAL_IMPOSSIBLE,
                        involved_entities=[constraint.subject, constraint.reference],
                        conflicting_constraints=[constraint],
                        severity=0.5,
                        description=f"Constraint {constraint.type} not satisfied for {constraint.subject}"
                    ))
        
        return violations
    
    def _multi_pass_refinement(self, scene: SceneDescription, 
                             initial_solution: GlobalSceneState) -> Tuple[GlobalSceneState, Dict[str, Any]]:
        """Perform multi-pass refinement to improve solution quality."""
        refinement_stats = {
            "refinement_passes": 0,
            "score_improvements": [],
            "convergence_reason": "max_passes"
        }
        
        current_solution = initial_solution
        
        for pass_num in range(self.max_refinement_passes):
            refinement_stats["refinement_passes"] += 1
            
            # Try local improvements
            improved_solution = self._local_improvement_pass(scene, current_solution)
            
            score_improvement = improved_solution.global_score - current_solution.global_score
            refinement_stats["score_improvements"].append(score_improvement)
            
            if score_improvement < self.min_score_improvement:
                refinement_stats["convergence_reason"] = "score_converged"
                break
            
            current_solution = improved_solution
            logger.debug(f"Refinement pass {pass_num + 1}: score improved by {score_improvement:.4f}")
        
        return current_solution, refinement_stats
    
    def _local_improvement_pass(self, scene: SceneDescription, 
                               current_solution: GlobalSceneState) -> GlobalSceneState:
        """Perform local improvements on current solution."""
        best_solution = current_solution
        
        # Try small adjustments to each entity
        for entity_id in current_solution.poses:
            original_pose = current_solution.poses[entity_id]
            
            # Try small position adjustments
            for dx, dy, dz in [(-0.05, 0, 0), (0.05, 0, 0), (0, -0.05, 0), (0, 0.05, 0), (0, 0, -0.01), (0, 0, 0.01)]:
                adjusted_pose = Pose(
                    position=original_pose.position + np.array([dx, dy, dz]),
                    orientation=original_pose.orientation
                )
                
                # Test this adjustment
                test_poses = current_solution.poses.copy()
                test_poses[entity_id] = adjusted_pose
                
                if self._is_placement_feasible(entity_id, adjusted_pose, scene, test_poses):
                    test_solution = self._evaluate_scene_state(scene, test_poses)
                    
                    if test_solution.global_score > best_solution.global_score:
                        best_solution = test_solution
        
        return best_solution
    
    def _get_entity_metadata(self, entity_id: str, scene: SceneDescription):
        """Get metadata for an entity (helper method)."""
        # Check objects
        for obj in scene.objects:
            if obj.object_id == entity_id:
                return self.metadata_extractor.get_metadata(obj.object_type)
        
        # Check robots
        for robot in scene.robots:
            if robot.robot_id == entity_id:
                return self.metadata_extractor.get_metadata(robot.robot_type)
        
        return None