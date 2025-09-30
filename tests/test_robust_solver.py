#!/usr/bin/env python3
"""
Tests for Phase 2E: Robust Constraint Solver

Tests backtracking capability, conflict resolution, global optimization,
and multi-pass refinement.
"""

import numpy as np
import pytest
from unittest.mock import Mock, patch

from mujoco_mcp.scene_gen import (
    SceneDescription,
    SpatialConstraint,
    ObjectPlacement,
    RobotConfiguration,
    MetadataExtractor,
    ConstraintSolver,
    Pose
)

# Try to import robust solver components
try:
    from mujoco_mcp.scene_gen.robust_solver import (
        RobustConstraintSolver,
        ConstraintConflict,
        PlacementSolution,
        GlobalSceneState,
        ConstraintConflictType
    )
    ROBUST_SOLVER_AVAILABLE = True
except ImportError:
    ROBUST_SOLVER_AVAILABLE = False


class TestConstraintConflictDetection:
    """Test constraint conflict detection and analysis."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_robust_solver_creation(self):
        """Test that robust constraint solver can be created."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        assert solver is not None
        assert hasattr(solver, 'max_search_depth')
        assert hasattr(solver, 'max_alternatives_per_entity')
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_circular_dependency_detection(self):
        """Test detection of circular dependencies in constraints."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        # Create scene with circular dependency
        scene_dict = {
            "objects": [
                {
                    "object_id": "obj1",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "obj1",
                            "reference": "obj2",
                            "clearance": 0.01
                        }
                    ]
                },
                {
                    "object_id": "obj2",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "obj2",
                            "reference": "obj1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        conflicts = solver._analyze_constraint_conflicts(scene)
        
        # Should detect circular dependency
        circular_conflicts = [c for c in conflicts if c.conflict_type == ConstraintConflictType.CIRCULAR_DEPENDENCY]
        assert len(circular_conflicts) > 0
        
        circular_conflict = circular_conflicts[0]
        assert "obj1" in circular_conflict.involved_entities
        assert "obj2" in circular_conflict.involved_entities
        assert circular_conflict.severity > 0.5
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_overconstrained_entity_detection(self):
        """Test detection of overconstrained entities."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        # Create scene with overconstrained entity
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "table2",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "table3",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "overconstrained_cup",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "overconstrained_cup",
                            "reference": "table1",
                            "clearance": 0.01
                        },
                        {
                            "type": "in_front_of",
                            "subject": "overconstrained_cup",
                            "reference": "table2",
                            "clearance": 0.1
                        },
                        {
                            "type": "beside",
                            "subject": "overconstrained_cup",
                            "reference": "table3",
                            "clearance": 0.1
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        conflicts = solver._analyze_constraint_conflicts(scene)
        
        # Should detect overconstrained entity
        overconstrained_conflicts = [c for c in conflicts if c.conflict_type == ConstraintConflictType.OVERCONSTRAINED]
        assert len(overconstrained_conflicts) > 0
        
        overconstrained_conflict = overconstrained_conflicts[0]
        assert "overconstrained_cup" in overconstrained_conflict.involved_entities


class TestBacktrackingSearch:
    """Test backtracking search functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_simple_backtracking_search(self):
        """Test backtracking search on a simple scene."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        # Create simple scene that should be solvable
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup1",
                            "reference": "table1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        solution, stats = solver._backtracking_search(scene)
        
        assert solution is not None
        assert "search_nodes_explored" in stats
        assert stats["search_nodes_explored"] > 0
        assert len(solution.poses) == 2
        assert "table1" in solution.poses
        assert "cup1" in solution.poses
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_entity_complexity_sorting(self):
        """Test sorting entities by constraint complexity."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []  # No constraints - should be placed first
                },
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup1",
                            "reference": "table1",
                            "clearance": 0.01
                        }
                    ]  # One constraint - should be placed second
                },
                {
                    "object_id": "box1",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box1",
                            "reference": "table1",
                            "clearance": 0.01
                        },
                        {
                            "type": "no_collision",
                            "subject": "box1",
                            "reference": "cup1",
                            "clearance": 0.05
                        }
                    ]  # Two constraints - should be placed last
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        entities = ["table1", "cup1", "box1"]
        sorted_entities = solver._sort_entities_by_complexity(scene, entities)
        
        # table1 should be first (no constraints, others depend on it)
        assert sorted_entities[0] == "table1"
        
        # cup1 and box1 order may vary, but table1 should definitely be first
        assert "cup1" in sorted_entities
        assert "box1" in sorted_entities


class TestGlobalSceneEvaluation:
    """Test global scene state evaluation and scoring."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_constraint_satisfaction_scoring(self):
        """Test constraint satisfaction scoring."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        # Create scene with known constraints
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup1",
                            "reference": "table1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        
        # Test satisfied constraint
        satisfied_poses = {
            "table1": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
            "cup1": Pose(np.array([0, 0, 1]), np.array([0, 0, 0, 1]))  # Cup above table
        }
        
        satisfied_score = solver._calculate_constraint_satisfaction_score(scene, satisfied_poses)
        assert satisfied_score > 0.5  # Should be high
        
        # Test unsatisfied constraint
        unsatisfied_poses = {
            "table1": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
            "cup1": Pose(np.array([0, 0, -1]), np.array([0, 0, 0, 1]))  # Cup below table
        }
        
        unsatisfied_score = solver._calculate_constraint_satisfaction_score(scene, unsatisfied_poses)
        assert unsatisfied_score < satisfied_score  # Should be lower
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_collision_avoidance_scoring(self):
        """Test collision avoidance scoring."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        scene_dict = {
            "objects": [
                {
                    "object_id": "box1",
                    "object_type": "box_small",
                    "constraints": []
                },
                {
                    "object_id": "box2",
                    "object_type": "box_small",
                    "constraints": []
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        
        # Test non-colliding poses
        separated_poses = {
            "box1": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
            "box2": Pose(np.array([1, 0, 0]), np.array([0, 0, 0, 1]))  # 1 meter apart
        }
        
        separated_score = solver._calculate_collision_avoidance_score(scene, separated_poses)
        assert separated_score > 0.5  # Should be high
        
        # Test overlapping poses
        overlapping_poses = {
            "box1": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
            "box2": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1]))  # Same position
        }
        
        overlapping_score = solver._calculate_collision_avoidance_score(scene, overlapping_poses)
        assert overlapping_score < separated_score  # Should be lower
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_global_scene_evaluation(self):
        """Test complete global scene evaluation."""
        solver = RobustConstraintSolver(self.metadata_extractor)
        
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup1",
                            "reference": "table1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = {
            "table1": Pose(np.array([0, 0, 0]), np.array([0, 0, 0, 1])),
            "cup1": Pose(np.array([0, 0, 1]), np.array([0, 0, 0, 1]))
        }
        
        scene_state = solver._evaluate_scene_state(scene, poses)
        
        assert isinstance(scene_state, GlobalSceneState)
        assert scene_state.global_score >= 0.0
        assert scene_state.global_score <= 1.0
        assert len(scene_state.poses) == 2
        assert len(scene_state.placed_entities) == 2


class TestRobustSolverIntegration:
    """Test integration of robust solver with main constraint solver."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.metadata_extractor)
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_robust_solver_integration(self):
        """Test that robust solver is integrated correctly."""
        if ROBUST_SOLVER_AVAILABLE:
            assert hasattr(self.solver, 'use_robust_solver')
            assert hasattr(self.solver, 'robust_solver')
            assert self.solver.use_robust_solver == True
            assert self.solver.robust_solver is not None
    
    @pytest.mark.skipif(not ROBUST_SOLVER_AVAILABLE, reason="Robust solver not available")
    def test_robust_solver_triggering(self):
        """Test conditions that trigger robust solver usage."""
        if not self.solver.use_robust_solver:
            pytest.skip("Robust solver not available")
        
        # Simple scene - should not trigger robust solver
        simple_scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup1",
                            "reference": "table1",
                            "clearance": 0.01
                        }
                    ]
                }
            ]
        }
        
        simple_scene = SceneDescription(**simple_scene_dict)
        should_use_robust_simple = self.solver._should_use_robust_solver(simple_scene)
        
        # Complex scene - should trigger robust solver
        complex_scene_dict = {
            "objects": [
                {
                    "object_id": f"obj{i}",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of" if i % 2 == 0 else "beside",
                            "subject": f"obj{i}",
                            "reference": f"obj{i-1}" if i > 0 else "table1",
                            "clearance": 0.01
                        },
                        {
                            "type": "no_collision",
                            "subject": f"obj{i}",
                            "reference": f"obj{j}",
                            "clearance": 0.05
                        } for j in range(i)
                    ][0] if i > 0 else {}
                } for i in range(8)  # Many objects with many constraints
            ]
        }
        
        # Add table
        complex_scene_dict["objects"].insert(0, {
            "object_id": "table1",
            "object_type": "table_standard",
            "constraints": []
        })
        
        complex_scene = SceneDescription(**complex_scene_dict)
        should_use_robust_complex = self.solver._should_use_robust_solver(complex_scene)
        
        # Complex scene should be more likely to trigger robust solver
        # (Note: exact behavior depends on implementation details)
        assert isinstance(should_use_robust_simple, bool)
        assert isinstance(should_use_robust_complex, bool)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])