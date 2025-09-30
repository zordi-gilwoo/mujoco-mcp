#!/usr/bin/env python3
"""
Tests for Phase 2B: Advanced Spatial Reasoning

Tests stable pose enumeration, workspace validation, robot reachability,
and enhanced constraint types.
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

# Try to import spatial reasoning components
try:
    from mujoco_mcp.scene_gen.spatial_reasoning import (
        AdvancedSpatialReasoner,
        StablePoseDatabase,
        RobotReachabilityChecker,
        StablePose,
        WorkspaceVolume,
        SupportSurface
    )
    SPATIAL_REASONING_AVAILABLE = True
except ImportError:
    SPATIAL_REASONING_AVAILABLE = False


class TestStablePoseDatabase:
    """Test stable pose database functionality."""
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_stable_pose_database_creation(self):
        """Test that stable pose database can be created and has default poses."""
        db = StablePoseDatabase()
        assert db is not None
        assert len(db.stable_poses) > 0
        
        # Should have poses for common objects
        assert "table_standard" in db.stable_poses
        assert "cup_ceramic_small" in db.stable_poses
        assert "box_small" in db.stable_poses
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_stable_pose_retrieval(self):
        """Test retrieval of stable poses with filtering."""
        db = StablePoseDatabase()
        
        # Get all poses for cup
        cup_poses = db.get_stable_poses("cup_ceramic_small")
        assert len(cup_poses) > 0
        assert all(isinstance(pose, StablePose) for pose in cup_poses)
        
        # Filter by support surface
        top_poses = db.get_stable_poses("cup_ceramic_small", support_surface=SupportSurface.BOTTOM)
        assert len(top_poses) >= 1
        assert all(pose.support_surface == SupportSurface.BOTTOM for pose in top_poses)
        
        # Filter by minimum stability
        stable_poses = db.get_stable_poses("cup_ceramic_small", min_stability=0.8)
        assert all(pose.stability_score >= 0.8 for pose in stable_poses)
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_stable_pose_sampling(self):
        """Test stable pose sampling with weighting."""
        db = StablePoseDatabase()
        
        # Sample poses multiple times
        samples = []
        for _ in range(10):
            pose = db.sample_stable_pose("cup_ceramic_small", min_stability=0.5)
            if pose:
                samples.append(pose)
        
        assert len(samples) > 0
        assert all(isinstance(pose, StablePose) for pose in samples)
        assert all(pose.stability_score >= 0.5 for pose in samples)
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_add_custom_stable_pose(self):
        """Test adding custom stable poses."""
        db = StablePoseDatabase()
        
        custom_pose = StablePose(
            object_type="custom_object",
            support_surface=SupportSurface.TOP,
            relative_position=np.array([0, 0, 0]),
            relative_orientation=np.array([0, 0, 0, 1]),
            stability_score=0.95,
            contact_area=0.1,
            description="Custom test pose"
        )
        
        db.add_stable_pose(custom_pose)
        
        # Should be able to retrieve it
        poses = db.get_stable_poses("custom_object")
        assert len(poses) == 1
        assert poses[0].description == "Custom test pose"


class TestWorkspaceVolume:
    """Test workspace volume functionality."""
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_workspace_volume_creation(self):
        """Test workspace volume creation and basic properties."""
        workspace = WorkspaceVolume(
            name="test_workspace",
            center=np.array([0.5, 0, 0.5]),
            extents=np.array([1.0, 1.0, 1.0]),
            orientation=np.array([0, 0, 0, 1]),
            accessibility_score=0.9,
            preferred_approach_directions=[np.array([1, 0, 0])]
        )
        
        assert workspace.name == "test_workspace"
        assert workspace.get_volume() == 1.0  # 1x1x1 cube
        np.testing.assert_array_equal(workspace.center, [0.5, 0, 0.5])
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_workspace_point_containment(self):
        """Test point containment checking."""
        workspace = WorkspaceVolume(
            name="test_workspace",
            center=np.array([0, 0, 0]),
            extents=np.array([2.0, 2.0, 2.0]),
            orientation=np.array([0, 0, 0, 1]),
            accessibility_score=1.0,
            preferred_approach_directions=[]
        )
        
        # Point inside workspace
        assert workspace.contains_point(np.array([0.5, 0.5, 0.5])) == True
        
        # Point outside workspace
        assert workspace.contains_point(np.array([2.0, 2.0, 2.0])) == False
        
        # Point on boundary
        assert workspace.contains_point(np.array([1.0, 0, 0])) == True


class TestRobotReachabilityChecker:
    """Test robot reachability checking."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_reachability_checker_creation(self):
        """Test that reachability checker can be created."""
        checker = RobotReachabilityChecker(self.metadata_extractor)
        assert checker is not None
        assert hasattr(checker, 'robot_specs')
        assert "franka_panda" in checker.robot_specs
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_reachability_checking(self):
        """Test basic reachability checking."""
        checker = RobotReachabilityChecker(self.metadata_extractor)
        
        robot_pose = Pose(
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1])
        )
        
        # Target within reach
        close_target = np.array([0.5, 0, 0.5])
        reachable, score = checker.check_reachability("franka_panda", robot_pose, close_target)
        assert reachable == True
        assert score > 0.0
        
        # Target too far
        far_target = np.array([2.0, 0, 0.5])
        reachable, score = checker.check_reachability("franka_panda", robot_pose, far_target)
        assert reachable == False
        assert score == 0.0
        
        # Target at unreachable height
        high_target = np.array([0.5, 0, 2.0])
        reachable, score = checker.check_reachability("franka_panda", robot_pose, high_target)
        assert reachable == False
        assert score == 0.0
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_optimal_base_position_finding(self):
        """Test finding optimal robot base position."""
        checker = RobotReachabilityChecker(self.metadata_extractor)
        
        # Create simple scene
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []
                }
            ]
        }
        scene = SceneDescription(**scene_dict)
        
        # Existing poses (just the table)
        existing_poses = {
            "table1": Pose(
                position=np.array([1, 0, 0]),
                orientation=np.array([0, 0, 0, 1])
            )
        }
        
        # Targets on the table
        targets = [np.array([1, 0, 0.8]), np.array([1.2, 0, 0.8])]
        
        optimal_pos, score = checker.find_optimal_base_position(
            "franka_panda", targets, scene, existing_poses
        )
        
        assert optimal_pos is not None
        assert len(optimal_pos) == 3
        assert score >= 0.0


class TestAdvancedSpatialReasoner:
    """Test the main advanced spatial reasoner."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_spatial_reasoner_creation(self):
        """Test that spatial reasoner can be created."""
        reasoner = AdvancedSpatialReasoner(self.metadata_extractor)
        assert reasoner is not None
        assert hasattr(reasoner, 'stable_pose_db')
        assert hasattr(reasoner, 'reachability_checker')
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_enhanced_object_placement(self):
        """Test enhanced object placement with stable poses."""
        reasoner = AdvancedSpatialReasoner(self.metadata_extractor)
        
        # Object to place
        obj = ObjectPlacement(
            object_id="test_cup",
            object_type="cup_ceramic_small",
            constraints=[]
        )
        
        # Support surface (table)
        support_metadata = self.metadata_extractor.get_metadata("table_standard")
        support_pose = Pose(
            position=np.array([0, 0, 0]),
            orientation=np.array([0, 0, 0, 1])
        )
        
        enhanced_pose, stability_score = reasoner.enhance_object_placement(
            obj, support_metadata, support_pose
        )
        
        assert enhanced_pose is not None
        assert isinstance(enhanced_pose, Pose)
        assert stability_score >= 0.0
        assert stability_score <= 1.0
        
        # Enhanced pose should be above the support surface
        assert enhanced_pose.position[2] > support_pose.position[2]
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_workspace_validation(self):
        """Test workspace validation for robots."""
        reasoner = AdvancedSpatialReasoner(self.metadata_extractor)
        
        # Create scene with robot and target objects
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
                    "constraints": []
                }
            ],
            "robots": [
                {
                    "robot_id": "robot1",
                    "robot_type": "franka_panda",
                    "joint_config": "ready",
                    "constraints": []
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        
        # Poses with robot close to targets
        poses = {
            "table1": Pose(np.array([1, 0, 0]), np.array([0, 0, 0, 1])),
            "cup1": Pose(np.array([1, 0, 0.8]), np.array([0, 0, 0, 1])),
            "robot1": Pose(np.array([0.5, 0, 0]), np.array([0, 0, 0, 1]))
        }
        
        robot_config = scene.robots[0]
        is_valid, score, suggestions = reasoner.validate_robot_workspace(
            robot_config, scene, poses
        )
        
        assert isinstance(is_valid, bool)
        assert 0.0 <= score <= 1.0
        assert isinstance(suggestions, list)


class TestEnhancedConstraintTypes:
    """Test new constraint types in Phase 2B."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.metadata_extractor)
    
    def test_new_constraint_type_validation(self):
        """Test that new constraint types are accepted in schema validation."""
        # Test new constraint types
        new_constraint_types = ['inside', 'aligned_with_axis', 'oriented_towards', 'within_reach']
        
        for constraint_type in new_constraint_types:
            constraint = SpatialConstraint(
                type=constraint_type,
                subject="test_subject",
                reference="test_reference",
                clearance=0.01
            )
            assert constraint.type == constraint_type
    
    def test_within_reach_constraint_solving(self):
        """Test solving scenes with within_reach constraints."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "cup1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "within_reach",
                            "subject": "cup1",
                            "reference": "robot1",
                            "clearance": 0.1
                        }
                    ]
                }
            ],
            "robots": [
                {
                    "robot_id": "robot1",
                    "robot_type": "franka_panda",
                    "joint_config": "ready",
                    "constraints": []
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = self.solver.solve(scene)
        
        assert len(poses) == 2
        assert "cup1" in poses
        assert "robot1" in poses
        
        # Cup should be positioned relative to robot
        robot_pos = poses["robot1"].position
        cup_pos = poses["cup1"].position
        
        # Should be reasonable distance
        distance = np.linalg.norm(cup_pos - robot_pos)
        assert 0.1 < distance < 1.0  # Reasonable manipulation distance
    
    @pytest.mark.skipif(not SPATIAL_REASONING_AVAILABLE, reason="Spatial reasoning not available")
    def test_enhanced_solver_integration(self):
        """Test that enhanced solver uses spatial reasoning when available."""
        # Should have spatial reasoning available if the module imported successfully
        if SPATIAL_REASONING_AVAILABLE:
            assert hasattr(self.solver, 'use_spatial_reasoning')
            assert hasattr(self.solver, 'spatial_reasoner')
            assert self.solver.use_spatial_reasoning is True
            assert self.solver.spatial_reasoner is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])