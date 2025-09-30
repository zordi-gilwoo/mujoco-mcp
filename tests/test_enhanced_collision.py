#!/usr/bin/env python3
"""
Tests for Phase 2A: Enhanced Collision Detection

Tests the rotation-aware AABB collision detection, physics-based validation,
and global collision optimization capabilities.
"""

import numpy as np
import pytest
from unittest.mock import Mock, patch

from mujoco_mcp.scene_gen import (
    SceneDescription,
    SpatialConstraint,
    ObjectPlacement,
    MetadataExtractor,
    ConstraintSolver,
    Pose
)

# Import enhanced collision detection components
from mujoco_mcp.scene_gen.enhanced_collision import (
    RotationAwareAABB,
    GlobalCollisionOptimizer,
    PhysicsCollisionValidator,
    quaternion_to_matrix,
    transform_bbox_with_rotation
)


class TestQuaternionMath:
    """Test quaternion and rotation utilities."""
    
    
    def test_quaternion_to_matrix(self):
        """Test quaternion to rotation matrix conversion."""
        # Identity quaternion should give identity matrix
        identity_quat = np.array([0, 0, 0, 1])
        identity_matrix = quaternion_to_matrix(identity_quat)
        expected = np.eye(3)
        np.testing.assert_allclose(identity_matrix, expected, atol=1e-10)
        
        # 90 degree rotation around Z axis
        z_90_quat = np.array([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
        z_90_matrix = quaternion_to_matrix(z_90_quat)
        
        # Should rotate [1,0,0] to [0,1,0]
        rotated = z_90_matrix @ np.array([1, 0, 0])
        expected = np.array([0, 1, 0])
        np.testing.assert_allclose(rotated, expected, atol=1e-10)
    
    
    def test_bbox_rotation_transform(self):
        """Test bounding box transformation with rotation."""
        # Unit cube centered at origin
        bbox_min = np.array([-0.5, -0.5, -0.5])
        bbox_max = np.array([0.5, 0.5, 0.5])
        
        # No rotation, just translation
        pose = Pose(
            position=np.array([1, 2, 3]),
            orientation=np.array([0, 0, 0, 1])
        )
        
        new_min, new_max = transform_bbox_with_rotation(bbox_min, bbox_max, pose)
        
        # Should just be translated
        expected_min = bbox_min + pose.position
        expected_max = bbox_max + pose.position
        np.testing.assert_allclose(new_min, expected_min, atol=1e-10)
        np.testing.assert_allclose(new_max, expected_max, atol=1e-10)
        
        # Test 45 degree rotation around Z - should expand the AABB
        z_45_quat = np.array([0, 0, np.sin(np.pi/8), np.cos(np.pi/8)])
        rotated_pose = Pose(
            position=np.array([0, 0, 0]),
            orientation=z_45_quat
        )
        
        rot_min, rot_max = transform_bbox_with_rotation(bbox_min, bbox_max, rotated_pose)
        
        # Rotated cube should have larger X and Y extents
        assert rot_max[0] - rot_min[0] > 1.0  # Should be > original width of 1.0
        assert rot_max[1] - rot_min[1] > 1.0  # Should be > original depth of 1.0


class TestRotationAwareAABB:
    """Test rotation-aware AABB collision detection."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    
    def test_rotation_aware_aabb_creation(self):
        """Test creation of rotation-aware AABB from metadata."""
        # Get a test object
        metadata = self.metadata_extractor.get_metadata("box_small")
        assert metadata is not None
        
        # Create pose with rotation
        pose = Pose(
            position=np.array([1, 2, 3]),
            orientation=np.array([0, 0, np.sin(np.pi/8), np.cos(np.pi/8)])  # 45 degrees around Z
        )
        
        # Create rotation-aware AABB
        aabb = RotationAwareAABB.from_metadata(metadata, pose)
        
        # Check that AABB is centered around the pose position
        center = (aabb.min + aabb.max) / 2
        np.testing.assert_allclose(center, pose.position, atol=1e-3)
        
        # Check that rotated AABB is larger than non-rotated
        no_rotation_pose = Pose(
            position=np.array([1, 2, 3]),
            orientation=np.array([0, 0, 0, 1])
        )
        no_rot_aabb = RotationAwareAABB.from_metadata(metadata, no_rotation_pose)
        
        # Rotated should have larger or equal extents due to rotation
        rot_extents = aabb.max - aabb.min
        no_rot_extents = no_rot_aabb.max - no_rot_aabb.min
        
        assert rot_extents[0] >= no_rot_extents[0] * 0.99  # Allow small numerical error
        assert rot_extents[1] >= no_rot_extents[1] * 0.99
    
    
    def test_overlap_volume_calculation(self):
        """Test overlap volume calculation between AABBs."""
        # Create two overlapping AABBs
        aabb1 = RotationAwareAABB(
            min_coords=np.array([0, 0, 0]),
            max_coords=np.array([2, 2, 2])
        )
        aabb2 = RotationAwareAABB(
            min_coords=np.array([1, 1, 1]),
            max_coords=np.array([3, 3, 3])
        )
        
        # Should overlap in 1x1x1 cube
        volume = aabb1.get_overlap_volume(aabb2)
        expected_volume = 1.0  # 1x1x1 overlap
        assert abs(volume - expected_volume) < 1e-6
        
        # Non-overlapping AABBs
        aabb3 = RotationAwareAABB(
            min_coords=np.array([5, 5, 5]),
            max_coords=np.array([6, 6, 6])
        )
        
        volume_no_overlap = aabb1.get_overlap_volume(aabb3)
        assert volume_no_overlap == 0.0
    
    
    def test_surface_contact_area(self):
        """Test surface contact area estimation."""
        # Two boxes in surface contact (one on top of other)
        aabb1 = RotationAwareAABB(
            min_coords=np.array([0, 0, 0]),
            max_coords=np.array([2, 2, 1])
        )
        aabb2 = RotationAwareAABB(
            min_coords=np.array([0, 0, 0.9]),  # Slightly overlapping on top
            max_coords=np.array([2, 2, 1.9])
        )
        
        # Should have contact area approximately 2x2 = 4
        contact_area = aabb1.get_surface_contact_area(aabb2)
        assert contact_area > 3.0  # Should be close to 4 with some tolerance


class TestPhysicsCollisionValidator:
    """Test physics-based collision validation."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    
    def test_physics_validator_creation(self):
        """Test that physics validator can be created."""
        validator = PhysicsCollisionValidator(self.metadata_extractor)
        assert validator is not None
        
        # Should handle MuJoCo availability gracefully
        assert hasattr(validator, 'physics_available')
    
    
    def test_collision_check_xml_generation(self):
        """Test XML generation for collision checking."""
        validator = PhysicsCollisionValidator(self.metadata_extractor)
        
        # Create simple scene
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
        poses = {
            "box1": Pose(
                position=np.array([0, 0, 0]),
                orientation=np.array([0, 0, 0, 1])
            ),
            "box2": Pose(
                position=np.array([1, 0, 0]),
                orientation=np.array([0, 0, 0, 1])
            )
        }
        
        xml_string = validator._build_collision_check_xml(scene, poses)
        
        # Check that XML contains expected elements
        assert "<mujoco>" in xml_string
        assert "<worldbody>" in xml_string
        assert "box1" in xml_string
        assert "box2" in xml_string
        assert "geom" in xml_string


class TestGlobalCollisionOptimizer:
    """Test global collision optimization."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
    
    
    def test_optimizer_creation(self):
        """Test that global collision optimizer can be created."""
        optimizer = GlobalCollisionOptimizer(self.metadata_extractor)
        assert optimizer is not None
        assert hasattr(optimizer, 'max_iterations')
        assert hasattr(optimizer, 'convergence_threshold')
    
    
    def test_collision_constraint_extraction(self):
        """Test extraction of collision constraints from scene."""
        optimizer = GlobalCollisionOptimizer(self.metadata_extractor)
        
        # Scene with collision constraints
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
                    "constraints": [
                        {
                            "type": "no_collision",
                            "subject": "box2",
                            "reference": "box1",
                            "clearance": 0.1
                        }
                    ]
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        collision_constraints = optimizer._get_collision_constraints(scene)
        
        assert len(collision_constraints) == 1
        assert collision_constraints[0].type == "no_collision"
        assert collision_constraints[0].subject == "box2"
        assert collision_constraints[0].reference == "box1"
    
    
    def test_entity_mobility_calculation(self):
        """Test calculation of entity mobility weights."""
        optimizer = GlobalCollisionOptimizer(self.metadata_extractor)
        
        # Scene with different constraint patterns
        scene_dict = {
            "objects": [
                {
                    "object_id": "table1",
                    "object_type": "table_standard",
                    "constraints": []  # No constraints = high mobility
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
                    ]  # One constraint = lower mobility
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        
        table_mobility = optimizer._get_entity_mobility("table1", scene)
        cup_mobility = optimizer._get_entity_mobility("cup1", scene)
        
        # Table should have very low mobility (large object, even with no constraints)
        # Cup should have higher mobility than table (smaller object)
        assert table_mobility < cup_mobility


class TestEnhancedConstraintSolver:
    """Test integration of enhanced collision detection with constraint solver."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.metadata_extractor)
    
    
    def test_enhanced_collision_integration(self):
        """Test that enhanced collision detection is integrated correctly."""
        # Should have enhanced collision available
        assert hasattr(self.solver, 'collision_optimizer')
        assert self.solver.collision_optimizer is not None
    
    def test_collision_resolution_with_rotation(self):
        """Test collision resolution works with rotated objects."""
        # Create scene with overlapping rotated boxes
        scene_dict = {
            "objects": [
                {
                    "object_id": "box1",
                    "object_type": "box_small",
                    "constraints": [],
                    "orientation": [0, 0, 0, 1]  # No rotation
                },
                {
                    "object_id": "box2",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "no_collision",
                            "subject": "box2",
                            "reference": "box1",
                            "clearance": 0.05
                        }
                    ],
                    "orientation": [0, 0, 0.707, 0.707]  # 90 degree rotation around Z
                }
            ]
        }
        
        scene = SceneDescription(**scene_dict)
        poses = self.solver.solve(scene)
        
        # Should successfully solve without collision
        assert len(poses) == 2
        assert "box1" in poses
        assert "box2" in poses
        
        # Boxes should be separated by at least the clearance distance
        box1_pos = poses["box1"].position
        box2_pos = poses["box2"].position
        distance = np.linalg.norm(box1_pos - box2_pos)
        
        # Should be separated by more than just the clearance due to object sizes
        assert distance > 0.05


if __name__ == "__main__":
    pytest.main([__file__, "-v"])