#!/usr/bin/env python3
"""
Tests for Phase 2D: Enhanced Asset Semantics

Tests rich asset metadata, grasp affordances, support surfaces,
workspace envelopes, and robot mounting rules.
"""

import numpy as np
import pytest
from unittest.mock import Mock, patch

from mujoco_mcp.scene_gen import (
    MetadataExtractor,
    ConstraintSolver
)

# Import enhanced semantics components
from mujoco_mcp.scene_gen.enhanced_semantics import (
    EnhancedAssetMetadata,
    EnhancedAssetDatabase,
    GraspAffordance,
    SupportSurfaceInfo,
    WorkspaceEnvelope,
    RobotMountingRule,
    AssetCategory,
    GraspType,
    SurfaceType
)


class TestGraspAffordance:
    """Test grasp affordance functionality."""
    
    
    def test_grasp_affordance_creation(self):
        """Test creating grasp affordances."""
        grasp = GraspAffordance(
            name="test_handle",
            grasp_type=GraspType.HANDLE_GRASP,
            position=np.array([0.05, 0, 0.05]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([1, 0, 0]),
            grasp_width=0.02,
            force_closure_quality=0.9,
            accessibility_score=0.8,
            description="Test handle grasp"
        )
        
        assert grasp.name == "test_handle"
        assert grasp.grasp_type == GraspType.HANDLE_GRASP
        assert grasp.force_closure_quality == 0.9
        assert grasp.accessibility_score == 0.8
        np.testing.assert_array_equal(grasp.position, [0.05, 0, 0.05])
        np.testing.assert_array_equal(grasp.approach_direction, [1, 0, 0])


class TestSupportSurfaceInfo:
    """Test support surface information."""
    
    
    def test_support_surface_creation(self):
        """Test creating support surface information."""
        surface = SupportSurfaceInfo(
            name="table_top",
            surface_type=SurfaceType.HORIZONTAL_STABLE,
            center_position=np.array([0, 0, 0.75]),
            normal_direction=np.array([0, 0, 1]),
            dimensions=np.array([1.2, 0.8]),
            load_capacity=50.0,
            stability_factor=0.95,
            material_friction=0.7,
            is_primary_function=True,
            description="Primary table surface"
        )
        
        assert surface.name == "table_top"
        assert surface.surface_type == SurfaceType.HORIZONTAL_STABLE
        assert surface.load_capacity == 50.0
        assert surface.stability_factor == 0.95
        assert surface.is_primary_function == True
        np.testing.assert_array_equal(surface.center_position, [0, 0, 0.75])
        np.testing.assert_array_equal(surface.normal_direction, [0, 0, 1])


class TestWorkspaceEnvelope:
    """Test workspace envelope functionality."""
    
    
    def test_workspace_envelope_creation(self):
        """Test creating workspace envelopes."""
        workspace = WorkspaceEnvelope(
            name="primary_workspace",
            center=np.array([0.6, 0, 0.5]),
            extents=np.array([1.0, 1.0, 0.8]),
            orientation=np.array([0, 0, 0, 1]),
            reachability_zones={"optimal": 0.9, "good": 0.7},
            task_preferences={"pick_and_place": 0.9, "assembly": 0.8},
            collision_margins={"front": 0.1, "sides": 0.15},
            description="Primary manipulation workspace"
        )
        
        assert workspace.name == "primary_workspace"
        assert workspace.reachability_zones["optimal"] == 0.9
        assert workspace.task_preferences["pick_and_place"] == 0.9
        assert workspace.collision_margins["front"] == 0.1
        np.testing.assert_array_equal(workspace.center, [0.6, 0, 0.5])
        np.testing.assert_array_equal(workspace.extents, [1.0, 1.0, 0.8])


class TestEnhancedAssetMetadata:
    """Test enhanced asset metadata functionality."""
    
    
    def test_enhanced_metadata_creation(self):
        """Test creating enhanced asset metadata."""
        # Create grasp affordance
        handle_grasp = GraspAffordance(
            name="handle",
            grasp_type=GraspType.HANDLE_GRASP,
            position=np.array([0.04, 0, 0.05]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([1, 0, 0]),
            grasp_width=0.02,
            force_closure_quality=0.9,
            accessibility_score=0.9
        )
        
        # Create support surface
        surface = SupportSurfaceInfo(
            name="top_surface",
            surface_type=SurfaceType.HORIZONTAL_STABLE,
            center_position=np.array([0, 0, 0.08]),
            normal_direction=np.array([0, 0, 1]),
            dimensions=np.array([0.07, 0.07]),
            load_capacity=1.0,
            stability_factor=0.7,
            material_friction=0.6
        )
        
        # Create enhanced metadata
        metadata = EnhancedAssetMetadata(
            asset_id="test_cup",
            asset_type="cup",
            category=AssetCategory.TABLEWARE,
            description="Test cup with enhanced semantics",
            grasp_affordances=[handle_grasp],
            support_surfaces=[surface],
            mass=0.2
        )
        
        assert metadata.asset_id == "test_cup"
        assert metadata.category == AssetCategory.TABLEWARE
        assert metadata.mass == 0.2
        assert len(metadata.grasp_affordances) == 1
        assert len(metadata.support_surfaces) == 1
    
    
    def test_grasp_affordance_queries(self):
        """Test querying grasp affordances."""
        # Create multiple grasp affordances
        handle_grasp = GraspAffordance(
            name="handle",
            grasp_type=GraspType.HANDLE_GRASP,
            position=np.array([0.04, 0, 0.05]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([1, 0, 0]),
            grasp_width=0.02,
            force_closure_quality=0.9,
            accessibility_score=0.9
        )
        
        top_grasp = GraspAffordance(
            name="top_rim",
            grasp_type=GraspType.TOP_GRASP,
            position=np.array([0, 0, 0.08]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([0, 0, -1]),
            grasp_width=0.08,
            force_closure_quality=0.7,
            accessibility_score=0.8
        )
        
        metadata = EnhancedAssetMetadata(
            asset_id="test_cup",
            asset_type="cup",
            category=AssetCategory.TABLEWARE,
            grasp_affordances=[handle_grasp, top_grasp]
        )
        
        # Test querying all grasps
        all_grasps = metadata.get_grasp_affordances()
        assert len(all_grasps) == 2
        
        # Test querying by type
        handle_grasps = metadata.get_grasp_affordances(GraspType.HANDLE_GRASP)
        assert len(handle_grasps) == 1
        assert handle_grasps[0].name == "handle"
        
        # Test getting best grasp
        best_grasp = metadata.get_best_grasp_affordance()
        assert best_grasp is not None
        assert best_grasp.name == "handle"  # Should be best due to higher quality
        
        # Test graspability
        assert metadata.is_graspable() == True
    
    
    def test_support_surface_queries(self):
        """Test querying support surfaces."""
        # Create support surface
        top_surface = SupportSurfaceInfo(
            name="top",
            surface_type=SurfaceType.HORIZONTAL_STABLE,
            center_position=np.array([0, 0, 0.75]),
            normal_direction=np.array([0, 0, 1]),
            dimensions=np.array([1.2, 0.8]),
            load_capacity=50.0,
            stability_factor=0.95,
            material_friction=0.7,
            is_primary_function=True
        )
        
        metadata = EnhancedAssetMetadata(
            asset_id="test_table",
            asset_type="table",
            category=AssetCategory.FURNITURE,
            support_surfaces=[top_surface]
        )
        
        # Test support surface queries
        all_surfaces = metadata.get_support_surfaces()
        assert len(all_surfaces) == 1
        
        primary_surface = metadata.get_primary_support_surface()
        assert primary_surface is not None
        assert primary_surface.name == "top"
        
        # Test support capabilities
        assert metadata.is_support_surface() == True
        assert metadata.can_support_load(10.0) == True
        assert metadata.can_support_load(100.0) == False


class TestEnhancedAssetDatabase:
    """Test enhanced asset database functionality."""
    
    
    def test_database_creation(self):
        """Test creating enhanced asset database."""
        # This should work with default assets even if file doesn't exist
        db = EnhancedAssetDatabase()
        assert db is not None
        assert hasattr(db, 'enhanced_assets')
        assert len(db.enhanced_assets) > 0
    
    
    def test_asset_retrieval(self):
        """Test retrieving assets from database."""
        db = EnhancedAssetDatabase()
        
        # Test getting specific asset
        table_metadata = db.get_enhanced_metadata("table_standard")
        assert table_metadata is not None
        assert table_metadata.asset_id == "table_standard"
        assert table_metadata.category == AssetCategory.FURNITURE
        
        # Test getting assets by category
        furniture_assets = db.get_assets_by_category(AssetCategory.FURNITURE)
        assert len(furniture_assets) > 0
        assert all(asset.category == AssetCategory.FURNITURE for asset in furniture_assets)
        
        # Test getting graspable assets
        graspable_assets = db.get_graspable_assets()
        assert len(graspable_assets) > 0
        assert all(asset.is_graspable() for asset in graspable_assets)
        
        # Test getting support surface assets
        support_assets = db.get_support_surface_assets()
        assert len(support_assets) > 0
        assert all(asset.is_support_surface() for asset in support_assets)
    
    
    def test_compatibility_queries(self):
        """Test finding compatible support surfaces."""
        db = EnhancedAssetDatabase()
        
        # Find support for light object
        light_supports = db.find_compatible_support(object_mass=1.0, required_stability=0.8)
        assert len(light_supports) > 0
        
        # Find support for heavy object
        heavy_supports = db.find_compatible_support(object_mass=40.0, required_stability=0.9)
        # Should find fewer options
        assert len(heavy_supports) <= len(light_supports)
        
        # All found supports should actually be able to handle the load
        for support in heavy_supports:
            assert support.can_support_load(40.0) == True


class TestEnhancedConstraintSolver:
    """Test integration of enhanced semantics with constraint solver."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
        self.solver = ConstraintSolver(self.metadata_extractor)
    
    
    def test_enhanced_semantics_integration(self):
        """Test that enhanced semantics is integrated with constraint solver."""
        assert hasattr(self.solver, 'enhanced_asset_db')
        assert self.solver.enhanced_asset_db is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])