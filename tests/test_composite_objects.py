#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Tests for composite objects (bins, totes, shelves) in scene generation.
"""

import pytest
import numpy as np
from mujoco_mcp.scene_gen.scene_schema import (
    SceneDescription,
    ObjectPlacement,
    SpatialConstraint,
)
from mujoco_mcp.scene_gen.metadata_extractor import MetadataExtractor
from mujoco_mcp.scene_gen.constraint_solver import ConstraintSolver
from mujoco_mcp.scene_gen.scene_xml_builder import SceneXMLBuilder


class TestCompositeObjects:
    """Test suite for composite objects (bins, totes, shelves)."""

    def setup_method(self):
        """Set up test fixtures."""
        self.metadata_extractor = MetadataExtractor()
        self.constraint_solver = ConstraintSolver(self.metadata_extractor)
        self.xml_builder = SceneXMLBuilder(self.metadata_extractor)

    def test_bin_metadata(self):
        """Test that bin metadata is properly loaded and computed."""
        # Get bin metadata with custom dimensions
        metadata = self.metadata_extractor.get_metadata_with_dimensions(
            "composite:bin",
            {"width": 0.5, "depth": 0.5, "height": 0.3, "wall_thickness": 0.01},
        )

        assert metadata is not None
        assert metadata.asset_id == "composite:bin"
        assert metadata.composite_shape == "bin"

        # Check bounding box
        bbox_min, bbox_max = metadata.get_bounding_box()
        assert bbox_min[0] == pytest.approx(-0.25)
        assert bbox_min[1] == pytest.approx(-0.25)
        assert bbox_min[2] == pytest.approx(0.0)
        assert bbox_max[0] == pytest.approx(0.25)
        assert bbox_max[1] == pytest.approx(0.25)
        assert bbox_max[2] == pytest.approx(0.3)

        # Check semantic points
        inside_bottom = metadata.get_semantic_point("inside_bottom")
        assert inside_bottom is not None
        assert inside_bottom[2] == pytest.approx(0.01)  # wall_thickness

        top_rim = metadata.get_semantic_point("top_rim")
        assert top_rim is not None
        assert top_rim[2] == pytest.approx(0.3)  # height

    def test_tote_metadata(self):
        """Test that tote metadata is properly loaded and computed."""
        metadata = self.metadata_extractor.get_metadata_with_dimensions(
            "composite:tote",
            {"width": 0.6, "depth": 0.4, "height": 0.35},
        )

        assert metadata is not None
        assert metadata.asset_id == "composite:tote"
        assert metadata.composite_shape == "tote"

        # Check dimensions are applied
        dims = metadata.get_dimensions()
        assert dims["width"] == pytest.approx(0.6)
        assert dims["depth"] == pytest.approx(0.4)
        assert dims["height"] == pytest.approx(0.35)

    def test_shelf_metadata(self):
        """Test that shelf metadata is properly loaded and computed."""
        metadata = self.metadata_extractor.get_metadata_with_dimensions(
            "composite:shelf",
            {"width": 0.6, "depth": 0.3, "height": 0.9, "num_shelves": 2},
        )

        assert metadata is not None
        assert metadata.asset_id == "composite:shelf"
        assert metadata.composite_shape == "shelf"

        # Check shelf semantic points are generated
        shelf_0 = metadata.get_semantic_point("shelf_0")
        assert shelf_0 is not None

        shelf_1 = metadata.get_semantic_point("shelf_1")
        assert shelf_1 is not None

        shelf_top = metadata.get_semantic_point("shelf_top")
        assert shelf_top is not None

    def test_bin_scene_generation(self):
        """Test generating a scene with a bin on a table."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="table",
                    object_type="table_standard",
                    constraints=[],
                ),
                ObjectPlacement(
                    object_id="storage_bin",
                    object_type="composite:bin",
                    dimensions={"width": 0.4, "depth": 0.4, "height": 0.3},
                    color=(0.6, 0.6, 0.6, 1.0),
                    constraints=[
                        SpatialConstraint(
                            type="on_top_of",
                            subject="storage_bin",
                            reference="table",
                            clearance=0.001,
                        )
                    ],
                ),
            ]
        )

        # Solve constraints
        poses = self.constraint_solver.solve(scene)

        assert "table" in poses
        assert "storage_bin" in poses

        # Bin should be on top of table
        table_top = poses["table"].position[2] + 0.75  # table height
        bin_bottom = poses["storage_bin"].position[2]
        assert bin_bottom == pytest.approx(table_top, abs=0.01)

        # Generate XML
        xml = self.xml_builder.build_scene(scene, poses)

        assert xml is not None
        assert "storage_bin" in xml
        assert "storage_bin_bottom" in xml
        assert "storage_bin_front_wall" in xml
        assert "storage_bin_back_wall" in xml
        assert "storage_bin_left_wall" in xml
        assert "storage_bin_right_wall" in xml

    def test_tote_with_object_inside(self):
        """Test placing an object inside a tote."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="table",
                    object_type="table_standard",
                    constraints=[],
                ),
                ObjectPlacement(
                    object_id="tote",
                    object_type="composite:tote",
                    dimensions={
                        "width": 0.5,
                        "depth": 0.5,
                        "height": 0.3,
                        "wall_thickness": 0.01,
                    },
                    constraints=[
                        SpatialConstraint(
                            type="on_top_of",
                            subject="tote",
                            reference="table",
                            clearance=0.001,
                        )
                    ],
                ),
                ObjectPlacement(
                    object_id="ball",
                    object_type="primitive:sphere",
                    dimensions={"radius": 0.05},
                    color=(1.0, 0.0, 0.0, 1.0),
                    constraints=[
                        SpatialConstraint(
                            type="inside", subject="ball", reference="tote", clearance=0.0
                        )
                    ],
                ),
            ]
        )

        # Solve constraints
        poses = self.constraint_solver.solve(scene)

        assert "tote" in poses
        assert "ball" in poses

        # Ball should be inside tote (above tote's interior bottom)
        tote_inside_bottom = poses["tote"].position[2] + 0.01  # wall_thickness
        ball_center = poses["ball"].position[2]
        ball_radius = 0.05

        # Ball bottom should be at or near interior floor
        ball_bottom = ball_center - ball_radius
        assert ball_bottom == pytest.approx(tote_inside_bottom, abs=0.01)

        # Ball center should be below tote top
        tote_top = poses["tote"].position[2] + 0.3
        assert ball_center < tote_top

    def test_shelf_scene_generation(self):
        """Test generating a scene with a shelf unit."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="shelf",
                    object_type="composite:shelf",
                    dimensions={
                        "width": 0.6,
                        "depth": 0.3,
                        "height": 0.9,
                        "num_shelves": 2,
                    },
                    color=(0.7, 0.5, 0.3, 1.0),
                    constraints=[],
                )
            ]
        )

        # Solve constraints
        poses = self.constraint_solver.solve(scene)

        assert "shelf" in poses

        # Generate XML
        xml = self.xml_builder.build_scene(scene, poses)

        assert xml is not None
        assert "shelf" in xml
        assert "shelf_back_wall" in xml
        assert "shelf_left_wall" in xml
        assert "shelf_right_wall" in xml
        assert "shelf_shelf_0" in xml  # Bottom shelf
        assert "shelf_shelf_1" in xml  # Interior shelf 1
        assert "shelf_shelf_2" in xml  # Interior shelf 2
        assert "shelf_shelf_top" in xml  # Top shelf

    def test_composite_validation(self):
        """Test that composite objects require proper dimensions."""
        # Missing dimensions should raise error
        with pytest.raises(ValueError, match="requires dimensions"):
            ObjectPlacement(
                object_id="bad_bin",
                object_type="composite:bin",
                constraints=[],
            )

        # Missing required dimension keys should raise error
        with pytest.raises(ValueError, match="missing dimensions"):
            ObjectPlacement(
                object_id="bad_tote",
                object_type="composite:tote",
                dimensions={"width": 0.5, "depth": 0.5},  # Missing height
                constraints=[],
            )

        # Valid composite should succeed
        obj = ObjectPlacement(
            object_id="good_bin",
            object_type="composite:bin",
            dimensions={"width": 0.4, "depth": 0.4, "height": 0.3},
            constraints=[],
        )
        assert obj.object_id == "good_bin"

    def test_bin_xml_structure(self):
        """Test that bin XML has correct structure with 5 geoms."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="bin",
                    object_type="composite:bin",
                    dimensions={"width": 0.4, "depth": 0.4, "height": 0.3},
                    constraints=[],
                )
            ]
        )

        poses = self.constraint_solver.solve(scene)
        xml = self.xml_builder.build_scene(scene, poses)

        # Should have 5 geoms: bottom, front, back, left, right
        assert xml.count('name="bin_bottom"') == 1
        assert xml.count('name="bin_front_wall"') == 1
        assert xml.count('name="bin_back_wall"') == 1
        assert xml.count('name="bin_left_wall"') == 1
        assert xml.count('name="bin_right_wall"') == 1

        # All should be box geoms
        assert xml.count('type="box"') >= 5

    def test_shelf_xml_structure(self):
        """Test that shelf XML has correct structure with shelves."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="shelf",
                    object_type="composite:shelf",
                    dimensions={
                        "width": 0.6,
                        "depth": 0.3,
                        "height": 0.9,
                        "num_shelves": 3,
                    },
                    constraints=[],
                )
            ]
        )

        poses = self.constraint_solver.solve(scene)
        xml = self.xml_builder.build_scene(scene, poses)

        # Should have 3 walls
        assert xml.count('name="shelf_back_wall"') == 1
        assert xml.count('name="shelf_left_wall"') == 1
        assert xml.count('name="shelf_right_wall"') == 1

        # Should have bottom shelf + 3 interior shelves + top shelf = 5 total
        assert xml.count('name="shelf_shelf_0"') == 1
        assert xml.count('name="shelf_shelf_1"') == 1
        assert xml.count('name="shelf_shelf_2"') == 1
        assert xml.count('name="shelf_shelf_3"') == 1
        assert xml.count('name="shelf_shelf_top"') == 1

    def test_composite_colors(self):
        """Test that composite objects respect custom colors."""
        scene = SceneDescription(
            objects=[
                ObjectPlacement(
                    object_id="blue_bin",
                    object_type="composite:bin",
                    dimensions={"width": 0.4, "depth": 0.4, "height": 0.3},
                    color=(0.0, 0.0, 1.0, 1.0),
                    constraints=[],
                )
            ]
        )

        poses = self.constraint_solver.solve(scene)
        xml = self.xml_builder.build_scene(scene, poses)

        # Color should be applied to geoms (approximately)
        assert "0.000 0.000 1.000 1.000" in xml or "0.0 0.0 1.0 1.0" in xml


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
