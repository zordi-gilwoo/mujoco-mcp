#!/usr/bin/env python3
"""Tests covering parametric primitive support for scene generation."""

import xml.etree.ElementTree as ET
import types

import numpy as np
import pytest
from pydantic import ValidationError

from mujoco_mcp.scene_gen import (
    ObjectPlacement,
    SceneDescription,
    SpatialConstraint,
    MetadataExtractor,
    SceneXMLBuilder,
    ConstraintSolver,
)
from mujoco_mcp.scene_gen.shared_types import Pose


def test_primitive_requires_dimensions_and_keys():
    """Primitive object placements must include all required dimension keys."""

    # Missing dimensions entirely should fail validation
    with pytest.raises(ValidationError):
        ObjectPlacement(object_id="pole", object_type="primitive:cylinder")

    # Missing one of the required keys should also fail validation
    with pytest.raises(ValidationError):
        ObjectPlacement(
            object_id="pole",
            object_type="primitive:cylinder",
            dimensions={"radius": 0.02},
        )

    # Non-primitive assets should not require dimensions
    tabletop = ObjectPlacement(object_id="table", object_type="table_standard")
    assert tabletop.dimensions is None


def test_primitive_dimension_bounds():
    """Primitive dimensions must be within reasonable bounds."""

    with pytest.raises(ValidationError):
        ObjectPlacement(
            object_id="bad_box",
            object_type="primitive:box",
            dimensions={"width": -0.1, "depth": 0.1, "height": 0.1},
        )

    with pytest.raises(ValidationError):
        ObjectPlacement(
            object_id="tiny_sphere",
            object_type="primitive:sphere",
            dimensions={"radius": 1e-5},
        )

    with pytest.raises(ValidationError):
        ObjectPlacement(
            object_id="huge_cylinder",
            object_type="primitive:cylinder",
            dimensions={"radius": 0.1, "height": 150.0},
        )


def test_metadata_with_dimensions_computes_symmetric_bounds():
    """Metadata extractor should produce symmetric bounds and semantic points."""

    extractor = MetadataExtractor()
    dims = {"width": 0.6, "depth": 0.4, "height": 0.2}

    metadata = extractor.get_metadata_with_dimensions("primitive:box", dims)
    assert metadata is not None

    bbox_min = np.array(metadata.bounding_box["min"], dtype=float)
    bbox_max = np.array(metadata.bounding_box["max"], dtype=float)
    np.testing.assert_allclose(bbox_min, [-0.3, -0.2, -0.1])
    np.testing.assert_allclose(bbox_max, [0.3, 0.2, 0.1])

    top_surface = metadata.get_semantic_point("top_surface")
    bottom_surface = metadata.get_semantic_point("bottom_surface")
    np.testing.assert_allclose(top_surface, [0.0, 0.0, 0.1])
    np.testing.assert_allclose(bottom_surface, [0.0, 0.0, -0.1])


def test_scene_xml_builder_formats_primitive_size_and_color():
    """XML builder must propagate primitive dimensions and RGBA into templates."""

    extractor = MetadataExtractor()
    builder = SceneXMLBuilder(extractor)

    ball = ObjectPlacement(
        object_id="ball",
        object_type="primitive:sphere",
        dimensions={"radius": 0.15},
        color=(0.1, 0.2, 0.3, 1.0),
    )

    scene = SceneDescription(objects=[ball], robots=[])
    poses = {
        "ball": Pose(
            position=np.array([0.1, -0.2, 0.15]), orientation=np.array([0.0, 0.0, 0.0, 1.0])
        )
    }

    xml_str = builder.build_scene(scene, poses)
    root = ET.fromstring(xml_str)

    ball_geom = root.find(".//body[@name='ball']/geom")
    assert ball_geom is not None
    assert ball_geom.get("type") == "sphere"
    assert ball_geom.get("size") == "0.150000"
    assert ball_geom.get("rgba") == "0.100 0.200 0.300 1.000"
    mass = float(ball_geom.get("mass"))
    assert mass == pytest.approx((4.0 / 3.0) * np.pi * 0.15 ** 3 * 780.0, rel=1e-3)


def test_constraint_solver_places_primitive_using_dimensions():
    """Constraint solver should honor primitive dimensions when solving placements."""

    table = ObjectPlacement(object_id="table", object_type="table_standard")
    pole = ObjectPlacement(
        object_id="pole",
        object_type="primitive:cylinder",
        dimensions={"radius": 0.05, "height": 0.6},
        constraints=[
            SpatialConstraint(
                type="on_top_of",
                subject="pole",
                reference="table",
                clearance=0.005,
            )
        ],
    )

    scene = SceneDescription(objects=[table, pole], robots=[])
    extractor = MetadataExtractor()
    solver = ConstraintSolver(extractor)
    solver.use_spatial_reasoning = False
    solver.use_enhanced_collision = False
    solver._enhance_poses_with_spatial_reasoning = types.MethodType(  # type: ignore[attr-defined]
        lambda _self, _scene, poses_map: poses_map,
        solver,
    )
    solver.collision_optimizer.optimize_poses = lambda _scene, poses_map: poses_map  # type: ignore[assignment]

    poses = solver.solve(scene)

    assert "table" in poses and "pole" in poses

    table_pose = poses["table"]
    pole_pose = poses["pole"]

    np.testing.assert_allclose(table_pose.position, [0.0, 0.0, 0.0])

    expected_center_z = 0.75 + 0.005 + 0.3  # table height + clearance + half pole height
    assert pole_pose.position[2] == pytest.approx(expected_center_z, abs=1e-6)


def test_on_top_of_respects_reference_orientation():
    """Rotated support surfaces should influence world placement of dependents."""

    base = ObjectPlacement(
        object_id="base",
        object_type="primitive:box",
        dimensions={"width": 0.4, "depth": 0.4, "height": 0.2},
        orientation=(0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)),  # 90 degrees about Y
    )
    link = ObjectPlacement(
        object_id="link",
        object_type="primitive:cylinder",
        dimensions={"radius": 0.05, "height": 0.3},
        constraints=[
            SpatialConstraint(
                type="on_top_of",
                subject="link",
                reference="base",
                clearance=0.01,
            )
        ],
    )

    scene = SceneDescription(objects=[base, link], robots=[])
    extractor = MetadataExtractor()
    solver = ConstraintSolver(extractor)
    solver.use_spatial_reasoning = False
    solver.use_enhanced_collision = False
    solver._enhance_poses_with_spatial_reasoning = types.MethodType(
        lambda _self, _scene, poses_map: poses_map,
        solver,
    )
    solver.collision_optimizer.optimize_poses = lambda _scene, poses_map: poses_map  # type: ignore[assignment]

    poses = solver.solve(scene)

    assert "link" in poses
    link_pose = poses["link"]

    ref_top_world = np.array([0.1, 0.0, 0.0])
    clearance_vector = np.array([0.01, 0.0, 0.0])
    bottom_offset = np.array([0.0, 0.0, -0.15])
    expected_position = ref_top_world + clearance_vector - bottom_offset

    np.testing.assert_allclose(link_pose.position, expected_position, atol=1e-6)
