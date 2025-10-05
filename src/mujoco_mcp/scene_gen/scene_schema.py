#!/usr/bin/env python3
"""
Scene Schema Models

Pydantic models for structured scene descriptions, including objects,
robots, spatial constraints, and validation rules.
"""

from typing import Dict, List, Optional, Tuple
from pydantic import BaseModel, Field, field_validator, model_validator
import logging

logger = logging.getLogger("mujoco_mcp.scene_gen.schema")


class SpatialConstraint(BaseModel):
    """
    Represents a spatial relationship between two objects or entities.

    Supported constraint types:
    - on_top_of: place subject on top of reference object
    - in_front_of: place subject in front of reference (+X direction)
    - beside: place subject beside reference (+Y direction with fallback to -Y)
    - no_collision: ensure subjects don't collide (AABB-based)
    """

    type: str = Field(..., description="Constraint type")
    subject: str = Field(..., description="ID of the object being positioned")
    reference: str = Field(..., description="ID of the reference object")
    clearance: float = Field(default=0.0, description="Minimum clearance distance in meters")
    offset: Optional[Tuple[float, float, float]] = Field(
        default=None, description="Optional 3D offset vector from default constraint position"
    )

    @field_validator("type")
    @classmethod
    def validate_constraint_type(cls, v):
        # Phase 2B: Extended constraint types for advanced spatial reasoning
        allowed_types = [
            "on_top_of",
            "in_front_of",
            "beside",
            "no_collision",
            "inside",
            "aligned_with_axis",
            "oriented_towards",
            "within_reach",
        ]
        if v not in allowed_types:
            raise ValueError(f"Constraint type must be one of {allowed_types}")
        return v


class ObjectPlacement(BaseModel):
    """
    Describes placement of an object in the scene with constraints.
    """

    object_id: str = Field(..., description="Unique identifier for this object instance")
    object_type: str = Field(..., description="Type/class of object (from assets_db)")
    constraints: List[SpatialConstraint] = Field(
        default_factory=list, description="Spatial constraints affecting this object's placement"
    )
    orientation: Optional[Tuple[float, float, float, float]] = Field(
        default=None, description="Quaternion orientation [x, y, z, w]. Default is [0, 0, 0, 1]"
    )
    dimensions: Optional[Dict[str, float]] = Field(
        default=None,
        description="Custom dimensions in meters (e.g., {'width': 0.5, 'height': 2.0})",
    )
    color: Optional[Tuple[float, float, float, float]] = Field(
        default=None,
        description="RGBA color values in [0,1] for MuJoCo geoms (e.g., (0.8, 0.2, 0.2, 1.0))",
    )

    @field_validator("object_id")
    @classmethod
    def validate_object_id(cls, v):
        if not v or not v.strip():
            raise ValueError("Object ID cannot be empty")
        return v

    @model_validator(mode="after")
    def validate_primitive_dimensions(self):
        """Ensure primitive objects receive the required dimension keys."""
        object_type = self.object_type or ""
        if not object_type.startswith("primitive:"):
            return self

        primitive_type = object_type.split(":", 1)[1]

        if not self.dimensions:
            raise ValueError(f"Primitive type {object_type} requires dimensions")

        required_keys = {
            "box": {"width", "depth", "height"},
            "sphere": {"radius"},
            "cylinder": {"radius", "height"},
            "capsule": {"radius", "length"},
            "ellipsoid": {"radius_x", "radius_y", "radius_z"},
        }

        if primitive_type in required_keys:
            missing = required_keys[primitive_type] - set(self.dimensions.keys())
            if missing:
                raise ValueError(
                    f"Primitive {primitive_type} missing dimensions: {sorted(missing)}"
                )

        # Sanity check on primitive dimensions to catch unreasonable values early
        for key, value in self.dimensions.items():
            if value <= 0:
                raise ValueError(f"Dimension '{key}' for {object_type} must be positive")
            if value < 1e-3:
                raise ValueError(
                    f"Dimension '{key}' for {object_type} is too small ({value}); minimum is 1e-3"
                )
            if value > 100:
                raise ValueError(
                    f"Dimension '{key}' for {object_type} is too large ({value}); maximum is 100"
                )

        return self


class RobotConfiguration(BaseModel):
    """
    Describes robot placement and joint configuration.
    """

    robot_id: str = Field(..., description="Unique identifier for this robot instance")
    robot_type: str = Field(..., description="Type of robot (from assets_db)")
    base_position: Optional[Tuple[float, float, float]] = Field(
        default=None, description="Base position [x, y, z]. If None, determined by constraints"
    )
    base_orientation: Optional[Tuple[float, float, float, float]] = Field(
        default=None,
        description="Base orientation quaternion [x, y, z, w]. Default is [0, 0, 0, 1]",
    )
    joint_config: str = Field(
        default="ready", description="Named joint configuration (e.g., 'ready', 'home', 'tucked')"
    )
    custom_joints: Optional[List[float]] = Field(
        default=None, description="Custom joint angles (overrides joint_config if provided)"
    )
    constraints: List[SpatialConstraint] = Field(
        default_factory=list, description="Spatial constraints affecting robot base placement"
    )


class SceneDescription(BaseModel):
    """
    Complete scene description including objects, robots, and workspace bounds.
    """

    objects: List[ObjectPlacement] = Field(
        default_factory=list, description="Objects to place in the scene"
    )
    robots: List[RobotConfiguration] = Field(
        default_factory=list, description="Robots to place in the scene"
    )
    workspace_bounds: Optional[Tuple[float, float, float, float, float, float]] = Field(
        default=None, description="Workspace bounds [x_min, y_min, z_min, x_max, y_max, z_max]"
    )

    @model_validator(mode="after")
    def validate_scene_consistency(self):
        """Validate that all constraint references exist in the scene."""
        objects = self.objects or []
        robots = self.robots or []

        # Collect all entity IDs
        all_ids = set()
        all_ids.update(obj.object_id for obj in objects)
        all_ids.update(robot.robot_id for robot in robots)

        # Check constraint references
        for obj in objects:
            for constraint in obj.constraints:
                if constraint.reference not in all_ids:
                    raise ValueError(
                        f"Object {obj.object_id} has constraint referencing "
                        f"unknown entity: {constraint.reference}"
                    )
                if constraint.subject != obj.object_id:
                    raise ValueError(
                        f"Object {obj.object_id} has constraint with mismatched "
                        f"subject: {constraint.subject}"
                    )

        for robot in robots:
            for constraint in robot.constraints:
                if constraint.reference not in all_ids:
                    raise ValueError(
                        f"Robot {robot.robot_id} has constraint referencing "
                        f"unknown entity: {constraint.reference}"
                    )
                if constraint.subject != robot.robot_id:
                    raise ValueError(
                        f"Robot {robot.robot_id} has constraint with mismatched "
                        f"subject: {constraint.subject}"
                    )

        return self

    def get_all_entity_ids(self) -> List[str]:
        """Get all entity IDs in the scene."""
        ids = []
        ids.extend(obj.object_id for obj in self.objects)
        ids.extend(robot.robot_id for robot in self.robots)
        return ids

    def get_constraints_for_entity(self, entity_id: str) -> List[SpatialConstraint]:
        """Get all constraints where the given entity is the subject."""
        constraints = []

        for obj in self.objects:
            if obj.object_id == entity_id:
                constraints.extend(obj.constraints)

        for robot in self.robots:
            if robot.robot_id == entity_id:
                constraints.extend(robot.constraints)

        return constraints

    def to_xml(self) -> str:
        """
        Convert this SceneDescription to MuJoCo XML.

        This completes the full pipeline:
        NL → SceneDescription → Poses → XML

        Returns:
            Complete MuJoCo XML string
        """
        from .metadata_extractor import MetadataExtractor
        from .constraint_solver import ConstraintSolver
        from .scene_xml_builder import SceneXMLBuilder

        # Initialize the pipeline components
        metadata_extractor = MetadataExtractor()
        constraint_solver = ConstraintSolver(metadata_extractor)
        xml_builder = SceneXMLBuilder(metadata_extractor)

        # Solve constraints to get poses
        poses = constraint_solver.solve(self)

        # Build XML from poses
        return xml_builder.build_scene(self, poses)
