#!/usr/bin/env python3
"""
Scene XML Builder

Composes complete MuJoCo XML scenes from asset templates and solved poses.
Generates a unified scene with proper worldbody structure, lighting, and floor.
"""

import logging
import math
import xml.etree.ElementTree as ET
from typing import Dict, Optional, Tuple

from .scene_schema import SceneDescription
from .constraint_solver import Pose
from .metadata_extractor import MetadataExtractor, AssetMetadata

logger = logging.getLogger("mujoco_mcp.scene_gen.xml_builder")


class SceneXMLBuilder:
    """
    Builds complete MuJoCo XML scenes from solved poses and asset metadata.

    Creates a unified scene with:
    - Floor plane and lighting
    - Objects positioned according to solved poses
    - Robots with joint configurations
    - Proper XML structure and naming
    """

    def __init__(self, metadata_extractor: MetadataExtractor, menagerie_loader=None):
        self.metadata_extractor = metadata_extractor
        self.menagerie_loader = menagerie_loader
        self._primitive_densities = {
            "box": 500.0,
            "sphere": 780.0,
            "cylinder": 780.0,
            "capsule": 780.0,
            "ellipsoid": 600.0,
        }
        # Robot model name mapping (scene_gen name -> menagerie name)
        self._robot_model_mapping = {
            "franka_panda": "franka_emika_panda",
        }

    def build_scene(
        self, scene: SceneDescription, poses: Dict[str, Pose], scene_name: str = "structured_scene"
    ) -> str:
        """
        Build complete MuJoCo XML from scene description and solved poses.

        Args:
            scene: Scene description with objects and robots
            poses: Solved poses for all entities
            scene_name: Name for the generated scene

        Returns:
            Complete MuJoCo XML as string
        """
        logger.info(f"Building XML scene '{scene_name}' with {len(poses)} entities")

        # Create root mujoco element
        mujoco = ET.Element("mujoco")
        mujoco.set("model", scene_name)

        # Add basic configuration
        self._add_compiler_config(mujoco)
        self._add_simulation_options(mujoco)
        self._add_defaults(mujoco)
        self._add_assets(mujoco)

        # Create worldbody
        worldbody = ET.SubElement(mujoco, "worldbody")

        # Add floor and lighting
        self._add_floor_and_lighting(worldbody)

        # Add objects
        for obj in scene.objects:
            if obj.object_id in poses:
                self._add_object_to_worldbody(
                    worldbody,
                    obj.object_id,
                    obj.object_type,
                    poses[obj.object_id],
                    dimensions=obj.dimensions,
                    color=obj.color,
                )
            else:
                logger.warning(f"No pose found for object {obj.object_id}")

        # Add robots
        for robot in scene.robots:
            if robot.robot_id in poses:
                self._add_robot_to_worldbody(worldbody, robot, poses[robot.robot_id])
            else:
                logger.warning(f"No pose found for robot {robot.robot_id}")

        # Convert to string with proper formatting
        xml_str = self._prettify_xml(mujoco)
        logger.info(f"Generated XML scene with {len(xml_str)} characters")

        return xml_str

    def _add_compiler_config(self, root: ET.Element):
        """Add compiler configuration."""
        compiler = ET.SubElement(root, "compiler")
        compiler.set("angle", "radian")
        compiler.set("meshdir", ".")
        compiler.set("texturedir", ".")

    def _add_simulation_options(self, root: ET.Element):
        """Add simulation options."""
        option = ET.SubElement(root, "option")
        option.set("timestep", "0.002")
        option.set("integrator", "RK4")
        option.set("gravity", "0 0 -9.81")

    def _add_defaults(self, root: ET.Element):
        """Add default element properties."""
        default = ET.SubElement(root, "default")

        # Default joint properties
        joint_default = ET.SubElement(default, "joint")
        joint_default.set("damping", "0.1")

        # Default geom properties
        geom_default = ET.SubElement(default, "geom")
        geom_default.set("contype", "1")
        geom_default.set("conaffinity", "1")
        geom_default.set("friction", "1 0.005 0.0001")

    def _add_assets(self, root: ET.Element):
        """Add asset definitions (textures, materials)."""
        asset = ET.SubElement(root, "asset")

        # Grid texture for floor
        texture = ET.SubElement(asset, "texture")
        texture.set("name", "grid")
        texture.set("type", "2d")
        texture.set("builtin", "checker")
        texture.set("width", "512")
        texture.set("height", "512")
        texture.set("rgb1", "0.1 0.2 0.3")
        texture.set("rgb2", "0.2 0.3 0.4")

        # Grid material
        material = ET.SubElement(asset, "material")
        material.set("name", "grid")
        material.set("texture", "grid")
        material.set("texrepeat", "8 8")
        material.set("texuniform", "true")
        material.set("reflectance", "0.2")

    def _add_floor_and_lighting(self, worldbody: ET.Element):
        """Add floor plane and lighting to worldbody."""
        # Floor
        floor = ET.SubElement(worldbody, "geom")
        floor.set("name", "floor")
        floor.set("size", "10 10 0.05")
        floor.set("type", "plane")
        floor.set("material", "grid")
        floor.set("pos", "0 0 0")

        # Main light
        light = ET.SubElement(worldbody, "light")
        light.set("name", "main_light")
        light.set("pos", "2 2 3")
        light.set("dir", "-1 -1 -1")
        light.set("diffuse", "0.8 0.8 0.8")
        light.set("specular", "0.3 0.3 0.3")

        # Ambient light
        light2 = ET.SubElement(worldbody, "light")
        light2.set("name", "ambient_light")
        light2.set("pos", "-2 -2 3")
        light2.set("dir", "1 1 -1")
        light2.set("diffuse", "0.4 0.4 0.4")
        light2.set("specular", "0.1 0.1 0.1")

    def _add_object_to_worldbody(
        self,
        worldbody: ET.Element,
        object_id: str,
        object_type: str,
        pose: Pose,
        dimensions: Optional[Dict[str, float]] = None,
        color: Optional[Tuple[float, float, float, float]] = None,
    ):
        """Add an object to the worldbody, supporting parametric primitives."""
        metadata = self.metadata_extractor.get_metadata_with_dimensions(object_type, dimensions)
        if metadata is None:
            metadata = self.metadata_extractor.get_metadata(object_type)

        if metadata and metadata.xml_template:
            # Use template from metadata
            xml_template = metadata.xml_template

            size_str = ""
            rgba_str = ""
            mass_str = ""

            if object_type.startswith("primitive:"):
                dims = dimensions or metadata.get_dimensions()
                size_str = self._compute_primitive_size(object_type, dims, metadata)
                rgba_str = (
                    self._format_rgba(color)
                    if color is not None
                    else self._format_rgba(self._default_primitive_rgba(metadata))
                )
                mass_value = self._compute_primitive_mass(object_type, dims)
                mass_str = f"{mass_value:.6f}"
            else:
                rgba_str = (
                    self._format_rgba(color)
                    if color is not None
                    else self._format_rgba(self._default_primitive_rgba(metadata))
                )

            formatted_xml = xml_template.format(
                name=object_id,
                pos=f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}",
                quat=f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}",
                size=size_str,
                rgba=rgba_str,
                mass=mass_str,
            )

            try:
                # Parse the formatted XML and add to worldbody
                object_element = ET.fromstring(formatted_xml)
                worldbody.append(object_element)
                logger.debug(f"Added object {object_id} using template")
            except ET.ParseError as e:
                logger.warning(f"Failed to parse XML template for {object_id}: {e}")
                self._add_fallback_object(
                    worldbody,
                    object_id,
                    object_type,
                    pose,
                    dimensions=dimensions,
                    color=color,
                )
        else:
            # Use fallback geometry
            self._add_fallback_object(
                worldbody,
                object_id,
                object_type,
                pose,
                dimensions=dimensions,
                color=color,
            )

    def _add_fallback_object(
        self,
        worldbody: ET.Element,
        object_id: str,
        object_type: str,
        pose: Pose,
        dimensions: Optional[Dict[str, float]] = None,
        color: Optional[Tuple[float, float, float, float]] = None,
    ):
        """Add a simple fallback object when template is not available."""
        body = ET.SubElement(worldbody, "body")
        body.set("name", object_id)
        body.set("pos", f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}")
        body.set(
            "quat",
            f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}",
        )

        # Get metadata for sizing
        metadata = self.metadata_extractor.get_metadata_with_dimensions(object_type, dimensions)
        if metadata is None:
            metadata = self.metadata_extractor.get_metadata(object_type)
        if metadata:
            dims = dimensions or metadata.get_dimensions()
            width = dims.get("width", 0.1)
            depth = dims.get("depth", 0.1)
            height = dims.get("height", 0.1)
        else:
            width = depth = height = 0.1
        mass_value = self._compute_fallback_mass(width, depth, height)

        # Create simple box geometry
        geom = ET.SubElement(body, "geom")
        geom.set("name", f"{object_id}_geom")
        geom.set("type", "box")
        geom.set("size", f"{width / 2:.6f} {depth / 2:.6f} {height / 2:.6f}")
        rgba_values = color if color is not None else self._default_primitive_rgba(metadata)
        geom.set("rgba", self._format_rgba(rgba_values))
        geom.set("mass", f"{mass_value:.6f}")
        geom.set("pos", f"0 0 {height / 2:.6f}")

        logger.debug(f"Added fallback object {object_id} as box")

    def _compute_primitive_size(
        self,
        object_type: str,
        dimensions: Dict[str, float],
        metadata: Optional[AssetMetadata] = None,
    ) -> str:
        """Convert primitive dimensions to MuJoCo size attributes."""

        primitive_shape = (metadata.primitive_shape if metadata else None) or object_type.split(
            ":", 1
        )[1]
        shape = primitive_shape.lower()

        if shape == "box":
            return (
                f"{dimensions.get('width', 0.1) / 2:.6f} "
                f"{dimensions.get('depth', 0.1) / 2:.6f} "
                f"{dimensions.get('height', 0.1) / 2:.6f}"
            )

        if shape == "sphere":
            return f"{dimensions.get('radius', 0.05):.6f}"

        if shape == "cylinder":
            return f"{dimensions.get('radius', 0.05):.6f} {dimensions.get('height', 0.1) / 2:.6f}"

        if shape == "capsule":
            return f"{dimensions.get('radius', 0.05):.6f} {dimensions.get('length', 0.1) / 2:.6f}"

        if shape == "ellipsoid":
            return (
                f"{dimensions.get('radius_x', 0.05):.6f} "
                f"{dimensions.get('radius_y', 0.05):.6f} "
                f"{dimensions.get('radius_z', 0.05):.6f}"
            )

        logger.warning(f"Unknown primitive shape '{primitive_shape}' - using fallback size")
        return "0.05 0.05 0.05"

    def _format_rgba(self, color: Tuple[float, float, float, float]) -> str:
        """Format RGBA tuple for MuJoCo XML."""

        return (
            f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {color[3]:.3f}"
            if color is not None
            else "0.700 0.700 0.700 1.000"
        )

    def _default_primitive_rgba(
        self, metadata: Optional[AssetMetadata]
    ) -> Tuple[float, float, float, float]:
        """Provide a default color for primitives if none specified."""

        if metadata and metadata.default_rgba:
            rgba = metadata.default_rgba
            if len(rgba) == 4:
                return (
                    float(rgba[0]),
                    float(rgba[1]),
                    float(rgba[2]),
                    float(rgba[3]),
                )
        return (0.7, 0.7, 0.7, 1.0)

    def _compute_primitive_mass(self, object_type: str, dimensions: Dict[str, float]) -> float:
        primitive_shape = object_type.split(":", 1)[1]
        density = self._primitive_densities.get(primitive_shape, 600.0)
        volume = self._compute_volume(primitive_shape, dimensions)
        mass = density * volume
        return max(mass, 1e-6)

    def _compute_volume(self, primitive_shape: str, dimensions: Dict[str, float]) -> float:
        shape = primitive_shape.lower()

        if shape == "box":
            return (
                dimensions.get("width", 0.1)
                * dimensions.get("depth", 0.1)
                * dimensions.get("height", 0.1)
            )
        if shape == "sphere":
            r = dimensions.get("radius", 0.05)
            return (4.0 / 3.0) * math.pi * r**3
        if shape == "cylinder":
            r = dimensions.get("radius", 0.05)
            h = dimensions.get("height", 0.1)
            return math.pi * r**2 * h
        if shape == "capsule":
            r = dimensions.get("radius", 0.05)
            length = dimensions.get("length", 0.1)
            cylinder_volume = math.pi * r**2 * length
            sphere_volume = (4.0 / 3.0) * math.pi * r**3
            return cylinder_volume + sphere_volume
        if shape == "ellipsoid":
            rx = dimensions.get("radius_x", 0.05)
            ry = dimensions.get("radius_y", 0.05)
            rz = dimensions.get("radius_z", 0.05)
            return (4.0 / 3.0) * math.pi * rx * ry * rz
        logger.warning("Unknown primitive shape '%s' while computing volume", primitive_shape)
        return 0.001

    def _compute_fallback_mass(self, width: float, depth: float, height: float) -> float:
        volume = width * depth * height
        density = self._primitive_densities.get("box", 500.0)
        return max(density * volume, 1e-6)

    def _add_robot_to_worldbody(self, worldbody: ET.Element, robot_config, pose: Pose):
        """Add a robot to the worldbody using file include."""
        robot_id = robot_config.robot_id
        robot_type = robot_config.robot_type

        # Try to load from Menagerie if loader is available
        if self.menagerie_loader:
            # Map scene_gen robot name to menagerie model name
            menagerie_name = self._robot_model_mapping.get(robot_type, robot_type)

            try:
                # Get the path to the robot XML file from Menagerie
                robot_file = self.menagerie_loader.get_model_file(menagerie_name)

                # Create a body wrapper with position/orientation and include the robot
                robot_body = ET.SubElement(worldbody, "body")
                robot_body.set("name", f"{robot_id}_base")
                robot_body.set(
                    "pos", f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}"
                )
                robot_body.set(
                    "quat",
                    f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}",
                )

                # Add include element pointing to the robot file
                include = ET.SubElement(robot_body, "include")
                include.set("file", str(robot_file))

                logger.info(f"Added robot {robot_id} from Menagerie via include: {robot_file}")
                return

            except Exception as e:
                logger.warning(f"Failed to load {robot_type} from Menagerie: {e}")
                logger.info("Falling back to simple robot representation")

        # Fallback: use simple geometric representation
        self._add_fallback_robot(worldbody, robot_id, robot_type, pose)

    def _configure_robot_joints(
        self, robot_element: ET.Element, robot_config, metadata: AssetMetadata
    ):
        """Configure robot joint positions based on configuration."""
        # Get joint configuration
        joint_values = None

        if robot_config.custom_joints:
            joint_values = robot_config.custom_joints
            logger.debug(f"Using custom joint config for {robot_config.robot_id}")
        elif robot_config.joint_config and metadata.joint_configs:
            if robot_config.joint_config in metadata.joint_configs:
                joint_values = metadata.joint_configs[robot_config.joint_config]
                logger.debug(
                    f"Using named joint config '{robot_config.joint_config}' for {robot_config.robot_id}"
                )

        if joint_values:
            # Find joint elements and set positions
            # This is a simplified implementation - a full version would need
            # to match joint names properly
            joints = robot_element.findall(".//joint")
            for i, joint in enumerate(joints):
                if i < len(joint_values):
                    # For revolute joints, we could set initial positions
                    # This is a placeholder - actual implementation depends on joint types
                    joint.set("pos", str(joint_values[i]))

    def _add_fallback_robot(
        self, worldbody: ET.Element, robot_id: str, robot_type: str, pose: Pose
    ):
        """Add a simple fallback robot when template is not available."""
        body = ET.SubElement(worldbody, "body")
        body.set("name", robot_id)
        body.set("pos", f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}")
        body.set(
            "quat",
            f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}",
        )

        # Create simple robot representation (base + arm)
        # Base
        base_geom = ET.SubElement(body, "geom")
        base_geom.set("name", f"{robot_id}_base")
        base_geom.set("type", "cylinder")
        base_geom.set("size", "0.1 0.15")
        base_geom.set("rgba", "0.2 0.2 0.2 1.0")
        base_geom.set("pos", "0 0 0.15")

        # Simplified arm
        arm_geom = ET.SubElement(body, "geom")
        arm_geom.set("name", f"{robot_id}_arm")
        arm_geom.set("type", "capsule")
        arm_geom.set("size", "0.05 0.3")
        arm_geom.set("rgba", "0.8 0.8 0.8 1.0")
        arm_geom.set("pos", "0.3 0 0.4")
        arm_geom.set("euler", "0 1.57 0")  # Point in +X direction

        logger.debug(f"Added fallback robot {robot_id}")

    def _prettify_xml(self, element: ET.Element) -> str:
        """Convert XML element to nicely formatted string."""
        # Create string representation
        rough_string = ET.tostring(element, encoding="unicode")

        # Basic formatting - add newlines after major elements
        formatted = rough_string.replace("><", ">\n<")

        # Simple indentation
        lines = formatted.split("\n")
        indented_lines = []
        indent_level = 0

        for line in lines:
            line = line.strip()
            if not line:
                continue

            # Decrease indent for closing tags
            if line.startswith("</"):
                indent_level = max(0, indent_level - 1)

            # Add indentation
            indented_lines.append("  " * indent_level + line)

            # Increase indent for opening tags (but not self-closing)
            if line.startswith("<") and not line.startswith("</") and not line.endswith("/>"):
                indent_level += 1

        return "\n".join(indented_lines)

    def validate_xml(self, xml_str: str) -> bool:
        """
        Validate that the generated XML is well-formed.

        Args:
            xml_str: XML string to validate

        Returns:
            True if XML is valid, False otherwise
        """
        try:
            ET.fromstring(xml_str)
            return True
        except ET.ParseError as e:
            logger.error(f"XML validation failed: {e}")
            return False
