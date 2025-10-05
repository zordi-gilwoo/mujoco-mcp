#!/usr/bin/env python3
"""
Metadata Extractor

Extracts geometric metadata from MuJoCo XML models and manages asset database.
Provides both static metadata loading and dynamic extraction capabilities.
"""

import json
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Optional, Tuple, List
import numpy as np

# Try to import MuJoCo, fall back to XML parsing only
try:
    import mujoco

    MUJOCO_AVAILABLE = True
except ImportError:
    MUJOCO_AVAILABLE = False
    logging.warning("MuJoCo not available, using XML parsing for metadata extraction")

logger = logging.getLogger("mujoco_mcp.scene_gen.metadata")


class AssetMetadata:
    """Container for asset metadata including dimensions and semantic points."""

    def __init__(self, asset_id: str, asset_type: str, **kwargs):
        self.asset_id = asset_id
        self.asset_type = asset_type
        self.description = kwargs.get("description", "")
        self.default_dimensions = kwargs.get("default_dimensions", {})
        self.semantic_points = kwargs.get("semantic_points", {})
        self.bounding_box = kwargs.get("bounding_box", {"min": [0, 0, 0], "max": [0.1, 0.1, 0.1]})
        self.xml_template = kwargs.get("xml_template", "")
        self.joint_configs = kwargs.get("joint_configs", {})
        self.primitive_shape = kwargs.get("primitive_shape")
        self.composite_shape = kwargs.get("composite_shape")
        self.required_dimensions = kwargs.get("required_dimensions", [])
        self.default_rgba = kwargs.get("default_rgba")

    def get_dimensions(self) -> Dict[str, float]:
        """Get asset dimensions (width, depth, height, etc.)."""
        return self.default_dimensions.copy()

    def get_bounding_box(self) -> Tuple[List[float], List[float]]:
        """Get bounding box as (min_coords, max_coords)."""
        return (self.bounding_box["min"], self.bounding_box["max"])

    def get_semantic_point(self, point_name: str) -> Optional[List[float]]:
        """Get coordinates of a semantic point (e.g., 'top_surface', 'center')."""
        return self.semantic_points.get(point_name)


class MetadataExtractor:
    """
    Extracts and manages asset metadata for scene generation.

    Supports both static metadata from JSON database and dynamic extraction
    from MuJoCo XML files.
    """

    def __init__(self, assets_db_path: Optional[str] = None):
        self.assets_db_path = assets_db_path or self._get_default_db_path()
        self._static_assets: Dict[str, AssetMetadata] = {}
        self._dynamic_cache: Dict[str, AssetMetadata] = {}
        self._load_static_assets()

    def _get_default_db_path(self) -> str:
        """Get default path to assets database."""
        return str(Path(__file__).parent / "assets_db.json")

    def _load_static_assets(self):
        """Load static asset metadata from JSON database."""
        try:
            with open(self.assets_db_path) as f:
                db = json.load(f)

            for asset_id, asset_data in db.get("assets", {}).items():
                # Fix the asset_type parameter name
                asset_data_fixed = asset_data.copy()
                if "type" in asset_data_fixed:
                    asset_data_fixed["asset_type"] = asset_data_fixed.pop("type")

                self._static_assets[asset_id] = AssetMetadata(asset_id=asset_id, **asset_data_fixed)

            logger.info(f"Loaded {len(self._static_assets)} static assets from database")

        except FileNotFoundError:
            logger.warning(f"Assets database not found at {self.assets_db_path}")
        except Exception as e:
            logger.error(f"Failed to load assets database: {e}")

    def get_metadata(self, asset_id: str) -> Optional[AssetMetadata]:
        """
        Get metadata for an asset, checking static database first,
        then dynamic cache.
        """
        # Check static database first
        if asset_id in self._static_assets:
            return self._static_assets[asset_id]

        # Check dynamic cache
        if asset_id in self._dynamic_cache:
            return self._dynamic_cache[asset_id]

        logger.warning(f"No metadata found for asset: {asset_id}")
        return None

    def get_metadata_with_dimensions(
        self,
        asset_id: str,
        dimensions: Optional[Dict[str, float]] = None,
    ) -> Optional[AssetMetadata]:
        """Return metadata customized for provided dimensions (primitives and composites)."""

        metadata = self.get_metadata(asset_id)
        if metadata is None:
            return None

        # Handle primitive types
        if asset_id.startswith("primitive:") and dimensions:
            primitive_shape = metadata.primitive_shape or asset_id.split(":", 1)[1]
            bbox_min, bbox_max = self._compute_primitive_bbox(primitive_shape, dimensions)
            semantic_points = self._compute_primitive_semantic_points(primitive_shape, dimensions)

            return AssetMetadata(
                asset_id=asset_id,
                asset_type=metadata.asset_type,
                description=metadata.description,
                default_dimensions=dimensions,
                semantic_points=semantic_points,
                bounding_box={"min": bbox_min.tolist(), "max": bbox_max.tolist()},
                xml_template=metadata.xml_template,
                primitive_shape=metadata.primitive_shape,
                composite_shape=metadata.composite_shape,
                required_dimensions=metadata.required_dimensions,
                default_rgba=metadata.default_rgba,
                joint_configs=metadata.joint_configs,
            )

        # Handle composite types
        if asset_id.startswith("composite:") and dimensions:
            composite_shape = metadata.composite_shape or asset_id.split(":", 1)[1]
            bbox_min, bbox_max = self._compute_composite_bbox(composite_shape, dimensions)
            semantic_points = self._compute_composite_semantic_points(composite_shape, dimensions)

            return AssetMetadata(
                asset_id=asset_id,
                asset_type=metadata.asset_type,
                description=metadata.description,
                default_dimensions=dimensions,
                semantic_points=semantic_points,
                bounding_box={"min": bbox_min.tolist(), "max": bbox_max.tolist()},
                xml_template=metadata.xml_template,
                primitive_shape=metadata.primitive_shape,
                composite_shape=metadata.composite_shape,
                required_dimensions=metadata.required_dimensions,
                default_rgba=metadata.default_rgba,
                joint_configs=metadata.joint_configs,
            )

        return metadata

    def extract_from_xml(self, xml_str: str, asset_id: str = "unknown") -> AssetMetadata:
        """
        Extract metadata from MuJoCo XML string.

        Uses MuJoCo physics engine if available, otherwise falls back to
        XML parsing heuristics.
        """
        if MUJOCO_AVAILABLE:
            return self._extract_with_mujoco(xml_str, asset_id)
        else:
            return self._extract_with_xml_parsing(xml_str, asset_id)

    def _extract_with_mujoco(self, xml_str: str, asset_id: str) -> AssetMetadata:
        """Extract metadata using MuJoCo physics engine."""
        try:
            # Create model from XML
            model = mujoco.MjModel.from_xml_string(xml_str)

            # Calculate bounding box from all geoms
            bbox_min = np.full(3, float("inf"))
            bbox_max = np.full(3, float("-inf"))

            for i in range(model.ngeom):
                geom_pos = model.geom_pos[i]
                geom_size = model.geom_size[i]
                geom_type = model.geom_type[i]

                # Estimate bounds based on geometry type
                if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                    # Box: size = [x, y, z] half-extents
                    local_min = geom_pos - geom_size[:3]
                    local_max = geom_pos + geom_size[:3]
                elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    # Cylinder: size = [radius, half-height]
                    r, h = geom_size[0], geom_size[1]
                    local_min = geom_pos + np.array([-r, -r, -h])
                    local_max = geom_pos + np.array([r, r, h])
                elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                    # Sphere: size = [radius]
                    r = geom_size[0]
                    local_min = geom_pos - np.array([r, r, r])
                    local_max = geom_pos + np.array([r, r, r])
                else:
                    # Conservative fallback
                    r = max(geom_size[:3]) if len(geom_size) > 0 else 0.1
                    local_min = geom_pos - np.array([r, r, r])
                    local_max = geom_pos + np.array([r, r, r])

                bbox_min = np.minimum(bbox_min, local_min)
                bbox_max = np.maximum(bbox_max, local_max)

            # Compute dimensions and semantic points
            dimensions = {
                "width": float(bbox_max[0] - bbox_min[0]),
                "depth": float(bbox_max[1] - bbox_min[1]),
                "height": float(bbox_max[2] - bbox_min[2]),
            }

            center = ((bbox_min + bbox_max) / 2).tolist()

            semantic_points = {
                "center": center,
                "bottom_surface": [center[0], center[1], float(bbox_min[2])],
                "top_surface": [center[0], center[1], float(bbox_max[2])],
            }

            metadata = AssetMetadata(
                asset_id=asset_id,
                asset_type="extracted",
                description="Extracted from XML via MuJoCo",
                default_dimensions=dimensions,
                semantic_points=semantic_points,
                bounding_box={"min": bbox_min.tolist(), "max": bbox_max.tolist()},
            )

            # Cache for future use
            self._dynamic_cache[asset_id] = metadata
            logger.info(f"Extracted metadata for {asset_id} using MuJoCo")

            return metadata

        except Exception as e:
            logger.warning(f"MuJoCo extraction failed for {asset_id}: {e}")
            return self._extract_with_xml_parsing(xml_str, asset_id)

    def _extract_with_xml_parsing(self, xml_str: str, asset_id: str) -> AssetMetadata:
        """Extract metadata using XML parsing heuristics."""
        try:
            root = ET.fromstring(xml_str)

            # Find all geom elements
            geoms = root.findall(".//geom")

            if not geoms:
                logger.warning(f"No geoms found in XML for {asset_id}")
                return self._create_fallback_metadata(asset_id)

            # Calculate rough bounding box from geom positions and sizes
            bbox_min = np.array([float("inf")] * 3)
            bbox_max = np.array([float("-inf")] * 3)

            for geom in geoms:
                pos = self._parse_vector(geom.get("pos", "0 0 0"))
                size = self._parse_vector(geom.get("size", "0.05 0.05 0.05"))
                geom_type = geom.get("type", "box")

                # Estimate bounds based on type
                if geom_type == "box":
                    local_min = pos - size
                    local_max = pos + size
                elif geom_type == "cylinder":
                    r = size[0] if len(size) > 0 else 0.05
                    h = size[1] if len(size) > 1 else 0.05
                    local_min = pos + np.array([-r, -r, -h])
                    local_max = pos + np.array([r, r, h])
                elif geom_type == "sphere":
                    r = size[0] if len(size) > 0 else 0.05
                    local_min = pos - np.array([r, r, r])
                    local_max = pos + np.array([r, r, r])
                else:
                    # Conservative fallback
                    r = 0.1
                    local_min = pos - np.array([r, r, r])
                    local_max = pos + np.array([r, r, r])

                bbox_min = np.minimum(bbox_min, local_min)
                bbox_max = np.maximum(bbox_max, local_max)

            # Handle case where no valid geoms were processed
            if np.any(np.isinf(bbox_min)) or np.any(np.isinf(bbox_max)):
                return self._create_fallback_metadata(asset_id)

            # Compute metadata
            dimensions = {
                "width": float(bbox_max[0] - bbox_min[0]),
                "depth": float(bbox_max[1] - bbox_min[1]),
                "height": float(bbox_max[2] - bbox_min[2]),
            }

            center = ((bbox_min + bbox_max) / 2).tolist()

            semantic_points = {
                "center": center,
                "bottom_surface": [center[0], center[1], float(bbox_min[2])],
                "top_surface": [center[0], center[1], float(bbox_max[2])],
            }

            metadata = AssetMetadata(
                asset_id=asset_id,
                asset_type="extracted",
                description="Extracted from XML via parsing",
                default_dimensions=dimensions,
                semantic_points=semantic_points,
                bounding_box={"min": bbox_min.tolist(), "max": bbox_max.tolist()},
            )

            self._dynamic_cache[asset_id] = metadata
            logger.info(f"Extracted metadata for {asset_id} using XML parsing")

            return metadata

        except Exception as e:
            logger.warning(f"XML parsing extraction failed for {asset_id}: {e}")
            return self._create_fallback_metadata(asset_id)

    def _parse_vector(self, vector_str: str) -> np.ndarray:
        """Parse vector string like '1.0 2.0 3.0' into numpy array."""
        try:
            return np.array([float(x) for x in vector_str.split()])
        except:
            return np.array([0.0, 0.0, 0.0])

    def _create_fallback_metadata(self, asset_id: str) -> AssetMetadata:
        """Create conservative fallback metadata when extraction fails."""
        logger.warning(f"Using fallback metadata for {asset_id}")

        return AssetMetadata(
            asset_id=asset_id,
            asset_type="fallback",
            description="Fallback metadata - extraction failed",
            default_dimensions={"width": 0.1, "depth": 0.1, "height": 0.1},
            semantic_points={
                "center": [0, 0, 0.05],
                "bottom_surface": [0, 0, 0],
                "top_surface": [0, 0, 0.1],
            },
            bounding_box={"min": [-0.05, -0.05, 0], "max": [0.05, 0.05, 0.1]},
        )

    def _compute_primitive_bbox(
        self,
        primitive_shape: str,
        dimensions: Dict[str, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute axis-aligned bounds for primitive geometry."""

        shape = primitive_shape.lower()

        if shape == "box":
            w = dimensions.get("width", 0.1)
            d = dimensions.get("depth", 0.1)
            h = dimensions.get("height", 0.1)
            return np.array([-w / 2, -d / 2, -h / 2]), np.array([w / 2, d / 2, h / 2])

        if shape == "sphere":
            r = dimensions.get("radius", 0.05)
            return np.array([-r, -r, -r]), np.array([r, r, r])

        if shape == "cylinder":
            r = dimensions.get("radius", 0.05)
            h = dimensions.get("height", 0.1)
            half_h = h / 2
            return np.array([-r, -r, -half_h]), np.array([r, r, half_h])

        if shape == "capsule":
            r = dimensions.get("radius", 0.05)
            length = dimensions.get("length", 0.1)
            half_height = (length / 2) + r
            return np.array([-r, -r, -half_height]), np.array([r, r, half_height])

        if shape == "ellipsoid":
            rx = dimensions.get("radius_x", 0.05)
            ry = dimensions.get("radius_y", 0.05)
            rz = dimensions.get("radius_z", 0.05)
            return np.array([-rx, -ry, -rz]), np.array([rx, ry, rz])

        logger.warning(f"Unknown primitive shape '{primitive_shape}' - using fallback bounds")
        return np.array([-0.05, -0.05, 0.0]), np.array([0.05, 0.05, 0.1])

    def _compute_primitive_semantic_points(
        self,
        primitive_shape: str,
        dimensions: Dict[str, float],
    ) -> Dict[str, List[float]]:
        """Compute semantic landmarks for primitive geometry."""

        shape = primitive_shape.lower()

        if shape == "box":
            h = dimensions.get("height", 0.1)
            half_h = h / 2
            return {
                "top_surface": [0.0, 0.0, half_h],
                "bottom_surface": [0.0, 0.0, -half_h],
                "center": [0.0, 0.0, 0.0],
            }

        if shape == "sphere":
            r = dimensions.get("radius", 0.05)
            return {
                "top_surface": [0.0, 0.0, r],
                "bottom_surface": [0.0, 0.0, -r],
                "center": [0.0, 0.0, 0.0],
            }

        if shape == "cylinder":
            h = dimensions.get("height", 0.1)
            half_h = h / 2
            return {
                "top_surface": [0.0, 0.0, half_h],
                "bottom_surface": [0.0, 0.0, -half_h],
                "center": [0.0, 0.0, 0.0],
            }

        if shape == "capsule":
            r = dimensions.get("radius", 0.05)
            length = dimensions.get("length", 0.1)
            half_height = (length / 2) + r
            return {
                "top_surface": [0.0, 0.0, half_height],
                "bottom_surface": [0.0, 0.0, -half_height],
                "center": [0.0, 0.0, 0.0],
            }

        if shape == "ellipsoid":
            rz = dimensions.get("radius_z", 0.05)
            return {
                "top_surface": [0.0, 0.0, rz],
                "bottom_surface": [0.0, 0.0, -rz],
                "center": [0.0, 0.0, 0.0],
            }

        return {
            "center": [0.0, 0.0, 0.0],
            "bottom_surface": [0.0, 0.0, 0.0],
            "top_surface": [0.0, 0.0, 0.0],
        }

    def _compute_composite_bbox(
        self,
        composite_shape: str,
        dimensions: Dict[str, float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute axis-aligned bounds for composite geometry (bins, totes, shelves)."""

        shape = composite_shape.lower()

        if shape in ["bin", "tote"]:
            w = dimensions.get("width", 0.4)
            d = dimensions.get("depth", 0.4)
            h = dimensions.get("height", 0.3)
            # Bounding box includes walls, floor sits at z=0
            return np.array([-w / 2, -d / 2, 0.0]), np.array([w / 2, d / 2, h])

        if shape == "shelf":
            w = dimensions.get("width", 0.6)
            d = dimensions.get("depth", 0.3)
            h = dimensions.get("height", 0.9)
            # Shelf has back and side walls, front open
            return np.array([-w / 2, -d / 2, 0.0]), np.array([w / 2, d / 2, h])

        # Fallback
        return np.array([-0.2, -0.2, 0.0]), np.array([0.2, 0.2, 0.3])

    def _compute_composite_semantic_points(
        self,
        composite_shape: str,
        dimensions: Dict[str, float],
    ) -> Dict[str, List[float]]:
        """Compute semantic points for composite geometry."""

        shape = composite_shape.lower()

        if shape in ["bin", "tote"]:
            w = dimensions.get("width", 0.4)
            d = dimensions.get("depth", 0.4)
            h = dimensions.get("height", 0.3)
            t = dimensions.get("wall_thickness", 0.01)

            return {
                "inside_bottom": [0.0, 0.0, t],  # Interior floor
                "inside_center": [0.0, 0.0, h / 2],  # Center of interior volume
                "top_rim": [0.0, 0.0, h],  # Top edge
                "bottom_surface": [0.0, 0.0, 0.0],  # Exterior bottom
                "center": [0.0, 0.0, h / 2],  # Overall center
            }

        if shape == "shelf":
            w = dimensions.get("width", 0.6)
            d = dimensions.get("depth", 0.3)
            h = dimensions.get("height", 0.9)
            t = dimensions.get("wall_thickness", 0.01)
            num_shelves = int(dimensions.get("num_shelves", 2))

            semantic_points = {
                "bottom_surface": [0.0, 0.0, 0.0],
                "center": [0.0, 0.0, h / 2],
                "front_center": [0.0, -d / 2, h / 2],  # Front opening center
                "shelf_0": [0.0, 0.0, t],  # Bottom shelf surface
            }

            # Add interior shelf positions
            if num_shelves > 0:
                shelf_spacing = (h - 2 * t) / (num_shelves + 1)
                for i in range(1, num_shelves + 1):
                    shelf_z = t + i * shelf_spacing
                    semantic_points[f"shelf_{i}"] = [0.0, 0.0, shelf_z]

            # Top shelf
            semantic_points["shelf_top"] = [0.0, 0.0, h - t]

            return semantic_points

        # Fallback
        return {
            "center": [0.0, 0.0, 0.15],
            "bottom_surface": [0.0, 0.0, 0.0],
            "top_surface": [0.0, 0.0, 0.3],
        }

    def ensure_metadata(self, asset_id: str, xml_path: Optional[str] = None) -> AssetMetadata:
        """
        Ensure metadata exists for an asset, extracting if necessary.

        Args:
            asset_id: Asset identifier
            xml_path: Optional path to XML file for dynamic extraction

        Returns:
            Asset metadata
        """
        metadata = self.get_metadata(asset_id)
        if metadata is not None:
            return metadata

        # Try to extract from XML file if path provided
        if xml_path and Path(xml_path).exists():
            try:
                with open(xml_path) as f:
                    xml_str = f.read()
                return self.extract_from_xml(xml_str, asset_id)
            except Exception as e:
                logger.error(f"Failed to read XML file {xml_path}: {e}")

        # Create fallback metadata
        return self._create_fallback_metadata(asset_id)

    def get_available_assets(self) -> List[str]:
        """Get list of all available asset IDs."""
        return list(self._static_assets.keys()) + list(self._dynamic_cache.keys())
