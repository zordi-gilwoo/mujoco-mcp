#!/usr/bin/env python3
"""
Enhanced Asset Semantics

Implements Phase 2D of the enhanced scene generation system:
1. Rich asset database with stable poses, grasp sites, support surfaces
2. Workspace envelope definitions
3. Robot mounting rules and configuration hints

This addresses the limitation in PR #12 of minimal semantic richness in the asset system.
"""

import json
import logging
import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, field
from pathlib import Path
from enum import Enum

from .metadata_extractor import AssetMetadata, MetadataExtractor
from .spatial_reasoning import SupportSurface, StablePose

logger = logging.getLogger("mujoco_mcp.scene_gen.enhanced_semantics")


class AssetCategory(Enum):
    """Categories of assets for semantic organization."""

    FURNITURE = "furniture"
    TABLEWARE = "tableware"
    CONTAINER = "container"
    TOOL = "tool"
    ROBOT = "robot"
    SENSOR = "sensor"
    OBSTACLE = "obstacle"


class GraspType(Enum):
    """Types of grasp affordances."""

    TOP_GRASP = "top_grasp"
    SIDE_GRASP = "side_grasp"
    HANDLE_GRASP = "handle_grasp"
    PINCH_GRASP = "pinch_grasp"
    WRAP_GRASP = "wrap_grasp"


class SurfaceType(Enum):
    """Types of support surfaces."""

    HORIZONTAL_STABLE = "horizontal_stable"
    HORIZONTAL_UNSTABLE = "horizontal_unstable"
    VERTICAL = "vertical"
    INCLINED = "inclined"
    CURVED = "curved"


@dataclass
class GraspAffordance:
    """Represents a grasp affordance on an object."""

    name: str
    grasp_type: GraspType
    position: np.ndarray  # [x, y, z] relative to object center
    orientation: np.ndarray  # quaternion [x, y, z, w] for gripper approach
    approach_direction: np.ndarray  # unit vector for approach
    grasp_width: float  # Required gripper opening
    force_closure_quality: float  # 0.0 to 1.0, higher is better
    accessibility_score: float  # 0.0 to 1.0, how easily accessible
    description: str = ""

    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.position = np.array(self.position, dtype=float)
        self.orientation = np.array(self.orientation, dtype=float)
        self.approach_direction = np.array(self.approach_direction, dtype=float)


@dataclass
class SupportSurfaceInfo:
    """Detailed information about a support surface."""

    name: str
    surface_type: SurfaceType
    center_position: np.ndarray  # [x, y, z] relative to object center
    normal_direction: np.ndarray  # unit vector pointing outward from surface
    dimensions: np.ndarray  # [width, depth] or [radius] for circular
    load_capacity: float  # Maximum load in kg
    stability_factor: float  # 0.0 to 1.0, stability of objects placed on this surface
    material_friction: float  # Coefficient of friction
    is_primary_function: bool = False  # True if this is the main support surface
    description: str = ""

    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.center_position = np.array(self.center_position, dtype=float)
        self.normal_direction = np.array(self.normal_direction, dtype=float)
        self.dimensions = np.array(self.dimensions, dtype=float)


@dataclass
class WorkspaceEnvelope:
    """Defines workspace envelope for robot operations."""

    name: str
    center: np.ndarray  # [x, y, z] center of workspace
    extents: np.ndarray  # [width, depth, height] extents
    orientation: np.ndarray  # quaternion [x, y, z, w] workspace orientation
    reachability_zones: Dict[str, float]  # zone_name -> reachability_score
    task_preferences: Dict[str, float]  # task_type -> preference_score
    collision_margins: Dict[str, float]  # direction -> minimum_clearance
    description: str = ""

    def __post_init__(self):
        """Ensure arrays are numpy arrays."""
        self.center = np.array(self.center, dtype=float)
        self.extents = np.array(self.extents, dtype=float)
        self.orientation = np.array(self.orientation, dtype=float)


@dataclass
class RobotMountingRule:
    """Rules for robot base placement and mounting."""

    surface_requirements: List[str]  # Required surface types for mounting
    minimum_clearances: Dict[str, float]  # direction -> minimum_clearance_meters
    preferred_orientations: List[np.ndarray]  # List of preferred quaternion orientations
    workspace_zones: List[WorkspaceEnvelope]  # Preferred workspace envelopes
    stability_requirements: Dict[str, float]  # requirement -> minimum_value
    description: str = ""

    def __post_init__(self):
        """Ensure orientation arrays are numpy arrays."""
        self.preferred_orientations = [
            np.array(orient, dtype=float) for orient in self.preferred_orientations
        ]


class EnhancedAssetMetadata(AssetMetadata):
    """
    Enhanced asset metadata with rich semantic information.
    Extends the basic AssetMetadata class from metadata_extractor.
    """

    def __init__(self, asset_id: str, asset_type: str, **kwargs):
        super().__init__(asset_id, asset_type, **kwargs)

        # Phase 2D: Enhanced semantic properties
        self.category = AssetCategory(kwargs.get("category", "obstacle"))
        self.grasp_affordances: List[GraspAffordance] = kwargs.get("grasp_affordances", [])
        self.support_surfaces: List[SupportSurfaceInfo] = kwargs.get("support_surfaces", [])
        self.stable_poses_enhanced: List[StablePose] = kwargs.get("stable_poses_enhanced", [])
        self.workspace_envelopes: List[WorkspaceEnvelope] = kwargs.get("workspace_envelopes", [])
        self.robot_mounting_rules: Optional[RobotMountingRule] = kwargs.get("robot_mounting_rules")

        # Functional properties
        self.manipulation_affordances = kwargs.get("manipulation_affordances", {})
        self.interaction_properties = kwargs.get("interaction_properties", {})
        self.safety_constraints = kwargs.get("safety_constraints", {})

        # Physical properties
        self.mass = kwargs.get("mass", 1.0)  # kg
        self.material_properties = kwargs.get("material_properties", {})
        self.center_of_mass = np.array(kwargs.get("center_of_mass", [0, 0, 0]))

    def get_grasp_affordances(
        self, grasp_type: Optional[GraspType] = None
    ) -> List[GraspAffordance]:
        """Get grasp affordances, optionally filtered by type."""
        if grasp_type is None:
            return self.grasp_affordances
        return [grasp for grasp in self.grasp_affordances if grasp.grasp_type == grasp_type]

    def get_best_grasp_affordance(
        self, grasp_type: Optional[GraspType] = None
    ) -> Optional[GraspAffordance]:
        """Get the best grasp affordance based on quality scores."""
        candidates = self.get_grasp_affordances(grasp_type)
        if not candidates:
            return None

        # Score based on force closure quality and accessibility
        def score_grasp(grasp):
            return (grasp.force_closure_quality + grasp.accessibility_score) / 2

        return max(candidates, key=score_grasp)

    def get_support_surfaces(
        self, surface_type: Optional[SurfaceType] = None, primary_only: bool = False
    ) -> List[SupportSurfaceInfo]:
        """Get support surfaces, optionally filtered by type."""
        surfaces = self.support_surfaces

        if surface_type is not None:
            surfaces = [s for s in surfaces if s.surface_type == surface_type]

        if primary_only:
            surfaces = [s for s in surfaces if s.is_primary_function]

        return surfaces

    def get_primary_support_surface(self) -> Optional[SupportSurfaceInfo]:
        """Get the primary support surface."""
        primary_surfaces = self.get_support_surfaces(primary_only=True)
        if primary_surfaces:
            return primary_surfaces[0]

        # Fallback to any horizontal stable surface
        horizontal_surfaces = self.get_support_surfaces(SurfaceType.HORIZONTAL_STABLE)
        if horizontal_surfaces:
            return horizontal_surfaces[0]

        return None

    def is_graspable(self) -> bool:
        """Check if the object has grasp affordances."""
        return len(self.grasp_affordances) > 0

    def is_support_surface(self) -> bool:
        """Check if the object can support other objects."""
        return len(self.support_surfaces) > 0

    def can_support_load(self, load_kg: float) -> bool:
        """Check if object can support a given load."""
        max_capacity = max((s.load_capacity for s in self.support_surfaces), default=0.0)
        return max_capacity >= load_kg


class EnhancedAssetDatabase:
    """
    Enhanced asset database with rich semantic information.
    Loads and manages enhanced asset metadata.
    """

    def __init__(self, database_path: Optional[str] = None):
        self.database_path = database_path or self._get_default_database_path()
        self.enhanced_assets: Dict[str, EnhancedAssetMetadata] = {}
        self._load_enhanced_database()

    def _get_default_database_path(self) -> str:
        """Get the default path to the enhanced asset database."""
        current_dir = Path(__file__).parent
        return str(current_dir / "enhanced_assets_db.json")

    def _load_enhanced_database(self):
        """Load the enhanced asset database."""
        try:
            if Path(self.database_path).exists():
                with open(self.database_path, "r") as f:
                    data = json.load(f)

                for asset_id, asset_data in data.get("assets", {}).items():
                    enhanced_metadata = self._create_enhanced_metadata(asset_id, asset_data)
                    self.enhanced_assets[asset_id] = enhanced_metadata

                logger.info(
                    f"Loaded {len(self.enhanced_assets)} enhanced assets from {self.database_path}"
                )
            else:
                logger.warning(
                    f"Enhanced asset database not found at {self.database_path}, creating default assets"
                )
                self._create_default_enhanced_assets()

        except Exception as e:
            logger.error(f"Error loading enhanced asset database: {e}")
            self._create_default_enhanced_assets()

    def _create_enhanced_metadata(
        self, asset_id: str, asset_data: Dict[str, Any]
    ) -> EnhancedAssetMetadata:
        """Create enhanced metadata from database entry."""
        # Convert grasp affordances
        grasp_affordances = []
        for grasp_data in asset_data.get("grasp_affordances", []):
            grasp = GraspAffordance(
                name=grasp_data["name"],
                grasp_type=GraspType(grasp_data["grasp_type"]),
                position=np.array(grasp_data["position"]),
                orientation=np.array(grasp_data["orientation"]),
                approach_direction=np.array(grasp_data["approach_direction"]),
                grasp_width=grasp_data["grasp_width"],
                force_closure_quality=grasp_data["force_closure_quality"],
                accessibility_score=grasp_data["accessibility_score"],
                description=grasp_data.get("description", ""),
            )
            grasp_affordances.append(grasp)

        # Convert support surfaces
        support_surfaces = []
        for surface_data in asset_data.get("support_surfaces", []):
            surface = SupportSurfaceInfo(
                name=surface_data["name"],
                surface_type=SurfaceType(surface_data["surface_type"]),
                center_position=np.array(surface_data["center_position"]),
                normal_direction=np.array(surface_data["normal_direction"]),
                dimensions=np.array(surface_data["dimensions"]),
                load_capacity=surface_data["load_capacity"],
                stability_factor=surface_data["stability_factor"],
                material_friction=surface_data["material_friction"],
                is_primary_function=surface_data.get("is_primary_function", False),
                description=surface_data.get("description", ""),
            )
            support_surfaces.append(surface)

        # Convert workspace envelopes
        workspace_envelopes = []
        for workspace_data in asset_data.get("workspace_envelopes", []):
            workspace = WorkspaceEnvelope(
                name=workspace_data["name"],
                center=np.array(workspace_data["center"]),
                extents=np.array(workspace_data["extents"]),
                orientation=np.array(workspace_data["orientation"]),
                reachability_zones=workspace_data.get("reachability_zones", {}),
                task_preferences=workspace_data.get("task_preferences", {}),
                collision_margins=workspace_data.get("collision_margins", {}),
                description=workspace_data.get("description", ""),
            )
            workspace_envelopes.append(workspace)

        # Convert robot mounting rules
        robot_mounting_rules = None
        if "robot_mounting_rules" in asset_data:
            rules_data = asset_data["robot_mounting_rules"]
            robot_mounting_rules = RobotMountingRule(
                surface_requirements=rules_data["surface_requirements"],
                minimum_clearances=rules_data["minimum_clearances"],
                preferred_orientations=[
                    np.array(orient) for orient in rules_data["preferred_orientations"]
                ],
                workspace_zones=[],  # Would be populated from workspace_envelopes
                stability_requirements=rules_data.get("stability_requirements", {}),
                description=rules_data.get("description", ""),
            )

        return EnhancedAssetMetadata(
            asset_id=asset_id,
            asset_type=asset_data.get("type", "object"),
            category=asset_data.get("category", "obstacle"),
            description=asset_data.get("description", ""),
            default_dimensions=asset_data.get("default_dimensions", {}),
            semantic_points=asset_data.get("semantic_points", {}),
            bounding_box=asset_data.get("bounding_box", {"min": [0, 0, 0], "max": [0.1, 0.1, 0.1]}),
            xml_template=asset_data.get("xml_template", ""),
            joint_configs=asset_data.get("joint_configs", {}),
            grasp_affordances=grasp_affordances,
            support_surfaces=support_surfaces,
            workspace_envelopes=workspace_envelopes,
            robot_mounting_rules=robot_mounting_rules,
            manipulation_affordances=asset_data.get("manipulation_affordances", {}),
            interaction_properties=asset_data.get("interaction_properties", {}),
            safety_constraints=asset_data.get("safety_constraints", {}),
            mass=asset_data.get("mass", 1.0),
            material_properties=asset_data.get("material_properties", {}),
            center_of_mass=asset_data.get("center_of_mass", [0, 0, 0]),
        )

    def _create_default_enhanced_assets(self):
        """Create default enhanced assets if database doesn't exist."""
        # Enhanced table with rich semantics
        table_grasp = GraspAffordance(
            name="table_edge_grasp",
            grasp_type=GraspType.SIDE_GRASP,
            position=np.array([0.6, 0, 0.75]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([-1, 0, 0]),
            grasp_width=0.02,
            force_closure_quality=0.6,
            accessibility_score=0.8,
            description="Grasp edge of table for repositioning",
        )

        table_surface = SupportSurfaceInfo(
            name="table_top",
            surface_type=SurfaceType.HORIZONTAL_STABLE,
            center_position=np.array([0, 0, 0.75]),
            normal_direction=np.array([0, 0, 1]),
            dimensions=np.array([1.2, 0.8]),
            load_capacity=50.0,
            stability_factor=0.95,
            material_friction=0.7,
            is_primary_function=True,
            description="Primary work surface for object placement",
        )

        self.enhanced_assets["table_standard"] = EnhancedAssetMetadata(
            asset_id="table_standard",
            asset_type="table",
            category=AssetCategory.FURNITURE,
            description="Standard rectangular work table with enhanced semantics",
            default_dimensions={"width": 1.2, "depth": 0.8, "height": 0.75},
            semantic_points={"top_surface": [0, 0, 0.75], "center": [0, 0, 0.375]},
            bounding_box={"min": [-0.6, -0.4, 0], "max": [0.6, 0.4, 0.75]},
            grasp_affordances=[table_grasp],
            support_surfaces=[table_surface],
            mass=25.0,
            material_properties={"friction": 0.7, "hardness": 0.8},
        )

        # Enhanced cup with multiple grasp options
        cup_handle_grasp = GraspAffordance(
            name="cup_handle",
            grasp_type=GraspType.HANDLE_GRASP,
            position=np.array([0.045, 0, 0.05]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([1, 0, 0]),
            grasp_width=0.02,
            force_closure_quality=0.9,
            accessibility_score=0.9,
            description="Primary handle grasp for manipulation",
        )

        cup_top_grasp = GraspAffordance(
            name="cup_rim",
            grasp_type=GraspType.TOP_GRASP,
            position=np.array([0, 0, 0.08]),
            orientation=np.array([0, 0, 0, 1]),
            approach_direction=np.array([0, 0, -1]),
            grasp_width=0.08,
            force_closure_quality=0.7,
            accessibility_score=0.8,
            description="Top grasp for careful manipulation",
        )

        self.enhanced_assets["cup_ceramic_small"] = EnhancedAssetMetadata(
            asset_id="cup_ceramic_small",
            asset_type="cup",
            category=AssetCategory.TABLEWARE,
            description="Small ceramic cup with handle and rim grasp options",
            default_dimensions={"radius": 0.04, "height": 0.08},
            semantic_points={"top_rim": [0, 0, 0.08], "center": [0, 0, 0.04]},
            bounding_box={"min": [-0.04, -0.04, 0], "max": [0.04, 0.04, 0.08]},
            grasp_affordances=[cup_handle_grasp, cup_top_grasp],
            mass=0.2,
            material_properties={"fragility": 0.8, "friction": 0.6},
        )

        # Enhanced robot with workspace envelopes
        robot_workspace = WorkspaceEnvelope(
            name="primary_manipulation_zone",
            center=np.array([0.6, 0.0, 0.5]),
            extents=np.array([1.0, 1.2, 0.8]),
            orientation=np.array([0, 0, 0, 1]),
            reachability_zones={"optimal": 0.9, "good": 0.7, "marginal": 0.4},
            task_preferences={"pick_and_place": 0.9, "assembly": 0.8, "inspection": 0.7},
            collision_margins={"front": 0.1, "sides": 0.15, "back": 0.2},
            description="Primary manipulation workspace for Franka Panda",
        )

        robot_mounting = RobotMountingRule(
            surface_requirements=["horizontal_stable"],
            minimum_clearances={"all_directions": 0.3, "front": 0.5},
            preferred_orientations=[np.array([0, 0, 0, 1])],
            workspace_zones=[robot_workspace],
            stability_requirements={"surface_load_capacity": 100.0},
            description="Mounting requirements for stable robot operation",
        )

        self.enhanced_assets["franka_panda"] = EnhancedAssetMetadata(
            asset_id="franka_panda",
            asset_type="robot",
            category=AssetCategory.ROBOT,
            description="Franka Emika Panda with enhanced workspace semantics",
            default_dimensions={"base_radius": 0.15, "reach": 0.855, "height": 1.0},
            semantic_points={"base": [0, 0, 0], "end_effector": [0.5, 0, 0.5]},
            bounding_box={"min": [-0.855, -0.855, 0], "max": [0.855, 0.855, 1.0]},
            workspace_envelopes=[robot_workspace],
            robot_mounting_rules=robot_mounting,
            joint_configs={
                "ready": [0, -0.785, 0, -2.356, 0, 1.571, 0.785],
                "home": [0, 0, 0, 0, 0, 0, 0],
                "tucked": [1.571, 0.785, 0, -1.571, 0, 2.356, 0.785],
            },
            mass=18.0,
            safety_constraints={"max_velocity": 2.0, "max_acceleration": 10.0},
        )

        logger.info("Created default enhanced assets")

    def get_enhanced_metadata(self, asset_id: str) -> Optional[EnhancedAssetMetadata]:
        """Get enhanced metadata for an asset."""
        return self.enhanced_assets.get(asset_id)

    def get_assets_by_category(self, category: AssetCategory) -> List[EnhancedAssetMetadata]:
        """Get all assets in a specific category."""
        return [asset for asset in self.enhanced_assets.values() if asset.category == category]

    def get_graspable_assets(self) -> List[EnhancedAssetMetadata]:
        """Get all assets that have grasp affordances."""
        return [asset for asset in self.enhanced_assets.values() if asset.is_graspable()]

    def get_support_surface_assets(self) -> List[EnhancedAssetMetadata]:
        """Get all assets that can support other objects."""
        return [asset for asset in self.enhanced_assets.values() if asset.is_support_surface()]

    def find_compatible_support(
        self, object_mass: float, required_stability: float = 0.8
    ) -> List[EnhancedAssetMetadata]:
        """Find assets that can support an object with given requirements."""
        compatible = []
        for asset in self.enhanced_assets.values():
            if asset.can_support_load(object_mass):
                primary_surface = asset.get_primary_support_surface()
                if primary_surface and primary_surface.stability_factor >= required_stability:
                    compatible.append(asset)
        return compatible

    def get_available_assets(self) -> List[str]:
        """Get list of all available asset IDs (for compatibility with basic database)."""
        return list(self.enhanced_assets.keys())

    def list_assets(self) -> List[str]:
        """List all available asset IDs."""
        return list(self.enhanced_assets.keys())
