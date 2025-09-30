#!/usr/bin/env python3
"""
Structured Scene Generation Module

This module provides infrastructure for generating structured MuJoCo scenes
from high-level descriptions or natural language prompts.

Key components:
- Asset metadata extraction and management
- Pydantic scene schema models
- Spatial constraint solving
- XML scene composition
- LLM integration (stubbed by default)
"""

from .scene_schema import (
    SpatialConstraint,
    ObjectPlacement,
    RobotConfiguration,
    SceneDescription,
)
from .shared_types import Pose, AABBBox
from .metadata_extractor import MetadataExtractor
from .constraint_solver import ConstraintSolver
from .scene_xml_builder import SceneXMLBuilder
from .llm_scene_generator import LLMSceneGenerator

# Phase 2A: Enhanced collision detection (optional import)
try:
    from .enhanced_collision import (
        GlobalCollisionOptimizer,
        RotationAwareAABB,
        PhysicsCollisionValidator,
        CollisionInfo
    )
    _enhanced_collision_exports = [
        "GlobalCollisionOptimizer",
        "RotationAwareAABB", 
        "PhysicsCollisionValidator",
        "CollisionInfo"
    ]
except ImportError:
    _enhanced_collision_exports = []

# Phase 2B: Advanced spatial reasoning (optional import)
try:
    from .spatial_reasoning import (
        AdvancedSpatialReasoner,
        StablePoseDatabase,
        RobotReachabilityChecker,
        StablePose,
        WorkspaceVolume,
        SupportSurface
    )
    _spatial_reasoning_exports = [
        "AdvancedSpatialReasoner",
        "StablePoseDatabase",
        "RobotReachabilityChecker", 
        "StablePose",
        "WorkspaceVolume",
        "SupportSurface"
    ]
except ImportError:
    _spatial_reasoning_exports = []

__all__ = [
    "SpatialConstraint",
    "ObjectPlacement", 
    "RobotConfiguration",
    "SceneDescription",
    "Pose",
    "AABBBox", 
    "MetadataExtractor",
    "ConstraintSolver",
    "SceneXMLBuilder", 
    "LLMSceneGenerator",
] + _enhanced_collision_exports + _spatial_reasoning_exports