#!/usr/bin/env python3
"""
Enhanced Structured Scene Generation Module

This module provides comprehensive infrastructure for generating structured MuJoCo scenes
from high-level descriptions or natural language prompts.

Enhanced features (Phases 2A-2E):
- Enhanced collision detection with rotation-aware AABB
- Advanced spatial reasoning with stable poses and reachability
- Symbolic plan interface for auditable NL→Plan→Scene separation
- Rich asset semantics with grasp sites and support surfaces
- Robust constraint solver with backtracking and global optimization
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

# Phase 2A: Enhanced collision detection
from .enhanced_collision import (
    GlobalCollisionOptimizer,
    RotationAwareAABB,
    PhysicsCollisionValidator,
    CollisionInfo
)

# Phase 2B: Advanced spatial reasoning
from .spatial_reasoning import (
    AdvancedSpatialReasoner,
    StablePoseDatabase,
    RobotReachabilityChecker,
    StablePose,
    WorkspaceVolume,
    SupportSurface
)

# Phase 2C: Symbolic plan interface
from .symbolic_plan import (
    SymbolicPlan,
    SymbolicOperation,
    SymbolicPlanGenerator,
    PlanToSceneConverter,
    PlanValidator,
    OperationType,
    ConstraintCategory
)

# Phase 2D: Enhanced asset semantics
from .enhanced_semantics import (
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

# Phase 2E: Robust constraint solver
from .robust_solver import (
    RobustConstraintSolver,
    ConstraintConflict,
    PlacementSolution,
    GlobalSceneState,
    ConstraintConflictType
)

__all__ = [
    # Core components
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
    
    # Phase 2A: Enhanced collision detection
    "GlobalCollisionOptimizer",
    "RotationAwareAABB", 
    "PhysicsCollisionValidator",
    "CollisionInfo",
    
    # Phase 2B: Advanced spatial reasoning
    "AdvancedSpatialReasoner",
    "StablePoseDatabase",
    "RobotReachabilityChecker", 
    "StablePose",
    "WorkspaceVolume",
    "SupportSurface",
    
    # Phase 2C: Symbolic plan interface
    "SymbolicPlan",
    "SymbolicOperation", 
    "SymbolicPlanGenerator",
    "PlanToSceneConverter",
    "PlanValidator",
    "OperationType",
    "ConstraintCategory",
    
    # Phase 2D: Enhanced asset semantics
    "EnhancedAssetMetadata",
    "EnhancedAssetDatabase",
    "GraspAffordance",
    "SupportSurfaceInfo",
    "WorkspaceEnvelope",
    "RobotMountingRule",
    "AssetCategory",
    "GraspType",
    "SurfaceType",
    
    # Phase 2E: Robust constraint solver
    "RobustConstraintSolver",
    "ConstraintConflict",
    "PlacementSolution", 
    "GlobalSceneState",
    "ConstraintConflictType",
]