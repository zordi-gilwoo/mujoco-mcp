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
from .metadata_extractor import MetadataExtractor
from .constraint_solver import ConstraintSolver
from .scene_xml_builder import SceneXMLBuilder
from .llm_scene_generator import LLMSceneGenerator

__all__ = [
    "SpatialConstraint",
    "ObjectPlacement", 
    "RobotConfiguration",
    "SceneDescription",
    "MetadataExtractor",
    "ConstraintSolver",
    "SceneXMLBuilder", 
    "LLMSceneGenerator",
]