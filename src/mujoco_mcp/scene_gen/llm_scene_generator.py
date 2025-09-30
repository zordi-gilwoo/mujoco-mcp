#!/usr/bin/env python3
"""
LLM Scene Generator

Generates scene descriptions from natural language prompts.
Includes stubbed LLM integration with environment variable gating.
"""

import json
import logging
import os
from typing import Dict, Any, Optional

from .scene_schema import SceneDescription

logger = logging.getLogger("mujoco_mcp.scene_gen.llm_generator")


class LLMSceneGenerator:
    """
    Generates scene descriptions from natural language using LLM integration.
    
    By default, returns canned examples. Real LLM integration can be enabled
    via environment variable STRUCTURED_SCENE_LLM=enabled.
    """
    
    def __init__(self):
        self.llm_enabled = os.getenv("STRUCTURED_SCENE_LLM", "disabled").lower() == "enabled"
        if self.llm_enabled:
            logger.info("LLM integration enabled - real LLM calls will be made")
        else:
            logger.info("LLM integration disabled - using canned examples")
    
    def generate_scene_description(self, prompt: str) -> SceneDescription:
        """
        Generate a scene description from natural language prompt.
        
        Args:
            prompt: Natural language description of desired scene
            
        Returns:
            SceneDescription object
            
        Raises:
            NotImplementedError: If LLM integration is enabled but not implemented
        """
        if self.llm_enabled:
            return self._generate_with_llm(prompt)
        else:
            return self._generate_canned_example(prompt)
    
    def generate_or_stub(self, prompt: str) -> Dict[str, Any]:
        """
        Generate scene description and return as JSON dict.
        
        This is the main entry point used by the MCP tool.
        
        Args:
            prompt: Natural language description
            
        Returns:
            Scene description as JSON-compatible dict
        """
        try:
            scene = self.generate_scene_description(prompt)
            return scene.model_dump()
        except Exception as e:
            logger.error(f"Failed to generate scene from prompt: {e}")
            # Return fallback canned example
            return self._get_canned_example_dict()
    
    def _generate_with_llm(self, prompt: str) -> SceneDescription:
        """
        Generate scene using real LLM integration.
        
        TODO: Implement actual LLM integration here.
        This could use OpenAI API, local models, or other LLM services.
        """
        raise NotImplementedError(
            "Real LLM integration not yet implemented. "
            "Set STRUCTURED_SCENE_LLM=disabled to use canned examples."
        )
    
    def _generate_canned_example(self, prompt: str) -> SceneDescription:
        """Generate a canned example scene based on prompt keywords."""
        # Analyze prompt for keywords to select appropriate example
        prompt_lower = prompt.lower()
        
        if any(word in prompt_lower for word in ["table", "cup", "robot", "arm"]):
            return self._get_table_cup_robot_scene()
        elif any(word in prompt_lower for word in ["collision", "box", "test"]):
            return self._get_collision_test_scene()
        elif any(word in prompt_lower for word in ["simple", "basic", "minimal"]):
            return self._get_simple_scene()
        else:
            # Default to table + cup + robot scene
            return self._get_table_cup_robot_scene()
    
    def _get_table_cup_robot_scene(self) -> SceneDescription:
        """Get the main canned example: table + cup + robot."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1", 
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.002
                        }
                    ]
                }
            ],
            "robots": [
                {
                    "robot_id": "franka_1",
                    "robot_type": "franka_panda",
                    "base_position": [0.6, 0.0, 0.0],
                    "base_orientation": [0, 0, 0, 1],
                    "joint_config": "ready",
                    "constraints": [
                        {
                            "type": "in_front_of", 
                            "subject": "franka_1",
                            "reference": "table_1",
                            "clearance": 0.15
                        }
                    ]
                }
            ],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0]
        }
        
        return SceneDescription(**scene_dict)
    
    def _get_collision_test_scene(self) -> SceneDescription:
        """Get collision testing scene with overlapping boxes."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard", 
                    "constraints": []
                },
                {
                    "object_id": "box_1",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_1", 
                            "reference": "table_1",
                            "clearance": 0.001
                        }
                    ]
                },
                {
                    "object_id": "box_2",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_2",
                            "reference": "table_1",
                            "clearance": 0.001
                        },
                        {
                            "type": "no_collision",
                            "subject": "box_2",
                            "reference": "box_1",
                            "clearance": 0.05
                        }
                    ]
                }
            ],
            "robots": [],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0]
        }
        
        return SceneDescription(**scene_dict)
    
    def _get_simple_scene(self) -> SceneDescription:
        """Get minimal scene with just a table and cup."""
        scene_dict = {
            "objects": [
                {
                    "object_id": "table_1",
                    "object_type": "table_standard",
                    "constraints": []
                },
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small", 
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.002
                        }
                    ]
                }
            ],
            "robots": [],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0]
        }
        
        return SceneDescription(**scene_dict)
    
    def _get_canned_example_dict(self) -> Dict[str, Any]:
        """Get the main canned example as a dictionary."""
        return self._get_table_cup_robot_scene().model_dump()
    
    def build_llm_prompt(self, user_prompt: str) -> str:
        """
        Build a structured prompt for LLM scene generation.
        
        This method would be used when real LLM integration is implemented.
        """
        system_prompt = """You are an expert at generating structured scene descriptions for robotic simulations.

Given a natural language description, generate a JSON scene description that follows this schema:

{
  "objects": [
    {
      "object_id": "unique_id",
      "object_type": "type_from_assets_db", 
      "constraints": [
        {
          "type": "constraint_type",
          "subject": "this_object_id",
          "reference": "other_object_id",
          "clearance": 0.01
        }
      ]
    }
  ],
  "robots": [
    {
      "robot_id": "unique_id",
      "robot_type": "robot_type_from_assets_db",
      "base_position": [x, y, z],
      "joint_config": "ready",
      "constraints": [...]
    }
  ],
  "workspace_bounds": [x_min, y_min, z_min, x_max, y_max, z_max]
}

Available object types: table_standard, cup_ceramic_small, box_small
Available robot types: franka_panda  
Available constraint types: on_top_of, in_front_of, beside, no_collision
Available joint configs: ready, home, tucked

Generate realistic scenes with proper spatial relationships."""

        full_prompt = f"{system_prompt}\n\nUser request: {user_prompt}\n\nGenerate JSON scene description:"
        
        return full_prompt