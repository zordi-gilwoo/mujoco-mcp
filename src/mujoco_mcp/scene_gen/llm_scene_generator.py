#!/usr/bin/env python3
"""
LLM Scene Generator

Generates scene descriptions from natural language prompts.
Includes stubbed LLM integration with environment variable gating.

Phase 2C Enhancement: Now supports symbolic plan generation as an intermediate step.
"""

import json
import logging
import os
from typing import Dict, Any, Optional

from .scene_schema import SceneDescription
from .symbolic_plan import SymbolicPlanGenerator, PlanToSceneConverter

logger = logging.getLogger("mujoco_mcp.scene_gen.llm_generator")


class LLMSceneGenerator:
    """
    Generates scene descriptions from natural language using enhanced pipeline.
    
    Always uses symbolic plan generation as intermediate step (Phase 2C),
    providing explicit NL→Plan→Scene separation for better auditability.
    
    By default, returns canned examples. Real LLM integration can be enabled
    via environment variable STRUCTURED_SCENE_LLM=enabled.
    """
    
    def __init__(self, metadata_extractor=None):
        # LLM configuration
        self.llm_enabled = os.getenv("STRUCTURED_SCENE_LLM", "disabled").lower() == "enabled"
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.model = os.getenv("OPENAI_MODEL", "gpt-4")
        self.max_tokens = int(os.getenv("OPENAI_MAX_TOKENS", "2000"))
        self.temperature = float(os.getenv("OPENAI_TEMPERATURE", "0.7"))
        
        if self.llm_enabled:
            if self.api_key:
                logger.info(f"LLM integration enabled - using model {self.model}")
            else:
                logger.warning("LLM integration enabled but OPENAI_API_KEY not found - will use symbolic plans")
                self.llm_enabled = False
        else:
            logger.info("LLM integration disabled - using symbolic plan generation")
        
        # Always initialize symbolic plan generator (Phase 2C) as fallback
        if metadata_extractor:
            self.plan_generator = SymbolicPlanGenerator(metadata_extractor)
            self.plan_converter = PlanToSceneConverter(metadata_extractor)
            logger.info("Symbolic plan interface initialized for NL→Plan→Scene pipeline")
        else:
            raise ValueError("MetadataExtractor is required for enhanced scene generation")
    
    def generate_scene_description(self, prompt: str) -> SceneDescription:
        """
        Generate a scene description from natural language prompt.
        
        Uses LLM if enabled and available, otherwise falls back to symbolic plan interface.
        
        Args:
            prompt: Natural language description of desired scene
            
        Returns:
            SceneDescription object
            
        Raises:
            ValueError: If no generation method is available
        """
        if self.llm_enabled and self.api_key:
            try:
                return self._generate_with_llm(prompt)
            except Exception as e:
                logger.warning(f"LLM generation failed, falling back to symbolic plans: {e}")
                return self._generate_with_symbolic_plan(prompt)
        else:
            return self._generate_with_symbolic_plan(prompt)
    
    def _generate_with_symbolic_plan(self, prompt: str) -> SceneDescription:
        """
        Generate scene using symbolic plan interface (Phase 2C).
        This provides explicit NL→Plan→Scene separation for auditability.
        """
        logger.info(f"Generating scene using symbolic plan interface: {prompt[:50]}...")
        
        # Step 1: NL → Symbolic Plan
        plan = self.plan_generator.generate_plan_from_nl(prompt)
        
        # Log plan details for auditability
        logger.info(f"Generated symbolic plan '{plan.plan_id}' with {len(plan.operations)} operations")
        if plan.validation_results.get("is_valid"):
            logger.info("Plan validation: PASSED")
        else:
            logger.warning(f"Plan validation: FAILED ({len(plan.validation_results.get('errors', []))} errors)")
            for error in plan.validation_results.get("errors", []):
                logger.warning(f"  - {error}")
        
        # Step 2: Plan → Scene Description
        scene = self.plan_converter.convert_plan_to_scene(plan)
        
        logger.info(f"Converted plan to scene with {len(scene.objects)} objects and {len(scene.robots)} robots")
        
        return scene
    
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
        Generate scene using real LLM integration with API key support.
        
        Supports OpenAI API and other LLM services through environment variables.
        """
        # Check for API key
        if not self.api_key:
            logger.error("OPENAI_API_KEY environment variable not set")
            raise ValueError(
                "LLM integration enabled but no API key found. "
                "Please set OPENAI_API_KEY environment variable."
            )
        
        try:
            import openai
        except ImportError:
            logger.error("OpenAI package not installed")
            raise ImportError(
                "OpenAI package required for LLM integration. "
                "Install with: pip install openai"
            )
        
        # Initialize OpenAI client
        client = openai.OpenAI(api_key=self.api_key)
        
        # Build structured prompt for scene generation
        system_prompt = self._get_system_prompt()
        user_message = f"""Natural Language Request: "{prompt}"

Please generate a complete scene description that fulfills this request. Consider:
- What objects are needed for the described scenario
- How objects should be spatially related 
- Whether robots are needed and how they should be positioned
- What constraints ensure a realistic, functional layout

Return only the JSON scene description without any additional text or formatting."""
        
        try:
            logger.info(f"Calling OpenAI API ({self.model}) for scene generation: {prompt[:50]}...")
            
            response = client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )
            
            # Extract and parse JSON response
            llm_response = response.choices[0].message.content.strip()
            logger.info(f"Received LLM response ({len(llm_response)} chars)")
            
            # Clean up response (remove markdown code blocks if present)
            if llm_response.startswith("```json"):
                llm_response = llm_response[7:-3].strip()
            elif llm_response.startswith("```"):
                llm_response = llm_response[3:-3].strip()
            
            # Parse JSON response
            try:
                scene_dict = json.loads(llm_response)
                scene = SceneDescription(**scene_dict)
                logger.info(f"Successfully generated scene with {len(scene.objects)} objects and {len(scene.robots)} robots")
                return scene
                
            except (json.JSONDecodeError, ValueError) as e:
                logger.error(f"Failed to parse LLM response as valid scene: {e}")
                logger.debug(f"Raw LLM response: {llm_response}")
                
                # Fallback to symbolic plan generation
                logger.info("Falling back to symbolic plan generation")
                return self._generate_with_symbolic_plan(prompt)
                
        except Exception as e:
            logger.error(f"OpenAI API call failed: {e}")
            
            # Fallback to symbolic plan generation
            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(prompt)
    
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
    
    def _get_system_prompt(self) -> str:
        """
        Get the system prompt for LLM scene generation.
        
        This prompt provides detailed instructions and schema information
        to help the LLM generate valid scene descriptions.
        """
        return """You are an expert at generating structured scene descriptions for robotic simulations in MuJoCo.

Your task is to convert natural language descriptions into valid JSON scene descriptions that follow the exact schema below.

## Schema:
```json
{
  "objects": [
    {
      "object_id": "unique_identifier",
      "object_type": "type_from_available_assets",
      "constraints": [
        {
          "type": "constraint_type",
          "subject": "this_object_id", 
          "reference": "target_object_id",
          "clearance": 0.01
        }
      ],
      "orientation": [x, y, z, w]  // optional quaternion
    }
  ],
  "robots": [
    {
      "robot_id": "unique_identifier",
      "robot_type": "robot_type_from_available_assets",
      "base_position": [x, y, z],  // optional, auto-positioned if omitted
      "base_orientation": [x, y, z, w],  // optional quaternion
      "joint_config": "config_name",
      "constraints": [
        // same constraint format as objects
      ]
    }
  ],
  "workspace_bounds": [x_min, y_min, z_min, x_max, y_max, z_max]  // optional
}
```

## Available Assets:
**Objects:**
- table_standard: Standard work table (0.8m x 1.2m x 0.05m)
- cup_ceramic_small: Small ceramic cup (0.08m diameter, 0.1m height)
- box_small: Small cardboard box (0.1m x 0.1m x 0.1m)
- shelf_small: Small storage shelf with multiple levels

**Robots:**
- franka_panda: 7-DOF robotic arm with gripper

## Available Constraint Types:
- **on_top_of**: Places subject on top of reference object
- **in_front_of**: Places subject in front (+X direction) of reference
- **beside**: Places subject beside (+Y direction) of reference  
- **no_collision**: Ensures minimum clearance between objects
- **within_reach**: Places subject within robot's reachable workspace
- **inside**: Places subject inside a container (like shelf)
- **aligned_with_axis**: Aligns subject along specified axis

## Available Joint Configurations:
- **ready**: Robot arm in ready position for manipulation
- **home**: Robot arm in home/neutral position
- **tucked**: Robot arm tucked against body

## Guidelines:
1. Use descriptive, unique IDs for objects and robots
2. Always specify clearance values (typically 0.001-0.05m)
3. Place tables first (no constraints), then objects on tables
4. Position robots with appropriate clearance from work surfaces
5. Use no_collision constraints to prevent overlapping objects
6. Consider workspace_bounds for complex scenes (default: [-2, -2, 0, 2, 2, 2])
7. Return only valid JSON without markdown formatting

## Example:
Input: "Create a workspace with a table, cup on the table, and robot arm nearby"
Output:
```json
{
  "objects": [
    {
      "object_id": "work_table",
      "object_type": "table_standard",
      "constraints": []
    },
    {
      "object_id": "coffee_cup",
      "object_type": "cup_ceramic_small",
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "coffee_cup",
          "reference": "work_table", 
          "clearance": 0.002
        }
      ]
    }
  ],
  "robots": [
    {
      "robot_id": "manipulator_arm",
      "robot_type": "franka_panda",
      "joint_config": "ready",
      "constraints": [
        {
          "type": "in_front_of",
          "subject": "manipulator_arm",
          "reference": "work_table",
          "clearance": 0.2
        }
      ]
    }
  ]
}
```

Generate realistic, physically plausible scenes. Respond with valid JSON only."""

    def build_llm_prompt(self, user_prompt: str) -> str:
        """
        Build a structured prompt for LLM scene generation.
        
        This method constructs the full conversation for the LLM API call.
        """
        system_prompt = self._get_system_prompt()
        
        # Add context about the specific request
        context_prompt = f"""
Natural Language Request: "{user_prompt}"

Please generate a complete scene description that fulfills this request. Consider:
- What objects are needed for the described scenario
- How objects should be spatially related 
- Whether robots are needed and how they should be positioned
- What constraints ensure a realistic, functional layout

Return only the JSON scene description without any additional text or formatting."""

        return system_prompt + context_prompt