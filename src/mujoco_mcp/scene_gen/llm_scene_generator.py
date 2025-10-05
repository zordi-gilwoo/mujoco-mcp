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
from typing import Dict, Any, Optional, List

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
        self.last_structured_json: Optional[Dict[str, Any]] = None
        self.last_llm_raw_json: Optional[Dict[str, Any]] = None
        self.last_symbolic_plan: Optional[Dict[str, Any]] = None
        self.last_llm_prompt: Optional[Dict[str, str]] = None
        self.last_constraint_fixes: Optional[List[Dict[str, str]]] = None
        self.last_generation_mode: str = "unknown"

        llm_flag = os.getenv("STRUCTURED_SCENE_LLM", "auto").lower()

        # Multi-provider support
        self.provider = os.getenv("LLM_PROVIDER", "openai").lower()
        self.supported_providers = ["openai", "claude", "gemini"]

        # Provider-specific configuration
        self._setup_provider_config()

        if llm_flag == "enabled":
            self.llm_enabled = True
        elif llm_flag == "disabled":
            self.llm_enabled = False
        else:
            # Auto-enable when an API key is available
            self.llm_enabled = bool(self.api_key)

        if self.llm_enabled and self.api_key:
            logger.info(f"LLM integration enabled - using {self.provider} with model {self.model}")
        elif self.llm_enabled and not self.api_key:
            logger.warning(
                f"LLM integration requested but {self.provider.upper()}_API_KEY not found - using symbolic plans"
            )
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

    def _setup_provider_config(self):
        """Setup configuration for the selected LLM provider."""
        if self.provider == "openai":
            self.api_key = os.getenv("OPENAI_API_KEY")
            self.model = os.getenv("OPENAI_MODEL", "gpt-4")
            self.max_tokens = int(os.getenv("OPENAI_MAX_TOKENS", "2000"))
            self.temperature = float(os.getenv("OPENAI_TEMPERATURE", "0.7"))
        elif self.provider == "claude":
            self.api_key = os.getenv("CLAUDE_API_KEY") or os.getenv("ANTHROPIC_API_KEY")
            self.model = os.getenv("CLAUDE_MODEL", "claude-3-sonnet-20241022")
            self.max_tokens = int(os.getenv("CLAUDE_MAX_TOKENS", "2000"))
            self.temperature = float(os.getenv("CLAUDE_TEMPERATURE", "0.7"))
        elif self.provider == "gemini":
            self.api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
            self.model = os.getenv("GEMINI_MODEL", "gemini-1.5-pro")
            self.max_tokens = int(os.getenv("GEMINI_MAX_TOKENS", "2000"))
            self.temperature = float(os.getenv("GEMINI_TEMPERATURE", "0.7"))
        else:
            raise ValueError(
                f"Unsupported LLM provider: {self.provider}. Supported: {self.supported_providers}"
            )

    def set_provider_config(self, provider: str, api_key: str, model: str = None):
        """Dynamically set provider configuration (for UI integration)."""
        if provider.lower() not in self.supported_providers:
            raise ValueError(
                f"Unsupported provider: {provider}. Supported: {self.supported_providers}"
            )

        self.provider = provider.lower()
        self.api_key = api_key

        # Set default models if not specified
        if model:
            self.model = model
        elif self.provider == "openai":
            self.model = "gpt-4"
        elif self.provider == "claude":
            self.model = "claude-3-sonnet-20241022"
        elif self.provider == "gemini":
            self.model = "gemini-1.5-pro"

        # Enable LLM if API key is provided
        self.llm_enabled = bool(self.api_key)

        logger.info(f"Provider configuration updated: {self.provider} with model {self.model}")

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
        # Reset trace metadata
        self.last_structured_json = None
        self.last_llm_raw_json = None
        self.last_symbolic_plan = None
        self.last_llm_prompt = None
        self.last_constraint_fixes = []
        self.last_generation_mode = "unknown"

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
        self.last_symbolic_plan = plan.to_dict()
        self.last_generation_mode = "symbolic"
        self.last_llm_prompt = None
        self.last_llm_raw_json = None

        # Log plan details for auditability
        logger.info(
            f"Generated symbolic plan '{plan.plan_id}' with {len(plan.operations)} operations"
        )
        if plan.validation_results.get("is_valid"):
            logger.info("Plan validation: PASSED")
        else:
            logger.warning(
                f"Plan validation: FAILED ({len(plan.validation_results.get('errors', []))} errors)"
            )
            for error in plan.validation_results.get("errors", []):
                logger.warning(f"  - {error}")

        # Step 2: Plan → Scene Description
        scene = self.plan_converter.convert_plan_to_scene(plan)
        self.last_structured_json = scene.model_dump()

        logger.info(
            f"Converted plan to scene with {len(scene.objects)} objects and {len(scene.robots)} robots"
        )

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
            logger.exception(f"Failed to generate scene from prompt: {e}")
            # Return fallback canned example
            return self._get_canned_example_dict()

    def generate_scene_with_trace(self, prompt: str) -> Dict[str, Any]:
        """
        Generate a scene while capturing trace metadata useful for debugging.

        Returns:
            Dict containing the SceneDescription plus rich trace artifacts.
        """

        scene = self.generate_scene_description(prompt)

        trace = {
            "scene_description": scene,
            "structured_scene_json": self.last_structured_json or scene.model_dump(),
            "symbolic_plan": self.last_symbolic_plan,
            "llm_prompt": self.last_llm_prompt,
            "llm_raw_json": self.last_llm_raw_json,
            "constraint_fixes": self.last_constraint_fixes,
            "generation_mode": self.last_generation_mode,
        }

        if self.last_generation_mode == "llm":
            trace["provider"] = self.provider
            trace["model"] = self.model

        return trace

    def _generate_with_llm(self, prompt: str) -> SceneDescription:
        """
        Generate scene using real LLM integration with multi-provider support.

        Supports OpenAI, Claude (Anthropic), and Gemini APIs.
        """
        # Check for API key
        if not self.api_key:
            logger.error(f"{self.provider.upper()}_API_KEY environment variable not set")
            raise ValueError(
                f"LLM integration enabled but no API key found. "
                f"Please set {self.provider.upper()}_API_KEY environment variable."
            )

        # Route to appropriate provider
        if self.provider == "openai":
            return self._generate_with_openai(prompt)
        elif self.provider == "claude":
            return self._generate_with_claude(prompt)
        elif self.provider == "gemini":
            return self._generate_with_gemini(prompt)
        else:
            raise ValueError(f"Unsupported provider: {self.provider}")

    def _generate_with_openai(self, prompt: str) -> SceneDescription:
        """Generate scene using OpenAI API."""
        try:
            import openai
        except ImportError:
            logger.exception("OpenAI package not installed")
            raise ImportError(
                "OpenAI package required for OpenAI integration. Install with: pip install openai"
            ) from None

        # Initialize OpenAI client
        client = openai.OpenAI(api_key=self.api_key)

        # Build structured prompt for scene generation
        prompt_bundle = self.build_llm_prompt(prompt)
        system_prompt = prompt_bundle["system_prompt"]
        user_message = prompt_bundle["user_message"]
        self.last_llm_prompt = prompt_bundle

        try:
            logger.info(f"Calling OpenAI API ({self.model}) for scene generation: {prompt[:50]}...")

            response = client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message},
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )

            # Extract and parse JSON response
            llm_response = response.choices[0].message.content.strip()
            logger.info(f"Received OpenAI response ({len(llm_response)} chars)")

            return self._parse_llm_response(llm_response, prompt)

        except Exception as e:
            logger.exception(f"OpenAI API call failed: {e}")
            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(prompt)

    def _generate_with_claude(self, prompt: str) -> SceneDescription:
        """Generate scene using Claude (Anthropic) API."""
        try:
            import anthropic
        except ImportError:
            logger.exception("Anthropic package not installed")
            raise ImportError(
                "Anthropic package required for Claude integration. "
                "Install with: pip install anthropic"
            ) from None

        # Initialize Anthropic client
        client = anthropic.Anthropic(api_key=self.api_key)

        # Build structured prompt for scene generation
        prompt_bundle = self.build_llm_prompt(prompt)
        system_prompt = prompt_bundle["system_prompt"]
        user_message = prompt_bundle["user_message"]
        self.last_llm_prompt = prompt_bundle

        try:
            logger.info(f"Calling Claude API ({self.model}) for scene generation: {prompt[:50]}...")

            response = client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                system=system_prompt,
                messages=[{"role": "user", "content": user_message}],
            )

            # Extract and parse JSON response
            llm_response = response.content[0].text.strip()
            logger.info(f"Received Claude response ({len(llm_response)} chars)")

            return self._parse_llm_response(llm_response, prompt)

        except Exception as e:
            logger.exception(f"Claude API call failed: {e}")
            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(prompt)

    def _generate_with_gemini(self, prompt: str) -> SceneDescription:
        """Generate scene using Google Gemini API."""
        try:
            import google.generativeai as genai
        except ImportError:
            logger.exception("Google GenerativeAI package not installed")
            raise ImportError(
                "Google GenerativeAI package required for Gemini integration. "
                "Install with: pip install google-generativeai"
            ) from None

        # Configure Gemini
        genai.configure(api_key=self.api_key)

        # Initialize model
        model = genai.GenerativeModel(self.model)

        # Build structured prompt for scene generation
        prompt_bundle = self.build_llm_prompt(prompt)
        system_prompt = prompt_bundle["system_prompt"]
        user_message = prompt_bundle["user_message"]
        full_prompt = prompt_bundle["combined"]
        self.last_llm_prompt = prompt_bundle

        try:
            logger.info(f"Calling Gemini API ({self.model}) for scene generation: {prompt[:50]}...")

            # Configure generation parameters
            generation_config = genai.types.GenerationConfig(
                temperature=self.temperature,
                max_output_tokens=self.max_tokens,
            )

            response = model.generate_content(full_prompt, generation_config=generation_config)

            # Extract and parse JSON response
            llm_response = response.text.strip()
            logger.info(f"Received Gemini response ({len(llm_response)} chars)")

            return self._parse_llm_response(llm_response, prompt)

        except Exception as e:
            logger.exception(f"Gemini API call failed: {e}")
            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(prompt)

    def _build_user_message(self, prompt: str) -> str:
        """Build user message for LLM API call."""
        return f"""Natural Language Request: "{prompt}"

Please generate a complete scene description that fulfills this request. Consider:
- What objects are needed for the described scenario
- How objects should be spatially related
- Whether robots are needed and how they should be positioned
- What constraints ensure a realistic, functional layout

Return only the JSON scene description without any additional text or formatting."""

    def _parse_llm_response(self, llm_response: str, original_prompt: str) -> SceneDescription:
        """Parse LLM response and return SceneDescription."""
        # Clean up response (remove markdown code blocks if present)
        if llm_response.startswith("```json"):
            llm_response = llm_response[7:-3].strip()
        elif llm_response.startswith("```"):
            llm_response = llm_response[3:-3].strip()

        # Parse JSON response
        try:
            logger.info(f"Parsing LLM response: {llm_response[:500]}...")
            scene_dict = json.loads(llm_response)
            self.last_llm_raw_json = scene_dict
            logger.info(
                f"Parsed JSON with objects={len(scene_dict.get('objects', []))}, robots={len(scene_dict.get('robots', []))}"
            )

            scene = SceneDescription(**scene_dict)
            self.last_structured_json = scene.model_dump()
            self.last_generation_mode = "llm"
            logger.info(
                f"✓ Successfully validated scene with {len(scene.objects)} objects and {len(scene.robots)} robots"
            )
            return scene

        except json.JSONDecodeError as e:
            logger.exception(f"❌ JSON parsing failed: {e}")
            logger.exception(f"Raw LLM response: {llm_response}")
            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(original_prompt)

        except ValueError as e:
            logger.exception(f"❌ Pydantic validation failed: {e}")
            logger.exception(
                f"Scene dict that failed validation: {json.dumps(scene_dict, indent=2)}"
            )

            # Try to fix common validation errors (constraint reference issues)
            fixed_scene_dict = self._attempt_fix_constraint_references(scene_dict)
            if fixed_scene_dict:
                try:
                    scene = SceneDescription(**fixed_scene_dict)
                    self.last_structured_json = scene.model_dump()
                    self.last_generation_mode = "llm"
                    logger.info(
                        f"✓ Fixed constraint references, validated scene with {len(scene.objects)} objects"
                    )
                    return scene
                except Exception as fix_error:
                    logger.exception(f"❌ Auto-fix failed: {fix_error}")

            logger.info("Falling back to symbolic plan generation")
            return self._generate_with_symbolic_plan(original_prompt)

    def _attempt_fix_constraint_references(
        self, scene_dict: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """
        Attempt to automatically fix invalid constraint references.
        Common issues: LLM references entity that doesn't exist, or uses similar name.
        """
        try:
            # Collect all valid entity IDs
            valid_ids = set()
            for obj in scene_dict.get("objects", []):
                valid_ids.add(obj.get("object_id"))
            for robot in scene_dict.get("robots", []):
                valid_ids.add(robot.get("robot_id"))

            if not valid_ids:
                return None

            logger.info(f"Valid entity IDs: {valid_ids}")

            # Check and fix constraint references
            fixed = False
            fixes: List[Dict[str, str]] = []

            for obj in scene_dict.get("objects", []):
                for constraint in obj.get("constraints", []):
                    ref = constraint.get("reference")
                    if ref and ref not in valid_ids:
                        # Try to find closest match
                        closest = self._find_closest_entity_id(ref, valid_ids)
                        if closest:
                            logger.warning(f"Fixing constraint reference: {ref} → {closest}")
                            constraint["reference"] = closest
                            fixed = True
                            fixes.append(
                                {
                                    "entity": constraint.get("subject", "unknown"),
                                    "original_reference": ref,
                                    "replacement_reference": closest,
                                }
                            )

            for robot in scene_dict.get("robots", []):
                for constraint in robot.get("constraints", []):
                    ref = constraint.get("reference")
                    if ref and ref not in valid_ids:
                        closest = self._find_closest_entity_id(ref, valid_ids)
                        if closest:
                            logger.warning(f"Fixing constraint reference: {ref} → {closest}")
                            constraint["reference"] = closest
                            fixed = True
                            fixes.append(
                                {
                                    "entity": constraint.get(
                                        "subject", robot.get("robot_id", "unknown")
                                    ),
                                    "original_reference": ref,
                                    "replacement_reference": closest,
                                }
                            )

            if fixed:
                self.last_constraint_fixes = fixes
                return scene_dict

            return None

        except Exception as e:
            logger.exception(f"Error attempting to fix constraints: {e}")
            return None

    def _find_closest_entity_id(self, invalid_ref: str, valid_ids: set) -> Optional[str]:
        """Find closest matching entity ID using simple string similarity."""
        if not valid_ids:
            return None

        # Simple matching: check if invalid_ref is substring of any valid ID, or vice versa
        invalid_lower = invalid_ref.lower()

        for valid_id in valid_ids:
            valid_lower = valid_id.lower()
            # Check if either is substring of the other
            if invalid_lower in valid_lower or valid_lower in invalid_lower:
                return valid_id

        # Fallback: return any ID (first one)
        return list(valid_ids)[0]

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
                {"object_id": "table_1", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.002,
                        }
                    ],
                },
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
                            "clearance": 0.15,
                        }
                    ],
                }
            ],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0],
        }

        return SceneDescription(**scene_dict)

    def _get_collision_test_scene(self) -> SceneDescription:
        """Get collision testing scene with overlapping boxes."""
        scene_dict = {
            "objects": [
                {"object_id": "table_1", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "box_1",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_1",
                            "reference": "table_1",
                            "clearance": 0.001,
                        }
                    ],
                },
                {
                    "object_id": "box_2",
                    "object_type": "box_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_2",
                            "reference": "table_1",
                            "clearance": 0.001,
                        },
                        {
                            "type": "no_collision",
                            "subject": "box_2",
                            "reference": "box_1",
                            "clearance": 0.05,
                        },
                    ],
                },
            ],
            "robots": [],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0],
        }

        return SceneDescription(**scene_dict)

    def _get_simple_scene(self) -> SceneDescription:
        """Get minimal scene with just a table and cup."""
        scene_dict = {
            "objects": [
                {"object_id": "table_1", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "cup_1",
                    "object_type": "cup_ceramic_small",
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cup_1",
                            "reference": "table_1",
                            "clearance": 0.002,
                        }
                    ],
                },
            ],
            "robots": [],
            "workspace_bounds": [-1.0, -1.0, 0.0, 1.5, 1.0, 2.0],
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
      "object_type": "type_from_available_assets_or_primitives",
      "dimensions": {"width": 0.5, "height": 2.0},  // REQUIRED for primitives, optional otherwise
      "color": [0.8, 0.2, 0.2, 1.0],  // optional RGBA values in [0,1]
      "constraints": [...],
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
      "constraints": [ ... ]
    }
  ],
  "workspace_bounds": [x_min, y_min, z_min, x_max, y_max, z_max]  // optional
}
```

## Available Primitive Types (use for custom-sized geometry):
- **primitive:box** - requires `width`, `depth`, `height` (meters)
- **primitive:sphere** - requires `radius` (meters)
- **primitive:cylinder** - requires `radius`, `height` (meters)
- **primitive:capsule** - requires `radius`, `length` (meters, length excludes hemispherical caps)
- **primitive:ellipsoid** - requires `radius_x`, `radius_y`, `radius_z` (meters)

## Available Predefined Assets:
**Objects:**
- table_standard: Standard work table (0.8m x 1.2m x 0.75m)
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
- **inside**: Places subject inside a container
- **aligned_with_axis**: Aligns subject along specified axis

## CRITICAL VALIDATION RULES:
1. **Every constraint.reference MUST be an object_id or robot_id that exists in the scene**
2. **Every constraint.subject MUST match the object_id of the object it belongs to**
3. **Do NOT reference non-existent entities** (e.g., if you create "cart_pole", do not reference "cart_top")

## Guidelines:
1. Use descriptive, unique IDs for objects and robots
2. Always specify clearance values (typically 0.001-0.05m)
3. Use primitive types whenever custom dimensions are required and always provide the required dimension keys
4. Colors are optional but must be RGBA values in [0,1] when provided
5. Anchor large support surfaces without constraints first, then place dependent objects
6. Use `no_collision` constraints to prevent overlaps when necessary
7. Return only valid JSON without markdown formatting

## Example 1 - Cart Pole with Custom Dimensions:
Input: "Create a cart pole with a 2m long pole"
Output:
```json
{
  "objects": [
    {
      "object_id": "cart",
      "object_type": "primitive:box",
      "dimensions": {
        "width": 0.4,
        "depth": 0.3,
        "height": 0.2
      },
      "color": [0.3, 0.3, 0.3, 1.0],
      "constraints": []
    },
    {
      "object_id": "pole",
      "object_type": "primitive:cylinder",
      "dimensions": {
        "radius": 0.02,
        "height": 2.0
      },
      "color": [0.8, 0.2, 0.2, 1.0],
      "orientation": [0.707, 0.0, 0.0, 0.707],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "pole",
          "reference": "cart",
          "clearance": 0.001
        }
      ]
    }
  ],
  "robots": []
}
```

## Example 2 - Ball on Table:
Input: "Place a red ball on a table"
Output:
```json
{
  "objects": [
    {
      "object_id": "table",
      "object_type": "table_standard",
      "constraints": []
    },
    {
      "object_id": "ball",
      "object_type": "primitive:sphere",
      "dimensions": {
        "radius": 0.1
      },
      "color": [1.0, 0.0, 0.0, 1.0],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "ball",
          "reference": "table",
          "clearance": 0.001
        }
      ]
    }
  ],
  "robots": []
}
```

## Example 3 - Line Up Objects on Table:
Input: "Place a table, then line up three cylinders on top: a 0.4m green post, a 0.8m orange post, and a 1.2m blue post"
Output:
```json
{
  "objects": [
    {
      "object_id": "table",
      "object_type": "table_standard",
      "constraints": []
    },
    {
      "object_id": "green_post",
      "object_type": "primitive:cylinder",
      "dimensions": {
        "radius": 0.05,
        "height": 0.4
      },
      "color": [0.0, 1.0, 0.0, 1.0],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "green_post",
          "reference": "table",
          "clearance": 0.001
        }
      ]
    },
    {
      "object_id": "orange_post",
      "object_type": "primitive:cylinder",
      "dimensions": {
        "radius": 0.05,
        "height": 0.8
      },
      "color": [1.0, 0.5, 0.0, 1.0],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "orange_post",
          "reference": "table",
          "clearance": 0.001
        },
        {
          "type": "beside",
          "subject": "orange_post",
          "reference": "green_post",
          "clearance": 0.1
        }
      ]
    },
    {
      "object_id": "blue_post",
      "object_type": "primitive:cylinder",
      "dimensions": {
        "radius": 0.05,
        "height": 1.2
      },
      "color": [0.0, 0.0, 1.0, 1.0],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "blue_post",
          "reference": "table",
          "clearance": 0.001
        },
        {
          "type": "beside",
          "subject": "blue_post",
          "reference": "orange_post",
          "clearance": 0.1
        }
      ]
    }
  ],
  "robots": []
}
```

**IMPORTANT**: When objects need both horizontal AND vertical positioning, specify BOTH constraint types:
- Use `on_top_of` to control Z position (vertical placement)
- Use `beside` or `in_front_of` to control XY position (horizontal placement)
The constraint solver will compose them correctly - horizontal constraints set XY, vertical constraints set Z.

**REMEMBER**: constraint references must match defined entity IDs exactly. Generate realistic, physically plausible scenes. Respond with valid JSON only."""

    def build_llm_prompt(self, user_prompt: str) -> Dict[str, str]:
        """Return a structured prompt bundle for LLM scene generation."""
        system_prompt = self._get_system_prompt()

        context_prompt = self._build_user_message(user_prompt)

        return {
            "system_prompt": system_prompt,
            "user_message": context_prompt,
            "combined": f"{system_prompt}\n\n{context_prompt}",
            "natural_language_request": user_prompt,
        }
