#!/usr/bin/env python3
"""Utility for prompting the LLM scene generator from the command line."""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Optional


def _load_scene_generator(model_override: Optional[str]):
    """Instantiate LLMSceneGenerator with OpenAI provider."""
    project_root = Path(__file__).resolve().parent
    sys.path.insert(0, str(project_root / "src"))

    from mujoco_mcp.scene_gen.metadata_extractor import MetadataExtractor
    from mujoco_mcp.scene_gen.llm_scene_generator import LLMSceneGenerator

    extractor = MetadataExtractor()
    generator = LLMSceneGenerator(extractor)

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY environment variable is not set")

    generator.set_provider_config("openai", api_key, model_override)
    return generator


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate MuJoCo XML scenes via LLM")
    parser.add_argument("prompt", help="Natural language description of the desired scene")
    parser.add_argument(
        "--model",
        default=os.getenv("OPENAI_MODEL"),
        help="Override the OpenAI model name (defaults to OPENAI_MODEL env or generator default)",
    )
    args = parser.parse_args()

    try:
        generator = _load_scene_generator(args.model)
    except Exception as exc:
        print(f"Failed to set up LLM generator: {exc}")
        return 1

    print("=== Natural Language Prompt ===")
    print(args.prompt)
    print()

    base_prompt = args.prompt
    current_prompt = base_prompt
    last_error = None

    for attempt in range(1, 4):
        print(f"=== Attempt {attempt} ===")
        print()

        try:
            trace = generator.generate_scene_with_trace(current_prompt)
        except Exception as exc:
            print(f"LLM scene generation failed: {exc}")
            last_error = exc
            if attempt == 3:
                return 1
            current_prompt = _augment_prompt_with_error(base_prompt, exc, None)
            continue

        scene = trace["scene_description"]
        structured = trace.get("structured_scene_json", scene.model_dump())
        raw_json = trace.get("llm_raw_json")
        llm_prompt = trace.get("llm_prompt")
        generation_mode = trace.get("generation_mode", "unknown")

        print(f"=== Generation Mode: {generation_mode} ===")
        provider = trace.get("provider")
        model = trace.get("model")
        if provider and model:
            print(f"Provider: {provider} | Model: {model}")
            print()

        if llm_prompt:
            print("=== LLM Prompt (system) ===")
            print(llm_prompt.get("system_prompt", "<missing system prompt>"))
            print()
            print("=== LLM Prompt (user) ===")
            print(llm_prompt.get("user_message", "<missing user prompt>"))
            print()

        if raw_json:
            print("=== Raw Structured JSON (LLM output) ===")
            print(json.dumps(raw_json, indent=2))
            print()

        print("=== Structured Scene JSON (validated) ===")
        print(json.dumps(structured, indent=2))
        print()

        symbolic_plan = trace.get("symbolic_plan")
        if symbolic_plan:
            print("=== Symbolic Plan (fallback) ===")
            print(json.dumps(symbolic_plan, indent=2))
            print()

        constraint_fixes = trace.get("constraint_fixes") or []
        if constraint_fixes:
            print("=== Constraint Fixes ===")
            print(json.dumps(constraint_fixes, indent=2))
            print()

        try:
            xml_output = scene.to_xml()
        except Exception as exc:
            print("Failed to convert structured scene to MuJoCo XML:")
            print(exc)
            last_error = exc
            if attempt == 3:
                return 1

            current_prompt = _augment_prompt_with_error(
                base_prompt,
                exc,
                structured,
            )
            continue

        print("=== Generated MuJoCo XML ===")
        print(xml_output)
        return 0

    if last_error:
        print("Scene generation failed after multiple attempts.")
        print(last_error)
    return 1


def _augment_prompt_with_error(base_prompt: str, error: Exception, structured_json: Optional[dict]) -> str:
    """Create a follow-up prompt that asks the LLM to correct a previous failure."""

    message_lines = [
        base_prompt,
        "",
        "The previous attempt failed to convert the generated scene to MuJoCo XML.",
        f"Error message: {error}",
    ]

    if structured_json is not None:
        message_lines.append(
            "Here is the JSON that failed—please respond with a corrected version that avoids the issue:"
        )
        message_lines.append(json.dumps(structured_json, indent=2))

    message_lines.append(
        "Return only the corrected JSON scene description without additional commentary."
    )

    return "\n".join(message_lines)


if __name__ == "__main__":
    sys.exit(main())
