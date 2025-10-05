#!/usr/bin/env python3
"""Regression tests for documented scene-generation prompts."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET

import pytest

from mujoco_mcp.scene_gen import ConstraintSolver, MetadataExtractor, SceneDescription, SceneXMLBuilder

from .prompt_scenarios import SCENE_PROMPTS


@pytest.mark.parametrize("scenario", SCENE_PROMPTS, ids=lambda s: s.slug)
def test_prompt_scenarios_generate_valid_xml(scenario):
    """Validate that each documented prompt has the metadata needed to build XML."""

    extractor = MetadataExtractor()

    # Ensure every object type is backed by metadata before solving.
    for obj in scenario.scene["objects"]:
        metadata = extractor.get_metadata(obj["object_type"])
        assert metadata is not None, f"Missing metadata for {obj['object_type']} in scenario {scenario.slug}"

    scene = SceneDescription(**scenario.scene)

    solver = ConstraintSolver(extractor)
    solver.use_spatial_reasoning = False
    solver.use_enhanced_collision = False
    solver._enhance_poses_with_spatial_reasoning = lambda _scene, poses_map: poses_map  # type: ignore[assignment]
    solver.collision_optimizer.optimize_poses = lambda _scene, poses_map: poses_map  # type: ignore[assignment]

    poses = solver.solve(scene)

    builder = SceneXMLBuilder(extractor)
    xml_str = builder.build_scene(scene, poses)

    # Verify the XML is well formed.
    ET.fromstring(xml_str)


def test_readme_prompt_table_matches_scenarios():
    """Keep README_SCENEGEN.md in sync with the scenario catalog."""

    expected_prompts = {scenario.prompt for scenario in SCENE_PROMPTS}
    expected_slugs = {scenario.slug for scenario in SCENE_PROMPTS}

    with open("README_SCENEGEN.md", "r", encoding="utf-8") as readme:
        content = readme.read()

    prompt_matches = re.findall(r"\|\s*([a-z0-9\-]+)\s*\|\s*`([^`]+)`\s*\|", content)
    readme_slugs = {slug for slug, _ in prompt_matches}
    readme_prompts = {prompt for _, prompt in prompt_matches}

    assert readme_slugs == expected_slugs, "Prompt slugs in README_SCENEGEN.md are out of sync with tests"
    assert readme_prompts == expected_prompts, "Prompt text in README_SCENEGEN.md is out of sync with tests"
