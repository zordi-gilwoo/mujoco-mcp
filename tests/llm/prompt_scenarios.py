"""Prompt scenarios used for documentation and automated tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class PromptScenario:
    slug: str
    prompt: str
    scene: Dict[str, object]


SCENE_PROMPTS: List[PromptScenario] = [
    PromptScenario(
        slug="cluttered-workbench",
        prompt="Create a cluttered workbench with a table, three custom-sized boxes, and a red sphere balanced on the tallest box.",
        scene={
            "objects": [
                {"object_id": "table", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "box_short",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.3, "depth": 0.2, "height": 0.1},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_short",
                            "reference": "table",
                            "clearance": 0.002,
                            "offset": [-0.3, -0.2, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "box_medium",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.25, "depth": 0.25, "height": 0.15},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_medium",
                            "reference": "table",
                            "clearance": 0.002,
                            "offset": [0.25, 0.2, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "box_tall",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.2, "depth": 0.2, "height": 0.3},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "box_tall",
                            "reference": "table",
                            "clearance": 0.002,
                            "offset": [0.0, -0.1, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "top_sphere",
                    "object_type": "primitive:sphere",
                    "dimensions": {"radius": 0.05},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "top_sphere",
                            "reference": "box_tall",
                            "clearance": 0.001,
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="cylinder-row",
        prompt="Place a table, then line up three cylinders on top of it: a 0.4 m green post, a 0.8 m orange post, and a 1.2 m blue post.",
        scene={
            "objects": [
                {"object_id": "table", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "cyl_short",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.05, "height": 0.4},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cyl_short",
                            "reference": "table",
                            "clearance": 0.001,
                            "offset": [-0.3, 0.0, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "cyl_mid",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.05, "height": 0.8},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cyl_mid",
                            "reference": "table",
                            "clearance": 0.001,
                            "offset": [0.0, 0.0, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "cyl_tall",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.05, "height": 1.2},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "cyl_tall",
                            "reference": "table",
                            "clearance": 0.001,
                            "offset": [0.3, 0.0, 0.0],
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="cart-pole",
        prompt="Build a cart-pole rig with a 0.4 m wide cart and a 1.8 m pole tilted forward 15 degrees.",
        scene={
            "objects": [
                {
                    "object_id": "cart",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.4, "depth": 0.3, "height": 0.2},
                    "constraints": [],
                },
                {
                    "object_id": "pole",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.03, "height": 1.8},
                    "orientation": [0.0, 0.130526, 0.0, 0.991445],
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "pole",
                            "reference": "cart",
                            "clearance": 0.002,
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="double-pendulum",
        prompt="Create a double pendulum with two 1.5 m vertical cylinders connected end to end.",
        scene={
            "objects": [
                {
                    "object_id": "base",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.4, "depth": 0.4, "height": 0.1},
                    "constraints": [],
                },
                {
                    "object_id": "lower_link",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.04, "height": 1.5},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "lower_link",
                            "reference": "base",
                            "clearance": 0.001,
                        }
                    ],
                },
                {
                    "object_id": "upper_link",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.04, "height": 1.5},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "upper_link",
                            "reference": "lower_link",
                            "clearance": 0.001,
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="stacked-tower",
        prompt="Stack three boxes of decreasing size and balance a 0.05 m radius sphere on the smallest box.",
        scene={
            "objects": [
                {
                    "object_id": "base_box",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.6, "depth": 0.6, "height": 0.2},
                    "constraints": [],
                },
                {
                    "object_id": "mid_box",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.4, "depth": 0.4, "height": 0.15},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "mid_box",
                            "reference": "base_box",
                            "clearance": 0.002,
                        }
                    ],
                },
                {
                    "object_id": "top_box",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.25, "depth": 0.25, "height": 0.12},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "top_box",
                            "reference": "mid_box",
                            "clearance": 0.002,
                        }
                    ],
                },
                {
                    "object_id": "tower_ball",
                    "object_type": "primitive:sphere",
                    "dimensions": {"radius": 0.05},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "tower_ball",
                            "reference": "top_box",
                            "clearance": 0.001,
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="storage-corner",
        prompt="Arrange a workspace with a table, a small storage shelf beside it, and a 0.6 m tall cylinder post on top of the table.",
        scene={
            "objects": [
                {"object_id": "table", "object_type": "table_standard", "constraints": []},
                {
                    "object_id": "shelf",
                    "object_type": "shelf_small",
                    "constraints": [
                        {
                            "type": "beside",
                            "subject": "shelf",
                            "reference": "table",
                            "clearance": 0.05,
                            "offset": [0.0, -0.8, 0.0],
                        }
                    ],
                },
                {
                    "object_id": "post",
                    "object_type": "primitive:cylinder",
                    "dimensions": {"radius": 0.04, "height": 0.6},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "post",
                            "reference": "table",
                            "clearance": 0.001,
                            "offset": [0.45, 0.0, 0.0],
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
    PromptScenario(
        slug="simple-crane",
        prompt="Build a simple crane using box primitives: a 0.6 m cubic base, a 2 m vertical column, and a 1.5 m horizontal boom.",
        scene={
            "objects": [
                {
                    "object_id": "base",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.6, "depth": 0.6, "height": 0.6},
                    "constraints": [],
                },
                {
                    "object_id": "column",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 0.15, "depth": 0.15, "height": 2.0},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "column",
                            "reference": "base",
                            "clearance": 0.001,
                        }
                    ],
                },
                {
                    "object_id": "boom",
                    "object_type": "primitive:box",
                    "dimensions": {"width": 1.5, "depth": 0.15, "height": 0.2},
                    "constraints": [
                        {
                            "type": "on_top_of",
                            "subject": "boom",
                            "reference": "column",
                            "clearance": 0.001,
                            "offset": [0.75, 0.0, 0.0],
                        }
                    ],
                },
            ],
            "robots": [],
        },
    ),
]
