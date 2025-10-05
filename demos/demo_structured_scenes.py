#!/usr/bin/env python3
"""
Structured Scene Generation Demo

Demonstrates the new structured scene generation capabilities including:
- Natural language scene creation
- JSON scene descriptions
- Constraint solving
- XML generation
- MCP tool integration
"""

import asyncio
import json
from src.mujoco_mcp.mcp_server_menagerie import handle_call_tool, handle_list_tools


async def demo_structured_scenes():
    """Demonstrate structured scene generation capabilities."""
    print("🏗️  Structured Scene Generation Demo")
    print("=" * 45)

    # Show available tools
    print("\n📋 Available MCP Tools:")
    tools = await handle_list_tools()
    for tool in tools:
        if "structured" in tool.name or "scene" in tool.name:
            print(f"  🔧 {tool.name}: {tool.description}")

    print(f"\n✅ Total tools available: {len(tools)}")

    # Demo 1: Natural Language Scene Generation
    print("\n" + "=" * 45)
    print("🗣️  Demo 1: Natural Language Scene Generation")
    print("=" * 45)

    prompts = [
        "Create a table with a cup on top and a robot arm nearby",
        "I need a simple collision test scene with two boxes",
        "Set up a basic manipulation workspace",
    ]

    for i, prompt in enumerate(prompts, 1):
        print(f"\n🎯 Example {i}: '{prompt}'")
        print("-" * 40)

        result = await handle_call_tool(
            "create_structured_scene", {"natural_language": prompt, "dry_run": True}
        )

        response = result[0].text
        if "✅" in response:
            # Extract summary from response
            if "scene_entities" in response:
                try:
                    # Parse the JSON summary from the response
                    start = response.find('{\n  "scene_entities"')
                    end = response.find("\n}", start) + 2
                    summary_json = response[start:end]
                    summary = json.loads(summary_json)

                    print(f"   📊 Entities: {summary['scene_entities']}")
                    print(f"   📦 Objects: {summary['objects']}")
                    print(f"   🤖 Robots: {summary['robots']}")
                    print(f"   📄 XML Size: {summary['xml_length']} chars")
                    print("   ✅ Generation successful!")
                except:
                    print("   ✅ Generation successful!")
            else:
                print("   ✅ Generation successful!")
        else:
            print("   ❌ Generation failed")

    # Demo 2: JSON Scene Description
    print("\n" + "=" * 45)
    print("📝 Demo 2: JSON Scene Description")
    print("=" * 45)

    # Complex scene with multiple constraints
    complex_scene = {
        "objects": [
            {"object_id": "workshop_table", "object_type": "table_standard", "constraints": []},
            {
                "object_id": "target_cup",
                "object_type": "cup_ceramic_small",
                "constraints": [
                    {
                        "type": "on_top_of",
                        "subject": "target_cup",
                        "reference": "workshop_table",
                        "clearance": 0.002,
                        "offset": [0.2, 0.1, 0.0],
                    }
                ],
            },
            {
                "object_id": "obstacle_box",
                "object_type": "box_small",
                "constraints": [
                    {
                        "type": "on_top_of",
                        "subject": "obstacle_box",
                        "reference": "workshop_table",
                        "clearance": 0.001,
                    },
                    {
                        "type": "no_collision",
                        "subject": "obstacle_box",
                        "reference": "target_cup",
                        "clearance": 0.08,
                    },
                ],
            },
        ],
        "robots": [
            {
                "robot_id": "manipulation_robot",
                "robot_type": "franka_panda",
                "joint_config": "ready",
                "base_position": [0.7, 0.0, 0.0],
                "constraints": [
                    {
                        "type": "in_front_of",
                        "subject": "manipulation_robot",
                        "reference": "workshop_table",
                        "clearance": 0.15,
                    }
                ],
            }
        ],
        "workspace_bounds": [-1.0, -1.0, 0.0, 2.0, 1.0, 2.0],
    }

    print("\n🔧 Complex Scene Configuration:")
    print(f"   📦 Objects: {len(complex_scene['objects'])}")
    print(f"   🤖 Robots: {len(complex_scene['robots'])}")

    # Count constraints
    total_constraints = 0
    for obj in complex_scene["objects"]:
        total_constraints += len(obj["constraints"])
    for robot in complex_scene["robots"]:
        total_constraints += len(robot["constraints"])
    print(f"   🔗 Constraints: {total_constraints}")

    result = await handle_call_tool(
        "create_structured_scene",
        {"scene_description_json": json.dumps(complex_scene), "dry_run": True},
    )

    response = result[0].text
    if "✅" in response:
        print("\n✅ Complex scene generation successful!")

        # Show XML preview
        if "XML Preview" in response:
            preview_start = response.find("XML Preview")
            preview_section = response[preview_start : preview_start + 300]
            print(f"\n📄 {preview_section}...")
    else:
        print("\n❌ Complex scene generation failed")
        print(response)

    # Demo 3: Feature Showcase
    print("\n" + "=" * 45)
    print("🌟 Demo 3: Feature Showcase")
    print("=" * 45)

    features = [
        "✅ Natural language to structured scene conversion",
        "✅ Pydantic schema validation with constraint checking",
        "✅ Spatial constraint solving (on_top_of, in_front_of, beside, no_collision)",
        "✅ Asset metadata system with fallback handling",
        "✅ Complete MuJoCo XML generation with lighting and floor",
        "✅ MCP tool integration with dry-run capability",
        "✅ Comprehensive error handling and validation",
        "✅ Extensible architecture for future enhancements",
    ]

    print("\n🚀 Structured Scene Generation Features:")
    for feature in features:
        print(f"   {feature}")

    print("\n📚 Available Assets:")
    from src.mujoco_mcp.scene_gen import MetadataExtractor

    extractor = MetadataExtractor()
    assets = extractor.get_available_assets()
    for asset in assets:
        metadata = extractor.get_metadata(asset)
        print(f"   📦 {asset}: {metadata.asset_type} - {metadata.description}")

    print("\n🔮 Phase 2 Roadmap (TODO comments in code):")
    roadmap = [
        "🔸 Real LLM integration (OpenAI API, local models)",
        "🔸 Advanced collision detection using MuJoCo contacts",
        "🔸 Robot reachability validation with forward kinematics",
        "🔸 Additional constraints (inside, aligned_with_axis)",
        "🔸 Backtracking constraint solver for conflict resolution",
        "🔸 Multi-table scene layout optimization",
    ]

    for item in roadmap:
        print(f"   {item}")

    print("\n" + "=" * 45)
    print("✨ Demo Complete! Structured Scene Generation Ready! ✨")
    print("=" * 45)


if __name__ == "__main__":
    asyncio.run(demo_structured_scenes())
