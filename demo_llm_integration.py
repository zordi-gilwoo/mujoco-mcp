#!/usr/bin/env python3
"""
Demo: LLM Integration for Structured Scene Generation

This demo shows how to use the enhanced LLM integration with API key support
for generating MuJoCo scenes from natural language descriptions.

Features demonstrated:
- OpenAI API integration with configurable parameters
- Graceful fallback to symbolic plan generation
- Error handling and configuration options
- Different scene generation modes
"""

import os
import asyncio
from src.mujoco_mcp.scene_gen import MetadataExtractor, LLMSceneGenerator


def demo_llm_integration():
    """Demonstrate LLM integration capabilities"""
    print("🚀 LLM Integration Demo for Structured Scene Generation")
    print("=" * 60)
    
    # Initialize components
    extractor = MetadataExtractor()
    
    print("\n📋 Configuration Options:")
    print("   STRUCTURED_SCENE_LLM=enabled/disabled  # Enable LLM integration")
    print("   OPENAI_API_KEY=your_key_here          # Required for LLM calls")
    print("   OPENAI_MODEL=gpt-4                    # Model selection")
    print("   OPENAI_TEMPERATURE=0.7                # Creativity level")
    print("   OPENAI_MAX_TOKENS=2000                # Response length")
    
    # Demo 1: Default symbolic plan generation
    print("\n🔧 Demo 1: Default Mode (Symbolic Plan Generation)")
    print("-" * 50)
    
    generator = LLMSceneGenerator(extractor)
    print(f"LLM enabled: {generator.llm_enabled}")
    print(f"Model: {generator.model}")
    
    scene = generator.generate_scene_description(
        "Create a robotics research lab with workbench and robot arm"
    )
    print(f"Generated scene: {len(scene.objects)} objects, {len(scene.robots)} robots")
    
    # Demo 2: LLM mode simulation (without real API key)
    print("\n🤖 Demo 2: LLM Mode Configuration")
    print("-" * 50)
    
    # Show how to configure for LLM mode
    print("To enable LLM integration:")
    print("1. Set environment variables:")
    print("   export STRUCTURED_SCENE_LLM=enabled")
    print("   export OPENAI_API_KEY=your_actual_api_key")
    print("   export OPENAI_MODEL=gpt-4  # or gpt-3.5-turbo")
    print("")
    print("2. Install OpenAI package:")
    print("   pip install openai")
    print("")
    print("3. The system will automatically:")
    print("   - Use LLM for scene generation")
    print("   - Fallback to symbolic plans if LLM fails")
    print("   - Handle API errors gracefully")
    
    # Demo 3: Advanced configuration
    print("\n⚙️ Demo 3: Advanced Configuration")
    print("-" * 50)
    
    os.environ.update({
        'OPENAI_MODEL': 'gpt-3.5-turbo',
        'OPENAI_TEMPERATURE': '0.3',
        'OPENAI_MAX_TOKENS': '1500'
    })
    
    generator_advanced = LLMSceneGenerator(extractor)
    print(f"Model: {generator_advanced.model}")
    print(f"Temperature: {generator_advanced.temperature}")
    print(f"Max tokens: {generator_advanced.max_tokens}")
    
    # Demo 4: Complex scene generation
    print("\n🏭 Demo 4: Complex Scene Examples")
    print("-" * 50)
    
    complex_prompts = [
        "Create a manufacturing cell with assembly table, parts bins, and collaborative robot",
        "Design a kitchen scene with countertops, appliances, and robotic chef assistant",
        "Build a laboratory setup with equipment table, sample containers, and precision robot",
        "Make a warehouse sorting area with conveyor, packages, and pick-and-place robot"
    ]
    
    for i, prompt in enumerate(complex_prompts, 1):
        scene = generator.generate_scene_description(prompt)
        print(f"{i}. {prompt[:50]}...")
        print(f"   → {len(scene.objects)} objects, {len(scene.robots)} robots")
    
    print("\n✅ Demo completed successfully!")
    print("\n📚 For more information:")
    print("   - See docs/STRUCTURED_SCENES.md for detailed documentation")
    print("   - Run `python demo_structured_scenes.py` for MCP tool integration")
    print("   - Check tests/test_structured_scene_generation.py for test examples")


async def demo_mcp_integration():
    """Demonstrate MCP tool integration with LLM support"""
    print("\n🔗 MCP Tool Integration with LLM")
    print("-" * 50)
    
    try:
        from src.mujoco_mcp.mcp_server_menagerie import handle_call_tool
        
        # Test MCP tool with natural language
        result = await handle_call_tool('create_structured_scene', {
            'natural_language': 'Create a pick-and-place workspace with parts and robot',
            'dry_run': True
        })
        
        response = result[0].text
        if '✅' in response:
            print("✅ MCP tool integration working")
            print("   Tool: create_structured_scene")
            print("   Input: natural_language + dry_run")
            print("   Output: Scene validation and XML generation")
        else:
            print("⚠️ MCP integration needs additional setup")
            
    except ImportError:
        print("ℹ️ MCP integration requires additional packages")
        print("   Install with: pip install mcp")


if __name__ == "__main__":
    demo_llm_integration()
    
    print("\n" + "="*60)
    asyncio.run(demo_mcp_integration())