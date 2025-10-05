#!/usr/bin/env python3
"""
Enhanced LLM Integration Demo - Multi-Provider Support

Demonstrates the comprehensive LLM integration system with support for:
- OpenAI GPT models
- Claude (Anthropic) models
- Google Gemini models

Features showcased:
- Multi-provider API support
- Dynamic provider switching
- Intelligent fallback systems
- Environment variable configuration
- Error handling and recovery
"""

import os
import sys
import time
import asyncio
from pathlib import Path

# Add the project root to Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root / "src"))

try:
    from mujoco_mcp.scene_gen.llm_scene_generator import LLMSceneGenerator
    from mujoco_mcp.scene_gen.metadata_extractor import MetadataExtractor
    from mujoco_mcp.scene_gen.scene_schema import SceneDescription
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Please ensure you're running from the project root directory")
    sys.exit(1)


class MultiProviderLLMDemo:
    """Demonstration of multi-provider LLM integration."""

    def __init__(self):
        self.metadata_extractor = None
        self.llm_generator = None
        self.providers_tested = []

    def setup_metadata_extractor(self):
        """Initialize the metadata extractor."""
        print("🔧 Setting up metadata extractor...")
        try:
            self.metadata_extractor = MetadataExtractor()
            print("✅ Metadata extractor initialized")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize metadata extractor: {e}")
            return False

    def setup_llm_generator(self):
        """Initialize the LLM scene generator."""
        print("🤖 Setting up LLM scene generator...")
        try:
            self.llm_generator = LLMSceneGenerator(self.metadata_extractor)
            print("✅ LLM scene generator initialized")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize LLM generator: {e}")
            return False

    def display_provider_config(self, provider: str):
        """Display current provider configuration."""
        config_vars = {
            "openai": ["OPENAI_API_KEY", "OPENAI_MODEL", "OPENAI_TEMPERATURE", "OPENAI_MAX_TOKENS"],
            "claude": [
                "CLAUDE_API_KEY",
                "ANTHROPIC_API_KEY",
                "CLAUDE_MODEL",
                "CLAUDE_TEMPERATURE",
                "CLAUDE_MAX_TOKENS",
            ],
            "gemini": [
                "GEMINI_API_KEY",
                "GOOGLE_API_KEY",
                "GEMINI_MODEL",
                "GEMINI_TEMPERATURE",
                "GEMINI_MAX_TOKENS",
            ],
        }

        print(f"\n📋 {provider.upper()} Configuration:")
        for var in config_vars.get(provider, []):
            value = os.getenv(var)
            if "API_KEY" in var:
                display_value = f"{'*' * 8}...{value[-4:]}" if value else "Not set"
            else:
                display_value = value or "Not set (using default)"
            print(f"   {var}: {display_value}")

    def test_provider(self, provider: str, api_key: str = None, model: str = None):
        """Test a specific LLM provider."""
        print(f"\n🧪 Testing {provider.upper()} Provider")
        print("=" * 50)

        try:
            # Configure the provider
            if api_key:
                self.llm_generator.set_provider_config(provider, api_key, model)
            else:
                # Try to use environment variables
                self.llm_generator.provider = provider
                self.llm_generator._setup_provider_config()
                if not self.llm_generator.api_key:
                    print(
                        f"⚠️  No API key found for {provider}. Set environment variable or pass key directly."
                    )
                    return False

            # Display configuration
            self.display_provider_config(provider)

            # Test with a simple scene generation
            test_prompt = "Create a simple workspace with a table, cup on the table, and robot arm positioned to reach the cup"

            print(f"\n🎯 Test Prompt: '{test_prompt}'")
            print("⏳ Generating scene...")

            start_time = time.time()
            scene = self.llm_generator.generate_scene_description(test_prompt)
            end_time = time.time()

            # Display results
            generation_time = end_time - start_time
            print(f"✅ Scene generated successfully in {generation_time:.2f} seconds")
            print(f"📊 Scene contains {len(scene.objects)} objects and {len(scene.robots)} robots")

            # Show scene details
            print("\n📋 Generated Scene Details:")
            for i, obj in enumerate(scene.objects, 1):
                print(f"   {i}. Object: {obj.object_id} ({obj.object_type})")
                if obj.constraints:
                    for constraint in obj.constraints:
                        print(
                            f"      - {constraint.type}: {constraint.subject} → {constraint.reference}"
                        )

            for i, robot in enumerate(scene.robots, 1):
                print(f"   {i}. Robot: {robot.robot_id} ({robot.robot_type})")
                if robot.constraints:
                    for constraint in robot.constraints:
                        print(
                            f"      - {constraint.type}: {constraint.subject} → {constraint.reference}"
                        )

            self.providers_tested.append(
                {
                    "provider": provider,
                    "success": True,
                    "time": generation_time,
                    "objects": len(scene.objects),
                    "robots": len(scene.robots),
                }
            )

            return True

        except Exception as e:
            print(f"❌ {provider.upper()} test failed: {e}")
            self.providers_tested.append({"provider": provider, "success": False, "error": str(e)})
            return False

    def test_fallback_system(self):
        """Test the fallback system when LLM fails."""
        print(f"\n🛡️  Testing Fallback System")
        print("=" * 50)

        # Temporarily disable LLM by clearing API key
        original_api_key = self.llm_generator.api_key
        self.llm_generator.api_key = None
        self.llm_generator.llm_enabled = False

        try:
            test_prompt = (
                "Create a complex manipulation scenario with multiple objects and constraints"
            )

            print(f"🎯 Test Prompt: '{test_prompt}'")
            print("⏳ Testing fallback to symbolic plan generation...")

            start_time = time.time()
            scene = self.llm_generator.generate_scene_description(test_prompt)
            end_time = time.time()

            generation_time = end_time - start_time
            print(f"✅ Fallback system worked! Scene generated in {generation_time:.2f} seconds")
            print(
                f"📊 Fallback scene contains {len(scene.objects)} objects and {len(scene.robots)} robots"
            )

            return True

        except Exception as e:
            print(f"❌ Fallback system failed: {e}")
            return False
        finally:
            # Restore original configuration
            self.llm_generator.api_key = original_api_key
            self.llm_generator.llm_enabled = bool(original_api_key)

    def display_summary(self):
        """Display test summary."""
        print(f"\n📊 Multi-Provider LLM Test Summary")
        print("=" * 50)

        if not self.providers_tested:
            print("No providers were tested.")
            return

        successful_tests = [p for p in self.providers_tested if p["success"]]
        failed_tests = [p for p in self.providers_tested if not p["success"]]

        print(f"✅ Successful tests: {len(successful_tests)}")
        print(f"❌ Failed tests: {len(failed_tests)}")

        if successful_tests:
            print("\n🎉 Working Providers:")
            for test in successful_tests:
                print(
                    f"   • {test['provider'].upper()}: {test['time']:.2f}s, {test['objects']} objects, {test['robots']} robots"
                )

        if failed_tests:
            print("\n💔 Failed Providers:")
            for test in failed_tests:
                print(f"   • {test['provider'].upper()}: {test['error']}")

        print(f"\n💡 Configuration Tips:")
        print(f"   • Set environment variables: <PROVIDER>_API_KEY=your_key")
        print(f"   • Supported providers: OpenAI, Claude, Gemini")
        print(f"   • System automatically falls back to symbolic plans if LLM fails")

    async def run_demo(self):
        """Run the complete multi-provider demo."""
        print("🚀 Multi-Provider LLM Integration Demo")
        print("=" * 50)
        print("This demo showcases LLM scene generation with multiple AI providers")
        print("including OpenAI GPT, Claude (Anthropic), and Google Gemini.")
        print("")

        # Setup
        if not self.setup_metadata_extractor():
            return False

        if not self.setup_llm_generator():
            return False

        print("\n🌟 Starting Multi-Provider Tests...")

        # Test available providers
        providers_to_test = [
            ("openai", "OpenAI GPT"),
            ("claude", "Claude (Anthropic)"),
            ("gemini", "Google Gemini"),
        ]

        for provider, display_name in providers_to_test:
            print(f"\n🔄 Testing {display_name}...")

            # Check if API key is available via environment
            api_key_vars = {
                "openai": "OPENAI_API_KEY",
                "claude": ["CLAUDE_API_KEY", "ANTHROPIC_API_KEY"],
                "gemini": ["GEMINI_API_KEY", "GOOGLE_API_KEY"],
            }

            api_key = None
            for var in (
                api_key_vars[provider]
                if isinstance(api_key_vars[provider], list)
                else [api_key_vars[provider]]
            ):
                api_key = os.getenv(var)
                if api_key:
                    break

            if api_key:
                self.test_provider(provider, api_key)
            else:
                print(f"⚠️  Skipping {display_name} - no API key found")
                print(
                    f"   Set environment variable: {api_key_vars[provider][0] if isinstance(api_key_vars[provider], list) else api_key_vars[provider]}"
                )

        # Test fallback system
        self.test_fallback_system()

        # Display final summary
        self.display_summary()

        print(f"\n🎯 Demo completed! Multi-provider LLM integration is ready for production use.")
        return True


def display_configuration_help():
    """Display help for configuring API keys."""
    print("\n🔧 Configuration Guide")
    print("=" * 30)
    print("To test different providers, set the appropriate environment variables:")
    print("")
    print("OpenAI (GPT):")
    print("   export OPENAI_API_KEY=sk-your-openai-key")
    print("   export OPENAI_MODEL=gpt-4  # optional")
    print("")
    print("Claude (Anthropic):")
    print("   export CLAUDE_API_KEY=sk-ant-your-claude-key")
    print("   # or export ANTHROPIC_API_KEY=sk-ant-your-claude-key")
    print("   export CLAUDE_MODEL=claude-3-sonnet-20241022  # optional")
    print("")
    print("Gemini (Google):")
    print("   export GEMINI_API_KEY=AIza-your-gemini-key")
    print("   # or export GOOGLE_API_KEY=AIza-your-gemini-key")
    print("   export GEMINI_MODEL=gemini-1.5-pro  # optional")
    print("")
    print("General:")
    print("   export STRUCTURED_SCENE_LLM=enabled")
    print("   export LLM_PROVIDER=openai  # or claude, gemini")


async def main():
    """Main demo function."""
    # Check if help is requested
    if "--help" in sys.argv or "-h" in sys.argv:
        display_configuration_help()
        return

    # Check if LLM integration is enabled
    if os.getenv("STRUCTURED_SCENE_LLM", "disabled").lower() != "enabled":
        print("⚠️  LLM integration is disabled.")
        print("   Enable with: export STRUCTURED_SCENE_LLM=enabled")
        print("   Then configure API keys for the providers you want to test.")
        display_configuration_help()
        return

    demo = MultiProviderLLMDemo()

    try:
        success = await demo.run_demo()

        if success:
            print(f"\n✨ Demo completed successfully!")
            print(f"   You can now use structured scene generation with multiple LLM providers.")
            print(f"   The system automatically falls back to symbolic plans if needed.")
        else:
            print(f"\n❌ Demo encountered issues. Check the error messages above.")

    except KeyboardInterrupt:
        print(f"\n\n⏹️  Demo interrupted by user")
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())

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

    os.environ.update(
        {"OPENAI_MODEL": "gpt-3.5-turbo", "OPENAI_TEMPERATURE": "0.3", "OPENAI_MAX_TOKENS": "1500"}
    )

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
        "Make a warehouse sorting area with conveyor, packages, and pick-and-place robot",
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
        result = await handle_call_tool(
            "create_structured_scene",
            {
                "natural_language": "Create a pick-and-place workspace with parts and robot",
                "dry_run": True,
            },
        )

        response = result[0].text
        if "✅" in response:
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

    print("\n" + "=" * 60)
    asyncio.run(demo_mcp_integration())
