#!/usr/bin/env python3
"""
Multi-Client MuJoCo MCP Demo
Demonstrates session isolation and concurrent client support
"""

import asyncio
import threading
import json
import time
from typing import List
from mujoco_mcp.mcp_server_menagerie import handle_call_tool, handle_list_tools


async def simulate_client(client_id: str, scene_type: str) -> List[str]:
    """Simulate a single MCP client session"""
    results = []

    results.append(f"\n🎯 Client {client_id} starting...")

    try:
        # Get session info
        session_info = await handle_call_tool("get_session_info", {})
        session_data = json.loads(session_info[0].text)
        current_session = session_data["current_session"]

        results.append(f"   📊 Session ID: {current_session['session_id']}")
        results.append(f"   🆔 Client ID: {current_session['client_id']}")
        results.append(f"   🔌 Viewer Port: {current_session['viewer_port']}")

        # Create a scene (this would normally connect to viewer server)
        results.append(f"   🎬 Creating {scene_type} scene...")
        create_result = await handle_call_tool("create_scene", {"scene_type": scene_type})
        results.append(f"   ✅ {create_result[0].text}")

        # Get updated session info
        session_info = await handle_call_tool("get_session_info", {})
        session_data = json.loads(session_info[0].text)
        current_session = session_data["current_session"]

        results.append(f"   📈 Active models: {list(current_session['active_models'].keys())}")

        # Simulate some operations
        results.append(f"   ⚡ Running simulation steps...")
        step_result = await handle_call_tool(
            "step_simulation", {"model_id": scene_type, "steps": 5}
        )
        results.append(f"   ✅ {step_result[0].text}")

        # Reset simulation
        results.append(f"   🔄 Resetting simulation...")
        reset_result = await handle_call_tool("reset_simulation", {"model_id": scene_type})
        results.append(f"   ✅ {reset_result[0].text}")

        results.append(f"   🏁 Client {client_id} completed successfully!")

    except Exception as e:
        results.append(f"   ❌ Client {client_id} error: {e}")

    return results


def run_client_in_thread(client_id: str, scene_type: str, results_dict: dict):
    """Run a client simulation in a separate thread with its own event loop"""

    async def async_client():
        return await simulate_client(client_id, scene_type)

    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        results_dict[client_id] = loop.run_until_complete(async_client())
    finally:
        loop.close()


async def demo_sequential_clients():
    """Demonstrate sequential client sessions"""
    print("\n" + "=" * 60)
    print("🔄 SEQUENTIAL CLIENTS DEMO")
    print("=" * 60)
    print("Testing session isolation with sequential clients...")

    # Test different scene types sequentially
    scene_types = ["pendulum", "cartpole", "pendulum"]
    client_results = []

    for i, scene_type in enumerate(scene_types, 1):
        results = await simulate_client(f"SEQ-{i}", scene_type)
        client_results.extend(results)

        # Small delay between clients
        await asyncio.sleep(0.1)

    # Print all results
    for result in client_results:
        print(result)

    # Show final session status
    print(f"\n📊 Final session status:")
    session_info = await handle_call_tool("get_session_info", {})
    session_data = json.loads(session_info[0].text)

    print(f"   Total active sessions: {session_data['all_sessions']['active_sessions']}")
    for session_id, info in session_data["all_sessions"]["sessions"].items():
        print(
            f"   - {info['client_id']}: {len(info['active_models'])} models, port {info['viewer_port']}"
        )


def demo_concurrent_clients():
    """Demonstrate concurrent client sessions"""
    print("\n" + "=" * 60)
    print("🚀 CONCURRENT CLIENTS DEMO")
    print("=" * 60)
    print("Testing true multi-client isolation with concurrent threads...")

    # Different clients with different scene types
    client_configs = [
        ("MULTI-A", "pendulum"),
        ("MULTI-B", "cartpole"),
        ("MULTI-C", "pendulum"),
    ]

    results_dict = {}
    threads = []

    # Start all clients concurrently
    for client_id, scene_type in client_configs:
        thread = threading.Thread(
            target=run_client_in_thread, args=(client_id, scene_type, results_dict)
        )
        threads.append(thread)
        thread.start()

    # Wait for all to complete
    for thread in threads:
        thread.join()

    # Print results from all clients
    for client_id in sorted(results_dict.keys()):
        for result in results_dict[client_id]:
            print(result)

    print(f"\n🎉 All {len(client_configs)} concurrent clients completed!")


async def demo_tools_overview():
    """Show available tools"""
    print("\n" + "=" * 60)
    print("🛠️  AVAILABLE TOOLS")
    print("=" * 60)

    tools = await handle_list_tools()
    print(f"Found {len(tools)} tools:")

    for tool in tools:
        print(f"  📦 {tool.name}")
        print(f"     {tool.description}")


async def main():
    """Main demo function"""
    print("🎮 MuJoCo MCP Multi-Client Demonstration")
    print("This demo shows how multiple MCP clients can work simultaneously")
    print("with isolated sessions and independent simulation environments.")

    # Show available tools
    await demo_tools_overview()

    # Demonstrate sequential clients (different sessions over time)
    await demo_sequential_clients()

    # Demonstrate concurrent clients (different threads simultaneously)
    demo_concurrent_clients()

    print("\n" + "=" * 60)
    print("✅ MULTI-CLIENT DEMO COMPLETE")
    print("=" * 60)
    print("\nKey Benefits Demonstrated:")
    print("  🔒 Session Isolation - Each client gets its own simulation space")
    print("  🔌 Port Allocation - Each client uses a different viewer port")
    print("  🏷️  Model Namespacing - Models are prefixed with session IDs")
    print("  📊 Resource Tracking - Sessions track their active models")
    print("  🧹 Clean Cleanup - Resources are properly managed")
    print("\nThis enables multiple AI agents or users to run")
    print("independent MuJoCo simulations simultaneously!")


if __name__ == "__main__":
    asyncio.run(main())
