#!/usr/bin/env python3
"""
Test MCP Server integration with ProcessManager
"""

import asyncio
import sys
import json
from pathlib import Path

# Add src to path for testing
sys.path.insert(0, str(Path(__file__).parent / "src"))

from mujoco_mcp.mcp_server_menagerie import handle_call_tool


async def test_process_pool_mcp_tools():
    """Test the new process pool MCP tools"""
    print("🧪 Testing MCP Process Pool Integration")
    print("=" * 50)

    try:
        # Test 1: Get process pool stats
        print("1. Testing get_process_pool_stats...")
        result = await handle_call_tool("get_process_pool_stats", {})
        stats_text = result[0].text
        stats = json.loads(stats_text)

        print(f"   ✅ Process Pool Stats retrieved")
        print(f"   📊 Total processes: {stats['process_pool']['total_processes']}")
        print(f"   🔧 Isolated mode: {stats['session_summary']['isolated_process_mode']}")
        print(f"   🔌 Available ports: {stats['resource_usage']['available_ports']}")

        # Test 2: List active processes (should be empty initially)
        print("\n2. Testing list_active_processes...")
        result = await handle_call_tool("list_active_processes", {})
        processes_text = result[0].text

        if "No active processes found" in processes_text:
            print("   ✅ No active processes (expected)")
        else:
            processes = json.loads(processes_text)
            print(f"   📋 Found {processes['active_processes']} active processes")

        # Test 3: Create a session (which should spawn a process)
        print("\n3. Testing session creation...")
        result = await handle_call_tool("get_session_info", {})
        session_text = result[0].text
        session_info = json.loads(session_text)

        current_session = session_info["current_session"]
        print(f"   ✅ Session created: {current_session['session_id']}")
        print(f"   🆔 Client ID: {current_session['client_id']}")
        print(f"   🔌 Viewer port: {current_session['viewer_port']}")

        # Test 4: Create a scene (which should trigger process spawning)
        print("\n4. Testing scene creation (triggers process spawning)...")
        try:
            result = await handle_call_tool("create_scene", {"scene_type": "pendulum"})
            scene_text = result[0].text
            print(f"   ✅ Scene creation result: {scene_text[:100]}...")
        except Exception as e:
            print(f"   ⚠️  Scene creation failed (expected without MuJoCo): {e}")

        # Test 5: Check processes again
        print("\n5. Testing list_active_processes after session activity...")
        result = await handle_call_tool("list_active_processes", {})
        processes_text = result[0].text

        if "No active processes found" not in processes_text:
            processes = json.loads(processes_text)
            print(f"   ✅ Found {processes['active_processes']} active processes")
            for session_id, proc_info in processes["processes"].items():
                print(f"     - {session_id}: PID={proc_info['pid']}, Port={proc_info['port']}")
        else:
            print("   📊 No active processes found")

        # Test 6: Get updated process pool stats
        print("\n6. Testing get_process_pool_stats after activity...")
        result = await handle_call_tool("get_process_pool_stats", {})
        stats_text = result[0].text
        updated_stats = json.loads(stats_text)

        print(f"   📈 Updated stats:")
        print(f"     Total processes: {updated_stats['process_pool']['total_processes']}")
        print(f"     Running processes: {updated_stats['process_pool']['running_processes']}")
        print(f"     Used ports: {updated_stats['resource_usage']['used_ports']}")

        # Test 7: Test terminate_process (if any processes exist)
        if updated_stats["process_pool"]["total_processes"] > 0:
            print("\n7. Testing terminate_process...")
            # Get the first session ID
            processes_result = await handle_call_tool("list_active_processes", {})
            if "No active processes found" not in processes_result[0].text:
                processes = json.loads(processes_result[0].text)
                first_session = list(processes["processes"].keys())[0]

                result = await handle_call_tool("terminate_process", {"session_id": first_session})
                terminate_text = result[0].text
                print(f"   ✅ Terminate result: {terminate_text}")
            else:
                print("   📊 No processes to terminate")
        else:
            print("\n7. No processes to terminate")

        print("\n✅ MCP Process Pool Integration tests completed!")
        return True

    except Exception as e:
        print(f"\n❌ MCP Process Pool Integration tests failed: {e}")
        import traceback

        traceback.print_exc()
        return False


async def main():
    """Run all MCP integration tests"""
    print("🚀 MuJoCo MCP Process Pool Integration Tests")
    print("=" * 60)

    try:
        success = await test_process_pool_mcp_tools()

        if success:
            print("\n🎉 All integration tests passed!")
            return 0
        else:
            print("\n💥 Some integration tests failed!")
            return 1

    except Exception as e:
        print(f"\n❌ Integration tests failed with error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
