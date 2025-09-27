#!/usr/bin/env python3
"""
Unified Multi-Client MuJoCo MCP Demo
Demonstrates both session-based and process-based client isolation approaches
"""

import asyncio
import threading
import json
import time
import sys
from pathlib import Path
from typing import List

# Add src to path for testing
sys.path.insert(0, str(Path(__file__).parent / "src"))

from mujoco_mcp.session_manager import SessionManager
from mujoco_mcp.process_manager import test_process_manager as process_manager


# Create a SessionManager that uses the test process manager for demos
class TestSessionManager(SessionManager):
    """SessionManager that uses test_process_manager for demonstration"""
    def __init__(self, use_isolated_processes: bool = True):
        super().__init__(use_isolated_processes)
        # Override the import to use test process manager
        global process_manager
        from mujoco_mcp.process_manager import test_process_manager
        import mujoco_mcp.session_manager
        mujoco_mcp.session_manager.process_manager = test_process_manager


class MockMCPContext:
    """Mock MCP context for demonstration"""
    def __init__(self, session_id: str):
        self.session_id = session_id


def print_section_header(title: str, char: str = "="):
    """Print a formatted section header"""
    width = 70
    print(f"\n{char * width}")
    print(f"{title:^{width}}")
    print(f"{char * width}")


async def simulate_client_session_based(client_id: str, scene_type: str, session_manager: SessionManager) -> List[str]:
    """Simulate a client session using session-based approach (original)"""
    results = []
    
    results.append(f"\n🎯 Session-Based Client {client_id} starting...")
    
    try:
        # Create mock context
        context = MockMCPContext(f"session_based_{client_id}")
        
        # Get or create session
        session = session_manager.get_or_create_session(context)
        results.append(f"   📊 Session ID: {session.session_id}")
        results.append(f"   🆔 Client ID: {session.client_id}")
        results.append(f"   🔌 Viewer Port: {session.viewer_port}")
        results.append(f"   🔗 Isolated Process: {session.use_isolated_process}")
        
        # Simulate getting viewer client (connects to shared viewer server)
        results.append(f"   🚀 Connecting to shared viewer server...")
        # Note: In session-based mode, would connect to existing viewer server
        
        # Simulate model creation
        session.active_models[scene_type] = f"{scene_type}_model"
        session.update_activity()
        results.append(f"   🎬 Created {scene_type} scene in shared process")
        
        # Simulate some processing time
        await asyncio.sleep(0.5)
        
        results.append(f"   📊 Active models: {list(session.active_models.keys())}")
        results.append(f"   🏁 Session-Based Client {client_id} completed!")
        
    except Exception as e:
        results.append(f"   ❌ Session-Based Client {client_id} error: {e}")
    
    return results


async def simulate_client_process_based(client_id: str, scene_type: str, session_manager: SessionManager) -> List[str]:
    """Simulate a client session using process-based approach (enhanced)"""
    results = []
    
    results.append(f"\n🎯 Process-Based Client {client_id} starting...")
    
    try:
        # Create mock context
        context = MockMCPContext(f"process_based_{client_id}")
        
        # Get or create session (will spawn isolated process)
        session = session_manager.get_or_create_session(context)
        results.append(f"   📊 Session ID: {session.session_id}")
        results.append(f"   🆔 Client ID: {session.client_id}")
        results.append(f"   🔌 Viewer Port: {session.viewer_port}")
        results.append(f"   🔗 Isolated Process: {session.use_isolated_process}")
        
        # Get viewer client (this will spawn the dedicated process)
        results.append(f"   🚀 Spawning dedicated viewer process...")
        client = session_manager.get_viewer_client(context)
        
        if client:
            results.append(f"   ✅ Dedicated process spawned on port: {client.port}")
            
            # Get process information
            process_info = process_manager.get_process_info(session.session_id)
            if process_info:
                results.append(f"   📈 Process PID: {process_info.pid}, Running: {process_info.is_running}")
        else:
            results.append(f"   ❌ Failed to spawn dedicated process")
            return results
        
        # Simulate model creation
        session.active_models[scene_type] = f"{scene_type}_model"
        session.update_activity()
        results.append(f"   🎬 Created {scene_type} scene in dedicated process")
        
        # Simulate some processing time
        await asyncio.sleep(0.5)
        
        results.append(f"   📊 Active models: {list(session.active_models.keys())}")
        results.append(f"   🏁 Process-Based Client {client_id} completed!")
        
    except Exception as e:
        results.append(f"   ❌ Process-Based Client {client_id} error: {e}")
    
    return results


def run_client_in_thread(client_func, client_id: str, scene_type: str, session_manager: SessionManager, results_dict: dict):
    """Run a client simulation in a separate thread with its own event loop"""
    
    async def async_client():
        return await client_func(client_id, scene_type, session_manager)
    
    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        results_dict[client_id] = loop.run_until_complete(async_client())
    finally:
        loop.close()


async def demo_session_based_approach():
    """Demonstrate the original session-based multi-client approach"""
    print_section_header("SESSION-BASED MULTI-CLIENT DEMO")
    print("Multiple clients share viewer server with session isolation")
    
    # Create session manager with session-based isolation
    session_manager = SessionManager(use_isolated_processes=False)
    
    # Test different scene types sequentially
    test_cases = [
        ("SB-1", "pendulum"),
        ("SB-2", "cartpole"),
        ("SB-3", "acrobot")
    ]
    
    client_results = []
    
    for client_id, scene_type in test_cases:
        results = await simulate_client_session_based(client_id, scene_type, session_manager)
        client_results.extend(results)
        await asyncio.sleep(0.1)
    
    # Print all results
    for result in client_results:
        print(result)
    
    # Show final session status
    print(f"\n📊 Final Session-Based System Status:")
    stats = session_manager.get_session_stats()
    print(f"   Active Sessions: {stats['active_sessions']}")
    print(f"   Isolated Process Mode: {stats['isolated_process_mode']}")
    
    for session_id, session_info in stats['sessions'].items():
        print(f"   Session {session_id}:")
        print(f"     - Client: {session_info['client_id']}")
        print(f"     - Port: {session_info['viewer_port']}")
        print(f"     - Models: {session_info['active_models']}")
    
    # Cleanup
    print(f"\n🧹 Cleaning up session-based sessions...")
    session_manager.cleanup_all_sessions()
    
    return session_manager


async def demo_process_based_approach():
    """Demonstrate the enhanced process-based multi-client approach"""
    print_section_header("PROCESS-BASED MULTI-CLIENT DEMO")
    print("Each client gets dedicated viewer process with complete isolation")
    
    # Create session manager with process-based isolation (using test process manager)
    session_manager = TestSessionManager(use_isolated_processes=True)
    
    # Test different scene types sequentially
    test_cases = [
        ("PB-1", "pendulum"),
        ("PB-2", "cartpole"),
        ("PB-3", "acrobot")
    ]
    
    client_results = []
    
    for client_id, scene_type in test_cases:
        results = await simulate_client_process_based(client_id, scene_type, session_manager)
        client_results.extend(results)
        await asyncio.sleep(0.2)
    
    # Print all results
    for result in client_results:
        print(result)
    
    # Show final session status
    print(f"\n📊 Final Process-Based System Status:")
    stats = session_manager.get_session_stats()
    print(f"   Active Sessions: {stats['active_sessions']}")
    print(f"   Isolated Process Mode: {stats['isolated_process_mode']}")
    print(f"   Process Pool Stats: {stats['process_stats']}")
    
    for session_id, session_info in stats['sessions'].items():
        print(f"   Session {session_id}:")
        print(f"     - Client: {session_info['client_id']}")
        if session_info.get('process'):
            proc = session_info['process']
            print(f"     - Process: PID={proc['pid']}, Port={proc['port']}, Running={proc['is_running']}")
        print(f"     - Models: {session_info['active_models']}")
    
    # Cleanup
    print(f"\n🧹 Cleaning up process-based sessions...")
    session_manager.cleanup_all_sessions()
    
    return session_manager


async def demo_concurrent_comparison():
    """Demonstrate concurrent clients using both approaches"""
    print_section_header("CONCURRENT MULTI-CLIENT COMPARISON")
    print("Running both approaches concurrently to show the differences")
    
    # Create both types of session managers
    session_manager_sb = SessionManager(use_isolated_processes=False)
    session_manager_pb = TestSessionManager(use_isolated_processes=True)
    
    # Test concurrent clients
    concurrent_tests = [
        (simulate_client_session_based, "CONC-SB-1", "pendulum", session_manager_sb),
        (simulate_client_session_based, "CONC-SB-2", "cartpole", session_manager_sb),
        (simulate_client_process_based, "CONC-PB-1", "pendulum", session_manager_pb),
        (simulate_client_process_based, "CONC-PB-2", "cartpole", session_manager_pb),
    ]
    
    print(f"Starting {len(concurrent_tests)} concurrent clients (mixed approaches)...")
    
    # Run all clients concurrently in separate threads
    results_dict = {}
    threads = []
    
    for client_func, client_id, scene_type, session_mgr in concurrent_tests:
        thread = threading.Thread(
            target=run_client_in_thread,
            args=(client_func, client_id, scene_type, session_mgr, results_dict)
        )
        threads.append(thread)
        thread.start()
    
    # Wait for all threads to complete
    for thread in threads:
        thread.join()
    
    # Print all results
    print(f"\n📋 Concurrent Client Results:")
    for client_id in sorted(results_dict.keys()):
        for result in results_dict[client_id]:
            print(result)
    
    # Show final comparison
    print(f"\n📊 Final Concurrent System Comparison:")
    stats_sb = session_manager_sb.get_session_stats()
    stats_pb = session_manager_pb.get_session_stats()
    
    print(f"   Session-Based: {stats_sb['active_sessions']} sessions")
    print(f"   Process-Based: {stats_pb['active_sessions']} sessions, {stats_pb['process_stats']['running_processes']} processes")
    
    # Cleanup both
    print(f"\n🧹 Cleaning up concurrent sessions...")
    session_manager_sb.cleanup_all_sessions()
    session_manager_pb.cleanup_all_sessions()


async def main():
    """Main demo runner"""
    print("🎭 Unified Multi-Client MuJoCo MCP Demo")
    print_section_header("ARCHITECTURE COMPARISON DEMO", "=")
    print("Comparing Session-Based vs Process-Based Multi-Client Architectures")
    
    try:
        # Demo 1: Session-based approach (original)
        await demo_session_based_approach()
        
        # Small break between demos
        await asyncio.sleep(1)
        
        # Demo 2: Process-based approach (enhanced)
        await demo_process_based_approach()
        
        # Small break between demos
        await asyncio.sleep(1)
        
        # Demo 3: Concurrent comparison
        await demo_concurrent_comparison()
        
        print_section_header("DEMO COMPLETED SUCCESSFULLY", "=")
        
        print(f"\n📋 Key Architecture Differences Demonstrated:")
        print(f"✅ Session-Based: Fast setup, shared resources, session isolation")
        print(f"✅ Process-Based: Complete isolation, crash protection, resource separation")
        print(f"✅ Both: Port management, client tracking, automatic cleanup")
        print(f"✅ Configurable: Choose approach based on deployment needs")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        return 1
    
    finally:
        # Final cleanup
        try:
            print(f"\n🧹 Final cleanup...")
            process_manager.cleanup_all()
            print("✅ Final cleanup completed")
        except Exception as e:
            print(f"❌ Final cleanup failed: {e}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))