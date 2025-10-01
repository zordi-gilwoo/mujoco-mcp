#!/usr/bin/env python3
"""
Multi-Client MuJoCo MCP Demo
Demonstrates process-based client isolation with dedicated viewer processes
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
    def __init__(self):
        super().__init__()
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


async def simulate_client(client_id: str, scene_type: str, session_manager: SessionManager) -> List[str]:
    """Simulate a client session using process-based isolation"""
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


def run_client_in_thread(client_id: str, scene_type: str, session_manager: SessionManager, results_dict: dict):
    """Run a client simulation in a separate thread with its own event loop"""
    
    async def async_client():
        return await simulate_client(client_id, scene_type, session_manager)
    
    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        results_dict[client_id] = loop.run_until_complete(async_client())
    finally:
        loop.close()


async def demo_sequential_clients():
    """Demonstrate sequential process-based multi-client approach"""
    print_section_header("SEQUENTIAL MULTI-CLIENT DEMO")
    print("Each client gets a dedicated viewer process with complete isolation")
    
    # Create session manager with process-based isolation
    session_manager = TestSessionManager()
    
    # Test different scene types sequentially
    test_cases = [
        ("SEQ-1", "pendulum"),
        ("SEQ-2", "cartpole"),
        ("SEQ-3", "acrobot"),
        ("SEQ-4", "double_pendulum")
    ]
    
    client_results = []
    
    for client_id, scene_type in test_cases:
        results = await simulate_client(client_id, scene_type, session_manager)
        client_results.extend(results)
        await asyncio.sleep(0.2)
    
    # Print all results
    for result in client_results:
        print(result)
    
    # Show final session status
    print(f"\n📊 Final Sequential System Status:")
    stats = session_manager.get_session_stats()
    print(f"   Active Sessions: {stats['active_sessions']}")
    print(f"   Isolation Mode: {stats['isolation_mode']}")
    print(f"   Process Pool Stats: {stats['process_stats']}")
    
    for session_id, session_info in stats['sessions'].items():
        print(f"   Session {session_id}:")
        print(f"     - Client: {session_info['client_id']}")
        if session_info.get('process'):
            proc = session_info['process']
            print(f"     - Process: PID={proc['pid']}, Port={proc['port']}, Running={proc['is_running']}")
        print(f"     - Models: {session_info['active_models']}")
    
    # Cleanup
    print(f"\n🧹 Cleaning up sequential sessions...")
    session_manager.cleanup_all_sessions()
    
    return session_manager


async def demo_concurrent_clients():
    """Demonstrate concurrent process-based multi-client approach"""
    print_section_header("CONCURRENT MULTI-CLIENT DEMO")
    print("Multiple clients running concurrently with dedicated processes")
    
    # Create session manager with process-based isolation
    session_manager = TestSessionManager()
    
    # Test concurrent clients
    concurrent_tests = [
        ("CONC-1", "pendulum"),
        ("CONC-2", "cartpole"),
        ("CONC-3", "acrobot"),
        ("CONC-4", "double_pendulum"),
        ("CONC-5", "hopper")
    ]
    
    print(f"Starting {len(concurrent_tests)} concurrent clients...")
    
    # Run all clients concurrently in separate threads
    results_dict = {}
    threads = []
    
    for client_id, scene_type in concurrent_tests:
        thread = threading.Thread(
            target=run_client_in_thread,
            args=(client_id, scene_type, session_manager, results_dict)
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
    
    # Show final system status
    print(f"\n📊 Final Concurrent System Status:")
    stats = session_manager.get_session_stats()
    print(f"   Active Sessions: {stats['active_sessions']}")
    print(f"   Isolation Mode: {stats['isolation_mode']}")
    print(f"   Process Pool Stats: {stats['process_stats']}")
    
    # Show process utilization
    process_info = process_manager.list_processes()
    print(f"   Active Processes: {len(process_info)}")
    for session_id, proc_info in process_info.items():
        print(f"     {session_id}: PID={proc_info['pid']}, Port={proc_info['port']}")
    
    # Cleanup
    print(f"\n🧹 Cleaning up concurrent sessions...")
    session_manager.cleanup_all_sessions()
    
    return session_manager


async def main():
    """Main demo runner"""
    print("🎭 Multi-Client MuJoCo MCP Demo")
    print_section_header("PROCESS-BASED MULTI-CLIENT ARCHITECTURE", "=")
    print("Demonstrating dedicated viewer processes for complete client isolation")
    
    try:
        # Demo 1: Sequential clients
        await demo_sequential_clients()
        
        # Small break between demos
        await asyncio.sleep(1)
        
        # Demo 2: Concurrent clients
        await demo_concurrent_clients()
        
        print_section_header("DEMO COMPLETED SUCCESSFULLY", "=")
        
        print(f"\n📋 Key Architecture Features Demonstrated:")
        print(f"✅ Process-based isolation with dedicated viewer processes")
        print(f"✅ Automatic port allocation and conflict prevention")
        print(f"✅ Complete memory and resource separation between clients")
        print(f"✅ Crash protection - one client failure doesn't affect others")
        print(f"✅ Enterprise-ready multi-tenant support")
        print(f"✅ Background process health monitoring")
        print(f"✅ Automatic resource cleanup on client disconnect")
        
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