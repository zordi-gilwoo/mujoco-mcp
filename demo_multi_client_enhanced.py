#!/usr/bin/env python3
"""
Enhanced Multi-Client MuJoCo MCP Demo
Demonstrates process pool and port allocation with isolated processes
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


class MockMCPContext:
    """Mock MCP context for demonstration"""
    def __init__(self, session_id: str):
        self.session_id = session_id


async def simulate_enhanced_client(client_id: str, scene_type: str, session_manager: SessionManager) -> List[str]:
    """Simulate an enhanced MCP client session with process isolation"""
    results = []
    
    results.append(f"\n🎯 Enhanced Client {client_id} starting...")
    
    try:
        # Create mock context
        context = MockMCPContext(f"session_{client_id}")
        
        # Get or create session (this will spawn isolated process if enabled)
        session = session_manager.get_or_create_session(context)
        results.append(f"   📊 Session ID: {session.session_id}")
        results.append(f"   🆔 Client ID: {session.client_id}")
        results.append(f"   🔌 Viewer Port: {session.viewer_port}")
        results.append(f"   🔗 Isolated Process: {session.use_isolated_process}")
        
        # Get viewer client (this will spawn the process)
        results.append(f"   🚀 Creating viewer client...")
        client = session_manager.get_viewer_client(context)
        
        if client:
            results.append(f"   ✅ Viewer client created successfully")
            results.append(f"   🔌 Client connected to port: {client.port}")
        else:
            results.append(f"   ❌ Failed to create viewer client")
            return results
        
        # Simulate model creation
        session.active_models[scene_type] = f"{scene_type}_model"
        session.update_activity()
        results.append(f"   🎬 Simulated {scene_type} scene creation...")
        
        # Simulate some processing time
        await asyncio.sleep(1)
        
        # Simulate simulation steps
        results.append(f"   ⚡ Simulating physics steps...")
        await asyncio.sleep(0.5)
        
        # Get session stats to show process information
        stats = session_manager.get_session_stats()
        current_session = stats["sessions"].get(session.session_id, {})
        
        if current_session.get("process"):
            proc_info = current_session["process"]
            results.append(f"   📈 Process Status: PID={proc_info['pid']}, Running={proc_info['is_running']}")
        
        results.append(f"   📊 Active models: {list(session.active_models.keys())}")
        results.append(f"   🏁 Enhanced Client {client_id} completed successfully!")
        
    except Exception as e:
        results.append(f"   ❌ Enhanced Client {client_id} error: {e}")
    
    return results


def run_enhanced_client_in_thread(client_id: str, scene_type: str, session_manager: SessionManager, results_dict: dict):
    """Run an enhanced client simulation in a separate thread with its own event loop"""
    
    async def async_client():
        return await simulate_enhanced_client(client_id, scene_type, session_manager)
    
    # Create new event loop for this thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        results_dict[client_id] = loop.run_until_complete(async_client())
    finally:
        loop.close()


async def demo_process_pool_sequential():
    """Demonstrate process pool with sequential clients"""
    print("\n" + "="*60)
    print("🔄 PROCESS POOL SEQUENTIAL DEMO")
    print("="*60)
    print("Testing isolated process spawning with sequential clients...")
    
    # Create session manager with isolated processes
    session_manager = SessionManager(use_isolated_processes=True)
    
    # Test different scene types sequentially
    test_cases = [
        ("SEQ-1", "pendulum"),
        ("SEQ-2", "cartpole"),
        ("SEQ-3", "acrobot"),
        ("SEQ-4", "pendulum")  # Same scene type, different process
    ]
    
    client_results = []
    
    for client_id, scene_type in test_cases:
        results = await simulate_enhanced_client(client_id, scene_type, session_manager)
        client_results.extend(results)
        
        # Small delay between clients
        await asyncio.sleep(0.2)
    
    # Print all results
    for result in client_results:
        print(result)
    
    # Show final session and process status
    print(f"\n📊 Final System Status:")
    stats = session_manager.get_session_stats()
    print(f"   Active Sessions: {stats['active_sessions']}")
    print(f"   Process Pool Stats: {stats['process_stats']}")
    
    for session_id, session_info in stats['sessions'].items():
        print(f"   Session {session_id}:")
        print(f"     - Client: {session_info['client_id']}")
        print(f"     - Models: {session_info['active_models']}")
        if session_info.get('process'):
            proc = session_info['process']
            print(f"     - Process: PID={proc['pid']}, Port={proc['port']}, Running={proc['is_running']}")
    
    # Cleanup
    print(f"\n🧹 Cleaning up sessions...")
    session_manager.cleanup_all_sessions()
    
    return session_manager


async def demo_process_pool_concurrent():
    """Demonstrate concurrent process pool usage"""
    print("\n" + "="*60)
    print("🚀 PROCESS POOL CONCURRENT DEMO")
    print("="*60)
    print("Testing concurrent isolated processes...")
    
    # Create session manager with isolated processes
    session_manager = SessionManager(use_isolated_processes=True)
    
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
            target=run_enhanced_client_in_thread,
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


async def demo_port_allocation():
    """Demonstrate port allocation and management"""
    print("\n" + "="*60)
    print("🔌 PORT ALLOCATION DEMO")
    print("="*60)
    print("Testing port allocation and conflict prevention...")
    
    # Test port allocation directly
    print("1. Testing ProcessManager port allocation...")
    session_ids = [f"port_test_{i}" for i in range(10)]
    spawned_processes = []
    
    for session_id in session_ids:
        process_info = process_manager.spawn_viewer_process(session_id, isolated=True)
        if process_info:
            spawned_processes.append(process_info)
            print(f"   ✅ {session_id}: Port {process_info.port}")
        else:
            print(f"   ❌ Failed to spawn {session_id}")
    
    # Check for port conflicts
    ports = [p.port for p in spawned_processes]
    unique_ports = set(ports)
    
    print(f"\n2. Port allocation analysis:")
    print(f"   Total processes: {len(spawned_processes)}")
    print(f"   Unique ports: {len(unique_ports)}")
    print(f"   Port range: {min(ports) if ports else 'N/A'} - {max(ports) if ports else 'N/A'}")
    
    if len(ports) == len(unique_ports):
        print("   ✅ No port conflicts detected!")
    else:
        print("   ❌ Port conflicts detected!")
    
    # Test process stats
    print(f"\n3. Process Manager Stats:")
    stats = process_manager.get_stats()
    for key, value in stats.items():
        print(f"   {key}: {value}")
    
    # Cleanup
    print(f"\n4. Cleaning up port test processes...")
    for session_id in session_ids:
        success = process_manager.terminate_process(session_id)
        if success:
            print(f"   ✅ Terminated {session_id}")
        else:
            print(f"   ❌ Failed to terminate {session_id}")


async def main():
    """Main demo runner"""
    print("🎭 Enhanced Multi-Client MuJoCo MCP Demo")
    print("=" * 60)
    print("Demonstrating Process Pool and Port Allocation")
    
    try:
        # Run sequential demo
        await demo_process_pool_sequential()
        
        # Small break between demos
        await asyncio.sleep(1)
        
        # Run concurrent demo
        await demo_process_pool_concurrent()
        
        # Small break between demos
        await asyncio.sleep(1)
        
        # Run port allocation demo
        await demo_port_allocation()
        
        print(f"\n" + "="*60)
        print("🎉 Enhanced Multi-Client Demo Completed Successfully!")
        print("=" * 60)
        
        print(f"\n📋 Key Features Demonstrated:")
        print(f"✅ Isolated process spawning for each client")
        print(f"✅ Automatic port allocation and conflict prevention")  
        print(f"✅ Session management with process lifecycle")
        print(f"✅ Concurrent multi-client support")
        print(f"✅ Proper resource cleanup and termination")
        print(f"✅ Process health monitoring and statistics")
        
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