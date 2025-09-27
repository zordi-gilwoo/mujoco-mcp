#!/usr/bin/env python3
"""
Test ProcessManager functionality for multi-client support
"""

import time
import asyncio
import sys
import os
import signal
from pathlib import Path

# Add src to path for testing
sys.path.insert(0, str(Path(__file__).parent / "src"))

from mujoco_mcp.process_manager import test_process_manager as process_manager
from mujoco_mcp.session_manager import SessionManager


def test_process_manager_basic():
    """Test basic ProcessManager functionality"""
    print("🧪 Testing ProcessManager Basic Functionality")
    print("=" * 50)
    
    try:
        # Test spawning a process
        print("1. Testing process spawning...")
        session_id = "test_session_1"
        
        process_info = process_manager.spawn_viewer_process(session_id, isolated=True)
        
        if process_info:
            print(f"   ✅ Process spawned successfully")
            print(f"   📊 PID: {process_info.pid}")
            print(f"   🔌 Port: {process_info.port}")
            print(f"   🌐 URL: {process_info.url}")
            print(f"   ▶️  Running: {process_info.is_running}")
        else:
            print("   ❌ Failed to spawn process")
            return False
        
        # Test process listing
        print("\n2. Testing process listing...")
        processes = process_manager.list_processes()
        print(f"   📋 Active processes: {len(processes)}")
        for sid, info in processes.items():
            print(f"   - {sid}: PID={info['pid']}, Port={info['port']}, Running={info['is_running']}")
        
        # Test process stats
        print("\n3. Testing process stats...")
        stats = process_manager.get_stats()
        print(f"   📊 Stats: {stats}")
        
        # Test process termination
        print("\n4. Testing process termination...")
        success = process_manager.terminate_process(session_id)
        if success:
            print("   ✅ Process terminated successfully")
        else:
            print("   ❌ Failed to terminate process")
            return False
        
        # Verify process is gone
        print("\n5. Verifying process cleanup...")
        processes_after = process_manager.list_processes()
        if session_id not in processes_after:
            print("   ✅ Process cleaned up successfully")
        else:
            print("   ❌ Process still exists after termination")
            return False
        
        print("\n✅ ProcessManager basic tests passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ ProcessManager basic tests failed: {e}")
        return False


def test_session_manager_with_processes():
    """Test SessionManager with ProcessManager integration"""
    print("\n🧪 Testing SessionManager with ProcessManager")
    print("=" * 50)
    
    try:
        # Create session manager with isolated processes enabled
        session_mgr = SessionManager(use_isolated_processes=True)
        
        print("1. Testing session creation with isolated processes...")
        
        # Simulate different MCP contexts
        contexts = [
            {"session_id": "test_session_A"},
            {"session_id": "test_session_B"},
            {"session_id": "test_session_C"}
        ]
        
        sessions = []
        for context in contexts:
            session = session_mgr.get_or_create_session(context)
            sessions.append(session)
            print(f"   ✅ Created session: {session.session_id}")
            print(f"      - Client ID: {session.client_id}")
            print(f"      - Use isolated process: {session.use_isolated_process}")
        
        print("\n2. Testing viewer client creation (which spawns processes)...")
        for i, context in enumerate(contexts):
            print(f"   Creating viewer client for session {i+1}...")
            client = session_mgr.get_viewer_client(context)
            if client:
                print(f"   ✅ Viewer client created successfully")
            else:
                print(f"   ❌ Failed to create viewer client")
                return False
        
        print("\n3. Testing session stats with process information...")
        stats = session_mgr.get_session_stats()
        print(f"   📊 Active sessions: {stats['active_sessions']}")
        print(f"   🔧 Isolated process mode: {stats['isolated_process_mode']}")
        print(f"   📈 Process stats: {stats['process_stats']}")
        
        for session_id, session_info in stats['sessions'].items():
            print(f"   - Session {session_id}:")
            print(f"     Client: {session_info['client_id']}")
            print(f"     Port: {session_info['viewer_port']}")
            if session_info.get('process'):
                proc = session_info['process']
                print(f"     Process: PID={proc['pid']}, Running={proc['is_running']}")
            else:
                print(f"     Process: Not spawned yet")
        
        print("\n4. Testing session cleanup...")
        session_mgr.cleanup_all_sessions()
        
        # Verify cleanup
        stats_after = session_mgr.get_session_stats()
        if stats_after['active_sessions'] == 0:
            print("   ✅ All sessions cleaned up successfully")
        else:
            print(f"   ❌ Sessions still active after cleanup: {stats_after['active_sessions']}")
            return False
        
        print("\n✅ SessionManager with ProcessManager tests passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ SessionManager with ProcessManager tests failed: {e}")
        return False


def test_multiple_concurrent_processes():
    """Test spawning multiple concurrent processes"""
    print("\n🧪 Testing Multiple Concurrent Processes")
    print("=" * 50)
    
    try:
        session_ids = [f"concurrent_session_{i}" for i in range(5)]
        spawned_processes = []
        
        print("1. Spawning multiple processes...")
        for session_id in session_ids:
            print(f"   Spawning process for {session_id}...")
            process_info = process_manager.spawn_viewer_process(session_id, isolated=True)
            if process_info:
                spawned_processes.append(process_info)
                print(f"   ✅ {session_id}: PID={process_info.pid}, Port={process_info.port}")
            else:
                print(f"   ❌ Failed to spawn process for {session_id}")
        
        print(f"\n2. Successfully spawned {len(spawned_processes)} processes")
        
        # Test that all processes are using different ports
        ports = [p.port for p in spawned_processes]
        unique_ports = set(ports)
        if len(ports) == len(unique_ports):
            print("   ✅ All processes using unique ports")
        else:
            print("   ❌ Port conflicts detected!")
            return False
        
        print("\n3. Testing concurrent process health...")
        time.sleep(2)  # Let processes stabilize
        
        alive_count = sum(1 for p in spawned_processes if p.is_running)
        print(f"   📊 Processes still running: {alive_count}/{len(spawned_processes)}")
        
        print("\n4. Cleaning up concurrent processes...")
        for session_id in session_ids:
            success = process_manager.terminate_process(session_id)
            if success:
                print(f"   ✅ Terminated {session_id}")
            else:
                print(f"   ❌ Failed to terminate {session_id}")
        
        print("\n✅ Multiple concurrent processes test passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ Multiple concurrent processes test failed: {e}")
        return False


def main():
    """Run all process manager tests"""
    print("🚀 MuJoCo MCP Process Manager Tests")
    print("=" * 60)
    
    tests = [
        ("Basic ProcessManager", test_process_manager_basic),
        ("SessionManager Integration", test_session_manager_with_processes),
        ("Concurrent Processes", test_multiple_concurrent_processes),
    ]
    
    results = {}
    passed = 0
    
    for test_name, test_func in tests:
        print(f"\n🔍 Running {test_name}...")
        try:
            success = test_func()
            results[test_name] = success
            if success:
                passed += 1
        except Exception as e:
            print(f"❌ {test_name} failed with exception: {e}")
            results[test_name] = False
    
    # Cleanup any remaining processes
    print("\n🧹 Final cleanup...")
    try:
        process_manager.cleanup_all()
        print("✅ Cleanup completed")
    except Exception as e:
        print(f"❌ Cleanup failed: {e}")
    
    # Print final results
    print(f"\n" + "=" * 60)
    print("📊 TEST RESULTS")
    print("=" * 60)
    
    for test_name, success in results.items():
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} {test_name}")
    
    print(f"\nTotal: {passed}/{len(tests)} tests passed")
    
    if passed == len(tests):
        print("🎉 All tests passed!")
        return 0
    else:
        print("💥 Some tests failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())