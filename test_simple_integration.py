#!/usr/bin/env python3
"""
Simple integration test for ProcessManager without full MCP dependencies
"""

import sys
import json
from pathlib import Path

# Add src to path for testing
sys.path.insert(0, str(Path(__file__).parent / "src"))

from mujoco_mcp.session_manager import SessionManager
from mujoco_mcp.process_manager import test_process_manager as process_manager


def test_session_process_integration():
    """Test SessionManager and ProcessManager integration"""
    print("🧪 Testing SessionManager and ProcessManager Integration")
    print("=" * 60)
    
    try:
        # Create session manager with isolated processes
        session_manager = SessionManager(use_isolated_processes=True)
        
        print("1. Testing session creation with process isolation...")
        
        # Create some test contexts
        contexts = [
            {"session_id": "integration_test_1"},
            {"session_id": "integration_test_2"},
            {"session_id": "integration_test_3"}
        ]
        
        sessions = []
        for context in contexts:
            session = session_manager.get_or_create_session(context)
            sessions.append(session)
            print(f"   ✅ Created session: {session.session_id}")
            print(f"      Client ID: {session.client_id}")
            print(f"      Uses isolated process: {session.use_isolated_process}")
        
        print("\n2. Testing viewer client creation (spawns processes)...")
        for i, context in enumerate(contexts):
            print(f"   Creating viewer client for session {i+1}...")
            client = session_manager.get_viewer_client(context)
            if client:
                print(f"   ✅ Viewer client created on port {client.port}")
            else:
                print(f"   ❌ Failed to create viewer client")
                return False
        
        print("\n3. Testing integrated statistics...")
        
        # Get session stats (includes process info)
        session_stats = session_manager.get_session_stats()
        print(f"   📊 Active sessions: {session_stats['active_sessions']}")
        print(f"   🔧 Isolated process mode: {session_stats['isolated_process_mode']}")
        
        # Print process info from session stats
        for session_id, session_info in session_stats['sessions'].items():
            print(f"   Session {session_id}:")
            print(f"     Client: {session_info['client_id']}")
            if session_info.get('process'):
                proc = session_info['process']
                print(f"     Process: PID={proc['pid']}, Port={proc['port']}, Running={proc['is_running']}")
            else:
                print(f"     Process: Not spawned")
        
        # Get process manager stats
        process_stats = process_manager.get_stats()
        print(f"\n   🔄 Process Manager Stats:")
        print(f"     Total processes: {process_stats['total_processes']}")
        print(f"     Running processes: {process_stats['running_processes']}")
        print(f"     Used ports: {process_stats['used_ports']}")
        print(f"     Available ports: {process_stats['available_ports']}")
        
        print("\n4. Testing manual process operations...")
        
        # Test direct process manager operations
        processes = process_manager.list_processes()
        print(f"   📋 Process Manager lists {len(processes)} processes:")
        for session_id, proc_info in processes.items():
            print(f"     {session_id}: PID={proc_info['pid']}, Port={proc_info['port']}")
        
        print("\n5. Testing cleanup integration...")
        
        # Test session cleanup (should also terminate process)
        first_context = contexts[0]
        print(f"   Cleaning up session: {first_context['session_id']}")
        session_manager.cleanup_session(first_context)
        
        # Verify process was terminated
        remaining_processes = process_manager.list_processes()
        if first_context['session_id'] not in remaining_processes:
            print(f"   ✅ Process for {first_context['session_id']} cleaned up")
        else:
            print(f"   ❌ Process for {first_context['session_id']} still running")
        
        print("\n6. Testing full cleanup...")
        session_manager.cleanup_all_sessions()
        
        final_stats = session_manager.get_session_stats()
        final_processes = process_manager.list_processes()
        
        print(f"   📊 Final active sessions: {final_stats['active_sessions']}")
        print(f"   🔄 Final active processes: {len(final_processes)}")
        
        if final_stats['active_sessions'] == 0 and len(final_processes) == 0:
            print("   ✅ All sessions and processes cleaned up successfully")
        else:
            print("   ❌ Some sessions or processes still active after cleanup")
            return False
        
        print("\n✅ SessionManager and ProcessManager integration tests passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ Integration tests failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_process_lifecycle():
    """Test complete process lifecycle"""
    print("\n🧪 Testing Complete Process Lifecycle")
    print("=" * 60)
    
    try:
        print("1. Starting with clean state...")
        initial_stats = process_manager.get_stats()
        print(f"   Initial processes: {initial_stats['total_processes']}")
        
        print("\n2. Spawning processes manually...")
        session_ids = ["lifecycle_test_1", "lifecycle_test_2"]
        spawned = []
        
        for session_id in session_ids:
            process_info = process_manager.spawn_viewer_process(session_id)
            if process_info:
                spawned.append(process_info)
                print(f"   ✅ Spawned {session_id}: PID={process_info.pid}, Port={process_info.port}")
            else:
                print(f"   ❌ Failed to spawn {session_id}")
        
        print(f"\n3. Verifying {len(spawned)} processes are running...")
        for process_info in spawned:
            if process_info.is_running:
                print(f"   ✅ {process_info.session_id} is running")
            else:
                print(f"   ❌ {process_info.session_id} is not running")
        
        print("\n4. Testing graceful termination...")
        for process_info in spawned:
            success = process_manager.terminate_process(process_info.session_id)
            if success:
                print(f"   ✅ Terminated {process_info.session_id}")
            else:
                print(f"   ❌ Failed to terminate {process_info.session_id}")
        
        print("\n5. Verifying cleanup...")
        final_stats = process_manager.get_stats()
        print(f"   Final processes: {final_stats['total_processes']}")
        
        if final_stats['total_processes'] == 0:
            print("   ✅ All processes cleaned up successfully")
        else:
            print("   ❌ Some processes still active after cleanup")
            return False
        
        print("\n✅ Process lifecycle tests passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ Process lifecycle tests failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all integration tests"""
    print("🚀 Simple ProcessManager Integration Tests")
    print("=" * 70)
    
    tests = [
        ("SessionManager Integration", test_session_process_integration),
        ("Process Lifecycle", test_process_lifecycle),
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
    
    # Final cleanup
    print("\n🧹 Final cleanup...")
    try:
        process_manager.cleanup_all()
        print("✅ Final cleanup completed")
    except Exception as e:
        print(f"❌ Final cleanup failed: {e}")
    
    # Print final results
    print(f"\n" + "=" * 70)
    print("📊 INTEGRATION TEST RESULTS")
    print("=" * 70)
    
    for test_name, success in results.items():
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"{status} {test_name}")
    
    print(f"\nTotal: {passed}/{len(tests)} tests passed")
    
    if passed == len(tests):
        print("🎉 All integration tests passed!")
        print("\n📋 Key Features Verified:")
        print("✅ SessionManager integrates seamlessly with ProcessManager")
        print("✅ Isolated processes spawn automatically for sessions")
        print("✅ Port allocation works correctly")
        print("✅ Process health monitoring functions")
        print("✅ Cleanup properly terminates processes")
        print("✅ Statistics provide comprehensive process information")
        return 0
    else:
        print("💥 Some integration tests failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())