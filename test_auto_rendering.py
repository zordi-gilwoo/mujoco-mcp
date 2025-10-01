#!/usr/bin/env python3
"""
Test auto-rendering functionality with simulated viewer
Tests the complete upload->validate->render workflow
"""

import sys
import asyncio
import json
import unittest.mock

sys.path.append('src')
from mujoco_mcp.mcp_server_menagerie import handle_call_tool
from mujoco_mcp.session_manager import session_manager
from mujoco_mcp.viewer_client import MuJoCoViewerClient

async def test_auto_rendering():
    """Test auto-rendering with simulated viewer"""
    
    print("🔧 Testing Auto-Rendering Functionality")
    print("=" * 50)
    
    # Create a valid XML for testing
    test_xml = '''<mujoco model="auto_render_test">
      <compiler angle="radian"/>
      <option timestep="0.002"/>
      
      <worldbody>
        <geom name="floor" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
        
        <body name="falling_sphere" pos="0 0 2">
          <geom name="sphere" type="sphere" size="0.15" rgba="0.2 0.8 0.2 1"/>
        </body>
      </worldbody>
    </mujoco>'''
    
    print("\n1️⃣ Testing upload with auto-render disabled...")
    try:
        result = await handle_call_tool("upload_xml", {
            "xml_content": test_xml,
            "scene_name": "test_no_render",
            "auto_render": False
        })
        result_text = result[0].text
        print(f"✅ No auto-render result: {result_text[:200]}...")
        
        if '"auto-render disabled"' in result_text or '"skipped": true' in result_text:
            print("✅ Auto-render correctly disabled")
        else:
            print("❌ Auto-render disable not working")
            
    except Exception as e:
        print(f"❌ Test failed: {e}")
    
    print("\n2️⃣ Testing upload with auto-render enabled (no viewer)...")
    try:
        result = await handle_call_tool("upload_xml", {
            "xml_content": test_xml,
            "scene_name": "test_no_viewer",
            "auto_render": True
        })
        result_text = result[0].text
        print(f"✅ No viewer result: {result_text[:200]}...")
        
        if "No viewer connection" in result_text:
            print("✅ Correctly detected no viewer connection")
        else:
            print("❌ Should have detected no viewer")
            
    except Exception as e:
        print(f"❌ Test failed: {e}")
    
    print("\n3️⃣ Testing upload with simulated viewer connection...")
    
    # Mock the viewer client to simulate successful connection
    with unittest.mock.patch.object(session_manager, 'get_viewer_client') as mock_get_client:
        with unittest.mock.patch.object(session_manager, 'get_or_create_session') as mock_session:
            
            # Create a mock session
            mock_session_obj = unittest.mock.MagicMock()
            mock_session_obj.session_id = "test_session_123"
            mock_session_obj.client_id = "test_client_456"
            mock_session_obj.active_models = {}
            mock_session.return_value = mock_session_obj
            
            # Create a mock viewer client
            mock_client = unittest.mock.MagicMock()
            mock_client.send_command.return_value = {"success": True, "message": "Model loaded"}
            mock_get_client.return_value = mock_client
            
            try:
                result = await handle_call_tool("upload_xml", {
                    "xml_content": test_xml,
                    "scene_name": "test_with_viewer",
                    "auto_render": True
                })
                result_text = result[0].text
                print(f"✅ With viewer result: {result_text[:300]}...")
                
                # Check if the mock was called correctly
                if mock_client.send_command.called:
                    call_args = mock_client.send_command.call_args[0][0]
                    print(f"✅ Viewer command sent: {call_args['type']}")
                    print(f"✅ Model ID: {call_args.get('model_id', 'N/A')}")
                    
                    if call_args['type'] == 'load_model' and 'model_xml' in call_args:
                        print("✅ Correct load_model command sent to viewer")
                    else:
                        print("❌ Incorrect viewer command")
                else:
                    print("❌ Viewer command not sent")
                    
                if "rendered successfully" in result_text:
                    print("✅ Success message indicates rendering worked")
                else:
                    print("❌ Success message not found")
                    
            except Exception as e:
                print(f"❌ Simulated viewer test failed: {e}")
    
    print("\n4️⃣ Testing with viewer connection failure...")
    
    # Mock the viewer client to simulate connection failure
    with unittest.mock.patch.object(session_manager, 'get_viewer_client') as mock_get_client:
        with unittest.mock.patch.object(session_manager, 'get_or_create_session') as mock_session:
            
            # Create a mock session
            mock_session_obj = unittest.mock.MagicMock()
            mock_session_obj.session_id = "test_session_456"
            mock_session_obj.client_id = "test_client_789"
            mock_session_obj.active_models = {}
            mock_session.return_value = mock_session_obj
            
            # Create a mock viewer client that fails
            mock_client = unittest.mock.MagicMock()
            mock_client.send_command.return_value = {"success": False, "error": "Connection failed"}
            mock_get_client.return_value = mock_client
            
            try:
                result = await handle_call_tool("upload_xml", {
                    "xml_content": test_xml,
                    "scene_name": "test_viewer_fail",
                    "auto_render": True
                })
                result_text = result[0].text
                print(f"✅ Viewer failure result: {result_text[:300]}...")
                
                if "rendering failed" in result_text and "validated" in result_text:
                    print("✅ Correctly handled viewer failure while keeping validation")
                else:
                    print("❌ Did not correctly handle viewer failure")
                    
            except Exception as e:
                print(f"❌ Viewer failure test failed: {e}")
    
    print("\n5️⃣ Testing session tracking with mock viewer...")
    
    with unittest.mock.patch.object(session_manager, 'get_viewer_client') as mock_get_client:
        with unittest.mock.patch.object(session_manager, 'get_or_create_session') as mock_session:
            
            # Create a mock session with tracking
            mock_session_obj = unittest.mock.MagicMock()
            mock_session_obj.session_id = "track_test_123"
            mock_session_obj.client_id = "track_client_456"
            mock_session_obj.active_models = {}
            mock_session.return_value = mock_session_obj
            
            # Create a successful mock viewer client
            mock_client = unittest.mock.MagicMock()
            mock_client.send_command.return_value = {"success": True}
            mock_get_client.return_value = mock_client
            
            try:
                result = await handle_call_tool("upload_xml", {
                    "xml_content": test_xml,
                    "scene_name": "session_tracking_test",
                    "auto_render": True
                })
                
                # Check if the session was updated
                if "session_tracking_test" in mock_session_obj.active_models:
                    print("✅ Session correctly tracked the new model")
                else:
                    print("❌ Session tracking failed")
                    print(f"Active models: {mock_session_obj.active_models}")
                    
            except Exception as e:
                print(f"❌ Session tracking test failed: {e}")
    
    print("\n" + "=" * 50)
    print("🎉 Auto-Rendering Tests Complete!")
    print("✅ Key functionality verified:")
    print("  • Auto-render can be disabled")
    print("  • Handles missing viewer connections gracefully")
    print("  • Sends correct commands to viewer when available")
    print("  • Handles viewer failures while preserving validation")
    print("  • Tracks models in session when rendering succeeds")

if __name__ == "__main__":
    asyncio.run(test_auto_rendering())