#!/usr/bin/env python3
"""
Test script for new download/upload functionality
Tests XML and Python script download/upload features
"""

import sys
import asyncio
import json
import base64
sys.path.append('src')

from mujoco_mcp.mcp_server_menagerie import handle_call_tool
from mujoco_mcp.file_handler import FileHandler

async def test_download_upload_features():
    """Test all download/upload features"""
    
    print("🧪 Testing Download/Upload Features")
    print("=" * 50)
    
    # Test 1: Download XML for built-in scene
    print("\n1. Testing XML download for built-in scene...")
    try:
        result = await handle_call_tool("download_xml", {"model_id": "pendulum"})
        result_text = result[0].text
        print(f"✅ Download XML result: {result_text[:200]}...")
        
        # Extract base64 content for further testing
        import re
        xml_match = re.search(r'"xml_content_base64":\s*"([^"]+)"', result_text)
        if xml_match:
            pendulum_xml_b64 = xml_match.group(1)
            pendulum_xml = base64.b64decode(pendulum_xml_b64).decode('utf-8')
            print(f"✅ Extracted XML content: {len(pendulum_xml)} characters")
        else:
            print("❌ Could not extract XML content")
            
    except Exception as e:
        print(f"❌ Download XML test failed: {e}")
    
    # Test 2: Download Python script
    print("\n2. Testing Python script download...")
    try:
        result = await handle_call_tool("download_python_script", {
            "model_id": "pendulum",
            "include_viewer_setup": True
        })
        result_text = result[0].text
        print(f"✅ Download Python script result: {result_text[:200]}...")
        
        # Extract script content
        script_match = re.search(r'"script_content_base64":\s*"([^"]+)"', result_text)
        if script_match:
            script_b64 = script_match.group(1)
            script_content = base64.b64decode(script_b64).decode('utf-8')
            print(f"✅ Generated script: {len(script_content)} characters")
            print(f"Script preview:\n{script_content[:300]}...")
        else:
            print("❌ Could not extract script content")
            
    except Exception as e:
        print(f"❌ Download Python script test failed: {e}")
    
    # Test 3: Upload valid XML
    print("\n3. Testing XML upload with valid content...")
    test_xml = '''<mujoco model="test_upload">
      <worldbody>
        <body name="test_body" pos="0 0 1">
          <geom name="test_sphere" type="sphere" size="0.2" rgba="1 0 0 1"/>
        </body>
      </worldbody>
    </mujoco>'''
    
    try:
        result = await handle_call_tool("upload_xml", {
            "xml_content": test_xml,
            "scene_name": "test_upload_scene",
            "auto_render": False  # Don't render to avoid viewer dependency
        })
        result_text = result[0].text
        print(f"✅ Upload XML result: {result_text[:300]}...")
        
    except Exception as e:
        print(f"❌ Upload XML test failed: {e}")
    
    # Test 4: Upload invalid XML
    print("\n4. Testing XML upload with invalid content...")
    invalid_xml = '''<invalid>
      <not_mujoco>This is not valid MuJoCo XML</not_mujoco>
    </invalid>'''
    
    try:
        result = await handle_call_tool("upload_xml", {
            "xml_content": invalid_xml,
            "scene_name": "invalid_scene"
        })
        result_text = result[0].text
        print(f"✅ Invalid XML handling: {result_text[:200]}...")
        
    except Exception as e:
        print(f"❌ Invalid XML test failed: {e}")
    
    # Test 5: Upload Python script
    print("\n5. Testing Python script upload...")
    test_script = '''#!/usr/bin/env python3
"""Test uploaded script"""

import mujoco
import numpy as np

def main():
    print("Hello from uploaded script!")
    # Simple MuJoCo test
    xml = """
    <mujoco>
      <worldbody>
        <geom type="plane" size="1 1 0.1"/>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    print(f"Model loaded with {model.nbody} bodies")

if __name__ == "__main__":
    main()
'''
    
    try:
        result = await handle_call_tool("upload_python_script", {
            "script_content": test_script,
            "script_name": "test_uploaded_script.py",
            "safe_mode": True
        })
        result_text = result[0].text
        print(f"✅ Upload Python script result: {result_text[:300]}...")
        
    except Exception as e:
        print(f"❌ Upload Python script test failed: {e}")
    
    # Test 6: Upload unsafe Python script
    print("\n6. Testing unsafe Python script detection...")
    unsafe_script = '''
import subprocess
import os

os.system("rm -rf /")  # Dangerous command
subprocess.call(["evil", "command"])
'''
    
    try:
        result = await handle_call_tool("upload_python_script", {
            "script_content": unsafe_script,
            "safe_mode": True
        })
        result_text = result[0].text
        print(f"✅ Unsafe script handling: {result_text[:200]}...")
        
    except Exception as e:
        print(f"❌ Unsafe script test failed: {e}")
    
    # Test 7: Test file handler directly
    print("\n7. Testing FileHandler directly...")
    try:
        fh = FileHandler()
        
        # Test file saving
        file_id = fh.save_upload_file(test_xml, "xml", "direct_test.xml")
        print(f"✅ Saved file with ID: {file_id}")
        
        # Test file listing
        files_info = fh.list_uploaded_files()
        print(f"✅ Files info: {json.dumps(files_info, indent=2)}")
        
        # Test file retrieval
        file_content = fh.get_file_content(file_id)
        if file_content:
            print(f"✅ Retrieved file: {file_content['name']} ({file_content['size']} bytes)")
        else:
            print("❌ Could not retrieve file")
            
    except Exception as e:
        print(f"❌ FileHandler direct test failed: {e}")
    
    print("\n" + "=" * 50)
    print("🎉 Download/Upload feature tests completed!")

if __name__ == "__main__":
    asyncio.run(test_download_upload_features())