#!/usr/bin/env python3
"""
End-to-end demo of download/upload workflow
Demonstrates the complete cycle: download XML -> modify -> upload -> auto-render
"""

import sys
import asyncio
import json
import base64
import re

sys.path.append("src")
from mujoco_mcp.mcp_server_menagerie import handle_call_tool


async def demo_workflow():
    """Demonstrate the complete download/upload workflow"""

    print("🔄 Download/Upload Workflow Demo")
    print("=" * 50)

    # Step 1: Download XML from built-in scene
    print("\n1️⃣ Downloading XML for pendulum scene...")
    try:
        result = await handle_call_tool("download_xml", {"model_id": "pendulum"})
        result_text = result[0].text

        # Extract XML content
        xml_match = re.search(r'"xml_content_base64":\s*"([^"]+)"', result_text)
        if xml_match:
            original_xml = base64.b64decode(xml_match.group(1)).decode("utf-8")
            print(f"✅ Downloaded XML ({len(original_xml)} characters)")
            print(f"Original XML preview:\n{original_xml[:200]}...")
        else:
            print("❌ Failed to extract XML")
            return
    except Exception as e:
        print(f"❌ Download failed: {e}")
        return

    # Step 2: Modify the XML (change sphere color and size)
    print("\n2️⃣ Modifying the XML...")
    modified_xml = original_xml.replace(
        'rgba="0.2 0.8 0.2 1"', 'rgba="0.8 0.2 0.8 1"'  # original green  # change to purple
    ).replace(
        'size="0.05"', 'size="0.08"'  # original size  # make it bigger
    )

    print(f"✅ Modified XML ({len(modified_xml)} characters)")
    print("Changes made: sphere color green→purple, size 0.05→0.08")

    # Step 3: Upload modified XML with auto-render disabled (no viewer dependency)
    print("\n3️⃣ Uploading modified XML...")
    try:
        result = await handle_call_tool(
            "upload_xml",
            {
                "xml_content": modified_xml,
                "scene_name": "modified_pendulum",
                "auto_render": False,  # Disable for demo
            },
        )
        result_text = result[0].text
        print(f"✅ Upload result: {result_text[:300]}...")

        # Extract file ID for later reference
        file_id_match = re.search(r'"file_id":\s*"([^"]+)"', result_text)
        if file_id_match:
            file_id = file_id_match.group(1)
            print(f"✅ File uploaded with ID: {file_id}")

    except Exception as e:
        print(f"❌ Upload failed: {e}")
        return

    # Step 4: Download Python script for the original scene
    print("\n4️⃣ Generating Python script for original scene...")
    try:
        result = await handle_call_tool(
            "download_python_script", {"model_id": "pendulum", "include_viewer_setup": True}
        )
        result_text = result[0].text

        # Extract script content
        script_match = re.search(r'"script_content_base64":\s*"([^"]+)"', result_text)
        if script_match:
            script_content = base64.b64decode(script_match.group(1)).decode("utf-8")
            print(f"✅ Generated Python script ({len(script_content)} characters)")

            # Show key parts of the script
            lines = script_content.split("\n")
            print("Script preview (first 10 lines):")
            for i, line in enumerate(lines[:10]):
                print(f"  {i+1:2d}: {line}")
            print("  ... (truncated)")

        else:
            print("❌ Failed to extract script")

    except Exception as e:
        print(f"❌ Script generation failed: {e}")

    # Step 5: Create a custom scene and upload it
    print("\n5️⃣ Creating and uploading a custom scene...")
    custom_xml = """<mujoco model="custom_scene">
      <compiler angle="radian"/>
      <option timestep="0.002"/>
      
      <worldbody>
        <geom name="floor" type="plane" size="2 2 0.1" rgba="0.7 0.7 0.7 1"/>
        
        <!-- Three spheres in a triangle -->
        <body name="sphere1" pos="0.5 0 0.5">
          <geom name="ball1" type="sphere" size="0.1" rgba="1 0 0 1"/>
        </body>
        
        <body name="sphere2" pos="-0.5 0 0.5">
          <geom name="ball2" type="sphere" size="0.1" rgba="0 1 0 1"/>
        </body>
        
        <body name="sphere3" pos="0 0.8 0.5">
          <geom name="ball3" type="sphere" size="0.1" rgba="0 0 1 1"/>
        </body>
        
        <!-- A box in the center -->
        <body name="center_box" pos="0 0 0.2">
          <geom name="box" type="box" size="0.2 0.2 0.2" rgba="1 1 0 1"/>
        </body>
      </worldbody>
    </mujoco>"""

    try:
        result = await handle_call_tool(
            "upload_xml",
            {
                "xml_content": custom_xml,
                "scene_name": "custom_spheres_and_box",
                "auto_render": False,  # Disable for demo
            },
        )
        result_text = result[0].text
        print(f"✅ Custom scene upload: {result_text[:200]}...")

    except Exception as e:
        print(f"❌ Custom scene upload failed: {e}")

    # Step 6: Test error handling with invalid XML
    print("\n6️⃣ Testing error handling with malformed XML...")
    invalid_xml = """<not_mujoco>
      <invalid_structure>
        This is not a valid MuJoCo XML file
      </invalid_structure>
    </not_mujoco>"""

    try:
        result = await handle_call_tool(
            "upload_xml", {"xml_content": invalid_xml, "scene_name": "invalid_test"}
        )
        result_text = result[0].text
        print(f"✅ Error handling: {result_text[:150]}...")

    except Exception as e:
        print(f"✅ Expected error caught: {e}")

    # Summary
    print("\n" + "=" * 50)
    print("🎉 Workflow Demo Complete!")
    print("✅ Successfully demonstrated:")
    print("  • XML download from built-in scenes")
    print("  • XML modification")
    print("  • XML upload with validation")
    print("  • Python script generation")
    print("  • Custom scene creation")
    print("  • Error handling for invalid XML")
    print("\n💡 Key Features:")
    print("  • Base64 encoding for safe content transport")
    print("  • Automatic MuJoCo validation")
    print("  • File tracking with unique IDs")
    print("  • Safety checks for Python scripts")
    print("  • Auto-rendering capability (when viewer is available)")


if __name__ == "__main__":
    asyncio.run(demo_workflow())
