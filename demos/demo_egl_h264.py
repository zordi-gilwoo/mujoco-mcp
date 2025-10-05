#!/usr/bin/env python3
"""
Demo: EGL Headless Rendering and H.264 Encoding
Showcases the new GPU rendering and video encoding capabilities
"""

import asyncio
import json
import base64
import time
from pathlib import Path

# Add src to path for imports
import sys

sys.path.insert(0, str(Path(__file__).parent / "src"))

from mujoco_mcp.mcp_server_headless import handle_call_tool, handle_list_tools


async def demo_gpu_support_check():
    """Demonstrate GPU support detection"""
    print("\n🔍 GPU Support Detection")
    print("=" * 50)

    result = await handle_call_tool("check_gpu_support", {})
    support_info = json.loads(result[0].text)

    print("EGL Rendering Support:")
    egl_info = support_info["egl_rendering"]
    print(f"  Available: {egl_info.get('egl_available', False)}")
    print(f"  GPU Ready: {egl_info.get('gpu_available', False)}")
    if egl_info.get("error"):
        print(f"  Issue: {egl_info['error']}")

    print("\nH.264 Encoding Support:")
    h264_info = support_info["h264_encoding"]
    print(f"  FFmpeg: {h264_info.get('ffmpeg_available', False)}")
    print(f"  PIL: {h264_info.get('pil_available', False)}")
    print(f"  Software H.264: {h264_info.get('software_encoder', False)}")

    hardware_encoders = h264_info.get("hardware_encoders", [])
    print(f"  Hardware Encoders: {len(hardware_encoders)}")
    for encoder in hardware_encoders:
        print(f"    - {encoder['name']} ({encoder['codec']})")

    return support_info


async def demo_egl_rendering():
    """Demonstrate EGL rendering capabilities"""
    print("\n🚀 EGL Rendering Demo")
    print("=" * 50)

    # Create simulation
    print("Creating pendulum simulation...")
    result = await handle_call_tool("create_scene", {"scene_type": "pendulum"})
    print("✅ Simulation created")

    # Enable EGL rendering
    print("Enabling EGL rendering...")
    result = await handle_call_tool(
        "enable_egl_rendering", {"model_id": "pendulum", "width": 640, "height": 480}
    )
    print(f"EGL Status: {result[0].text}")

    # Render a frame
    print("Rendering frame...")
    result = await handle_call_tool(
        "render_egl_frame", {"model_id": "pendulum", "width": 640, "height": 480}
    )

    if len(result) > 0 and hasattr(result[0], "data"):
        print("✅ Frame rendered successfully (image data received)")
        print(f"   Image size: ~{len(result[0].data) * 3/4:.0f} bytes (base64)")
    else:
        print(f"Frame rendering result: {result[0].text}")


async def demo_h264_encoding():
    """Demonstrate H.264 video encoding"""
    print("\n🎬 H.264 Encoding Demo")
    print("=" * 50)

    # Ensure we have a simulation
    try:
        await handle_call_tool("get_state", {"model_id": "pendulum"})
    except:
        # Create if doesn't exist
        await handle_call_tool("create_scene", {"scene_type": "pendulum"})

    # Setup H.264 encoder
    print("Setting up H.264 encoder...")
    result = await handle_call_tool(
        "setup_h264_encoder",
        {"model_id": "pendulum", "width": 320, "height": 240, "fps": 10, "bitrate": "500K"},
    )
    print(f"Encoder Status: {result[0].text}")

    # Record short video
    print("Recording 3-second video...")
    start_time = time.time()

    result = await handle_call_tool(
        "record_h264_video", {"model_id": "pendulum", "duration": 3.0, "return_bytes": True}
    )

    encode_time = time.time() - start_time

    if len(result) > 0 and hasattr(result[0], "resource"):
        resource = result[0].resource
        if hasattr(resource, "uri"):
            video_uri = resource.uri
            video_size = len(video_uri) * 3 // 4  # Approximate size from base64
            print(f"✅ Video encoded successfully!")
            print(f"   Duration: 3.0 seconds")
            print(f"   File size: ~{video_size} bytes")
            print(f"   Encoding time: {encode_time:.2f} seconds")
            print(f"   Compression ratio: ~{(320*240*3*30)//video_size:.1f}:1")
        else:
            print(f"Video resource received but no URI: {resource}")
    else:
        print(f"Video encoding result: {result[0].text}")


async def demo_streaming_encoder():
    """Demonstrate streaming encoder capabilities"""
    print("\n📡 Streaming Encoder Demo")
    print("=" * 50)

    # Setup streaming encoder
    print("Setting up streaming encoder...")
    result = await handle_call_tool(
        "setup_h264_encoder",
        {
            "model_id": "pendulum",
            "width": 320,
            "height": 240,
            "fps": 15,
            "bitrate": "400K",
            "streaming": True,
        },
    )
    print(f"Streaming Encoder: {result[0].text}")

    print("Streaming would work in real-time applications...")
    print("(Chunks are generated as frames are added)")


async def demo_performance_comparison():
    """Demonstrate performance differences"""
    print("\n⚡ Performance Comparison")
    print("=" * 50)

    # Test rendering performance
    print("Testing rendering performance...")

    start_time = time.time()
    for i in range(10):
        result = await handle_call_tool(
            "render_egl_frame", {"model_id": "pendulum", "width": 320, "height": 240}
        )
        # Step simulation for animation
        await handle_call_tool("step_simulation", {"model_id": "pendulum", "steps": 5})

    total_time = time.time() - start_time
    fps = 10 / total_time

    print(f"Rendered 10 frames in {total_time:.2f} seconds")
    print(f"Effective FPS: {fps:.1f}")

    if fps > 20:
        print("✅ Performance: Excellent (suitable for real-time)")
    elif fps > 10:
        print("✅ Performance: Good (suitable for most applications)")
    else:
        print("⚠️ Performance: Limited (may need optimization)")


async def cleanup():
    """Clean up simulation"""
    print("\n🧹 Cleanup")
    print("=" * 50)

    result = await handle_call_tool("close_simulation", {"model_id": "pendulum"})
    print(f"Cleanup: {result[0].text}")


async def main():
    """Run the complete demo"""
    print("🎭 MuJoCo MCP: EGL & H.264 Feature Demo")
    print("=" * 60)

    try:
        # Check support
        support_info = await demo_gpu_support_check()

        # Demo EGL rendering
        await demo_egl_rendering()

        # Demo H.264 encoding if available
        if support_info["h264_encoding"].get("ffmpeg_available", False):
            await demo_h264_encoding()
            await demo_streaming_encoder()
        else:
            print("\n⚠️ Skipping H.264 demos (FFmpeg not available)")

        # Performance test
        await demo_performance_comparison()

        print("\n🎉 Demo completed successfully!")

    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Always cleanup
        await cleanup()


if __name__ == "__main__":
    asyncio.run(main())
