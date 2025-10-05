#!/usr/bin/env python3
"""
Test EGL Headless Rendering and H.264 Encoding Features
"""

import sys
import os
import time
import json
from pathlib import Path
import numpy as np

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent / "src"))

try:
    from mujoco_mcp.egl_renderer import EGLRenderer, check_egl_support, create_egl_renderer
    from mujoco_mcp.h264_encoder import H264Encoder, StreamingEncoder, check_h264_support
    from mujoco_mcp.mcp_server_headless import HeadlessSimulation, get_scene_xml
    import mujoco
except ImportError as e:
    print(f"❌ Import Error: {e}")
    sys.exit(1)


def test_egl_support():
    """Test EGL support detection"""
    print("\n🔍 Testing EGL Support Detection")
    print("=" * 50)

    support_info = check_egl_support()
    print(f"EGL Available: {support_info.get('egl_available', False)}")
    print(f"GPU Available: {support_info.get('gpu_available', False)}")

    if support_info.get("error"):
        print(f"Error: {support_info['error']}")
    else:
        print(f"GPU Vendor: {support_info.get('vendor', 'Unknown')}")
        print(f"GPU Renderer: {support_info.get('renderer', 'Unknown')}")
        print(f"OpenGL Version: {support_info.get('version', 'Unknown')}")

    return support_info.get("egl_available", False)


def test_h264_support():
    """Test H.264 encoder support detection"""
    print("\n🎥 Testing H.264 Support Detection")
    print("=" * 50)

    support_info = check_h264_support()
    print(f"FFmpeg Available: {support_info.get('ffmpeg_available', False)}")
    print(f"PIL Available: {support_info.get('pil_available', False)}")
    print(f"Software H.264: {support_info.get('software_encoder', False)}")

    hardware_encoders = support_info.get("hardware_encoders", [])
    print(f"Hardware Encoders: {len(hardware_encoders)}")
    for encoder in hardware_encoders:
        print(f"  - {encoder['name']} ({encoder['codec']})")

    return support_info.get("ffmpeg_available", False)


def test_egl_rendering():
    """Test EGL rendering functionality"""
    print("\n🚀 Testing EGL Rendering")
    print("=" * 50)

    try:
        # Create a simple pendulum model
        xml_string = get_scene_xml("pendulum")
        model = mujoco.MjModel.from_xml_string(xml_string)
        data = mujoco.MjData(model)

        # Test EGL renderer creation
        egl_renderer = create_egl_renderer(model, 320, 240)
        if not egl_renderer:
            print("❌ EGL renderer creation failed")
            return False

        # Test rendering
        with egl_renderer:
            frame = egl_renderer.render(data)
            print(f"✅ EGL rendering successful: {frame.shape}")

            # Step simulation and render again
            mujoco.mj_step(model, data)
            frame2 = egl_renderer.render(data)
            print(f"✅ EGL animation frame: {frame2.shape}")

            # Check if frames are different (animation working)
            if not np.array_equal(frame, frame2):
                print("✅ Animation detected - frames are different")
            else:
                print("⚠️ Frames are identical - may not be animating")

        return True

    except Exception as e:
        print(f"❌ EGL rendering test failed: {e}")
        return False


def test_h264_encoding():
    """Test H.264 encoding functionality"""
    print("\n🎬 Testing H.264 Encoding")
    print("=" * 50)

    try:
        # Create test frames
        width, height = 320, 240
        fps = 10  # Lower FPS for faster testing

        encoder = H264Encoder(width, height, fps, bitrate="500K", preset="ultrafast")
        print(f"Encoder info: {encoder.get_info()}")

        # Generate test frames
        num_frames = 30  # 3 seconds at 10 fps
        for i in range(num_frames):
            # Create animated test frame
            frame = np.zeros((height, width, 3), dtype=np.uint8)

            # Moving colored rectangle
            x = int((i / num_frames) * (width - 50))
            y = height // 2 - 25
            frame[y : y + 50, x : x + 50] = [255, 100, 100]  # Red rectangle

            encoder.add_frame(frame)

        print(f"Added {num_frames} frames to encoder")

        # Encode to bytes
        video_data = encoder.encode_to_bytes()
        print(f"✅ H.264 encoding successful: {len(video_data)} bytes")

        # Test streaming encoder
        streaming_encoder = StreamingEncoder(width, height, fps, bitrate="400K")
        chunk_count = 0

        for i in range(num_frames):
            frame = np.ones((height, width, 3), dtype=np.uint8) * (i * 8 % 256)
            chunk = streaming_encoder.add_frame(frame)
            if chunk:
                chunk_count += 1
                print(f"Got streaming chunk {chunk_count}: {len(chunk)} bytes")

        # Flush remaining frames
        final_chunk = streaming_encoder.flush()
        if final_chunk:
            chunk_count += 1
            print(f"Final chunk: {len(final_chunk)} bytes")

        print(f"✅ Streaming encoding successful: {chunk_count} chunks")
        return True

    except Exception as e:
        print(f"❌ H.264 encoding test failed: {e}")
        return False


def test_headless_simulation_with_features():
    """Test headless simulation with EGL and H.264 features"""
    print("\n🎯 Testing Headless Simulation with New Features")
    print("=" * 50)

    try:
        # Create headless simulation
        xml_string = get_scene_xml("pendulum")
        sim = HeadlessSimulation("test_pendulum", xml_string)

        # Test EGL rendering
        egl_success = sim.enable_egl_rendering(320, 240)
        print(f"EGL rendering enabled: {egl_success}")

        # Test frame rendering
        frame = sim.render_frame(320, 240)
        print(f"✅ Frame rendered: {frame.shape}")

        # Test H.264 encoder setup
        h264_success = sim.setup_h264_encoder(320, 240, fps=10)
        print(f"H.264 encoder setup: {h264_success}")

        # Test streaming encoder setup
        streaming_success = sim.setup_streaming_encoder(320, 240, fps=10)
        print(f"Streaming encoder setup: {streaming_success}")

        # Record a short video
        frames_recorded = 0
        for i in range(20):  # 2 seconds at 10 fps
            sim.step(1)
            frame = sim.render_frame(320, 240)
            chunk = sim.add_frame_to_encoder(frame)
            frames_recorded += 1

            if chunk:
                print(f"Got streaming chunk: {len(chunk)} bytes")

        print(f"✅ Recorded {frames_recorded} frames")

        # Test video encoding
        video_data = sim.encode_video()
        if video_data:
            print(f"✅ Video encoded: {len(video_data)} bytes")

        # Cleanup
        sim.close()
        print("✅ Simulation closed successfully")

        return True

    except Exception as e:
        print(f"❌ Headless simulation test failed: {e}")
        return False


def benchmark_performance():
    """Benchmark EGL vs software rendering performance"""
    print("\n⚡ Performance Benchmark")
    print("=" * 50)

    try:
        xml_string = get_scene_xml("pendulum")
        model = mujoco.MjModel.from_xml_string(xml_string)
        data = mujoco.MjData(model)

        width, height = 640, 480
        num_frames = 50

        # Benchmark software rendering
        print("Testing software rendering...")
        start_time = time.time()

        try:
            software_renderer = mujoco.Renderer(model, height, width)
            for i in range(num_frames):
                mujoco.mj_step(model, data)
                software_renderer.update_scene(data)
                frame = software_renderer.render()

            software_time = time.time() - start_time
            software_fps = num_frames / software_time
            print(f"Software rendering: {software_fps:.1f} FPS ({software_time:.2f}s)")

        except Exception as e:
            print(f"Software rendering failed: {e}")
            software_fps = 0

        # Reset simulation
        mujoco.mj_resetData(model, data)

        # Benchmark EGL rendering
        print("Testing EGL rendering...")
        egl_renderer = create_egl_renderer(model, width, height)

        if egl_renderer:
            start_time = time.time()

            with egl_renderer:
                for i in range(num_frames):
                    mujoco.mj_step(model, data)
                    frame = egl_renderer.render(data)

            egl_time = time.time() - start_time
            egl_fps = num_frames / egl_time
            print(f"EGL rendering: {egl_fps:.1f} FPS ({egl_time:.2f}s)")

            if software_fps > 0:
                speedup = egl_fps / software_fps
                print(f"EGL speedup: {speedup:.1f}x")
        else:
            print("EGL rendering not available")

        return True

    except Exception as e:
        print(f"❌ Performance benchmark failed: {e}")
        return False


def main():
    """Run all tests"""
    print("🧪 MuJoCo MCP EGL & H.264 Feature Tests")
    print("=" * 60)

    results = {}

    # Test EGL support
    results["egl_support"] = test_egl_support()

    # Test H.264 support
    results["h264_support"] = test_h264_support()

    # Test EGL rendering if available
    if results["egl_support"]:
        results["egl_rendering"] = test_egl_rendering()
    else:
        print("\n⚠️ Skipping EGL rendering tests (not available)")
        results["egl_rendering"] = False

    # Test H.264 encoding if available
    if results["h264_support"]:
        results["h264_encoding"] = test_h264_encoding()
    else:
        print("\n⚠️ Skipping H.264 encoding tests (not available)")
        results["h264_encoding"] = False

    # Test integrated features
    results["headless_features"] = test_headless_simulation_with_features()

    # Run performance benchmark
    results["performance_benchmark"] = benchmark_performance()

    # Summary
    print("\n📊 Test Results Summary")
    print("=" * 60)

    passed = sum(results.values())
    total = len(results)

    for test_name, passed_test in results.items():
        status = "✅ PASS" if passed_test else "❌ FAIL"
        print(f"{test_name:20} : {status}")

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All tests passed!")
        return 0
    else:
        print("⚠️ Some tests failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
