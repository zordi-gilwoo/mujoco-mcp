#!/usr/bin/env python3
"""
Test script for RL Environment Viewer functionality
"""

import asyncio
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from mujoco_mcp.rl_runner import rl_runner, create_and_run_rl_environment


async def test_rl_viewer_functionality():
    """Test the complete RL viewer functionality"""
    print("🧪 Testing RL Environment Viewer Functionality")
    print("=" * 60)

    # Test environment configuration
    config = {
        "robot_type": "simple_arm",
        "task_type": "reaching",
        "max_episode_steps": 100,
        "action_space_type": "continuous",
        "reward_scale": 1.0,
    }

    xml_content = """<mujoco model="simple_arm">
    <option timestep="0.002"/>
    <worldbody>
        <body name="base" pos="0 0 0">
            <geom name="base_geom" type="cylinder" size="0.1 0.1" rgba="0.5 0.5 0.5 1"/>
            <body name="link1" pos="0 0 0.1">
                <joint name="joint1" type="hinge" axis="0 0 1"/>
                <geom name="link1_geom" type="capsule" size="0.05 0.2" rgba="0.8 0.2 0.2 1"/>
                <body name="link2" pos="0 0 0.2">
                    <joint name="joint2" type="hinge" axis="0 1 0"/>
                    <geom name="link2_geom" type="capsule" size="0.04 0.15" rgba="0.2 0.8 0.2 1"/>
                    <body name="end_effector" pos="0 0 0.15">
                        <geom name="ee_geom" type="sphere" size="0.03" rgba="1 0 0 1"/>
                    </body>
                </body>
            </body>
        </body>
        <body name="target" pos="0.3 0.0 0.3">
            <geom name="target_geom" type="sphere" size="0.05" rgba="0 1 0 0.7"/>
        </body>
    </worldbody>
</mujoco>"""

    print("1. Testing RL environment creation...")
    success = await rl_runner.create_environment(config)
    if success:
        print("✅ RL environment created successfully")
    else:
        print("❌ Failed to create RL environment")
        return False

    print("\n2. Testing environment status...")
    status = rl_runner.get_status()
    print(f"   Environment type: {status['environment_type']}")
    print(f"   Is running: {status['is_running']}")

    print("\n3. Testing random action simulation (brief)...")
    if await rl_runner.start_random_actions(num_steps=50, step_delay=0.001):
        print("✅ Random actions started successfully")

        # Let it run briefly
        await asyncio.sleep(1)

        # Check status during run
        status = rl_runner.get_status()
        print(f"   Steps completed: {status['total_steps']}")
        print(f"   Episodes: {status['current_episode']}")
        print(f"   Total reward: {status['total_reward']:.3f}")

        # Stop the environment
        await rl_runner.stop_environment()
        print("✅ RL environment stopped successfully")
    else:
        print("❌ Failed to start random actions")
        return False

    print("\n4. Testing cleanup...")
    rl_runner.cleanup()
    print("✅ Cleanup completed")

    print("\n🎉 All RL viewer tests passed!")
    return True


if __name__ == "__main__":
    try:
        result = asyncio.run(test_rl_viewer_functionality())
        sys.exit(0 if result else 1)
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        sys.exit(1)
