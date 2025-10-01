#!/usr/bin/env python3
"""
Basic example: Demonstrates how to start and interact with MuJoCo MCP server
"""

import sys
import time
import threading
import logging
from model_context_protocol import MCPClient

# Import mujoco_mcp module
try:
    import mujoco_mcp
except ImportError:
    print("mujoco_mcp module not found. Please ensure the module is installed.")
    print("Try running: pip install -e .")
    sys.exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mcp_example")

# MuJoCo model XML - simple pendulum
PENDULUM_XML = """
<mujoco model="pendulum">
  <option timestep="0.01" />
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
      <geom name="pole" type="capsule" size="0.045 0.5" rgba="0 0.7 0.7 1" fromto="0 0 0 0 0 -1"/>
      <site name="tip" pos="0 0 -1" size="0.01"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="1" ctrllimited="true" ctrlrange="-2.0 2.0"/>
  </actuator>
  <sensor>
    <jointpos name="joint_pos" joint="hinge"/>
    <jointvel name="joint_vel" joint="hinge"/>
  </sensor>
</mujoco>
"""


def start_server():
    server_thread = threading.Thread(
        target=mujoco_mcp.start,
        kwargs={"host": "localhost", "port": 8000, "blocking": True},
        daemon=True,
    )
    server_thread.start()
    return server_thread


def run_client():

    server_thread = start_server()

    try:
        run_client()
    except KeyboardInterrupt:


if __name__ == "__main__":
    main()
