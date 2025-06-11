#!/usr/bin/env python3
"""
Basic functional tests for MuJoCo MCP package
"""

import unittest
import numpy as np
from mujoco_mcp import MuJoCoSimulation

# Simple test model XML
TEST_MODEL_XML = """
<mujoco model="test">
  <option timestep="0.01" />
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body name="box" pos="0 0 0.5">
      <joint name="free" type="free"/>
      <geom name="box" type="box" size="0.1 0.1 0.1" rgba="0 0.7 0.7 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="x" joint="free" gear="1 0 0 0 0 0"/>
    <motor name="y" joint="free" gear="0 1 0 0 0 0"/>
    <motor name="z" joint="free" gear="0 0 1 0 0 0"/>
  </actuator>
  <sensor>
    <framepos name="box_pos" objtype="body" objname="box"/>
  </sensor>
</mujoco>
"""


class TestMuJoCoSimulation(unittest.TestCase):
    """Test MuJoCo simulation basic functionality"""
    
    def setUp(self):
        """Set up before each test"""
        self.sim = MuJoCoSimulation(model_xml=TEST_MODEL_XML)
    
    def test_initialization(self):
        """Test simulation initialization"""
        self.assertIsNotNone(self.sim.model)
        self.assertIsNotNone(self.sim.data)
        self.assertIsNotNone(self.sim.sim_id)
        self.assertTrue(self.sim.is_initialized())
    
    def test_step(self):
        """Test simulation stepping"""
        initial_time = self.sim.get_time()
        self.sim.step(1)
        new_time = self.sim.get_time()
        self.assertGreater(new_time, initial_time)
    
    def test_reset(self):
        """Test simulation reset"""
        # Run some steps
        self.sim.step(10)
        current_time = self.sim.get_time()
        self.assertGreater(current_time, 0)
        
        # Reset
        self.sim.reset()
        reset_time = self.sim.get_time()
        self.assertEqual(reset_time, 0.0)
    
    def test_joint_positions(self):
        """Test joint position get/set"""
        # Get initial positions
        initial_pos = self.sim.get_joint_positions()
        self.assertEqual(len(initial_pos), 7)  # 7 DOF for free joint
        
        # Set new positions
        new_pos = [0.5, -0.5, 1.0, 1.0, 0.0, 0.0, 0.0]
        self.sim.set_joint_positions(new_pos)
        
        # Verify positions were set
        read_pos = self.sim.get_joint_positions()
        np.testing.assert_array_almost_equal(new_pos, read_pos, decimal=5)
    
    def test_joint_velocities(self):
        """Test joint velocity get/set"""
        # Get initial velocities
        initial_vel = self.sim.get_joint_velocities()
        self.assertEqual(len(initial_vel), 6)  # 6 DOF velocity for free joint
        
        # Set new velocities
        new_vel = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]
        self.sim.set_joint_velocities(new_vel)
        
        # Verify velocities were set
        read_vel = self.sim.get_joint_velocities()
        np.testing.assert_array_almost_equal(new_vel, read_vel, decimal=5)
    
    def test_control(self):
        """Test control application"""
        # Apply control
        control = [1.0, 0.0, 0.0]  # Force in x direction
        self.sim.apply_control(control)
        
        # Get initial position
        initial_pos = self.sim.get_joint_positions()
        
        # Step simulation
        self.sim.step(10)
        
        # Check that position changed (object should move due to force)
        new_pos = self.sim.get_joint_positions()
        self.assertGreater(new_pos[0], initial_pos[0])  # x position should increase
    
    def test_sensor_data(self):
        """Test sensor data retrieval"""
        sensor_data = self.sim.get_sensor_data()
        self.assertIn("box_pos", sensor_data)
        self.assertIsInstance(sensor_data["box_pos"], list)
    
    def test_rigid_body_states(self):
        """Test rigid body state retrieval"""
        body_states = self.sim.get_rigid_body_states()
        self.assertIn("box", body_states)
        self.assertIn("position", body_states["box"])
        self.assertIn("orientation", body_states["box"])
    
    def test_model_info(self):
        """Test model information retrieval"""
        self.assertGreater(self.sim.get_num_joints(), 0)
        self.assertGreater(self.sim.get_num_actuators(), 0)
        self.assertGreater(self.sim.get_timestep(), 0)
        
        joint_names = self.sim.get_joint_names()
        self.assertIsInstance(joint_names, list)
        self.assertGreater(len(joint_names), 0)


if __name__ == "__main__":
    unittest.main()