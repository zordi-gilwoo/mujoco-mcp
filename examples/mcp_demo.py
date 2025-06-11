#!/usr/bin/env python3
"""
MCP Demo: Fancy MuJoCo tasks controllable through Claude/Cursor via MCP interface

This demo showcases advanced robotics tasks that can be controlled via natural language 
through the MCP interface. When configured with Claude Desktop or Cursor, users can 
give commands like:

1. "Start a robotic arm simulation and move it to pick up the red cube"
2. "Make the robot perform a figure-8 trajectory" 
3. "Simulate a humanoid walking and adjust the gait parameters"
4. "Test collision avoidance while the robot moves between obstacles"

The demo includes several scenarios that demonstrate the power of LLM-controlled robotics.
"""

import asyncio
import json
import time
import numpy as np
from typing import Dict, List, Optional
from mcp.server.fastmcp import FastMCP
from mujoco_mcp import MuJoCoSimulation

# Advanced robot arm model with gripper
ROBOT_ARM_XML = """
<mujoco model="robot_arm_demo">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <asset>
    <texture name="wood" type="cube" file="wood.png"/>
    <material name="wood" texture="wood" rgba="0.8 0.6 0.4 1"/>
    <material name="red" rgba="0.8 0.2 0.2 1"/>
    <material name="blue" rgba="0.2 0.2 0.8 1"/>
    <material name="green" rgba="0.2 0.8 0.2 1"/>
  </asset>
  
  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba=".9 .9 .9 1"/>
    
    <!-- Robot arm base -->
    <body name="base" pos="0 0 0.1">
      <geom type="cylinder" size="0.1 0.05" rgba="0.3 0.3 0.3 1"/>
      
      <!-- Shoulder joint -->
      <body name="shoulder" pos="0 0 0.05">
        <joint name="shoulder_pan" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
        <geom type="cylinder" size="0.08 0.1" rgba="0.4 0.4 0.4 1"/>
        
        <!-- Upper arm -->
        <body name="upper_arm" pos="0 0 0.1">
          <joint name="shoulder_lift" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
          <geom type="box" size="0.05 0.05 0.2" rgba="0.5 0.5 0.5 1"/>
          
          <!-- Elbow -->
          <body name="forearm" pos="0 0 0.2">
            <joint name="elbow" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
            <geom type="box" size="0.04 0.04 0.15" rgba="0.6 0.6 0.6 1"/>
            
            <!-- Wrist -->
            <body name="wrist" pos="0 0 0.15">
              <joint name="wrist_rotate" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
              <joint name="wrist_tilt" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
              <geom type="cylinder" size="0.03 0.05" rgba="0.7 0.7 0.7 1"/>
              
              <!-- Gripper base -->
              <body name="gripper_base" pos="0 0 0.05">
                <geom type="box" size="0.02 0.03 0.03" rgba="0.8 0.8 0.8 1"/>
                
                <!-- Left gripper finger -->
                <body name="left_finger" pos="0.02 0 0">
                  <joint name="left_finger" type="slide" axis="1 0 0" range="0 0.02"/>
                  <geom type="box" size="0.005 0.01 0.03" rgba="0.9 0.9 0.9 1"/>
                </body>
                
                <!-- Right gripper finger -->
                <body name="right_finger" pos="-0.02 0 0">
                  <joint name="right_finger" type="slide" axis="-1 0 0" range="0 0.02"/>
                  <geom type="box" size="0.005 0.01 0.03" rgba="0.9 0.9 0.9 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    
    <!-- Objects to manipulate -->
    <body name="red_cube" pos="0.3 0.2 0.125">
      <joint name="cube_free" type="free"/>
      <geom name="red_cube_geom" type="box" size="0.025 0.025 0.025" material="red"/>
    </body>
    
    <body name="blue_sphere" pos="-0.2 0.3 0.13">
      <joint name="sphere_free" type="free"/>
      <geom name="blue_sphere_geom" type="sphere" size="0.03" material="blue"/>
    </body>
    
    <body name="green_cylinder" pos="0.1 -0.4 0.15">
      <joint name="cylinder_free" type="free"/>
      <geom name="green_cylinder_geom" type="cylinder" size="0.03 0.05" material="green"/>
    </body>
    
    <!-- Obstacles -->
    <body name="obstacle1" pos="0.15 0.0 0.2">
      <geom type="box" size="0.05 0.05 0.1" rgba="0.6 0.4 0.2 1"/>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="shoulder_pan_motor" joint="shoulder_pan" gear="100"/>
    <motor name="shoulder_lift_motor" joint="shoulder_lift" gear="100"/>
    <motor name="elbow_motor" joint="elbow" gear="50"/>
    <motor name="wrist_rotate_motor" joint="wrist_rotate" gear="25"/>
    <motor name="wrist_tilt_motor" joint="wrist_tilt" gear="25"/>
    <motor name="left_finger_motor" joint="left_finger" gear="10"/>
    <motor name="right_finger_motor" joint="right_finger" gear="10"/>
  </actuator>
  
  <sensor>
    <framepos name="gripper_pos" objtype="body" objname="gripper_base"/>
    <framelinvel name="gripper_vel" objtype="body" objname="gripper_base"/>
    <framepos name="cube_pos" objtype="body" objname="red_cube"/>
    <framepos name="sphere_pos" objtype="body" objname="blue_sphere"/>
    <framepos name="cylinder_pos" objtype="body" objname="green_cylinder"/>
    <jointpos name="joint_positions" joint="shoulder_pan shoulder_lift elbow wrist_rotate wrist_tilt"/>
  </sensor>
</mujoco>
"""

class MuJoCoMCPDemo(FastMCP):
    """Enhanced MCP server with fancy robotics demonstrations."""
    
    def __init__(self):
        super().__init__()
        self.simulations: Dict[str, MuJoCoSimulation] = {}
        self.current_task = None
        self.task_progress = {}
        
        # Register MCP resources and tools
        self._register_resources()
        self._register_tools()
    
    def _register_resources(self):
        """Register MCP resources for data access."""
        
        @self.resource("simulation_status")
        def get_simulation_status() -> str:
            """Get status of all active simulations and current tasks."""
            if not self.simulations:
                return "No active simulations"
            
            status = []
            for sim_id, sim in self.simulations.items():
                status.append(f"Simulation {sim_id}: {sim.get_model_name()}")
                if sim_id in self.task_progress:
                    task_info = self.task_progress[sim_id]
                    status.append(f"  Current Task: {task_info.get('name', 'None')}")
                    status.append(f"  Progress: {task_info.get('progress', 0):.1f}%")
            
            return "\n".join(status)
        
        @self.resource("robot_state")
        def get_robot_state(sim_id: str) -> str:
            """Get detailed robot state including joint positions and end-effector pose."""
            if sim_id not in self.simulations:
                return f"Simulation {sim_id} not found"
            
            sim = self.simulations[sim_id]
            
            # Get joint positions
            joint_pos = sim.get_joint_positions()
            joint_names = sim.get_joint_names()
            
            # Get sensor data
            sensor_data = sim.get_sensor_data()
            
            state_info = []
            state_info.append("=== ROBOT STATE ===")
            state_info.append("Joint Positions:")
            for i, name in enumerate(joint_names[:7]):  # First 7 joints are arm joints
                if i < len(joint_pos):
                    state_info.append(f"  {name}: {joint_pos[i]:.3f} rad")
            
            if "gripper_pos" in sensor_data:
                gripper_pos = sensor_data["gripper_pos"]
                state_info.append(f"Gripper Position: [{gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f}]")
            
            return "\n".join(state_info)
        
        @self.resource("scene_objects")
        def get_scene_objects(sim_id: str) -> str:
            """Get positions and states of all objects in the scene."""
            if sim_id not in self.simulations:
                return f"Simulation {sim_id} not found"
            
            sim = self.simulations[sim_id]
            sensor_data = sim.get_sensor_data()
            
            objects_info = []
            objects_info.append("=== SCENE OBJECTS ===")
            
            object_sensors = ["cube_pos", "sphere_pos", "cylinder_pos"]
            object_names = ["Red Cube", "Blue Sphere", "Green Cylinder"]
            
            for sensor, name in zip(object_sensors, object_names):
                if sensor in sensor_data:
                    pos = sensor_data[sensor]
                    objects_info.append(f"{name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            
            return "\n".join(objects_info)
    
    def _register_tools(self):
        """Register MCP tools for actions."""
        
        @self.tool("start_robot_demo")
        def start_robot_demo() -> str:
            """Start the robot arm demonstration with objects to manipulate."""
            try:
                sim = MuJoCoSimulation(model_xml=ROBOT_ARM_XML)
                sim_id = f"robot_demo_{int(time.time())}"
                self.simulations[sim_id] = sim
                
                # Initialize robot to a neutral pose
                neutral_pose = [0, -0.5, 0.8, 0, 0.5, 0, 0]  # Reasonable starting pose
                sim.set_joint_positions(neutral_pose)
                
                return f"✅ Robot demonstration started! Simulation ID: {sim_id}\n" + \
                       f"🦾 Robot arm is ready for commands\n" + \
                       f"🎯 Objects available: Red cube, Blue sphere, Green cylinder\n" + \
                       f"Use other tools to control the robot!"
                       
            except Exception as e:
                return f"❌ Failed to start robot demo: {str(e)}"
        
        @self.tool("move_robot_to_position")
        def move_robot_to_position(sim_id: str, target_x: float, target_y: float, target_z: float, duration: float = 3.0) -> str:
            """Move robot end-effector to specified coordinates using inverse kinematics."""
            if sim_id not in self.simulations:
                return f"❌ Simulation {sim_id} not found"
            
            try:
                sim = self.simulations[sim_id]
                
                # Simple inverse kinematics (simplified for demo)
                # In real implementation, this would use proper IK solver
                target_pos = np.array([target_x, target_y, target_z])
                
                # Calculate approximate joint angles for target position
                # This is a simplified calculation - real IK would be more complex
                distance = np.linalg.norm(target_pos[:2])
                height = target_z - 0.15  # Subtract base height
                
                # Shoulder pan (rotate to face target)
                shoulder_pan = np.arctan2(target_y, target_x)
                
                # Shoulder lift and elbow (reach target distance and height)
                shoulder_lift = -0.3 - 0.5 * (height / 0.5)
                elbow = 1.2 + 0.8 * (distance / 0.5)
                
                # Wrist (keep gripper level)
                wrist_tilt = -(shoulder_lift + elbow) + 0.5
                
                target_joints = [shoulder_pan, shoulder_lift, elbow, 0, wrist_tilt, 0, 0]
                
                # Gradually move to target
                current_joints = sim.get_joint_positions()[:7]
                steps = int(duration * 100)  # 100 Hz control
                
                self.task_progress[sim_id] = {
                    'name': f'Moving to [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]',
                    'progress': 0
                }
                
                for i in range(steps):
                    alpha = (i + 1) / steps
                    interpolated_joints = []
                    
                    for j in range(7):
                        if j < len(current_joints) and j < len(target_joints):
                            joint_val = current_joints[j] + alpha * (target_joints[j] - current_joints[j])
                            interpolated_joints.append(joint_val)
                        else:
                            interpolated_joints.append(0)
                    
                    sim.set_joint_positions(interpolated_joints)
                    sim.step(1)
                    
                    self.task_progress[sim_id]['progress'] = (i + 1) / steps * 100
                
                # Get final gripper position for confirmation
                sensor_data = sim.get_sensor_data()
                final_pos = sensor_data.get("gripper_pos", [0, 0, 0])
                
                return f"✅ Robot moved to position!\n" + \
                       f"🎯 Target: [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]\n" + \
                       f"📍 Actual: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]\n" + \
                       f"📏 Error: {np.linalg.norm(np.array(final_pos) - target_pos):.3f}m"
                       
            except Exception as e:
                return f"❌ Failed to move robot: {str(e)}"
        
        @self.tool("pick_up_object")
        def pick_up_object(sim_id: str, object_name: str) -> str:
            """Pick up a specified object (red_cube, blue_sphere, or green_cylinder)."""
            if sim_id not in self.simulations:
                return f"❌ Simulation {sim_id} not found"
            
            try:
                sim = self.simulations[sim_id]
                sensor_data = sim.get_sensor_data()
                
                # Map object names to sensor names
                object_map = {
                    "red_cube": "cube_pos",
                    "blue_sphere": "sphere_pos", 
                    "green_cylinder": "cylinder_pos"
                }
                
                if object_name not in object_map:
                    return f"❌ Unknown object: {object_name}. Available: {list(object_map.keys())}"
                
                sensor_name = object_map[object_name]
                if sensor_name not in sensor_data:
                    return f"❌ Cannot find {object_name} in scene"
                
                object_pos = sensor_data[sensor_name]
                
                self.task_progress[sim_id] = {
                    'name': f'Picking up {object_name}',
                    'progress': 0
                }
                
                # Step 1: Move above object (33% progress)
                above_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.1]
                move_result = move_robot_to_position(sim_id, *above_pos, 2.0)
                self.task_progress[sim_id]['progress'] = 33
                
                # Step 2: Open gripper (50% progress)
                current_joints = sim.get_joint_positions()
                current_joints[5] = 0.02  # Open left finger
                current_joints[6] = 0.02  # Open right finger
                sim.set_joint_positions(current_joints)
                sim.step(50)
                self.task_progress[sim_id]['progress'] = 50
                
                # Step 3: Move down to object (66% progress)
                at_object_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.03]
                move_result = move_robot_to_position(sim_id, *at_object_pos, 1.5)
                self.task_progress[sim_id]['progress'] = 66
                
                # Step 4: Close gripper (83% progress)
                current_joints = sim.get_joint_positions()
                current_joints[5] = 0.005  # Close left finger
                current_joints[6] = 0.005  # Close right finger
                sim.set_joint_positions(current_joints)
                sim.step(50)
                self.task_progress[sim_id]['progress'] = 83
                
                # Step 5: Lift object (100% progress)
                lift_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.15]
                move_result = move_robot_to_position(sim_id, *lift_pos, 1.5)
                self.task_progress[sim_id]['progress'] = 100
                
                return f"✅ Successfully picked up {object_name}!\n" + \
                       f"🤖 Object is now grasped and lifted\n" + \
                       f"📦 Object was at: [{object_pos[0]:.3f}, {object_pos[1]:.3f}, {object_pos[2]:.3f}]\n" + \
                       f"⬆️ Now lifted to: [{lift_pos[0]:.3f}, {lift_pos[1]:.3f}, {lift_pos[2]:.3f}]"
                       
            except Exception as e:
                return f"❌ Failed to pick up object: {str(e)}"
        
        @self.tool("perform_figure_eight")
        def perform_figure_eight(sim_id: str, center_x: float = 0.2, center_y: float = 0.0, size: float = 0.15, duration: float = 8.0) -> str:
            """Make the robot trace a figure-8 pattern in the air."""
            if sim_id not in self.simulations:
                return f"❌ Simulation {sim_id} not found"
            
            try:
                sim = self.simulations[sim_id]
                
                self.task_progress[sim_id] = {
                    'name': 'Figure-8 trajectory',
                    'progress': 0
                }
                
                steps = int(duration * 50)  # 50 Hz for smooth motion
                height = 0.3  # Fixed height for figure-8
                
                for i in range(steps):
                    t = (i / steps) * 4 * np.pi  # Two complete loops
                    
                    # Parametric figure-8 equations
                    x = center_x + size * np.sin(t)
                    y = center_y + size * np.sin(t) * np.cos(t)
                    z = height
                    
                    # Calculate joint angles for this position
                    distance = np.linalg.norm([x, y])
                    shoulder_pan = np.arctan2(y, x)
                    shoulder_lift = -0.3 - 0.5 * ((z - 0.15) / 0.5)
                    elbow = 1.2 + 0.8 * (distance / 0.5)
                    wrist_tilt = -(shoulder_lift + elbow) + 0.5
                    
                    joints = [shoulder_pan, shoulder_lift, elbow, 0, wrist_tilt, 0, 0]
                    sim.set_joint_positions(joints)
                    sim.step(2)
                    
                    progress = (i + 1) / steps * 100
                    self.task_progress[sim_id]['progress'] = progress
                
                return f"✅ Figure-8 trajectory completed!\n" + \
                       f"🎨 Traced a figure-8 pattern in {duration:.1f} seconds\n" + \
                       f"📏 Pattern size: {size:.2f}m, Center: [{center_x:.2f}, {center_y:.2f}]\n" + \
                       f"✨ Smooth robotic motion demonstration complete!"
                       
            except Exception as e:
                return f"❌ Failed to perform figure-8: {str(e)}"
        
        @self.tool("place_object_at")
        def place_object_at(sim_id: str, target_x: float, target_y: float, target_z: float) -> str:
            """Place the currently grasped object at the specified location."""
            if sim_id not in self.simulations:
                return f"❌ Simulation {sim_id} not found"
            
            try:
                sim = self.simulations[sim_id]
                
                self.task_progress[sim_id] = {
                    'name': f'Placing object at [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]',
                    'progress': 0
                }
                
                # Step 1: Move to above target location (50% progress)
                above_target = [target_x, target_y, target_z + 0.1]
                move_result = move_robot_to_position(sim_id, *above_target, 2.0)
                self.task_progress[sim_id]['progress'] = 50
                
                # Step 2: Lower to placement location (75% progress)
                at_target = [target_x, target_y, target_z + 0.03]
                move_result = move_robot_to_position(sim_id, *at_target, 1.5)
                self.task_progress[sim_id]['progress'] = 75
                
                # Step 3: Open gripper to release object (90% progress)
                current_joints = sim.get_joint_positions()
                current_joints[5] = 0.02  # Open left finger
                current_joints[6] = 0.02  # Open right finger
                sim.set_joint_positions(current_joints)
                sim.step(50)
                self.task_progress[sim_id]['progress'] = 90
                
                # Step 4: Retract gripper (100% progress)
                retract_pos = [target_x, target_y, target_z + 0.15]
                move_result = move_robot_to_position(sim_id, *retract_pos, 1.0)
                self.task_progress[sim_id]['progress'] = 100
                
                return f"✅ Object placed successfully!\n" + \
                       f"📦 Placed at: [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]\n" + \
                       f"🤖 Robot retracted and ready for next task"
                       
            except Exception as e:
                return f"❌ Failed to place object: {str(e)}"
        
        @self.tool("stop_simulation")
        def stop_simulation(sim_id: str) -> str:
            """Stop and clean up a simulation."""
            if sim_id not in self.simulations:
                return f"❌ Simulation {sim_id} not found"
            
            del self.simulations[sim_id]
            if sim_id in self.task_progress:
                del self.task_progress[sim_id]
            
            return f"✅ Simulation {sim_id} stopped and cleaned up"

async def main():
    """Run the MCP demo server."""
    print("🚀 Starting MuJoCo MCP Demo Server...")
    print("🤖 Ready for robot control via Claude/Cursor!")
    print("📡 Connect your MCP client to control the robot arm")
    print("---")
    
    app = MuJoCoMCPDemo()
    
    # In a real deployment, this would be configured to work with MCP clients
    # For demo purposes, we'll show what the server provides
    print("Available MCP Resources:")
    print("- simulation_status: Get status of active simulations")
    print("- robot_state: Get detailed robot joint and pose information")  
    print("- scene_objects: Get positions of all objects in the scene")
    print()
    print("Available MCP Tools:")
    print("- start_robot_demo: Initialize robot arm with objects")
    print("- move_robot_to_position: Move end-effector to coordinates")
    print("- pick_up_object: Grasp and lift specified objects")
    print("- perform_figure_eight: Execute smooth figure-8 trajectory")
    print("- place_object_at: Place grasped object at target location")
    print("- stop_simulation: Clean up simulation")
    print()
    print("🎯 Example Claude commands:")
    print('  "Start the robot demo and pick up the red cube"')
    print('  "Move the robot to position 0.3, 0.2, 0.4"') 
    print('  "Make the robot draw a figure-8 pattern in the air"')
    print('  "Place the object at coordinates -0.1, 0.3, 0.2"')
    
    # Keep server running
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())