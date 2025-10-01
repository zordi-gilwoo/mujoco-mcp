#!/usr/bin/env python3
"""
MuJoCo Simplified Demo Script
Demonstrates basic MuJoCo functionality

Usage:
python simple_demo.py
"""

import numpy as np
import mujoco
import argparse
import logging
from typing import List

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("mujoco_simple_demo")

# Example MuJoCo model XML (simple manipulator and some objects)
EXAMPLE_MODEL_XML = """
<mujoco>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="5 5 0.1" rgba=".9 .9 .9 1"/>

        <!-- Simple robot -->
        <body name="robot1" pos="0 0 0.5">
            <joint name="robot1_base_rot" type="ball"/>
            <geom name="robot1_base" type="cylinder" size="0.1 0.1" rgba="0.7 0.7 0.7 1"/>
            <body name="robot1_arm" pos="0 0 0.1">
                <joint name="robot1_shoulder" axis="0 1 0" range="-180 180"/>
                <geom name="robot1_arm_geom" type="capsule" size="0.05"
                      fromto="0 0 0 0.5 0 0" rgba="0.7 0.7 0.7 1"/>
                <body name="robot1_forearm" pos="0.5 0 0">
                    <joint name="robot1_elbow" axis="0 1 0" range="-90 90"/>
                    <geom name="robot1_forearm_geom" type="capsule" size="0.04"
                          fromto="0 0 0 0.5 0 0" rgba="0.7 0.7 0.7 1"/>
                    <body name="robot1_wrist" pos="0.5 0 0">
                        <joint name="robot1_wrist_rot" axis="1 0 0" range="-180 180"/>
                        <geom name="robot1_wrist_geom" type="sphere" size="0.05"
                              rgba="0.5 0.5 0.5 1"/>
                        <site name="robot1_grasp_site" pos="0 0 0.1" size="0.02"/>
                    </body>
                </body>
            </body>
        </body>

        <body name="red_cube" pos="1 0 0.1">
            <joint type="free"/>
            <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
        </body>

        <body name="blue_cube" pos="0 1 0.1">
            <joint type="free"/>
            <geom type="box" size="0.1 0.1 0.1" rgba="0 0 1 1"/>
        </body>

        <body name="green_sphere" pos="-1 0 0.1">
            <joint type="free"/>
            <geom type="sphere" size="0.1" rgba="0 1 0 1"/>
        </body>
    </worldbody>

    <actuator>
        <motor name="robot1_base_rot_motor" joint="robot1_base_rot"/>
        <motor name="robot1_shoulder_motor" joint="robot1_shoulder"/>
        <motor name="robot1_elbow_motor" joint="robot1_elbow"/>
        <motor name="robot1_wrist_motor" joint="robot1_wrist_rot"/>
    </actuator>
</mujoco>
"""


class MuJoCoSimulation:

    def _find_bodies(self):
        """Find objects and robot parts in the scene"""
        self.body_names = {}
        self.robot_parts = {}

        # Get all body indices and names
        for i in range(self.model.nbody):
            name = self.model.body(i).name
            if name:
                self.body_names[name] = i

                # Identify robot parts
                if name.startswith("robot1_"):
                    self.robot_parts[name] = i

        for _ in range(num_steps):
            mujoco.mj_step(self.model, self.data)

    def reset(self):
        if self.renderer:
            self.renderer.update_scene(self.data)
            return self.renderer.render()
        return None

    def set_joint_positions(self, positions: List[float]):
        for i, pos in enumerate(positions):
            if i < self.model.nq:
                self.data.qpos[i] = pos
        mujoco.mj_forward(self.model, self.data)

    def get_joint_positions(self) -> List[float]:
        for i, ctrl in enumerate(control):
            if i < self.model.nu:
                self.data.ctrl[i] = ctrl

    def get_body_position(self, body_name: str) -> List[float] | None:
        if body_name in self.body_names:
            body_id = self.body_names[body_name]
            return self.data.body(body_id).xpos.copy()
        return None

    def apply_force(self, body_name: str, force: List[float]):
        shoulder_idx = -1
        elbow_idx = -1
        wrist_idx = -1

        for i in range(self.model.njnt):
            name = self.model.joint(i).name
            if name == "robot1_shoulder":
                shoulder_idx = self.model.joint(i).qposadr[0]
            elif name == "robot1_elbow":
                elbow_idx = self.model.joint(i).qposadr[0]
            elif name == "robot1_wrist_rot":
                wrist_idx = self.model.joint(i).qposadr[0]

        if shoulder_idx >= 0:
            self.data.qpos[shoulder_idx] = np.deg2rad(shoulder_angle)
        if elbow_idx >= 0:
            self.data.qpos[elbow_idx] = np.deg2rad(elbow_angle)
        if wrist_idx >= 0:
            self.data.qpos[wrist_idx] = np.deg2rad(wrist_angle)

            shoulder_idx = -1
            elbow_idx = -1

            for j in range(self.model.njnt):
                name = self.model.joint(j).name
                if name == "robot1_shoulder":
                    shoulder_idx = self.model.joint(j).qposadr[0]
                elif name == "robot1_elbow":
                    elbow_idx = self.model.joint(j).qposadr[0]

            wrist_pos = self.get_body_position("robot1_wrist")
            distance = np.linalg.norm(np.array(target_pos) - wrist_pos)

            if distance < 0.2:
    sim.apply_force("green_sphere", [10.0, 0.0, 5.0])
    for _ in range(5):
        sim.step(20)
        pos = sim.get_body_position("green_sphere")
    run_demo()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
