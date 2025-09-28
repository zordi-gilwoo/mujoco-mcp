"""Built-in MuJoCo scene XML definitions for quick loading.

This module centralizes small, self-contained scenes that can be loaded without
network access. It keeps the server-side logic lightweight while still
providing visually distinct environments for basic demos.
"""

from __future__ import annotations

from typing import Dict, Optional

_DEFAULT_SCENE_NAME = "pendulum"

_PENDULUM_XML = """
<mujoco model="pendulum">
    <compiler angle="radian"/>

    <asset>
        <material name="MatPlane" reflectance="0.5" texture="TexPlane" texrepeat="1 1" texuniform="true"/>
        <texture name="TexPlane" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 .2 .3" width="512" height="512"/>
    </asset>

    <default>
        <joint damping="0.05"/>
        <geom contype="0" friction="1 0.1 0.1" rgba="0.7 0.7 0 1"/>
    </default>

    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <camera name="track" mode="trackcom" pos="0 -2 1" xyaxes="1 0 0 0 1 2"/>

        <geom name="floor" pos="0 0 -0.5" size="2 2 0.1" type="plane" material="MatPlane"/>

        <body name="pole" pos="0 0 1">
            <joint name="hinge" type="hinge" axis="1 0 0" pos="0 0 0"/>
            <geom name="pole" type="capsule" size="0.045" fromto="0 0 0 0 0 -1" rgba="0 0.7 0.7 1"/>

            <body name="mass" pos="0 0 -1">
                <geom name="mass" type="sphere" size="0.15" rgba="1 0 0 1"/>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor joint="hinge" gear="1" ctrllimited="true" ctrlrange="-3 3"/>
    </actuator>
</mujoco>
""".strip()

_CARTPOLE_XML = """
<mujoco model="cartpole">
    <option timestep="0.002"/>

    <worldbody>
        <geom name="track" type="plane" size="5 0.1 0.05" rgba="0.2 0.3 0.2 1"/>

        <body name="cart" pos="0 0 0.1">
            <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
            <geom name="cart_geom" type="box" size="0.12 0.1 0.06" rgba="0.8 0.2 0.2 1"/>

            <body name="pole" pos="0 0 0.1">
                <joint name="hinge" type="hinge" axis="0 1 0" range="-1.57 1.57" damping="0.05"/>
                <geom name="pole_geom" type="capsule" size="0.02 0.6" rgba="0.2 0.8 0.2 1" fromto="0 0 0 0 0 0.6"/>
                <body name="pole_tip" pos="0 0 0.6">
                    <geom name="pole_cap" type="sphere" size="0.04" rgba="0.1 0.6 0.9 1"/>
                </body>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor joint="slider" gear="15" ctrllimited="true" ctrlrange="-3 3"/>
    </actuator>
</mujoco>
""".strip()

_FRANKA_PANDA_XML = """
<mujoco model="franka_panda_demo">
    <option timestep="0.002"/>

    <asset>
        <material name="MatBase" rgba="0.2 0.2 0.2 1"/>
        <material name="MatLink" rgba="0.2 0.6 0.8 1"/>
        <material name="MatEE" rgba="0.9 0.3 0.2 1"/>
    </asset>

    <worldbody>
        <light name="light" pos="1 1 2" dir="-1 -1 -2" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
        <camera name="track" pos="2 -2 1.5" lookat="0 0 0.5"/>
        <geom name="table" type="box" size="0.4 0.4 0.02" pos="0 0 -0.02" rgba="0.4 0.3 0.2 1"/>

        <body name="base" pos="0 0 0">
            <geom name="base_geom" type="cylinder" size="0.12 0.08" material="MatBase"/>

            <body name="link1" pos="0 0 0.08">
                <joint name="joint1" type="hinge" axis="0 0 1" limited="true" range="-2.897 2.897"/>
                <geom name="link1_geom" type="capsule" size="0.04 0.15" material="MatLink" fromto="0 0 0 0 0 0.3"/>

                <body name="link2" pos="0 0 0.3">
                    <joint name="joint2" type="hinge" axis="0 1 0" limited="true" range="-1.762 1.762"/>
                    <geom name="link2_geom" type="capsule" size="0.035 0.25" material="MatLink" fromto="0 0 0 0 0 0.28"/>

                    <body name="link3" pos="0 0 0.28">
                        <joint name="joint3" type="hinge" axis="0 0 1" limited="true" range="-2.897 2.897"/>
                        <geom name="link3_geom" type="capsule" size="0.03 0.2" material="MatLink" fromto="0 0 0 0 0 0.22"/>

                        <body name="link4" pos="0 0 0.22">
                            <joint name="joint4" type="hinge" axis="0 1 0" limited="true" range="-3.071 0"/>
                            <geom name="link4_geom" type="capsule" size="0.028 0.18" material="MatLink" fromto="0 0 0 0 0 0.2"/>

                            <body name="link5" pos="0 0 0.2">
                                <joint name="joint5" type="hinge" axis="0 0 1" limited="true" range="-2.897 2.897"/>
                                <geom name="link5_geom" type="capsule" size="0.025 0.16" material="MatLink" fromto="0 0 0 0 0 0.18"/>

                                <body name="link6" pos="0 0 0.18">
                                    <joint name="joint6" type="hinge" axis="0 1 0" limited="true" range="-0.017 3.752"/>
                                    <geom name="link6_geom" type="capsule" size="0.022 0.14" material="MatLink" fromto="0 0 0 0 0 0.16"/>

                                    <body name="end_effector" pos="0 0 0.16">
                                        <joint name="joint7" type="hinge" axis="0 0 1" limited="true" range="-2.897 2.897"/>
                                        <geom name="eef_geom" type="sphere" size="0.05" material="MatEE"/>
                                        <site name="eef_site" pos="0 0 0" size="0.01" rgba="0.1 0.8 0.3 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>

        <body name="target" pos="0.5 0 0.4">
            <geom name="target_geom" type="sphere" size="0.04" rgba="0 1 0 0.7"/>
        </body>
    </worldbody>

    <actuator>
        <position joint="joint1" kp="200"/>
        <position joint="joint2" kp="200"/>
        <position joint="joint3" kp="200"/>
        <position joint="joint4" kp="160"/>
        <position joint="joint5" kp="140"/>
        <position joint="joint6" kp="120"/>
        <position joint="joint7" kp="100"/>
    </actuator>
</mujoco>
""".strip()

_SCENE_ALIASES: Dict[str, str] = {
    "pendulum": "pendulum",
    "simple pendulum": "pendulum",
    "double pendulum": "pendulum",  # Fallback until a dedicated model exists
    "cartpole": "cartpole",
    "cart pole": "cartpole",
    "cart-pole": "cartpole",
    "cart": "cartpole",
    "pendulum cart": "cartpole",
    "franka": "franka_panda",
    "panda": "franka_panda",
    "franka panda": "franka_panda",
    "franka_emika_panda": "franka_panda",
}

_BUILTIN_SCENES: Dict[str, str] = {
    "pendulum": _PENDULUM_XML,
    "cartpole": _CARTPOLE_XML,
    "franka_panda": _FRANKA_PANDA_XML,
}


def normalize_scene_name(name: Optional[str]) -> Optional[str]:
    """Map a free-form name to a known scene key."""
    if not name:
        return None

    key = name.strip().lower()
    if not key:
        return None

    return _SCENE_ALIASES.get(key, key if key in _BUILTIN_SCENES else None)


def get_builtin_scene(name: Optional[str]) -> Optional[str]:
    """Return the XML string for a built-in scene if it exists."""
    key = normalize_scene_name(name)
    if key is None:
        return None
    return _BUILTIN_SCENES.get(key)


def default_scene_xml() -> str:
    """Return the default scene XML."""
    return _BUILTIN_SCENES[_DEFAULT_SCENE_NAME]


def list_builtin_scenes() -> Dict[str, str]:
    """Expose a copy of the built-in scene mapping."""
    return dict(_BUILTIN_SCENES)

