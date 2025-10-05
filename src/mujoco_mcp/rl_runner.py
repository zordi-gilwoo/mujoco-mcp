#!/usr/bin/env python3
"""
RL Environment Runner for MuJoCo MCP Viewer
Handles execution of RL environments with random actions and viewer integration
"""

import asyncio
import logging
import json
import time
import numpy as np
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict

from .rl_integration import (
    MuJoCoRLEnvironment,
    RLConfig,
    create_reaching_env,
    create_balancing_env,
    create_walking_env,
)
from .viewer_client import MuJoCoViewerClient, viewer_manager

logger = logging.getLogger(__name__)


@dataclass
class RLRunnerStatus:
    """Status of the RL environment runner"""

    is_running: bool = False
    current_episode: int = 0
    total_steps: int = 0
    total_reward: float = 0.0
    average_reward: float = 0.0
    environment_type: str = ""
    last_action: Optional[np.ndarray] = None
    last_observation: Optional[np.ndarray] = None


class RLEnvironmentRunner:
    """Manages RL environment execution with viewer integration"""

    def __init__(self, viewer_port: int = 8888):
        self.viewer_port = viewer_port
        self.viewer_client = None
        self.current_env = None
        self.status = RLRunnerStatus()
        self.is_running = False
        self.runner_task = None

    async def create_environment(self, config: Dict[str, Any]) -> bool:
        """Create RL environment from configuration"""
        try:
            # Parse configuration
            rl_config = RLConfig(
                robot_type=config.get("robot_type", "simple_arm"),
                task_type=config.get("task_type", "reaching"),
                max_episode_steps=config.get("max_episode_steps", 1000),
                action_space_type=config.get("action_space_type", "continuous"),
                reward_scale=config.get("reward_scale", 1.0),
            )

            # Create environment using factory functions
            if rl_config.task_type == "reaching":
                self.current_env = create_reaching_env(rl_config.robot_type)
            elif rl_config.task_type == "balancing":
                self.current_env = create_balancing_env()
            elif rl_config.task_type == "walking":
                self.current_env = create_walking_env(rl_config.robot_type)
            else:
                # Default to reaching task
                self.current_env = create_reaching_env(rl_config.robot_type)

            # Update status
            self.status.environment_type = f"{rl_config.task_type}_{rl_config.robot_type}"

            logger.info(f"Created RL environment: {self.status.environment_type}")
            return True

        except Exception as e:
            logger.error(f"Failed to create RL environment: {e}")
            return False

    async def connect_viewer(self, model_id: str = "rl_environment") -> bool:
        """Connect to MuJoCo viewer"""
        try:
            # Get or create viewer client
            self.viewer_client = viewer_manager.get_client(model_id)
            if not self.viewer_client:
                if viewer_manager.create_client(model_id, self.viewer_port):
                    self.viewer_client = viewer_manager.get_client(model_id)
                else:
                    logger.error("Failed to create viewer client")
                    return False

            logger.info(f"Connected to viewer for model: {model_id}")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to viewer: {e}")
            return False

    async def load_environment_to_viewer(
        self, xml_content: str, model_id: str = "rl_environment"
    ) -> bool:
        """Load RL environment XML to viewer"""
        try:
            if not self.viewer_client:
                await self.connect_viewer(model_id)

            if self.viewer_client:
                result = self.viewer_client.load_model(xml_content, model_id)
                if result.get("success", False):
                    logger.info("RL environment loaded to viewer")
                    return True
                else:
                    logger.error(f"Failed to load environment to viewer: {result}")
                    return False
            else:
                logger.error("No viewer client available")
                return False

        except Exception as e:
            logger.error(f"Error loading environment to viewer: {e}")
            return False

    async def start_random_actions(self, num_steps: int = 1000, step_delay: float = 0.02) -> bool:
        """Start running RL environment with random actions"""
        if self.is_running or not self.current_env:
            return False

        self.is_running = True
        self.status.is_running = True
        self.status.current_episode = 0
        self.status.total_steps = 0
        self.status.total_reward = 0.0

        # Start the runner task
        self.runner_task = asyncio.create_task(self._run_random_actions_loop(num_steps, step_delay))

        logger.info(f"Started RL environment with random actions for {num_steps} steps")
        return True

    async def stop_environment(self):
        """Stop RL environment execution"""
        self.is_running = False
        self.status.is_running = False

        if self.runner_task:
            self.runner_task.cancel()
            try:
                await self.runner_task
            except asyncio.CancelledError:
                pass
            self.runner_task = None

        logger.info("RL environment stopped")

    async def _run_random_actions_loop(self, num_steps: int, step_delay: float):
        """Main loop for running random actions"""
        try:
            obs, info = self.current_env.reset()
            self.status.last_observation = obs
            episode_rewards = []
            current_episode_reward = 0.0

            for step in range(num_steps):
                if not self.is_running:
                    break

                # Sample random action
                action = self.current_env.action_space.sample()
                self.status.last_action = action

                # Execute action
                obs, reward, terminated, truncated, info = self.current_env.step(action)

                # Update status
                self.status.last_observation = obs
                self.status.total_steps += 1
                current_episode_reward += reward
                self.status.total_reward += reward

                # Update viewer with current state
                if self.viewer_client and step % 5 == 0:  # Update every 5 steps to reduce load
                    await self._update_viewer_state(obs, action, reward)

                # Check if episode ended
                if terminated or truncated:
                    episode_rewards.append(current_episode_reward)
                    self.status.current_episode += 1
                    self.status.average_reward = (
                        np.mean(episode_rewards) if episode_rewards else 0.0
                    )

                    logger.info(
                        f"Episode {self.status.current_episode} finished: reward={current_episode_reward:.3f}"
                    )

                    # Reset environment
                    obs, info = self.current_env.reset()
                    current_episode_reward = 0.0

                # Small delay for visualization
                await asyncio.sleep(step_delay)

            logger.info(
                f"Completed {num_steps} steps across {self.status.current_episode} episodes"
            )

        except Exception as e:
            logger.error(f"Error in random actions loop: {e}")
        finally:
            self.is_running = False
            self.status.is_running = False

    async def _update_viewer_state(
        self, observation: np.ndarray, action: np.ndarray, reward: float
    ):
        """Update viewer with current environment state"""
        try:
            if self.viewer_client:
                # Send state update command (this would need to be implemented in viewer client)
                state_data = {
                    "observation": (
                        observation.tolist() if hasattr(observation, "tolist") else observation
                    ),
                    "action": action.tolist() if hasattr(action, "tolist") else action,
                    "reward": float(reward),
                    "step": self.status.total_steps,
                }
                # For now, just log the state - in full implementation this would update the viewer
                logger.debug(f"Step {self.status.total_steps}: reward={reward:.3f}")

        except Exception as e:
            logger.debug(f"Failed to update viewer state: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Get current runner status"""
        status_dict = asdict(self.status)

        # Convert numpy arrays to lists for JSON serialization
        if status_dict["last_action"] is not None:
            status_dict["last_action"] = status_dict["last_action"].tolist()
        if status_dict["last_observation"] is not None:
            status_dict["last_observation"] = status_dict["last_observation"].tolist()

        return status_dict

    def cleanup(self):
        """Cleanup resources"""
        if self.current_env:
            self.current_env.close()
            self.current_env = None

        if self.viewer_client:
            self.viewer_client = None


# Global RL runner instance
rl_runner = RLEnvironmentRunner()


async def create_and_run_rl_environment(
    config: Dict[str, Any],
    xml_content: str,
    num_steps: int = 1000,
    model_id: str = "rl_environment",
) -> Dict[str, Any]:
    """
    High-level function to create, load, and run RL environment
    """
    try:
        # Create environment
        if not await rl_runner.create_environment(config):
            return {"success": False, "error": "Failed to create RL environment"}

        # Load to viewer
        if not await rl_runner.load_environment_to_viewer(xml_content, model_id):
            return {"success": False, "error": "Failed to load environment to viewer"}

        # Start random actions
        if not await rl_runner.start_random_actions(num_steps):
            return {"success": False, "error": "Failed to start random actions"}

        return {"success": True, "status": rl_runner.get_status()}

    except Exception as e:
        logger.error(f"Error in create_and_run_rl_environment: {e}")
        return {"success": False, "error": str(e)}


async def stop_rl_environment() -> Dict[str, Any]:
    """Stop the current RL environment"""
    try:
        await rl_runner.stop_environment()
        return {"success": True}
    except Exception as e:
        logger.error(f"Error stopping RL environment: {e}")
        return {"success": False, "error": str(e)}


def get_rl_status() -> Dict[str, Any]:
    """Get current RL environment status"""
    return rl_runner.get_status()


# Example usage
async def main():
    """Example usage of RL environment runner"""
    config = {
        "robot_type": "simple_arm",
        "task_type": "reaching",
        "max_episode_steps": 500,
        "action_space_type": "continuous",
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

    print("🤖 Starting RL Environment Runner Example")
    print("=" * 50)

    # Create and run environment
    result = await create_and_run_rl_environment(config, xml_content, num_steps=100)
    print(f"Start result: {result}")

    # Let it run for a bit
    await asyncio.sleep(5)

    # Check status
    status = get_rl_status()
    print(f"Status: {status}")

    # Stop environment
    stop_result = await stop_rl_environment()
    print(f"Stop result: {stop_result}")


if __name__ == "__main__":
    asyncio.run(main())
