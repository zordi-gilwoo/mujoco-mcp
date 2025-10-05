# Advanced Features Guide

## Overview

MuJoCo MCP provides advanced features for robotics research and development.

---

## 🎮 Advanced Control Algorithms

### PID Controllers

```python
from mujoco_mcp.advanced_controllers import PIDController, PIDConfig

# Create controller
config = PIDConfig(kp=5.0, ki=0.1, kd=0.5)
pid = PIDController(config)

# Control loop
command = pid.update(target, current, dt=0.02)
```

### Trajectory Planning

```python
from mujoco_mcp.advanced_controllers import TrajectoryPlanner

planner = TrajectoryPlanner()

# Minimum jerk trajectory
positions, velocities, accelerations = planner.minimum_jerk_trajectory(
    start_pos=np.array([0, 0, 0]),
    end_pos=np.array([1, 0.5, 0]),
    duration=5.0
)

# Spline through waypoints
waypoints = np.array([[0, 0], [1, 1], [2, 0]])
times = np.array([0, 1, 2])
traj = planner.spline_trajectory(waypoints, times)
```

---

## 🤖 Multi-Robot Coordination

### Formation Control

```python
from mujoco_mcp.multi_robot_coordinator import MultiRobotCoordinator

coordinator = MultiRobotCoordinator()

# Add robots
coordinator.add_robot("arm1", "franka_panda", {"manipulation": True})
coordinator.add_robot("arm2", "ur5e", {"manipulation": True})

# Start coordination
coordinator.start_coordination()

# Formation task
task_id = coordinator.formation_control(
    robots=["arm1", "arm2"],
    formation_type="line",
    spacing=1.5
)
```

### Cooperative Manipulation

```python
# Cooperative grasping
task_id = coordinator.cooperative_manipulation(
    robots=["arm1", "arm2"],
    target_object="box",
    approach_positions={
        "arm1": np.array([0.3, 0.1, 0.5]),
        "arm2": np.array([0.3, -0.1, 0.5])
    }
)
```

---

## 🔬 Sensor Feedback

### Sensor Suite

```python
from mujoco_mcp.sensor_feedback import create_robot_sensor_suite

# Create sensors
sensor_manager = create_robot_sensor_suite("franka_panda", n_joints=7)
sensor_manager.start_sensing()

# Get readings
readings = sensor_manager.get_latest_readings()
```

### Closed-Loop Control

```python
from mujoco_mcp.sensor_feedback import create_feedback_controller

controller = create_feedback_controller("franka_panda")

# Set target
controller.set_target({
    "joint_position": np.array([0, -0.785, 0, -1.57, 0, 1.57, 0])
})

# Control loop
controller.update_state(sensor_readings)
commands = controller.compute_control()
```

---

## 🧠 Reinforcement Learning

### Create RL Environment

```python
from mujoco_mcp.rl_integration import create_reaching_env, RLTrainer

# Create environment
env = create_reaching_env("franka_panda")
trainer = RLTrainer(env)

# Run baseline
baseline_results = trainer.random_policy_baseline(num_episodes=10)

# Evaluate policy
results = trainer.evaluate_policy(my_policy, num_episodes=10)
```

### Custom RL Tasks

```python
from mujoco_mcp.rl_integration import MuJoCoRLEnvironment, RLConfig

config = RLConfig(
    robot_type="franka_panda",
    task_type="reaching",
    max_episode_steps=1000
)

env = MuJoCoRLEnvironment(config)

# Standard Gymnasium interface
obs, info = env.reset()
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

---

## 📊 Physics Benchmarking

### Benchmark Suite

```python
from benchmarks.physics_benchmarks import BenchmarkSuite

suite = BenchmarkSuite("results/")
results = suite.run_all_benchmarks(parallel=True)

# View results
for result in results:
    print(f"{result.test_name}: {'PASS' if result.success else 'FAIL'}")
```

### Available Benchmarks
- Simulation stability (energy conservation)
- Performance (FPS, CPU, memory)
- Accuracy (vs analytical solutions)
- Scalability (increasing complexity)

---

## 📈 Real-time Visualization

### State Monitoring

```python
from mujoco_mcp.visualization_tools import RobotStateMonitor

monitor = RobotStateMonitor(viewer_client)
monitor.start_monitoring("robot_model", update_rate=50.0)

# Show visualizations
monitor.show_joint_positions()
monitor.show_dashboard()

# Export data
monitor.export_data("robot_data.json")
```

### Trajectory Visualization

```python
from mujoco_mcp.visualization_tools import TrajectoryVisualizer

viz = TrajectoryVisualizer()
viz.add_trajectory("planned", planned_positions)
viz.add_trajectory("actual", actual_positions)

# Show 3D plot
viz.create_3d_plot()
viz.show_plot()
```

---

## 🔧 Complete Example

```python
import asyncio
from mujoco_mcp.multi_robot_coordinator import MultiRobotCoordinator
from mujoco_mcp.visualization_tools import RobotStateMonitor

async def main():
    # Initialize
    coordinator = MultiRobotCoordinator()
    coordinator.add_robot("arm1", "franka_panda", {"manipulation": True})
    coordinator.add_robot("arm2", "ur5e", {"manipulation": True})
    
    # Start systems
    coordinator.start_coordination()
    monitor = RobotStateMonitor(coordinator.viewer_client)
    monitor.start_monitoring("arm1")
    
    # Execute task
    task_id = coordinator.cooperative_manipulation(
        robots=["arm1", "arm2"],
        target_object="box",
        approach_positions={
            "arm1": np.array([0.3, 0.1, 0.5]),
            "arm2": np.array([0.3, -0.1, 0.5])
        }
    )
    
    # Wait for completion
    while True:
        status = coordinator.get_task_status(task_id)
        if status in ["completed", "failed"]:
            break
        await asyncio.sleep(0.1)
    
    # Cleanup
    monitor.export_data("task_results.json")
    coordinator.stop_coordination()

asyncio.run(main())
```

---

## 📚 API Modules

- `advanced_controllers` - PID, trajectory planning, optimization
- `multi_robot_coordinator` - Multi-robot coordination and tasks
- `sensor_feedback` - Sensor processing and closed-loop control
- `rl_integration` - Reinforcement learning environments
- `visualization_tools` - Real-time plotting and monitoring
- `physics_benchmarks` - Performance and accuracy testing

For detailed API documentation, see [API_REFERENCE.md](../../API_REFERENCE.md).

---

**Last Updated**: October 2025