# 🤖 Setting up MuJoCo MCP with Claude Desktop/Cursor

This guide shows how to configure Claude Desktop or Cursor to control MuJoCo physics simulations through natural language commands.

## 📋 Prerequisites

1. **Python 3.8+** with MuJoCo MCP installed:
   ```bash
   git clone https://github.com/yourusername/mujoco-mcp.git
   cd mujoco-mcp
   pip install -e .
   ```

2. **Claude Desktop** (latest version) or **Cursor IDE**

3. **MuJoCo** physics engine (installed automatically with the package)

## 🚀 Quick Start

### Step 1: Start the MCP Server

```bash
# Navigate to the project directory
cd mujoco-mcp

# Start the enhanced demo server
python examples/mcp_demo.py
```

The server will start and display:
```
🚀 Starting MuJoCo MCP Demo Server...
🤖 Ready for robot control via Claude/Cursor!
📡 Connect your MCP client to control the robot arm
```

### Step 2: Configure Claude Desktop

1. **Open Claude Desktop Settings**
   - Click the settings gear icon
   - Navigate to "Developer" or "Integrations" section

2. **Add MCP Server Configuration**
   ```json
   {
     "mcpServers": {
       "mujoco-physics": {
         "command": "python",
         "args": ["/path/to/mujoco-mcp/examples/mcp_demo.py"],
         "env": {
           "PYTHONPATH": "/path/to/mujoco-mcp/src"
         }
       }
     }
   }
   ```

3. **Restart Claude Desktop**

### Step 3: Configure Cursor IDE

1. **Install MCP Extension** (if available)
   - Open Extensions panel (Ctrl/Cmd+Shift+X)
   - Search for "Model Context Protocol" or "MCP"
   - Install and enable

2. **Configure MCP Settings**
   - Open Cursor settings (Ctrl/Cmd+,)
   - Navigate to Extensions → MCP
   - Add server configuration:
   ```json
   {
     "server": "http://localhost:8000",
     "name": "MuJoCo Physics"
   }
   ```

## 🎯 Example Commands

Once configured, you can control the robot with natural language:

### 🚀 Starting a Simulation
```
"Start the robot demonstration with the arm and objects"
```
**What happens:** Initializes a robot arm simulation with colored objects to manipulate

### 🎯 Basic Movement  
```
"Move the robot arm to position 0.3, 0.2, 0.4"
```
**What happens:** Robot calculates inverse kinematics and smoothly moves to coordinates

### 📦 Object Manipulation
```
"Pick up the red cube and place it at coordinates -0.1, 0.3, 0.2"
```
**What happens:** 
1. Robot moves to red cube location
2. Opens gripper
3. Grasps the cube
4. Lifts and moves to target location
5. Places cube and retracts

### 🎨 Advanced Trajectories
```
"Make the robot trace a figure-8 pattern in the air above the table"
```
**What happens:** Robot executes smooth parametric trajectory forming figure-8

### 📊 Status Queries
```
"What's the current status of the simulation?"
"Show me the robot's joint positions"
"Where are all the objects in the scene?"
```
**What happens:** Returns detailed state information

## 🔧 Advanced Configuration

### Custom Model Loading
```
"Start a simulation with a custom robot model from /path/to/model.xml"
```

### Multi-Robot Scenarios
```json
{
  "mcpServers": {
    "mujoco-multi": {
      "command": "python",
      "args": ["/path/to/mujoco-mcp/examples/multi_robot_demo.py"],
      "env": {
        "MAX_ROBOTS": "3",
        "ENABLE_COLLISION_DETECTION": "true"
      }
    }
  }
}
```

### Physics Parameters
```
"Change the simulation timestep to 0.001 seconds"
"Set gravity to Mars conditions (3.71 m/s²)"
"Enable collision detection between all objects"
```

## 🛠️ Troubleshooting

### Server Connection Issues
```bash
# Check if server is running
curl -X POST http://localhost:8000/mcp/ping

# Debug mode with verbose logging
PYTHONPATH=/path/to/mujoco-mcp/src python examples/mcp_demo.py --debug
```

### Permission Errors
```bash
# Ensure MuJoCo license (if required)
export MUJOCO_GL=egl  # For headless servers

# Fix Python path issues
pip install -e . --user
```

### Performance Optimization
```json
{
  "env": {
    "MUJOCO_GL": "egl",
    "OMP_NUM_THREADS": "4",
    "SIMULATION_FREQ": "100"
  }
}
```

## 🎭 Demo Scenarios

### 1. **Pick and Place Assembly Line**
```
"Set up an assembly line simulation where the robot picks up components and assembles them in sequence"
```

### 2. **Collision Avoidance Navigation**  
```
"Create obstacles in the workspace and have the robot navigate around them to reach the target"
```

### 3. **Dynamic Grasping**
```
"Simulate moving objects on a conveyor belt and have the robot catch them"
```

### 4. **Multi-Modal Control**
```
"Show me the robot camera view while it performs the pick and place task"
```

## 📈 Performance Tips

1. **Optimize Simulation Frequency**
   - Use 100-200 Hz for smooth visualization
   - Use 1000+ Hz for precise control tasks

2. **Batch Operations**
   ```
   "Plan a trajectory to visit all three objects in optimal order, then execute it"
   ```

3. **Parallel Simulations**
   ```
   "Run the same task in 5 parallel simulations with different random conditions"
   ```

## 🔍 Debugging

### View Real-time State
```python
# Add to your commands
"Show me a live update of joint positions every 0.1 seconds"
"Plot the end-effector trajectory during the last movement"
```

### Error Handling
```
"If the robot fails to grasp the object, try adjusting the approach angle"
"Detect collisions and stop motion if any occur"
```

## 🌟 Advanced Features

### 1. **Reinforcement Learning Integration**
```
"Train the robot to learn optimal grasping strategies for different object shapes"
```

### 2. **Vision-Based Control**
```
"Use the robot's camera to identify and pick up only red objects"
```

### 3. **Force Feedback**
```
"Apply gentle force when contacting objects to avoid damage"
```

### 4. **Human-Robot Interaction**
```
"Simulate a human hand in the workspace and have the robot collaborate safely"
```

## 📚 Resources

- **MuJoCo Documentation**: https://mujoco.readthedocs.io/
- **Model Context Protocol**: https://spec.modelcontextprotocol.io/
- **Example Models**: `/path/to/mujoco-mcp/models/`
- **API Reference**: `/path/to/mujoco-mcp/docs/api.md`

## 🆘 Support

If you encounter issues:

1. **Check the logs**: `tail -f /tmp/mujoco_mcp.log`
2. **Validate configuration**: `python -m mujoco_mcp.validate_config`
3. **Test basic functionality**: `python examples/basic_example.py`
4. **Report issues**: https://github.com/yourusername/mujoco-mcp/issues

---

## 🎉 You're Ready!

Your MuJoCo MCP setup is complete! You can now:

- 🤖 Control robots with natural language
- 🎯 Perform complex manipulation tasks  
- 🎨 Create smooth trajectories
- 📊 Monitor simulation states
- 🔬 Run physics experiments

**Try your first command:**
```
"Start the robot demo and make it wave hello!"
```

Happy robotics! 🚀🤖