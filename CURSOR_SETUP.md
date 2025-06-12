# Cursor MCP Setup Guide for MuJoCo MCP v0.6.0

## 📋 Quick Setup Instructions

### Step 1: Update Your Cursor MCP Configuration

Replace your current `~/.cursor/mcp.json` file with the updated configuration that includes our MuJoCo MCP v0.6.0 server.

**Location**: `~/.cursor/mcp.json`

**Updated Configuration**:
```json
{
  "figma": {
    "Framelink Figma MCP": {
      "command": "npx",
      "args": ["-y", "figma-developer-mcp", "--figma-api-key=figd_kFSGWosDyYRkWKUfZZurv_0zGlEqYvIHZCY3nrFK", "--stdio"]
    }
  },
  "github": {
    "command": "npx", 
    "args": [
      "-y",
      "@modelcontextprotocol/server-github"
    ],
    "env": {
      "GITHUB_PERSONAL_ACCESS_TOKEN": "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"
    }
  },
  "sequential-thinking": {
    "command": "npx",
    "args": [
      "-y",
      "@modelcontextprotocol/server-sequential-thinking"
    ]
  },
  "mujoco-mcp": {
    "command": "python",
    "args": ["-m", "mujoco_mcp"],
    "cwd": "/Users/robert/workspace/04-code/03-mujoco/mujoco-mcp",
    "env": {
      "PYTHONUNBUFFERED": "1",
      "PYTHONPATH": "/Users/robert/workspace/04-code/03-mujoco/mujoco-mcp/src",
      "MUJOCO_MCP_LOG_LEVEL": "INFO"
    }
  },
  "browsermcp": {
    "command": "npx",
    "args": [
      "@browsermcp/mcp@latest"
    ]
  }
}
```

### Step 2: Install MuJoCo MCP

```bash
cd /Users/robert/workspace/04-code/03-mujoco/mujoco-mcp
pip install -e .
```

### Step 3: Test the Configuration

```bash
# Test that the server can start
python -m mujoco_mcp --check

# Test server startup (Ctrl+C to stop)
python -m mujoco_mcp --debug
```

### Step 4: Restart Cursor

After updating the MCP configuration:
1. Close Cursor completely
2. Reopen Cursor 
3. The MuJoCo MCP server should now be available

## 🧪 Testing Your Setup

### In Cursor, try these commands:

**Basic Test**:
```
Can you check what MCP servers are available?
```

**MuJoCo Specific Test**:
```
Create a pendulum simulation using MuJoCo
```

**Advanced Test**:
```
Set up a robot arm simulation and move it to position [1.0, 0.5, 1.2]
```

## 🔍 Key Changes Made

### ✅ Fixed Configuration Issues:
1. **Correct Command**: Changed from `"-m", "mcp.run"` to `"-m", "mujoco_mcp"`
2. **Proper Path**: Updated to use our v0.6.0 project path
3. **Environment Setup**: Added proper PYTHONPATH and environment variables
4. **Working Directory**: Set correct `cwd` for the project

### ✅ Enhanced Environment:
- `PYTHONUNBUFFERED=1`: Ensures real-time logging
- `PYTHONPATH`: Points to our source directory
- `MUJOCO_MCP_LOG_LEVEL=INFO`: Enables informative logging

## 📊 Available MuJoCo MCP Features

Once connected, you'll have access to **98 tools** including:

### 🎮 Basic Control
- `load_model` - Load MuJoCo models
- `step_simulation` - Advance simulation
- `reset_simulation` - Reset to initial state
- `get_state` - Query current state

### 🤖 Robot Control  
- `set_joint_positions` - Control robot joints
- `move_to_position` - Move end-effector
- `apply_control` - Apply control inputs
- `grasp_object` - Manipulation tasks

### 🧠 AI Features
- `create_rl_environment` - Setup RL training
- `train_policy` - Train AI policies  
- `execute_command` - Natural language control
- `optimize_parameters` - Parameter tuning

### 📊 Visualization
- `render_frame` - Generate images
- `get_camera_image` - Camera views
- `visualize_contacts` - Contact forces

## ❗ Troubleshooting

### Server Won't Start
```bash
# Check dependencies
python -c "import mujoco; print('MuJoCo OK')"
python -c "import mcp; print('MCP OK')"

# Check configuration
python -m mujoco_mcp --check
```

### Connection Issues
1. Ensure Cursor is completely restarted
2. Check the MCP configuration file syntax
3. Verify the project path is correct
4. Check logs in Cursor's developer console

### Permission Issues
```bash
# Make sure the project is properly installed
cd /Users/robert/workspace/04-code/03-mujoco/mujoco-mcp
pip install -e .
```

## 🎯 Expected Results

After successful setup, you should see:
- ✅ MuJoCo MCP server listed in Cursor's MCP connections
- ✅ Ability to create physics simulations through natural language
- ✅ Access to 98 MuJoCo tools and 3 resources
- ✅ Real-time simulation control and visualization

## 📞 Support

If you encounter issues:
1. Check the server logs: `python -m mujoco_mcp --debug`
2. Run integration tests: `python test_mcp_integration.py`
3. Review the configuration guide: `CONFIG.md`

Happy simulating! 🚀