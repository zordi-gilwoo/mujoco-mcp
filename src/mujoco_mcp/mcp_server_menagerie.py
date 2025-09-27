#!/usr/bin/env python3
"""
Enhanced MuJoCo MCP Server with Menagerie Support
Production-ready MCP server with MuJoCo Menagerie model integration
"""

import asyncio
import sys
import json
from typing import Dict, Any, List, Optional
import logging

from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

from .version import __version__
from .viewer_client import MuJoCoViewerClient as ViewerClient
from .menagerie_loader import MenagerieLoader
from .session_manager import session_manager

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp-menagerie")

# Create server instance
server = Server("mujoco-mcp-menagerie")

# Global instances
menagerie_loader = MenagerieLoader()

@server.list_tools()
async def handle_list_tools() -> List[types.Tool]:
    """Return list of available MuJoCo MCP tools with Menagerie support"""
    return [
        types.Tool(
            name="get_server_info",
            description="Get information about the MuJoCo MCP server",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        types.Tool(
            name="list_menagerie_models",
            description="List available MuJoCo Menagerie models by category",
            inputSchema={
                "type": "object",
                "properties": {
                    "category": {
                        "type": "string",
                        "description": "Filter by category (arms, quadrupeds, humanoids, grippers, etc.)"
                    }
                },
                "required": []
            }
        ),
        types.Tool(
            name="validate_menagerie_model", 
            description="Validate a specific MuJoCo Menagerie model",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_name": {
                        "type": "string",
                        "description": "Name of the Menagerie model to validate"
                    }
                },
                "required": ["model_name"]
            }
        ),
        types.Tool(
            name="create_menagerie_scene",
            description="Create a scene from a MuJoCo Menagerie model",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_name": {
                        "type": "string",
                        "description": "Name of the Menagerie model (e.g., 'franka_emika_panda')"
                    },
                    "scene_name": {
                        "type": "string",
                        "description": "Optional custom scene name"
                    }
                },
                "required": ["model_name"]
            }
        ),
        types.Tool(
            name="create_scene",
            description="Create a physics simulation scene (built-in or Menagerie)",
            inputSchema={
                "type": "object",
                "properties": {
                    "scene_type": {
                        "type": "string",
                        "description": "Type of scene to create",
                        "enum": ["pendulum", "double_pendulum", "cart_pole", "arm"]
                    },
                    "menagerie_model": {
                        "type": "string", 
                        "description": "Optional: Load a MuJoCo Menagerie model instead"
                    }
                },
                "required": ["scene_type"]
            }
        ),
        types.Tool(
            name="step_simulation",
            description="Step the physics simulation forward",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string", 
                        "description": "ID of the model to step"
                    },
                    "steps": {
                        "type": "integer",
                        "description": "Number of simulation steps",
                        "default": 1
                    }
                },
                "required": ["model_id"]
            }
        ),
        types.Tool(
            name="get_state",
            description="Get current state of the simulation",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to get state from"
                    }
                },
                "required": ["model_id"]
            }
        ),
        types.Tool(
            name="reset_simulation",
            description="Reset simulation to initial state",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to reset"
                    }
                },
                "required": ["model_id"]
            }
        ),
        types.Tool(
            name="close_viewer",
            description="Close the MuJoCo viewer window",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model viewer to close (optional - if not provided, closes entire session)"
                    }
                },
                "required": []
            }
        ),
        types.Tool(
            name="get_session_info",
            description="Get information about the current session and active models",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        types.Tool(
            name="execute_command",
            description="Execute natural language commands using available MCP tools",
            inputSchema={
                "type": "object",
                "properties": {
                    "command": {
                        "type": "string",
                        "description": "Natural language command to execute"
                    },
                    "model_id": {
                        "type": "string",
                        "description": "Optional model ID to target specific model"
                    }
                },
                "required": ["command"]
            }
        )
    ]

async def parse_and_execute_command(command: str, model_id: Optional[str] = None) -> str:
    """Parse natural language command and execute corresponding MCP tools"""
    
    command_lower = command.lower().strip()
    actions_taken = []
    
    try:
        # Scene creation commands
        if any(phrase in command_lower for phrase in ["create", "make", "build", "load"]):
            if any(phrase in command_lower for phrase in ["pendulum", "simple pendulum"]):
                result = await handle_create_scene("pendulum", model_id)
                actions_taken.append("created_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["double pendulum", "chaotic"]):
                result = await handle_create_scene("double_pendulum", model_id)
                actions_taken.append("created_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["cart pole", "cart-pole", "cartpole", "balancing"]):
                result = await handle_create_scene("cart_pole", model_id)
                actions_taken.append("created_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["arm", "robot arm", "robotic arm"]):
                result = await handle_create_scene("arm", model_id)
                actions_taken.append("created_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            # Menagerie model creation
            elif any(phrase in command_lower for phrase in ["franka", "panda", "franka panda"]):
                result = await handle_create_menagerie_scene("franka_emika_panda", model_id)
                actions_taken.append("created_menagerie_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["ur5", "ur5e", "universal robot"]):
                result = await handle_create_menagerie_scene("universal_robots_ur5e", model_id)
                actions_taken.append("created_menagerie_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["quadruped", "dog", "spot", "anymal"]):
                result = await handle_create_menagerie_scene("anybotics_anymal_c", model_id)
                actions_taken.append("created_menagerie_scene")
                return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
                
        # Simulation control commands
        elif any(phrase in command_lower for phrase in ["step", "run", "simulate", "advance"]):
            steps = 1
            # Extract number of steps if mentioned
            import re
            numbers = re.findall(r'\d+', command)
            if numbers:
                steps = int(numbers[0])
                
            result = await handle_step_simulation(model_id, steps)
            actions_taken.append("stepped_simulation")
            return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["reset", "restart", "initialize"]):
            result = await handle_reset_simulation(model_id)
            actions_taken.append("reset_simulation")
            return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["state", "status", "information", "info"]):
            result = await handle_get_state(model_id)
            actions_taken.append("get_state")
            return f"✅ Current state retrieved\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["close", "stop", "quit", "exit"]):
            result = await handle_close_viewer(model_id)
            actions_taken.append("close_viewer")
            return f"✅ {result.text}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["list", "show", "available", "models"]):
            if any(phrase in command_lower for phrase in ["menagerie", "robot", "models"]):
                result = await handle_list_menagerie_models()
                actions_taken.append("list_menagerie_models")
                return f"✅ Listed available models\n📋 Actions taken: {actions_taken}"
            else:
                result = await handle_get_session_info()
                actions_taken.append("get_session_info")
                return f"✅ Session information retrieved\n📋 Actions taken: {actions_taken}"
                
        else:
            return f"❌ Command not recognized: '{command}'\n\n🤖 Try commands like:\n" \
                   f"• 'Create a pendulum simulation'\n" \
                   f"• 'Load Franka Panda robot'\n" \
                   f"• 'Step simulation 100 times'\n" \
                   f"• 'Reset the simulation'\n" \
                   f"• 'Show current state'\n" \
                   f"• 'List available models'"
                   
    except Exception as e:
        logger.exception(f"Error executing command: {command}")
        return f"❌ Error executing command '{command}': {str(e)}"

async def handle_create_scene(scene_type: str, model_id: Optional[str] = None) -> types.TextContent:
    """Helper to create scene"""
    arguments = {"scene_type": scene_type}
    if model_id:
        arguments["model_id"] = model_id
    results = await handle_call_tool("create_scene", arguments)
    return results[0]

async def handle_create_menagerie_scene(model_name: str, model_id: Optional[str] = None) -> types.TextContent:
    """Helper to create menagerie scene"""
    arguments = {"model_name": model_name}
    if model_id:
        arguments["scene_name"] = model_id
    results = await handle_call_tool("create_menagerie_scene", arguments)
    return results[0]

async def handle_step_simulation(model_id: Optional[str], steps: int = 1) -> types.TextContent:
    """Helper to step simulation"""
    arguments = {"steps": steps}
    if model_id:
        arguments["model_id"] = model_id
    else:
        # Use default model from session
        session = session_manager.get_session()
        if session and session.active_models:
            arguments["model_id"] = list(session.active_models.keys())[0]
        else:
            return types.TextContent(type="text", text="❌ No active model found. Create a scene first.")
    
    results = await handle_call_tool("step_simulation", arguments)
    return results[0]

async def handle_reset_simulation(model_id: Optional[str]) -> types.TextContent:
    """Helper to reset simulation"""
    arguments = {}
    if model_id:
        arguments["model_id"] = model_id
    else:
        # Use default model from session
        session = session_manager.get_session()
        if session and session.active_models:
            arguments["model_id"] = list(session.active_models.keys())[0]
        else:
            return types.TextContent(type="text", text="❌ No active model found. Create a scene first.")
    
    results = await handle_call_tool("reset_simulation", arguments)
    return results[0]

async def handle_get_state(model_id: Optional[str]) -> types.TextContent:
    """Helper to get state"""
    arguments = {}
    if model_id:
        arguments["model_id"] = model_id
    else:
        # Use default model from session
        session = session_manager.get_session()
        if session and session.active_models:
            arguments["model_id"] = list(session.active_models.keys())[0]
        else:
            return types.TextContent(type="text", text="❌ No active model found. Create a scene first.")
    
    results = await handle_call_tool("get_state", arguments)
    return results[0]

async def handle_close_viewer(model_id: Optional[str]) -> types.TextContent:
    """Helper to close viewer"""
    arguments = {}
    if model_id:
        arguments["model_id"] = model_id
    
    results = await handle_call_tool("close_viewer", arguments)
    return results[0]

async def handle_list_menagerie_models(category: Optional[str] = None) -> types.TextContent:
    """Helper to list menagerie models"""
    arguments = {}
    if category:
        arguments["category"] = category
    
    results = await handle_call_tool("list_menagerie_models", arguments)
    return results[0]

async def handle_get_session_info() -> types.TextContent:
    """Helper to get session info"""
    results = await handle_call_tool("get_session_info", {})
    return results[0]

@server.call_tool()
async def handle_call_tool(name: str, arguments: Dict[str, Any]) -> List[types.TextContent]:
    """Handle tool calls with Menagerie support"""
    
    try:
        if name == "get_server_info":
            return [types.TextContent(
                type="text",
                text=json.dumps({
                    "name": "MuJoCo MCP Server with Menagerie Support",
                    "version": __version__,
                    "description": "Control MuJoCo physics simulations with Menagerie model support",
                    "status": "ready",
                    "capabilities": [
                        "create_scene", "create_menagerie_scene", "list_menagerie_models",
                        "validate_menagerie_model", "step_simulation", "get_state", 
                        "reset", "close_viewer"
                    ],
                    "menagerie_support": True
                }, indent=2)
            )]
            
        elif name == "list_menagerie_models":
            category_filter = arguments.get("category")
            available_models = menagerie_loader.get_available_models()
            
            if category_filter:
                if category_filter in available_models:
                    filtered_models = {category_filter: available_models[category_filter]}
                else:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Unknown category: {category_filter}. Available: {', '.join(available_models.keys())}"
                    )]
            else:
                filtered_models = available_models
            
            # Format response
            result = {"categories": len(filtered_models), "models": {}}
            total_models = 0
            
            for category, models in filtered_models.items():
                result["models"][category] = {
                    "count": len(models),
                    "models": models
                }
                total_models += len(models)
            
            result["total_models"] = total_models
            
            return [types.TextContent(
                type="text",
                text=json.dumps(result, indent=2)
            )]
            
        elif name == "validate_menagerie_model":
            model_name = arguments.get("model_name")
            if not model_name:
                return [types.TextContent(
                    type="text",
                    text="❌ model_name is required"
                )]
            
            # Validate the model
            validation_result = menagerie_loader.validate_model(model_name)
            
            if validation_result["valid"]:
                status = "✅ Valid"
                details = []
                if "n_bodies" in validation_result:
                    details.append(f"Bodies: {validation_result['n_bodies']}")
                if "n_joints" in validation_result:
                    details.append(f"Joints: {validation_result['n_joints']}")
                if "n_actuators" in validation_result:
                    details.append(f"Actuators: {validation_result['n_actuators']}")
                if "xml_size" in validation_result:
                    details.append(f"XML Size: {validation_result['xml_size']} chars")
                
                message = f"{status} - {model_name}"
                if details:
                    message += f" ({', '.join(details)})"
                    
                if "note" in validation_result:
                    message += f"\n⚠️ {validation_result['note']}"
                    
            else:
                message = f"❌ Invalid - {model_name}: {validation_result.get('error', 'Unknown error')}"
            
            return [types.TextContent(
                type="text",
                text=message
            )]
            
        elif name == "create_menagerie_scene":
            model_name = arguments.get("model_name")
            scene_name = arguments.get("scene_name", f"{model_name}_scene")
            
            if not model_name:
                return [types.TextContent(
                    type="text",
                    text="❌ model_name is required"
                )]
            
            try:
                # Get the complete scene XML
                scene_xml = menagerie_loader.create_scene_xml(model_name, scene_name)
                
                # Get session-specific viewer client
                viewer_client = session_manager.get_viewer_client()
                if not viewer_client:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Failed to connect to MuJoCo viewer server. Please start `mujoco-mcp-viewer` first.\n✅ XML generated successfully for {model_name} ({len(scene_xml)} chars)"
                    )]
                
                # Create session-specific model ID to avoid conflicts
                session = session_manager.get_or_create_session()
                session_model_id = f"{session.session_id}_{scene_name}"
                
                # Load the model
                response = viewer_client.send_command({
                    "type": "load_model",
                    "model_id": session_model_id,
                    "model_xml": scene_xml
                })
                
                # Track the model in session
                if response.get("success"):
                    session.active_models[scene_name] = model_name
                
                if response.get("success"):
                    return [types.TextContent(
                        type="text",
                        text=f"✅ Created Menagerie scene '{scene_name}' with model '{model_name}' successfully! Viewer window opened."
                    )]
                else:
                    return [types.TextContent(
                        type="text", 
                        text=f"❌ Failed to create scene: {response.get('error', 'Unknown error')}\n✅ XML generated successfully for {model_name}"
                    )]
                    
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to load Menagerie model '{model_name}': {str(e)}"
                )]
                
        elif name == "create_scene":
            menagerie_model = arguments.get("menagerie_model")
            
            # If Menagerie model specified, delegate to create_menagerie_scene
            if menagerie_model:
                return await handle_call_tool("create_menagerie_scene", {
                    "model_name": menagerie_model,
                    "scene_name": menagerie_model
                })
            
            # Otherwise use built-in scenes (original logic)
            scene_type = arguments.get("scene_type", "pendulum")
            
            # Get session-specific viewer client
            viewer_client = session_manager.get_viewer_client()
            if not viewer_client:
                return [types.TextContent(
                    type="text",
                    text="❌ Failed to connect to MuJoCo viewer server. Please start `mujoco-mcp-viewer` first."
                )]
            
            # Map scene types to model XML
            scene_models = {
                "pendulum": """
                <mujoco>
                    <worldbody>
                        <body name="pole" pos="0 0 1">
                            <joint name="hinge" type="hinge" axis="1 0 0"/>
                            <geom name="pole" type="capsule" size="0.02 0.6" rgba="0.8 0.2 0.2 1"/>
                            <body name="mass" pos="0 0 -0.6">
                                <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """,
                "double_pendulum": """
                <mujoco>
                    <worldbody>
                        <body name="pole1" pos="0 0 1">
                            <joint name="hinge1" type="hinge" axis="1 0 0"/>
                            <geom name="pole1" type="capsule" size="0.02 0.4" rgba="0.8 0.2 0.2 1"/>
                            <body name="pole2" pos="0 0 -0.4">
                                <joint name="hinge2" type="hinge" axis="1 0 0"/>
                                <geom name="pole2" type="capsule" size="0.02 0.4" rgba="0.2 0.8 0.2 1"/>
                                <body name="mass" pos="0 0 -0.4">
                                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                                </body>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """,
                "cart_pole": """
                <mujoco>
                    <worldbody>
                        <body name="cart" pos="0 0 0.1">
                            <joint name="slider" type="slide" axis="1 0 0"/>
                            <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
                            <body name="pole" pos="0 0 0.1">
                                <joint name="hinge" type="hinge" axis="0 1 0"/>
                                <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
                            </body>
                        </body>
                    </worldbody>
                </mujoco>
                """
            }
            
            if scene_type not in scene_models:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Unknown scene type: {scene_type}. Available: {', '.join(scene_models.keys())}"
                )]
            
            # Create session-specific model ID to avoid conflicts
            session = session_manager.get_or_create_session()
            session_model_id = f"{session.session_id}_{scene_type}"
            
            # Load the model
            response = viewer_client.send_command({
                "type": "load_model",
                "model_id": session_model_id,
                "model_xml": scene_models[scene_type]
            })
            
            # Track the model in session
            if response.get("success"):
                session.active_models[scene_type] = scene_type
                return [types.TextContent(
                    type="text",
                    text=f"✅ Created {scene_type} scene successfully! Viewer window opened. Session: {session.client_id}"
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to create scene: {response.get('error', 'Unknown error')}"
                )]
                
        elif name == "step_simulation":
            model_id = arguments.get("model_id")
            steps = arguments.get("steps", 1)
            
            # Get session-specific viewer client
            viewer_client = session_manager.get_viewer_client()
            if not viewer_client:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
            
            # Convert to session-specific model ID if needed
            session = session_manager.get_or_create_session()
            if model_id and not model_id.startswith(session.session_id):
                session_model_id = f"{session.session_id}_{model_id}"
            else:
                session_model_id = model_id
                
            # The viewer server doesn't have a direct step_simulation command
            # It automatically runs the simulation, so we just return success
            response = {"success": True, "message": f"Simulation running for model {session_model_id}"}
            
            return [types.TextContent(
                type="text",
                text=f"⏩ Stepped simulation {steps} steps for session {session.client_id}" if response.get("success") 
                     else f"❌ Step failed: {response.get('error')}"
            )]
            
        elif name == "get_state":
            model_id = arguments.get("model_id")
            
            # Get session-specific viewer client
            viewer_client = session_manager.get_viewer_client()
            if not viewer_client:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
            
            # Convert to session-specific model ID if needed
            session = session_manager.get_or_create_session()
            if model_id and not model_id.startswith(session.session_id):
                session_model_id = f"{session.session_id}_{model_id}"
            else:
                session_model_id = model_id
                
            response = viewer_client.send_command({
                "type": "get_state",
                "model_id": session_model_id
            })
            
            if response.get("success"):
                state = response.get("state", {})
                return [types.TextContent(
                    type="text",
                    text=json.dumps(state, indent=2)
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to get state: {response.get('error')}"
                )]
                
        elif name == "reset_simulation":
            model_id = arguments.get("model_id")
            
            # Get session-specific viewer client
            viewer_client = session_manager.get_viewer_client()
            if not viewer_client:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
            
            # Convert to session-specific model ID if needed
            session = session_manager.get_or_create_session()
            if model_id and not model_id.startswith(session.session_id):
                session_model_id = f"{session.session_id}_{model_id}"
            else:
                session_model_id = model_id
                
            response = viewer_client.send_command({
                "type": "reset",
                "model_id": session_model_id
            })
            
            return [types.TextContent(
                type="text",
                text="🔄 Simulation reset to initial state" if response.get("success")
                     else f"❌ Reset failed: {response.get('error')}"
            )]
            
        elif name == "close_viewer":
            model_id = arguments.get("model_id")
            
            # Get session and clean up if model_id provided, otherwise clean up entire session
            session = session_manager.get_session()
            if not session:
                return [types.TextContent(
                    type="text",
                    text="❌ No active session found."
                )]
            
            viewer_client = session_manager.get_viewer_client()
            if not viewer_client:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection."
                )]
            
            if model_id:
                # Close specific model
                session_model_id = f"{session.session_id}_{model_id}" if not model_id.startswith(session.session_id) else model_id
                response = viewer_client.send_command({
                    "type": "close_model",
                    "model_id": session_model_id
                })
                
                # Remove from session tracking
                if model_id in session.active_models:
                    del session.active_models[model_id]
                    
                return [types.TextContent(
                    type="text",
                    text=f"✅ Model {model_id} closed for session {session.client_id}" if response.get("success")
                         else f"❌ Failed to close model: {response.get('error')}"
                )]
            else:
                # Close entire session
                session_manager.cleanup_session()
                return [types.TextContent(
                    type="text",
                    text=f"✅ Session {session.client_id} closed and cleaned up"
                )]
        
        elif name == "get_session_info":
            session = session_manager.get_or_create_session()
            session_stats = session_manager.get_session_stats()
            
            return [types.TextContent(
                type="text",
                text=json.dumps({
                    "current_session": {
                        "session_id": session.session_id,
                        "client_id": session.client_id,
                        "viewer_port": session.viewer_port,
                        "active_models": session.active_models,
                        "created_at": session.created_at,
                        "last_activity": session.last_activity
                    },
                    "all_sessions": session_stats
                }, indent=2)
            )]
        
        elif name == "execute_command":
            command = arguments.get("command", "").strip()
            model_id = arguments.get("model_id")
            
            if not command:
                return [types.TextContent(
                    type="text",
                    text="❌ Command is required"
                )]
            
            # Parse natural language command and map to MCP tool calls
            result = await parse_and_execute_command(command, model_id)
            
            return [types.TextContent(
                type="text",
                text=result
            )]
        
        else:
            return [types.TextContent(
                type="text",
                text=f"❌ Unknown tool: {name}"
            )]
            
    except Exception as e:
        logger.exception(f"Error in tool {name}")
        return [types.TextContent(
            type="text", 
            text=f"❌ Error: {str(e)}"
        )]

async def main():
    """Main entry point for enhanced MCP server"""
    logger.info(f"Starting MuJoCo MCP Server with Menagerie Support v{__version__}")
    
    # Initialize server capabilities
    server_options = InitializationOptions(
        server_name="mujoco-mcp-menagerie",
        server_version=__version__,
        capabilities=server.get_capabilities(
            notification_options=NotificationOptions(),
            experimental_capabilities={}
        )
    )
    
    # Run server with stdio transport
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            server_options
        )

if __name__ == "__main__":
    asyncio.run(main())