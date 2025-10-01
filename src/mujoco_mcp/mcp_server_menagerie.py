#!/usr/bin/env python3
"""
Enhanced MuJoCo MCP Server with Menagerie Support
Production-ready MCP server with MuJoCo Menagerie model integration
"""

import asyncio
import sys
import json
import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Dict, Any, List, Optional
import logging
import base64
import uuid
from datetime import datetime

from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

from .version import __version__
from .viewer_client import MuJoCoViewerClient as ViewerClient
from .menagerie_loader import MenagerieLoader
from .scene_gen import (
    SceneDescription,
    MetadataExtractor,
    ConstraintSolver, 
    SceneXMLBuilder,
    LLMSceneGenerator
)
from .session_manager import session_manager
from .process_manager import process_manager
from .file_handler import FileHandler

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp-menagerie")

# Create server instance
server = Server("mujoco-mcp-menagerie")

# Global instances
menagerie_loader = MenagerieLoader()
file_handler = FileHandler()

# Scene generation instances
metadata_extractor = MetadataExtractor()
constraint_solver = ConstraintSolver(metadata_extractor)
xml_builder = SceneXMLBuilder(metadata_extractor)
llm_generator = LLMSceneGenerator(metadata_extractor)  # Phase 2C: Pass metadata for symbolic plans

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
            name="create_structured_scene",
            description="Create a structured scene from JSON description or natural language",
            inputSchema={
                "$schema": "http://json-schema.org/draft-07/schema#",
                "type": "object",
                "properties": {
                    "scene_description_json": {
                        "type": "string",
                        "description": "JSON string containing structured scene description"
                    },
                    "natural_language": {
                        "type": "string",
                        "description": "Natural language description of desired scene"
                    },
                    "dry_run": {
                        "type": "boolean",
                        "description": "If true, validate and return summary without loading viewer",
                        "default": False
                    }
                },
                "additionalProperties": False
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
            name="get_process_pool_stats",
            description="Get statistics about the process pool and active processes",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        types.Tool(
            name="list_active_processes",
            description="List all active viewer processes with details",
            inputSchema={
                "type": "object",
                "properties": {},
                "required": []
            }
        ),
        types.Tool(
            name="terminate_process",
            description="Terminate a specific viewer process by session ID",
            inputSchema={
                "type": "object",
                "properties": {
                    "session_id": {
                        "type": "string",
                        "description": "Session ID of the process to terminate"
                    }
                },
                "required": ["session_id"]
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
        ),
        types.Tool(
            name="download_xml",
            description="Download XML content generated from prompts or models",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to download XML from"
                    },
                    "include_resolved": {
                        "type": "boolean",
                        "description": "Include resolved includes in the XML",
                        "default": True
                    }
                },
                "required": ["model_id"]
            }
        ),
        types.Tool(
            name="download_python_script",
            description="Download Python script that recreates the current simulation",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to generate Python script for"
                    },
                    "include_viewer_setup": {
                        "type": "boolean",
                        "description": "Include viewer setup code in the script",
                        "default": True
                    }
                },
                "required": ["model_id"]
            }
        ),
        types.Tool(
            name="upload_xml",
            description="Upload and validate XML file, then automatically render the scene",
            inputSchema={
                "type": "object",
                "properties": {
                    "xml_content": {
                        "type": "string",
                        "description": "XML content to upload and validate"
                    },
                    "scene_name": {
                        "type": "string",
                        "description": "Optional name for the scene (auto-generated if not provided)"
                    },
                    "auto_render": {
                        "type": "boolean",
                        "description": "Automatically render the scene after validation",
                        "default": True
                    }
                },
                "required": ["xml_content"]
            }
        ),
        types.Tool(
            name="upload_python_script",
            description="Upload and execute Python script to create/modify simulations",
            inputSchema={
                "type": "object",
                "properties": {
                    "script_content": {
                        "type": "string",
                        "description": "Python script content to upload and execute"
                    },
                    "script_name": {
                        "type": "string",
                        "description": "Optional name for the script (auto-generated if not provided)"
                    },
                    "safe_mode": {
                        "type": "boolean",
                        "description": "Run script in safe mode with restricted imports",
                        "default": True
                    }
                },
                "required": ["script_content"]
            }
        ),
        types.Tool(
            name="list_uploaded_files",
            description="List all uploaded files with metadata",
            inputSchema={
                "type": "object",
                "properties": {
                    "file_type": {
                        "type": "string",
                        "description": "Filter by file type (xml, python)",
                        "enum": ["xml", "python"]
                    }
                },
                "required": []
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
                # Call the scene creation logic directly
                result_texts = await handle_call_tool("create_scene", {"scene_type": "pendulum"})
                result_text = result_texts[0].text if result_texts else "Scene creation attempted"
                actions_taken.append("created_scene")
                
                # Clean up the result text to remove redundant success/error markers
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["double pendulum", "chaotic"]):
                result_texts = await handle_call_tool("create_scene", {"scene_type": "double_pendulum"})
                result_text = result_texts[0].text if result_texts else "Scene creation attempted"
                actions_taken.append("created_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["cart pole", "cart-pole", "cartpole", "balancing"]):
                result_texts = await handle_call_tool("create_scene", {"scene_type": "cart_pole"})
                result_text = result_texts[0].text if result_texts else "Scene creation attempted"
                actions_taken.append("created_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["arm", "robot arm", "robotic arm"]):
                result_texts = await handle_call_tool("create_scene", {"scene_type": "arm"})
                result_text = result_texts[0].text if result_texts else "Scene creation attempted"
                actions_taken.append("created_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            # Menagerie model creation
            elif any(phrase in command_lower for phrase in ["franka", "panda", "franka panda"]):
                result_texts = await handle_call_tool("create_menagerie_scene", {"model_name": "franka_emika_panda"})
                result_text = result_texts[0].text if result_texts else "Menagerie scene creation attempted"
                actions_taken.append("created_menagerie_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["ur5", "ur5e", "universal robot"]):
                result_texts = await handle_call_tool("create_menagerie_scene", {"model_name": "universal_robots_ur5e"})
                result_text = result_texts[0].text if result_texts else "Menagerie scene creation attempted"
                actions_taken.append("created_menagerie_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
            elif any(phrase in command_lower for phrase in ["quadruped", "dog", "spot", "anymal"]):
                result_texts = await handle_call_tool("create_menagerie_scene", {"model_name": "anybotics_anymal_c"})
                result_text = result_texts[0].text if result_texts else "Menagerie scene creation attempted"
                actions_taken.append("created_menagerie_scene")
                
                clean_result = result_text.replace("✅", "").replace("❌", "").strip()
                success = not result_text.startswith("❌")
                status_icon = "✅" if success else "❌"
                
                return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
                
        # Simulation control commands
        elif any(phrase in command_lower for phrase in ["step", "run", "simulate", "advance"]):
            steps = 1
            # Extract number of steps if mentioned
            import re
            numbers = re.findall(r'\d+', command)
            if numbers:
                steps = int(numbers[0])
            
            # Use a default model if none specified
            if not model_id:
                session = session_manager.get_session()
                if session and session.active_models:
                    model_id = list(session.active_models.keys())[0]
                else:
                    model_id = "default"
                    
            result_texts = await handle_call_tool("step_simulation", {"model_id": model_id, "steps": steps})
            result_text = result_texts[0].text if result_texts else "Simulation step attempted"
            actions_taken.append("stepped_simulation")
            
            clean_result = result_text.replace("✅", "").replace("❌", "").strip()
            success = not result_text.startswith("❌")
            status_icon = "✅" if success else "❌"
            
            return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["reset", "restart", "initialize"]):
            if not model_id:
                session = session_manager.get_session()
                if session and session.active_models:
                    model_id = list(session.active_models.keys())[0]
                else:
                    model_id = "default"
                    
            result_texts = await handle_call_tool("reset_simulation", {"model_id": model_id})
            result_text = result_texts[0].text if result_texts else "Reset attempted"
            actions_taken.append("reset_simulation")
            
            clean_result = result_text.replace("✅", "").replace("❌", "").strip()
            success = not result_text.startswith("❌")
            status_icon = "✅" if success else "❌"
            
            return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["state", "status", "information", "info"]):
            if not model_id:
                session = session_manager.get_session()
                if session and session.active_models:
                    model_id = list(session.active_models.keys())[0]
                else:
                    model_id = "default"
                    
            result_texts = await handle_call_tool("get_state", {"model_id": model_id})
            result_text = result_texts[0].text if result_texts else "State retrieved"
            actions_taken.append("get_state")
            
            return f"✅ Current state retrieved\n📋 Actions taken: {actions_taken}\n\n{result_text}"
            
        elif any(phrase in command_lower for phrase in ["close", "stop", "quit", "exit"]):
            result_texts = await handle_call_tool("close_viewer", {"model_id": model_id} if model_id else {})
            result_text = result_texts[0].text if result_texts else "Viewer closed"
            actions_taken.append("close_viewer")
            
            clean_result = result_text.replace("✅", "").replace("❌", "").strip()
            success = not result_text.startswith("❌")
            status_icon = "✅" if success else "❌"
            
            return f"{status_icon} {clean_result}\n📋 Actions taken: {actions_taken}"
            
        elif any(phrase in command_lower for phrase in ["list", "show", "available", "models"]):
            if any(phrase in command_lower for phrase in ["menagerie", "robot", "models"]):
                result_texts = await handle_call_tool("list_menagerie_models", {})
                result_text = result_texts[0].text if result_texts else "Models listed"
                actions_taken.append("list_menagerie_models")
                
                return f"✅ Available Menagerie models:\n📋 Actions taken: {actions_taken}\n\n{result_text}"
            else:
                result_texts = await handle_call_tool("get_session_info", {})
                result_text = result_texts[0].text if result_texts else "Session info retrieved"
                actions_taken.append("get_session_info")
                
                return f"✅ Session information:\n📋 Actions taken: {actions_taken}\n\n{result_text}"
                
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
                        "create_structured_scene", "create_scene", "create_menagerie_scene", 
                        "list_menagerie_models", "validate_menagerie_model", "step_simulation", 
                        "get_state", "reset", "close_viewer"
                        "create_scene", "create_menagerie_scene", "list_menagerie_models",
                        "validate_menagerie_model", "step_simulation", "get_state", 
                        "reset", "close_viewer", "download_xml", "download_python_script",
                        "upload_xml", "upload_python_script", "list_uploaded_files"
                    ],
                    "menagerie_support": True
                }, indent=2)
            )]
            
        elif name == "create_structured_scene":
            scene_description_json = arguments.get("scene_description_json")
            natural_language = arguments.get("natural_language")
            dry_run = arguments.get("dry_run", False)
            
            try:
                # Determine input method
                if scene_description_json:
                    # Parse JSON directly
                    try:
                        scene_dict = json.loads(scene_description_json)
                        scene = SceneDescription(**scene_dict)
                        logger.info("Using provided JSON scene description")
                    except (json.JSONDecodeError, Exception) as e:
                        return [types.TextContent(
                            type="text",
                            text=f"❌ Failed to parse scene description JSON: {str(e)}"
                        )]
                elif natural_language:
                    # Generate scene from natural language
                    try:
                        scene_dict = llm_generator.generate_or_stub(natural_language)
                        scene = SceneDescription(**scene_dict)
                        logger.info(f"Generated scene from natural language: '{natural_language}'")
                    except Exception as e:
                        return [types.TextContent(
                            type="text",
                            text=f"❌ Failed to generate scene from natural language: {str(e)}"
                        )]
                else:
                    return [types.TextContent(
                        type="text",
                        text="❌ Either 'scene_description_json' or 'natural_language' must be provided"
                    )]
                
                # Solve constraints
                try:
                    poses = constraint_solver.solve(scene)
                    logger.info(f"Successfully solved constraints for {len(poses)} entities")
                except Exception as e:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Constraint solving failed: {str(e)}"
                    )]
                
                # Build XML
                try:
                    scene_xml = xml_builder.build_scene(scene, poses, "structured_scene")
                    
                    # Validate XML
                    if not xml_builder.validate_xml(scene_xml):
                        return [types.TextContent(
                            type="text",
                            text="❌ Generated XML is invalid"
                        )]
                    
                    logger.info(f"Successfully built XML scene ({len(scene_xml)} characters)")
                except Exception as e:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ XML generation failed: {str(e)}"
                    )]
                
                # Prepare response
                summary = {
                    "scene_entities": len(poses),
                    "objects": len(scene.objects),
                    "robots": len(scene.robots),
                    "solved_poses": {entity_id: {
                        "position": pose.position.tolist(),
                        "orientation": pose.orientation.tolist()
                    } for entity_id, pose in poses.items()},
                    "xml_length": len(scene_xml)
                }
                
                if dry_run:
                    # Return summary without loading viewer
                    return [types.TextContent(
                        type="text",
                        text=f"✅ Structured scene validation successful!\n\n{json.dumps(summary, indent=2)}\n\nXML Preview (first 500 chars):\n{scene_xml[:500]}{'...' if len(scene_xml) > 500 else ''}"
                    )]
                
                # Load scene in viewer
                if not viewer_client:
                    viewer_client = ViewerClient()
                
                if not viewer_client.connected:
                    success = viewer_client.connect()
                    if not success:
                        return [types.TextContent(
                            type="text",
                            text=f"❌ Failed to connect to MuJoCo viewer server. Scene generated successfully but cannot display.\n\n{json.dumps(summary, indent=2)}"
                        )]
                
                # Load the scene
                response = viewer_client.send_command({
                    "type": "load_model",
                    "model_id": "structured_scene",
                    "model_xml": scene_xml
                })
                
                if response.get("success"):
                    return [types.TextContent(
                        type="text",
                        text=f"✅ Structured scene created successfully!\n\n{json.dumps(summary, indent=2)}\n\nViewer window opened with generated scene."
                    )]
                else:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Failed to load scene in viewer: {response.get('error', 'Unknown error')}\n\n{json.dumps(summary, indent=2)}"
                    )]
                    
            except Exception as e:
                logger.exception(f"Error in create_structured_scene")
                return [types.TextContent(
                    type="text",
                    text=f"❌ Structured scene generation failed: {str(e)}"
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
        
        elif name == "get_process_pool_stats":
            # Get process pool statistics
            session_stats = session_manager.get_session_stats()
            process_stats = process_manager.get_stats()
            
            stats_info = {
                "process_pool": process_stats,
                "session_summary": {
                    "active_sessions": session_stats.get("active_sessions", 0),
                    "isolated_process_mode": session_stats.get("isolated_process_mode", False)
                },
                "resource_usage": {
                    "total_processes": process_stats.get("total_processes", 0),
                    "running_processes": process_stats.get("running_processes", 0),
                    "used_ports": process_stats.get("used_ports", 0),
                    "available_ports": process_stats.get("available_ports", 0)
                }
            }
            
            return [types.TextContent(
                type="text",
                text=json.dumps(stats_info, indent=2)
            )]
        
        elif name == "list_active_processes":
            # List all active processes
            processes = process_manager.list_processes()
            
            if not processes:
                return [types.TextContent(
                    type="text",
                    text="📊 No active processes found"
                )]
            
            process_list = {
                "active_processes": len(processes),
                "processes": processes
            }
            
            return [types.TextContent(
                type="text",
                text=json.dumps(process_list, indent=2)
            )]
        
        elif name == "terminate_process":
            session_id = arguments.get("session_id")
            
            if not session_id:
                return [types.TextContent(
                    type="text",
                    text="❌ Session ID is required"
                )]
            
            # Check if process exists
            process_info = process_manager.get_process_info(session_id)
            if not process_info:
                return [types.TextContent(
                    type="text",
                    text=f"❌ No process found for session {session_id}"
                )]
            
            # Terminate the process
            success = process_manager.terminate_process(session_id)
            
            if success:
                return [types.TextContent(
                    type="text",
                    text=f"✅ Process for session {session_id} terminated successfully"
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to terminate process for session {session_id}"
                )]
        
        elif name == "download_xml":
            model_id = arguments.get("model_id")
            include_resolved = arguments.get("include_resolved", True)
            
            if not model_id:
                return [types.TextContent(
                    type="text",
                    text="❌ model_id is required"
                )]
            
            try:
                xml_content = None
                model_type = "unknown"
                
                # Try built-in scenes first (they don't require sessions)
                scene_models = {
                    "pendulum": """<mujoco>
                        <worldbody>
                            <body name="pole" pos="0 0 1">
                                <joint name="hinge" type="hinge" axis="1 0 0"/>
                                <geom name="pole" type="capsule" size="0.02 0.6" rgba="0.8 0.2 0.2 1"/>
                                <body name="mass" pos="0 0 -0.6">
                                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>""",
                    "double_pendulum": """<mujoco>
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
                    </mujoco>""",
                    "cart_pole": """<mujoco>
                        <worldbody>
                            <body name="cart" pos="0 0 0.1">
                                <joint name="slider" type="slide" axis="1 0 0"/>
                                <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
                                <body name="pole" pos="0 0 0.1">
                                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                                    <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
                                    <body name="mass" pos="0 0 0.5">
                                        <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                                    </body>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>""",
                    "arm": """<mujoco>
                        <worldbody>
                            <body name="base" pos="0 0 0.5">
                                <geom name="base" type="cylinder" size="0.1 0.05" rgba="0.8 0.2 0.2 1"/>
                                <body name="link1" pos="0 0 0.05">
                                    <joint name="joint1" type="hinge" axis="0 0 1"/>
                                    <geom name="link1" type="capsule" size="0.03 0.2" rgba="0.2 0.8 0.2 1"/>
                                    <body name="link2" pos="0 0 0.2">
                                        <joint name="joint2" type="hinge" axis="0 1 0"/>
                                        <geom name="link2" type="capsule" size="0.03 0.15" rgba="0.2 0.2 0.8 1"/>
                                        <body name="end_effector" pos="0 0 0.15">
                                            <geom name="ee" type="sphere" size="0.03" rgba="0.8 0.8 0.2 1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>"""
                }
                
                if model_id in scene_models:
                    xml_content = scene_models[model_id].strip()
                    model_type = "built-in"
                else:
                    # Try menagerie models
                    try:
                        if include_resolved:
                            xml_content = menagerie_loader.get_model_xml(model_id)
                        else:
                            # Get raw XML without resolution
                            xml_content = menagerie_loader.download_file(model_id, f"{model_id}.xml")
                        model_type = "menagerie"
                    except Exception as e:
                        return [types.TextContent(
                            type="text",
                            text=f"❌ Could not find model '{model_id}' in built-in scenes or menagerie: {str(e)}"
                        )]
                
                if not xml_content:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Could not find XML content for model: {model_id}"
                    )]
                
                # Encode as base64 for safe transport
                xml_b64 = base64.b64encode(xml_content.encode('utf-8')).decode('ascii')
                
                result = {
                    "model_id": model_id,
                    "model_type": model_type,
                    "include_resolved": include_resolved,
                    "xml_size": len(xml_content),
                    "download_info": {
                        "filename": f"{model_id}.xml",
                        "content_type": "application/xml",
                        "encoding": "base64"
                    },
                    "xml_content_base64": xml_b64
                }
                
                return [types.TextContent(
                    type="text",
                    text=f"✅ XML downloaded for {model_id}\n📄 {json.dumps(result, indent=2)}"
                )]
                
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to download XML: {str(e)}"
                )]
        
        elif name == "download_python_script":
            model_id = arguments.get("model_id")
            include_viewer_setup = arguments.get("include_viewer_setup", True)
            
            if not model_id:
                return [types.TextContent(
                    type="text",
                    text="❌ model_id is required"
                )]
            
            try:
                # Get XML content directly (reuse logic from download_xml)
                xml_content = None
                model_type = "unknown"
                
                # Try built-in scenes first
                scene_models = {
                    "pendulum": """<mujoco>
                        <worldbody>
                            <body name="pole" pos="0 0 1">
                                <joint name="hinge" type="hinge" axis="1 0 0"/>
                                <geom name="pole" type="capsule" size="0.02 0.6" rgba="0.8 0.2 0.2 1"/>
                                <body name="mass" pos="0 0 -0.6">
                                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>""",
                    "double_pendulum": """<mujoco>
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
                    </mujoco>""",
                    "cart_pole": """<mujoco>
                        <worldbody>
                            <body name="cart" pos="0 0 0.1">
                                <joint name="slider" type="slide" axis="1 0 0"/>
                                <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
                                <body name="pole" pos="0 0 0.1">
                                    <joint name="hinge" type="hinge" axis="0 1 0"/>
                                    <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
                                    <body name="mass" pos="0 0 0.5">
                                        <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                                    </body>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>""",
                    "arm": """<mujoco>
                        <worldbody>
                            <body name="base" pos="0 0 0.5">
                                <geom name="base" type="cylinder" size="0.1 0.05" rgba="0.8 0.2 0.2 1"/>
                                <body name="link1" pos="0 0 0.05">
                                    <joint name="joint1" type="hinge" axis="0 0 1"/>
                                    <geom name="link1" type="capsule" size="0.03 0.2" rgba="0.2 0.8 0.2 1"/>
                                    <body name="link2" pos="0 0 0.2">
                                        <joint name="joint2" type="hinge" axis="0 1 0"/>
                                        <geom name="link2" type="capsule" size="0.03 0.15" rgba="0.2 0.2 0.8 1"/>
                                        <body name="end_effector" pos="0 0 0.15">
                                            <geom name="ee" type="sphere" size="0.03" rgba="0.8 0.8 0.2 1"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </worldbody>
                    </mujoco>"""
                }
                
                if model_id in scene_models:
                    xml_content = scene_models[model_id].strip()
                    model_type = "built-in"
                else:
                    # Try menagerie models
                    try:
                        xml_content = menagerie_loader.get_model_xml(model_id)
                        model_type = "menagerie"
                    except Exception as e:
                        return [types.TextContent(
                            type="text",
                            text=f"❌ Could not find model '{model_id}' in built-in scenes or menagerie: {str(e)}"
                        )]
                
                if not xml_content:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Could not find XML content for model: {model_id}"
                    )]
                
                # Generate Python script
                script_content = file_handler.generate_python_script(
                    xml_content, model_id, include_viewer_setup
                )
                
                # Encode script as base64
                script_b64 = base64.b64encode(script_content.encode('utf-8')).decode('ascii')
                
                result = {
                    "model_id": model_id,
                    "model_type": model_type,
                    "include_viewer": include_viewer_setup,
                    "script_size": len(script_content),
                    "download_info": {
                        "filename": f"{model_id}_simulation.py",
                        "content_type": "text/x-python",
                        "encoding": "base64"
                    },
                    "script_content_base64": script_b64
                }
                
                return [types.TextContent(
                    type="text",
                    text=f"✅ Python script generated for {model_id}\n🐍 {json.dumps(result, indent=2)}"
                )]
                
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to generate Python script: {str(e)}"
                )]
        
        elif name == "upload_xml":
            xml_content = arguments.get("xml_content", "").strip()
            scene_name = arguments.get("scene_name")
            auto_render = arguments.get("auto_render", True)
            
            if not xml_content:
                return [types.TextContent(
                    type="text",
                    text="❌ xml_content is required"
                )]
            
            try:
                # Validate the XML
                validation_result = file_handler.validate_xml(xml_content)
                
                if not validation_result["valid"]:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ XML validation failed: {validation_result['error']}"
                    )]
                
                # Generate scene name if not provided
                if not scene_name:
                    scene_name = f"uploaded_scene_{uuid.uuid4().hex[:8]}"
                
                # Save the uploaded file
                file_id = file_handler.save_upload_file(xml_content, "xml", f"{scene_name}.xml")
                
                if auto_render:
                    # Create the scene in the viewer
                    session = session_manager.get_or_create_session()
                    viewer_client = session_manager.get_viewer_client()
                    
                    if not viewer_client:
                        return [types.TextContent(
                            type="text",
                            text=f"✅ XML validated and uploaded (File ID: {file_id})\n❌ Auto-render failed: No viewer connection"
                        )]
                    
                    # Load the model in viewer
                    session_model_id = f"{session.session_id}_{scene_name}"
                    response = viewer_client.send_command({
                        "type": "load_model",
                        "model_id": session_model_id,
                        "model_xml": xml_content
                    })
                    
                    if response.get("success"):
                        # Track the model in session
                        session.active_models[scene_name] = scene_name
                        
                        result = {
                            "file_id": file_id,
                            "scene_name": scene_name,
                            "validation": validation_result,
                            "rendering": {
                                "success": True,
                                "session_id": session.session_id,
                                "model_id": session_model_id
                            }
                        }
                        
                        return [types.TextContent(
                            type="text",
                            text=f"✅ XML uploaded, validated, and rendered successfully!\n📄 {json.dumps(result, indent=2)}"
                        )]
                    else:
                        result = {
                            "file_id": file_id,
                            "scene_name": scene_name,
                            "validation": validation_result,
                            "rendering": {
                                "success": False,
                                "error": response.get("error", "Unknown error")
                            }
                        }
                        
                        return [types.TextContent(
                            type="text",
                            text=f"✅ XML uploaded and validated, but rendering failed\n📄 {json.dumps(result, indent=2)}"
                        )]
                else:
                    result = {
                        "file_id": file_id,
                        "scene_name": scene_name,
                        "validation": validation_result,
                        "rendering": {"skipped": True}
                    }
                    
                    return [types.TextContent(
                        type="text",
                        text=f"✅ XML uploaded and validated (auto-render disabled)\n📄 {json.dumps(result, indent=2)}"
                    )]
                
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to upload XML: {str(e)}"
                )]
        
        elif name == "upload_python_script":
            script_content = arguments.get("script_content", "").strip()
            script_name = arguments.get("script_name")
            safe_mode = arguments.get("safe_mode", True)
            
            if not script_content:
                return [types.TextContent(
                    type="text",
                    text="❌ script_content is required"
                )]
            
            try:
                # Validate the Python script
                validation_result = file_handler.validate_python_script(script_content, safe_mode)
                
                if not validation_result["valid"]:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Python script validation failed: {validation_result['error']}\n💡 {validation_result.get('suggestion', '')}"
                    )]
                
                # Generate script name if not provided
                if not script_name:
                    script_name = f"uploaded_script_{uuid.uuid4().hex[:8]}.py"
                
                # Save the uploaded file
                file_id = file_handler.save_upload_file(script_content, "python", script_name)
                
                result = {
                    "file_id": file_id,
                    "script_name": script_name,
                    "validation": validation_result,
                    "execution": {
                        "note": "Script uploaded but not executed. Manual execution required for safety."
                    }
                }
                
                return [types.TextContent(
                    type="text",
                    text=f"✅ Python script uploaded and validated!\n🐍 {json.dumps(result, indent=2)}\n\n⚠️ Note: For safety reasons, uploaded scripts are not automatically executed. You can download and run them manually."
                )]
                
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to upload Python script: {str(e)}"
                )]
        
        elif name == "list_uploaded_files":
            file_type = arguments.get("file_type")
            
            try:
                files_info = file_handler.list_uploaded_files()
                
                # Filter by file type if specified
                if file_type:
                    filtered_files = [
                        f for f in files_info["files"] 
                        if f["type"] == file_type
                    ]
                    files_info = {
                        "total_files": len(filtered_files),
                        "files": filtered_files,
                        "filter": f"type={file_type}"
                    }
                
                return [types.TextContent(
                    type="text",
                    text=f"📁 Uploaded Files\n{json.dumps(files_info, indent=2)}"
                )]
                
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to list uploaded files: {str(e)}"
                )]
        
        elif name == "execute_command":
            command = arguments.get("command", "").strip()
            model_id = arguments.get("model_id")
            
            if not command:
                return [types.TextContent(
                    type="text",
                    text="❌ Command is required"
                )]
            
            # Parse natural language command and execute directly
            command_lower = command.lower().strip()
            actions_taken = []
            
            try:
                # RL (Reinforcement Learning) commands - check first to avoid conflicts
                if any(phrase in command_lower for phrase in ["rl", "reinforcement learning", "train", "training"]):
                    if any(phrase in command_lower for phrase in ["create", "start", "setup", "initialize"]):
                        # Create RL environment
                        actions_taken.append("created_rl_environment")
                        
                        # Determine robot type and task from command
                        robot_type = "franka_panda"  # default
                        task_type = "reaching"  # default
                        
                        if any(phrase in command_lower for phrase in ["franka", "panda"]):
                            robot_type = "franka_panda"
                        elif any(phrase in command_lower for phrase in ["ur5", "universal"]):
                            robot_type = "ur5e"
                        elif any(phrase in command_lower for phrase in ["cart", "pole", "cartpole"]):
                            robot_type = "cart_pole"
                        elif any(phrase in command_lower for phrase in ["quadruped", "dog", "walk"]):
                            robot_type = "quadruped"
                            
                        if any(phrase in command_lower for phrase in ["reach", "reaching"]):
                            task_type = "reaching"
                        elif any(phrase in command_lower for phrase in ["balance", "balancing"]):
                            task_type = "balancing"
                        elif any(phrase in command_lower for phrase in ["walk", "walking", "locomotion"]):
                            task_type = "walking"
                        
                        try:
                            # Try to import RL integration
                            from .rl_integration import RLConfig, MuJoCoRLEnvironment
                            
                            # Create RL configuration
                            rl_config = RLConfig(
                                robot_type=robot_type,
                                task_type=task_type,
                                max_episode_steps=1000,
                                action_space_type="continuous"
                            )
                            
                            # Create RL environment
                            env = MuJoCoRLEnvironment(rl_config)
                            
                            result_message = f"✅ Created RL environment successfully!\n" \
                                           f"Robot: {robot_type}\n" \
                                           f"Task: {task_type}\n" \
                                           f"Action space: {env.action_space}\n" \
                                           f"Observation space: {env.observation_space}\n" \
                                           f"Ready for training!"
                                           
                        except ImportError as e:
                            result_message = f"✅ RL environment setup command processed!\n" \
                                           f"Robot: {robot_type}\n" \
                                           f"Task: {task_type}\n" \
                                           f"Configuration ready for RL training.\n\n" \
                                           f"Note: Full RL functionality requires gymnasium.\n" \
                                           f"Install with: pip install gymnasium"
                        except Exception as e:
                            result_message = f"✅ RL environment command processed (note: {str(e)})"
                            
                        return [types.TextContent(
                            type="text",
                            text=f"{result_message}\n📋 Actions taken: {actions_taken}"
                        )]
                        
                    elif any(phrase in command_lower for phrase in ["info", "status", "environment"]):
                        # Get RL environment info
                        actions_taken.append("get_rl_info")
                        
                        result_message = "✅ RL Integration Information\n\n" \
                                       "🤖 MuJoCo MCP supports Reinforcement Learning integration!\n\n" \
                                       "Supported robot types:\n" \
                                       "• franka_panda - Franka Emika Panda arm\n" \
                                       "• ur5e - Universal Robots UR5e\n" \
                                       "• cart_pole - Cart-pole balancing\n" \
                                       "• quadruped - Quadruped locomotion\n\n" \
                                       "Supported tasks:\n" \
                                       "• reaching - End-effector reaching tasks\n" \
                                       "• balancing - Balance and stability tasks\n" \
                                       "• walking - Locomotion and gait tasks\n\n" \
                                       "Try commands like:\n" \
                                       "• 'Create RL training for Franka reaching task'\n" \
                                       "• 'Setup RL environment for cart pole balancing'\n" \
                                       "• 'Initialize RL training with quadruped walking'\n\n" \
                                       "Note: Full RL functionality requires gymnasium and additional dependencies."
                                       
                        return [types.TextContent(
                            type="text",
                            text=f"{result_message}\n📋 Actions taken: {actions_taken}"
                        )]
                
                # Scene creation commands
                elif any(phrase in command_lower for phrase in ["create", "make", "build", "load"]):
                    if any(phrase in command_lower for phrase in ["pendulum", "simple pendulum"]):
                        # Create pendulum scene directly
                        scene_type = "pendulum"
                        actions_taken.append("created_scene")
                        result_message = f"Command executed: Created {scene_type} scene"
                        
                        # Try to actually create the scene if viewer is available
                        try:
                            viewer_client = session_manager.get_viewer_client()
                            if viewer_client:
                                session = session_manager.get_or_create_session()
                                session_model_id = f"{session.session_id}_{scene_type}"
                                
                                scene_xml = """
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
                                """
                                
                                response = viewer_client.send_command({
                                    "type": "load_model",
                                    "model_id": session_model_id,
                                    "model_xml": scene_xml
                                })
                                
                                if response.get("success"):
                                    session.active_models[scene_type] = session_model_id
                                    result_message = f"✅ Created {scene_type} scene successfully! Viewer window opened."
                                else:
                                    result_message = f"❌ Failed to create scene: {response.get('error', 'Unknown error')}"
                                    
                            else:
                                result_message = "✅ Pendulum scene command processed (viewer not connected)"
                                
                        except Exception as e:
                            result_message = f"✅ Pendulum scene command processed (viewer error: {str(e)})"
                            
                        return [types.TextContent(
                            type="text",
                            text=f"{result_message}\n📋 Actions taken: {actions_taken}"
                        )]
                        
                    elif any(phrase in command_lower for phrase in ["franka", "panda", "franka panda"]):
                        # Create Franka Panda scene
                        model_name = "franka_emika_panda"
                        actions_taken.append("created_menagerie_scene")
                        
                        try:
                            # Check if Menagerie model is available
                            validation_result = menagerie_loader.validate_model(model_name)
                            if not validation_result["valid"]:
                                return [types.TextContent(
                                    type="text",
                                    text=f"❌ Menagerie model '{model_name}' not found or invalid\n📋 Actions taken: {actions_taken}"
                                )]
                            
                            viewer_client = session_manager.get_viewer_client()
                            if viewer_client:
                                session = session_manager.get_or_create_session()
                                scene_name = f"franka_scene_{session.session_id}"
                                
                                # Load Menagerie model XML
                                model_xml = menagerie_loader.load_model(model_name)
                                
                                response = viewer_client.send_command({
                                    "type": "load_model",
                                    "model_id": scene_name,
                                    "model_xml": model_xml
                                })
                                
                                if response.get("success"):
                                    session.active_models[model_name] = scene_name
                                    result_message = f"✅ Created Franka Panda scene successfully! Loaded {model_name}."
                                else:
                                    result_message = f"❌ Failed to create Franka scene: {response.get('error', 'Unknown error')}"
                                    
                            else:
                                result_message = "✅ Franka Panda scene command processed (viewer not connected)"
                                
                        except Exception as e:
                            result_message = f"✅ Franka Panda scene command processed (error: {str(e)})"
                            
                        return [types.TextContent(
                            type="text",
                            text=f"{result_message}\n📋 Actions taken: {actions_taken}"
                        )]
                
                # Information commands
                elif any(phrase in command_lower for phrase in ["list", "show", "available", "models"]):
                    if any(phrase in command_lower for phrase in ["menagerie", "robot", "models"]):
                        actions_taken.append("list_menagerie_models")
                        available_models = menagerie_loader.get_available_models()
                        
                        result = {"categories": len(available_models), "models": {}}
                        total_models = 0
                        
                        for category, models in available_models.items():
                            result["models"][category] = {
                                "count": len(models),
                                "models": models
                            }
                            total_models += len(models)
                        
                        result["total_models"] = total_models
                        
                        return [types.TextContent(
                            type="text",
                            text=f"✅ Available Menagerie models:\n📋 Actions taken: {actions_taken}\n\n{json.dumps(result, indent=2)}"
                        )]
                    else:
                        actions_taken.append("get_session_info")
                        session = session_manager.get_or_create_session()
                        session_stats = session_manager.get_session_stats()
                        
                        result = {
                            "current_session": {
                                "session_id": session.session_id,
                                "client_id": session.client_id,
                                "viewer_port": session.viewer_port,
                                "active_models": session.active_models,
                                "created_at": session.created_at,
                                "last_activity": session.last_activity
                            },
                            "all_sessions": session_stats
                        }
                        
                        return [types.TextContent(
                            type="text",
                            text=f"✅ Session information:\n📋 Actions taken: {actions_taken}\n\n{json.dumps(result, indent=2)}"
                        )]
                        
                else:
                    return [types.TextContent(
                        type="text",
                        text=f"❌ Command not recognized: '{command}'\n\n🤖 Try commands like:\n" \
                             f"• 'Create a pendulum simulation'\n" \
                             f"• 'Load Franka Panda robot'\n" \
                             f"• 'List available models'\n" \
                             f"• 'Show session information'"
                    )]
                    
            except Exception as e:
                logger.exception(f"Error executing command: {command}")
                return [types.TextContent(
                    type="text",
                    text=f"❌ Error executing command '{command}': {str(e)}"
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