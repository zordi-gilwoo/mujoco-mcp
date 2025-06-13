#!/usr/bin/env python3
"""
MuJoCo MCP Server for stdio transport
Production-ready MCP server that works with Claude Desktop and other MCP clients
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

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp")

# Create server instance
server = Server("mujoco-mcp")

# Global viewer client
viewer_client: Optional[ViewerClient] = None

@server.list_tools()
async def handle_list_tools() -> List[types.Tool]:
    """Return list of available MuJoCo MCP tools"""
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
            name="create_scene",
            description="Create a physics simulation scene",
            inputSchema={
                "type": "object",
                "properties": {
                    "scene_type": {
                        "type": "string",
                        "description": "Type of scene to create",
                        "enum": ["pendulum", "double_pendulum", "cart_pole", "arm"]
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
                        "description": "ID of the model viewer to close"
                    }
                },
                "required": ["model_id"]
            }
        )
    ]

@server.call_tool()
async def handle_call_tool(name: str, arguments: Dict[str, Any]) -> List[types.TextContent]:
    """Handle tool calls"""
    global viewer_client
    
    try:
        if name == "get_server_info":
            return [types.TextContent(
                type="text",
                text=json.dumps({
                    "name": "MuJoCo MCP Server",
                    "version": __version__,
                    "description": "Control MuJoCo physics simulations through MCP",
                    "status": "ready",
                    "capabilities": ["create_scene", "step_simulation", "get_state", "reset", "close_viewer"]
                }, indent=2)
            )]
            
        elif name == "create_scene":
            scene_type = arguments.get("scene_type", "pendulum")
            
            # Initialize viewer client if not exists
            if not viewer_client:
                viewer_client = ViewerClient()
                
            # Connect to viewer server
            if not viewer_client.connected:
                success = viewer_client.connect()
                if not success:
                    return [types.TextContent(
                        type="text",
                        text="❌ Failed to connect to MuJoCo viewer server. Please start `mujoco-mcp-viewer` first."
                    )]
            
            # Create the scene
            response = viewer_client.send_command({
                "type": "create_scene",
                "scene_type": scene_type
            })
            
            if response.get("success"):
                return [types.TextContent(
                    type="text",
                    text=f"✅ Created {scene_type} scene successfully! Viewer window opened."
                )]
            else:
                return [types.TextContent(
                    type="text", 
                    text=f"❌ Failed to create scene: {response.get('error', 'Unknown error')}"
                )]
                
        elif name == "step_simulation":
            model_id = arguments.get("model_id")
            steps = arguments.get("steps", 1)
            
            if not viewer_client or not viewer_client.connected:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
                
            response = viewer_client.send_command({
                "type": "step_simulation", 
                "model_id": model_id,
                "steps": steps
            })
            
            return [types.TextContent(
                type="text",
                text=f"⏩ Stepped simulation {steps} steps" if response.get("success") 
                     else f"❌ Step failed: {response.get('error')}"
            )]
            
        elif name == "get_state":
            model_id = arguments.get("model_id")
            
            if not viewer_client or not viewer_client.connected:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
                
            response = viewer_client.send_command({
                "type": "get_state",
                "model_id": model_id
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
            
            if not viewer_client or not viewer_client.connected:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection. Create a scene first."
                )]
                
            response = viewer_client.send_command({
                "type": "reset_simulation",
                "model_id": model_id
            })
            
            return [types.TextContent(
                type="text",
                text="🔄 Simulation reset to initial state" if response.get("success")
                     else f"❌ Reset failed: {response.get('error')}"
            )]
            
        elif name == "close_viewer":
            model_id = arguments.get("model_id")
            
            if not viewer_client or not viewer_client.connected:
                return [types.TextContent(
                    type="text",
                    text="❌ No active viewer connection."
                )]
                
            response = viewer_client.send_command({
                "type": "close_viewer",
                "model_id": model_id
            })
            
            # Close our connection too
            if viewer_client:
                viewer_client.disconnect()
                viewer_client = None
                
            return [types.TextContent(
                type="text",
                text="❌ Viewer closed" if response.get("success")
                     else f"❌ Failed to close: {response.get('error')}"
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
    """Main entry point for MCP server"""
    logger.info(f"Starting MuJoCo MCP Server v{__version__}")
    
    # Initialize server capabilities
    server_options = InitializationOptions(
        server_name="mujoco-mcp",
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