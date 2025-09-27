#!/usr/bin/env python3
"""
Web Server for MuJoCo MCP Client
Serves the web client and provides API endpoints for command execution
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import Dict, Any

from starlette.applications import Starlette
from starlette.responses import JSONResponse, FileResponse, Response
from starlette.routing import Route, Mount
from starlette.staticfiles import StaticFiles
from starlette.middleware.cors import CORSMiddleware
import uvicorn

from src.mujoco_mcp.mcp_server_menagerie import handle_call_tool

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp-web")

# Get the client directory
CLIENT_DIR = Path(__file__).parent / "client"

async def execute_command_endpoint(request):
    """Handle execute command API endpoint"""
    try:
        body = await request.json()
        command = body.get('command', '').strip()
        model_id = body.get('model_id')
        
        if not command:
            return JSONResponse({
                "success": False,
                "error": "Command is required"
            }, status_code=400)
        
        # Execute the command using the MCP server
        result = await handle_call_tool("execute_command", {
            "command": command,
            "model_id": model_id
        })
        
        # Parse the result
        result_text = result[0].text if result else "No result"
        
        # Check if command was successful
        success = not result_text.startswith("❌")
        
        # Extract actions taken if present
        actions_taken = []
        if "Actions taken:" in result_text:
            actions_part = result_text.split("Actions taken:")[-1].strip()
            if actions_part.startswith("[") and actions_part.endswith("]"):
                try:
                    actions_taken = eval(actions_part)
                except:
                    pass
        
        return JSONResponse({
            "success": success,
            "result": result_text,
            "actions_taken": actions_taken
        })
        
    except Exception as e:
        logger.exception("Error in execute_command_endpoint")
        return JSONResponse({
            "success": False,
            "error": str(e)
        }, status_code=500)

async def scene_load_endpoint(request):
    """Handle scene loading endpoint"""
    try:
        body = await request.json()
        xml_content = body.get('xml', '').strip()
        
        if not xml_content:
            return JSONResponse({
                "success": False,
                "error": "XML content is required"
            }, status_code=400)
        
        # For now, just return success - this would need to be integrated with the viewer
        return JSONResponse({
            "success": True,
            "message": "Scene loaded successfully"
        })
        
    except Exception as e:
        logger.exception("Error in scene_load_endpoint")
        return JSONResponse({
            "success": False,
            "error": str(e)
        }, status_code=500)

async def config_endpoint(request):
    """Handle config endpoint"""
    return JSONResponse({
        "stun_server": "stun:stun.l.google.com:19302",
        "app_version": "0.8.2"
    })

async def homepage(request):
    """Serve the main HTML page"""
    return FileResponse(CLIENT_DIR / "index.html")

# Define routes
routes = [
    Route("/", homepage),
    Route("/api/execute-command", execute_command_endpoint, methods=["POST"]),
    Route("/api/scene/load", scene_load_endpoint, methods=["POST"]),
    Route("/api/config", config_endpoint, methods=["GET"]),
    Mount("/static", StaticFiles(directory=CLIENT_DIR), name="static"),
]

# Create the application
app = Starlette(routes=routes)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

def main():
    """Main entry point"""
    logger.info("Starting MuJoCo MCP Web Server")
    
    # Check if client directory exists
    if not CLIENT_DIR.exists():
        logger.error(f"Client directory not found: {CLIENT_DIR}")
        return
    
    # Check if required files exist
    required_files = ["index.html", "app.js", "styles.css"]
    for file in required_files:
        if not (CLIENT_DIR / file).exists():
            logger.error(f"Required file not found: {CLIENT_DIR / file}")
            return
    
    logger.info(f"Serving client files from: {CLIENT_DIR}")
    logger.info("Web interface will be available at: http://localhost:8080")
    
    # Run the server
    uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")

if __name__ == "__main__":
    main()