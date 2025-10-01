"""FastAPI server for the remote viewer with WebRTC signaling."""

import asyncio
import logging
from pathlib import Path
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from .config import ViewerConfig
from .signaling import SignalingServer
from .logging_utils import setup_logging

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for FastAPI app."""
    # Startup
    logger.info(f"Starting MuJoCo Remote Viewer server")
    logger.info(f"Configuration: {app.state.config.to_dict()}")
    
    # Start simulation if configured to auto-start
    app.state.signaling_server.simulation.start()
    
    yield
    
    # Shutdown
    logger.info("Shutting down MuJoCo Remote Viewer server")
    
    # Stop simulation
    app.state.signaling_server.simulation.stop()
    
    # Cleanup peer connections
    for client_id in list(app.state.signaling_server.peer_connections.keys()):
        await app.state.signaling_server._cleanup_client(client_id)


def create_app(config: ViewerConfig = None) -> FastAPI:
    """Create and configure the FastAPI application.
    
    Args:
        config: Viewer configuration (uses defaults if None)
        
    Returns:
        Configured FastAPI application
    """
    if config is None:
        config = ViewerConfig.from_env()
    
    # Setup logging
    setup_logging(config.log_level)
    
    # Create FastAPI app
    app = FastAPI(
        title="MuJoCo Remote Viewer",
        description="Python-based headless WebRTC MuJoCo viewer",
        version="0.1.0",
        lifespan=lifespan,
    )
    
    # Add CORS middleware for development
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"] if config.debug_mode else [f"http://{config.host}:{config.port}"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    
    # Create signaling server
    signaling_server = SignalingServer(config)
    
    # Store config and signaling server in app state
    app.state.config = config
    app.state.signaling_server = signaling_server
    
    # Setup routes
    _setup_routes(app, config, signaling_server)
    
    return app


def _setup_routes(app: FastAPI, config: ViewerConfig, signaling_server: SignalingServer):
    """Setup API routes and static file serving.
    
    Args:
        app: FastAPI application
        config: Viewer configuration
        signaling_server: Signaling server instance
    """
    
    # Static files (client assets)
    client_dir = Path(__file__).parent.parent / "client"
    if client_dir.exists():
        app.mount("/static", StaticFiles(directory=client_dir), name="static")
    
    @app.get("/", response_class=HTMLResponse)
    async def serve_client():
        """Serve the main client HTML page."""
        try:
            client_file = client_dir / "index.html"
            if client_file.exists():
                with open(client_file, 'r') as f:
                    return HTMLResponse(f.read())
            else:
                return HTMLResponse("""
                <html>
                    <head><title>MuJoCo Remote Viewer</title></head>
                    <body>
                        <h1>MuJoCo Remote Viewer</h1>
                        <p>Client files not found. Please check the installation.</p>
                        <p>Expected client files in: {}</p>
                    </body>
                </html>
                """.format(client_dir))
        except Exception as e:
            logger.error(f"Error serving client: {e}")
            return HTMLResponse(f"<html><body><h1>Error</h1><p>{e}</p></body></html>", status_code=500)
    
    @app.get("/api/config")
    async def get_config():
        """Get viewer configuration."""
        return JSONResponse(config.to_dict())
    
    @app.get("/api/stats")
    async def get_stats():
        """Get server statistics."""
        return JSONResponse(signaling_server.get_stats())
    
    @app.get("/api/health")
    async def health_check():
        """Health check endpoint."""
        return JSONResponse({
            "status": "healthy",
            "version": "0.1.0",
            "clients_connected": signaling_server.stats["clients_connected"],
        })

    @app.post("/api/execute-command")
    async def execute_command(request: Request):
        """Execute simple text commands against the running simulation."""
        try:
            payload = await request.json()
        except Exception as exc:
            return JSONResponse({"success": False, "error": str(exc)}, status_code=400)

        command = (payload.get("command") or "").strip()
        if not command:
            return JSONResponse({"success": False, "error": "Command is required"}, status_code=400)

        result = await signaling_server.process_text_command(command)
        status_code = 200 if result.get("success") else 400
        return JSONResponse(result, status_code=status_code)

    @app.post("/api/scene/load")
    async def load_scene(request: Request):
        """Load a scene from XML."""
        try:
            data = await request.json()
            xml = data.get('xml')
            
            if not xml:
                return JSONResponse({"error": "XML content is required"}, status_code=400)
            
            # Basic XML validation
            try:
                from xml.dom import minidom
                minidom.parseString(xml)
            except Exception as e:
                return JSONResponse({"error": f"Invalid XML: {str(e)}"}, status_code=400)
            
            # Store the scene XML in the signaling server (could be used by simulation)
            signaling_server.current_scene_xml = xml
            
            # Broadcast scene update to all connected clients
            await signaling_server.broadcast_scene_update(xml)
            
            return JSONResponse({
                "success": True,
                "message": "Scene loaded successfully"
            })
            
        except Exception as e:
            logger.error(f"Error loading scene: {e}")
            return JSONResponse({"error": str(e)}, status_code=500)
    
    @app.websocket("/ws/signaling")
    async def websocket_signaling(websocket: WebSocket):
        """WebSocket endpoint for WebRTC signaling."""
        await signaling_server.handle_websocket(websocket)


def run_server(host: str = None, port: int = None, config: ViewerConfig = None):
    """Run the viewer server with uvicorn.
    
    Args:
        host: Server host (overrides config)
        port: Server port (overrides config)
        config: Viewer configuration
    """
    import uvicorn
    
    if config is None:
        config = ViewerConfig.from_env()
    
    # Override config with parameters
    if host is not None:
        config.host = host
    if port is not None:
        config.port = port
    
    app = create_app(config)
    
    logger.info(f"Starting server on {config.host}:{config.port}")
    
    uvicorn.run(
        app,
        host=config.host,
        port=config.port,
        log_level=config.log_level.lower(),
        access_log=config.debug_mode,
    )


if __name__ == "__main__":
    # Allow running this module directly for development
    run_server()
