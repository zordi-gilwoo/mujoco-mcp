"""
MuJoCo Viewer Client - Socket communication module
Connect to standalone MuJoCo Viewer Server process
Enhanced version: includes auto-reconnect, health check and process management
"""

import json
import socket
import logging
import time
import subprocess
import sys
import os
from typing import Dict, Any

logger = logging.getLogger("mujoco_mcp.viewer_client")


class MuJoCoViewerClient:
    """Client for connecting to MuJoCo Viewer Server - Enhanced version"""

    def __init__(self, host: str = "localhost", port: int = 8888):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.auto_start = True  # Auto-start viewer server
        self.reconnect_attempts = 3
        self.reconnect_delay = 2.0

    def connect(self) -> bool:
        """Connect to MuJoCo Viewer Server - supports auto-start and retry"""
        for attempt in range(self.reconnect_attempts):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(15.0)  # Increased timeout for model replacement
                self.socket.connect((self.host, self.port))
                self.connected = True
                logger.info(f"Connected to MuJoCo Viewer Server at {self.host}:{self.port}")
                return True
            except Exception as e:
                logger.warning(f"Connection attempt {attempt + 1} failed: {e}")

                # Try to start viewer server after first failure
                if attempt == 0 and self.auto_start:
                    logger.info("Attempting to start MuJoCo Viewer Server...")
                    if self._start_viewer_server():
                        time.sleep(3)  # Wait for server to start
                        continue

                if attempt < self.reconnect_attempts - 1:
                    time.sleep(self.reconnect_delay)

        logger.error(f"Failed to connect after {self.reconnect_attempts} attempts")
        self.connected = False
        return False

    def disconnect(self):
        """Disconnect"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        logger.info("Disconnected from MuJoCo Viewer Server")

    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Send command to viewer server and get response"""
        if not self.connected or not self.socket:
            return {"success": False, "error": "Not connected to viewer server"}

        try:
            # Send command
            command_json = json.dumps(command)
            self.socket.send(command_json.encode("utf-8"))

            # Receive response - support larger messages
            response_data = b""
            while True:
                chunk = self.socket.recv(8192)
                if not chunk:
                    break
                response_data += chunk

                # Check if we received complete JSON (ends with newline)
                if response_data.endswith(b"\n"):
                    break

                # Prevent infinite waiting
                if len(response_data) > 1024 * 1024:  # 1MB limit
                    raise ValueError("Response too large")

            return json.loads(response_data.decode("utf-8").strip())

        except Exception as e:
            logger.exception(f"Failed to send command: {e}")
            return {"success": False, "error": str(e)}

    def ping(self) -> bool:
        """Test connection health - enhanced version"""
        if not self.connected:
            # Try to reconnect
            if not self.connect():
                return False

        try:
            response = self.send_command({"type": "ping"})
            return response.get("success", False)
        except:
            # Connection may be broken, try to reconnect
            self.connected = False
            if self.connect():
                response = self.send_command({"type": "ping"})
                return response.get("success", False)
            return False

    def load_model(self, model_source: str, model_id: str = None) -> Dict[str, Any]:
        """Load MuJoCo model to viewer

        Args:
            model_source: XML string or XML file path
            model_id: model ID
        """
        cmd = {
            "type": "load_model",
            "model_xml": model_source,  # Keep backward compatibility, but can actually be file path
        }
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def replace_model(self, model_source: str, model_id: str = None) -> Dict[str, Any]:
        """Replace current model (close existing viewer and load new model)

        Args:
            model_source: XML string or XML file path
            model_id: model ID
        """
        cmd = {
            "type": "replace_model",
            "model_xml": model_source,  # Keep backward compatibility, but can actually be file path
        }
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def start_viewer(self) -> Dict[str, Any]:
        """Start viewer GUI"""
        return self.send_command({"type": "start_viewer"})

    def get_state(self, model_id: str = None) -> Dict[str, Any]:
        """Get simulation state"""
        cmd = {"type": "get_state"}
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def set_control(self, control: list) -> Dict[str, Any]:
        """Set control input"""
        return self.send_command({"type": "set_control", "control": control})

    def set_joint_positions(self, positions: list, model_id: str = None) -> Dict[str, Any]:
        """Set joint positions"""
        cmd = {"type": "set_joint_positions", "positions": positions}
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def reset_simulation(self, model_id: str = None) -> Dict[str, Any]:
        """Reset simulation"""
        cmd = {"type": "reset"}
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def close_viewer(self) -> Dict[str, Any]:
        """Close viewer GUI window"""
        return self.send_command({"type": "close_viewer"})

    def shutdown_server(self) -> Dict[str, Any]:
        """Shutdown entire viewer server"""
        return self.send_command({"type": "shutdown_server"})

    def capture_render(
        self, model_id: str = None, width: int = 640, height: int = 480
    ) -> Dict[str, Any]:
        """Capture current rendered image"""
        cmd = {"type": "capture_render", "width": width, "height": height}
        if model_id:
            cmd["model_id"] = model_id
        return self.send_command(cmd)

    def _start_viewer_server(self) -> bool:
        """Try to start MuJoCo Viewer Server - support macOS mjpython"""
        try:
            # Find viewer server script
            script_paths = [
                "mujoco_viewer_server.py",
                os.path.join(os.path.dirname(__file__), "..", "..", "mujoco_viewer_server.py"),
                os.path.join(os.getcwd(), "mujoco_viewer_server.py"),
            ]

            viewer_script = None
            for path in script_paths:
                if os.path.exists(path):
                    viewer_script = os.path.abspath(path)
                    break

            if not viewer_script:
                logger.error("Could not find mujoco_viewer_server.py")
                return False

            # Check if we need to use mjpython (macOS)
            python_executable = sys.executable
            if sys.platform == "darwin":  # macOS
                # Try to find mjpython
                mjpython_result = subprocess.run(
                    ["which", "mjpython"], capture_output=True, text=True
                )
                if mjpython_result.returncode == 0:
                    mjpython_path = mjpython_result.stdout.strip()
                    if mjpython_path:
                        python_executable = mjpython_path
                        logger.info(f"Using mjpython for macOS: {mjpython_path}")
                else:
                    logger.warning("mjpython not found on macOS, viewer may not work properly")

            # Start process
            cmd = [python_executable, viewer_script, "--port", str(self.port)]
            logger.info(f"Starting viewer with command: {' '.join(cmd)}")

            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,  # Independent process group
            )

            logger.info(f"Started MuJoCo Viewer Server (PID: {process.pid})")
            return True

        except Exception as e:
            logger.exception(f"Failed to start viewer server: {e}")
            return False

    def get_diagnostics(self) -> Dict[str, Any]:
        """Get connection diagnostic information"""
        diagnostics = {
            "host": self.host,
            "port": self.port,
            "connected": self.connected,
            "socket_alive": self.socket is not None,
            "ping_result": False,
            "viewer_process": self._check_viewer_process(),
        }

        if self.connected:
            diagnostics["ping_result"] = self.ping()

        return diagnostics

    def _check_viewer_process(self) -> bool:
        """Check if viewer process is running"""
        try:
            # Use lsof to check port
            result = subprocess.run(
                ["lsof", "-ti", f":{self.port}"], capture_output=True, text=True
            )
            return bool(result.stdout.strip())
        except:
            return False


class ViewerManager:
    """Manage multiple viewer client connections"""

    def __init__(self):
        self.clients = {}  # model_id -> ViewerClient
        self.default_port = 8888

    def create_client(self, model_id: str, port: int | None = None) -> bool:
        """Create viewer client for specific model"""
        if port is None:
            port = self.default_port

        client = MuJoCoViewerClient(port=port)
        if client.connect():
            self.clients[model_id] = client
            logger.info(f"Created viewer client for model {model_id}")
            return True
        else:
            logger.error(f"Failed to create viewer client for model {model_id}")
            return False

    def get_client(self, model_id: str) -> MuJoCoViewerClient | None:
        """Get viewer client for specified model"""
        return self.clients.get(model_id)

    def remove_client(self, model_id: str):
        """Remove viewer client"""
        if model_id in self.clients:
            self.clients[model_id].disconnect()
            del self.clients[model_id]
            logger.info(f"Removed viewer client for model {model_id}")

    def disconnect_all(self):
        """Disconnect all connections"""
        for model_id in list(self.clients.keys()):
            self.remove_client(model_id)


# Global viewer manager instance
viewer_manager = ViewerManager()


# Diagnostic information functions
def get_system_diagnostics() -> Dict[str, Any]:
    """Get system diagnostic information"""
    diagnostics = {
        "viewer_manager": {
            "active_clients": len(viewer_manager.clients),
            "client_ids": list(viewer_manager.clients.keys()),
            "default_port": viewer_manager.default_port,
        },
        "clients": {},
    }

    for model_id, client in viewer_manager.clients.items():
        diagnostics["clients"][model_id] = client.get_diagnostics()

    return diagnostics


def get_viewer_client(model_id: str) -> MuJoCoViewerClient | None:
    """Convenience function to get viewer client for specified model"""
    return viewer_manager.get_client(model_id)


def ensure_viewer_connection(model_id: str) -> bool:
    """Convenience function to ensure viewer connection exists - enhanced version"""
    client = viewer_manager.get_client(model_id)
    if client and client.connected and client.ping():
        return True

    # If connection doesn't exist or is broken, try to reconnect
    logger.info(f"Creating new viewer connection for model {model_id}")

    # Multiple attempts
    for attempt in range(3):
        if viewer_manager.create_client(model_id):
            return True
        logger.warning(f"Connection attempt {attempt + 1} failed for model {model_id}")
        if attempt < 2:
            time.sleep(2)

    # Finally provide detailed diagnostics
    client = viewer_manager.get_client(model_id)
    if client:
        diagnostics = client.get_diagnostics()
        logger.error(f"Connection diagnostics: {diagnostics}")

    return False
