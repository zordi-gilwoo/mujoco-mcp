#!/usr/bin/env python3
"""
Mock Viewer Server for testing ProcessManager without MuJoCo dependencies
"""

import argparse
import socket
import threading
import time
import json
import logging
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mock_viewer_server")


class MockViewerServer:
    """Mock viewer server that simulates a MuJoCo viewer for testing"""

    def __init__(self, port: int = 8888, session_id: str = None):
        self.port = port
        self.session_id = session_id or "mock_session"
        self.running = False
        self.socket = None
        self.clients = []

    def start(self):
        """Start the mock server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(("localhost", self.port))
            self.socket.listen(5)

            self.running = True
            logger.info(
                f"Mock viewer server started on port {self.port} for session {self.session_id}"
            )

            while self.running:
                try:
                    client_socket, addr = self.socket.accept()
                    logger.info(f"Client connected from {addr}")

                    # Handle client in a separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client, args=(client_socket, addr), daemon=True
                    )
                    client_thread.start()

                except Exception as e:
                    if self.running:
                        logger.error(f"Error accepting client: {e}")
                    break

        except Exception as e:
            logger.error(f"Failed to start mock server: {e}")
            sys.exit(1)
        finally:
            if self.socket:
                self.socket.close()

    def _handle_client(self, client_socket: socket.socket, addr):
        """Handle a client connection"""
        try:
            while self.running:
                data = client_socket.recv(1024)
                if not data:
                    break

                try:
                    message = json.loads(data.decode())
                    response = self._process_message(message)
                    client_socket.send(json.dumps(response).encode())
                except json.JSONDecodeError:
                    response = {"error": "Invalid JSON"}
                    client_socket.send(json.dumps(response).encode())

        except Exception as e:
            logger.error(f"Error handling client {addr}: {e}")
        finally:
            client_socket.close()
            logger.info(f"Client {addr} disconnected")

    def _process_message(self, message):
        """Process a message from the client"""
        command = message.get("command", "unknown")

        if command == "ping":
            return {
                "success": True,
                "pong": True,
                "session_id": self.session_id,
                "port": self.port,
                "server_type": "mock",
            }
        elif command == "load_model":
            return {"success": True, "message": f"Mock model loaded in session {self.session_id}"}
        elif command == "step_simulation":
            return {
                "success": True,
                "message": f"Mock simulation step completed in session {self.session_id}",
            }
        else:
            return {"success": False, "error": f"Unknown command: {command}"}

    def stop(self):
        """Stop the server"""
        self.running = False
        if self.socket:
            self.socket.close()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Mock MuJoCo Viewer Server")
    parser.add_argument("--port", type=int, default=8888, help="Port to bind to")
    parser.add_argument("--session-id", type=str, help="Session ID for isolation")
    parser.add_argument("--isolated", action="store_true", help="Run in isolated mode")

    args = parser.parse_args()

    logger.info(f"Starting mock viewer server on port {args.port}")
    if args.session_id:
        logger.info(f"Session ID: {args.session_id}")
    if args.isolated:
        logger.info("Running in isolated mode")

    server = MockViewerServer(port=args.port, session_id=args.session_id)

    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
        server.stop()
    except Exception as e:
        logger.error(f"Server error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
