"""
Session Manager for Multi-Client MuJoCo MCP Server
Handles client session isolation and resource management with process-based isolation

Each client gets a dedicated MuJoCo viewer process for complete isolation:
- Memory and resource separation
- Crash protection between clients
- Security boundaries at the process level
- Enterprise-ready multi-tenant support
"""

import logging
import time
import threading
import uuid
from typing import Dict, Optional, Any
from dataclasses import dataclass

from .viewer_client import MuJoCoViewerClient, ViewerManager, viewer_manager
from .process_manager import process_manager

logger = logging.getLogger("mujoco_mcp.session_manager")


@dataclass
class SessionInfo:
    """Information about an active MCP client session with process isolation

    Each session gets a dedicated viewer process for complete isolation.

    Attributes:
        session_id: Unique identifier for the session
        client_id: Human-readable client identifier
        created_at: Session creation timestamp
        last_activity: Last activity timestamp for cleanup
        viewer_port: Port allocated by ProcessManager for the dedicated viewer process
        active_models: Dictionary of loaded models (model_id -> scene_type/name)
    """

    session_id: str
    client_id: str
    created_at: float
    last_activity: float
    viewer_port: int
    active_models: Dict[str, str]  # model_id -> scene_type/name

    def update_activity(self):
        """Update last activity timestamp for session cleanup management"""
        self.last_activity = time.time()

    def is_stale(self, timeout: float = 1800.0) -> bool:
        """Check if session is stale (inactive for too long)

        Args:
            timeout: Inactivity timeout in seconds (default: 30 minutes)

        Returns:
            True if session should be cleaned up
        """
        return (time.time() - self.last_activity) > timeout


class SessionManager:
    """Manages MCP client sessions with dedicated process-based isolation

    Each client session gets its own dedicated MuJoCo viewer process providing:
    - Complete memory and resource isolation
    - Crash protection between clients
    - Security boundaries at the process level
    - Automatic port allocation and process lifecycle management
    - Enterprise-ready multi-tenant support
    """

    def __init__(self):
        """Initialize SessionManager with process-based isolation"""
        self.sessions: Dict[str, SessionInfo] = {}
        self._lock = threading.RLock()

        logger.info("SessionManager initialized with process-based isolation")

    def get_or_create_session(self, context: Any = None) -> SessionInfo:
        """Get existing session or create new one with dedicated process isolation

        Args:
            context: MCP context containing session information

        Returns:
            SessionInfo object with session details and process configuration
        """
        session_id = self._extract_session_id(context)

        with self._lock:
            # Return existing session if found
            if session_id in self.sessions:
                session = self.sessions[session_id]
                session.update_activity()
                logger.debug(f"Retrieved existing session {session_id}")
                return session

            # Create new session with process isolation
            client_id = f"client_{len(self.sessions) + 1}_{uuid.uuid4().hex[:8]}"

            # ProcessManager will allocate port dynamically when process is spawned
            viewer_port = 0  # Will be set when process is spawned

            # Create session info
            session = SessionInfo(
                session_id=session_id,
                client_id=client_id,
                created_at=time.time(),
                last_activity=time.time(),
                viewer_port=viewer_port,
                active_models={},
            )

            self.sessions[session_id] = session

            logger.info(
                f"Created new process-based session {session_id} for {client_id} with dynamic port allocation"
            )

            return session

    def get_session(self, context: Any = None) -> Optional[SessionInfo]:
        """Get existing session without creating new one"""
        session_id = self._extract_session_id(context)
        with self._lock:
            session = self.sessions.get(session_id)
            if session:
                session.update_activity()
            return session

    def get_viewer_client(self, context: Any = None) -> Optional[MuJoCoViewerClient]:
        """Get or create viewer client for the session with dedicated process isolation

        Args:
            context: MCP context containing session information

        Returns:
            MuJoCoViewerClient connected to the session's dedicated viewer process
        """
        session = self.get_or_create_session(context)

        # Use ProcessManager for isolated process management
        process_info = process_manager.get_process_info(session.session_id)

        if not process_info:
            # Spawn new isolated process
            process_info = process_manager.spawn_viewer_process(session.session_id, isolated=True)
            if not process_info:
                logger.error(f"Failed to spawn isolated process for session {session.session_id}")
                return None

            # Update session with actual port from process
            session.viewer_port = process_info.port
            logger.info(
                f"Spawned isolated process for session {session.session_id} on port {process_info.port}"
            )

        # Create viewer client that connects to the isolated process
        client = viewer_manager.get_client(session.session_id)
        if not client:
            success = viewer_manager.create_client(session.session_id, process_info.port)
            if success:
                client = viewer_manager.get_client(session.session_id)
                logger.info(f"Created viewer client for isolated session {session.session_id}")
            else:
                logger.error(
                    f"Failed to create viewer client for isolated session {session.session_id}"
                )
                return None

        return client

    def cleanup_session(self, context: Any = None):
        """Clean up session and its dedicated process resources

        Args:
            context: MCP context containing session information
        """
        session_id = self._extract_session_id(context)

        with self._lock:
            if session_id in self.sessions:
                session = self.sessions[session_id]

                # Clean up the dedicated isolated process
                process_manager.terminate_process(session_id)
                logger.info(f"Terminated isolated process for session {session_id}")

                # Clean up all viewer clients for this session's models
                for model_id in list(session.active_models.keys()):
                    full_model_id = f"{session.session_id}_{model_id}"
                    viewer_manager.remove_client(full_model_id)

                # Remove primary session client
                viewer_manager.remove_client(session.session_id)

                del self.sessions[session_id]
                logger.info(f"Cleaned up session {session_id}")

    def cleanup_all_sessions(self):
        """Clean up all sessions and their resources"""
        with self._lock:
            session_ids = list(self.sessions.keys())
            for session_id in session_ids:
                self.cleanup_session({"session_id": session_id})

            # Also cleanup any orphaned processes
            process_manager.cleanup_all()

    def cleanup_stale_sessions(self, timeout: float = 1800.0):
        """Clean up stale sessions"""
        stale_sessions = []

        with self._lock:
            for session_id, session in self.sessions.items():
                if session.is_stale(timeout):
                    stale_sessions.append(session_id)

        for session_id in stale_sessions:
            logger.info(f"Cleaning up stale session {session_id}")
            # Create a dummy context for cleanup
            self.cleanup_session({"session_id": session_id})

    def get_session_stats(self) -> Dict[str, Any]:
        """Get statistics about active sessions and their dedicated processes

        Returns:
            Dictionary containing session and process statistics
        """
        with self._lock:
            stats = {
                "active_sessions": len(self.sessions),
                "isolation_mode": "process-based",
                "sessions": {},
                "process_stats": process_manager.get_stats(),
            }

            for session_id, session in self.sessions.items():
                session_info = {
                    "client_id": session.client_id,
                    "created_at": session.created_at,
                    "last_activity": session.last_activity,
                    "viewer_port": session.viewer_port,
                    "active_models": list(session.active_models.keys()),
                    "age_seconds": time.time() - session.created_at,
                    "idle_seconds": time.time() - session.last_activity,
                }

                # Add process information for the dedicated process
                process_info = process_manager.get_process_info(session_id)
                if process_info:
                    session_info["process"] = {
                        "pid": process_info.pid,
                        "port": process_info.port,
                        "url": process_info.url,
                        "is_running": process_info.is_running,
                        "age_seconds": process_info.age_seconds,
                    }
                else:
                    session_info["process"] = None

                stats["sessions"][session_id] = session_info

            return stats

    def _extract_session_id(self, context: Any = None) -> str:
        """Extract session ID from MCP context or generate one

        Args:
            context: MCP context object or dictionary containing session information

        Returns:
            Unique session identifier string
        """
        if context and hasattr(context, "session_id"):
            return context.session_id
        elif context and isinstance(context, dict) and "session_id" in context:
            return context["session_id"]
        else:
            # Generate thread-based session ID for testing/development
            # In production MCP server, this would come from the connection context
            thread_id = threading.current_thread().ident
            return f"session_{thread_id}"


# Global session manager instance with process-based isolation
# Provides enterprise-ready multi-client support with complete process isolation
session_manager = SessionManager()
