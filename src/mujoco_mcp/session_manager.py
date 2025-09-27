"""
Session Manager for Multi-Client MuJoCo MCP Server
Handles client session isolation and resource management

Supports two isolation modes:
- Session-based: Multiple clients share viewer server with session isolation (fast, lightweight)
- Process-based: Each client gets dedicated viewer process (complete isolation, enterprise-ready)
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
    """Information about an active MCP client session
    
    Attributes:
        session_id: Unique identifier for the session
        client_id: Human-readable client identifier
        created_at: Session creation timestamp
        last_activity: Last activity timestamp for cleanup
        viewer_port: Port for viewer connection (allocated by SessionManager or ProcessManager)
        active_models: Dictionary of loaded models (model_id -> scene_type/name)
        use_isolated_process: Whether this session uses dedicated process isolation
    """
    session_id: str
    client_id: str
    created_at: float
    last_activity: float
    viewer_port: int
    active_models: Dict[str, str]  # model_id -> scene_type/name
    use_isolated_process: bool = False
    
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
    """Manages MCP client sessions and their associated resources
    
    Supports two isolation modes:
    1. Session-based isolation (use_isolated_processes=False):
       - Multiple clients share a single viewer server process
       - Session isolation through unique session IDs and ports
       - Fast connection, lightweight resource usage
       - Suitable for development and single-user scenarios
       
    2. Process-based isolation (use_isolated_processes=True):
       - Each client gets a dedicated viewer server process
       - Complete memory and resource isolation
       - Crash protection and security boundaries
       - Suitable for production and multi-tenant deployments
    """
    
    def __init__(self, use_isolated_processes: bool = True):
        """Initialize SessionManager
        
        Args:
            use_isolated_processes: Whether to use process-based isolation by default
                                  False = session-based isolation (lightweight)
                                  True = process-based isolation (enterprise-ready)
        """
        self.sessions: Dict[str, SessionInfo] = {}
        self._lock = threading.RLock()
        
        # Session-based isolation settings (used when use_isolated_processes=False)
        self._base_port = 8888
        self._port_counter = 0
        
        # Process isolation configuration
        self.use_isolated_processes = use_isolated_processes
        
        logger.info(f"SessionManager initialized with {'process-based' if use_isolated_processes else 'session-based'} isolation")
        
    def get_or_create_session(self, context: Any = None, use_isolated_process: bool = None) -> SessionInfo:
        """Get existing session or create new one for the MCP client
        
        Args:
            context: MCP context containing session information
            use_isolated_process: Override default isolation mode for this session
                                None = use manager default
                                False = session-based isolation
                                True = process-based isolation
                                
        Returns:
            SessionInfo object with session details and configuration
        """
        session_id = self._extract_session_id(context)
        
        with self._lock:
            # Return existing session if found
            if session_id in self.sessions:
                session = self.sessions[session_id]
                session.update_activity()
                logger.debug(f"Retrieved existing session {session_id}")
                return session
            
            # Create new session
            client_id = f"client_{len(self.sessions) + 1}_{uuid.uuid4().hex[:8]}"
            
            # Determine isolation mode for this session
            if use_isolated_process is None:
                use_isolated_process = self.use_isolated_processes
            
            # Allocate port based on isolation mode
            if use_isolated_process:
                # ProcessManager will allocate its own port dynamically
                viewer_port = 0  # Will be set when process is spawned
            else:
                # Use traditional sequential port allocation for session-based mode
                viewer_port = self._allocate_port()
            
            # Create session info
            session = SessionInfo(
                session_id=session_id,
                client_id=client_id,
                created_at=time.time(),
                last_activity=time.time(),
                viewer_port=viewer_port,
                active_models={},
                use_isolated_process=use_isolated_process
            )
            
            self.sessions[session_id] = session
            
            isolation_type = "process-based" if use_isolated_process else "session-based"
            port_info = f"port {viewer_port}" if viewer_port > 0 else "dynamic port allocation"
            logger.info(f"Created new {isolation_type} session {session_id} for {client_id} with {port_info}")
            
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
        """Get or create viewer client for the session"""
        session = self.get_or_create_session(context)
        
        if session.use_isolated_process:
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
                logger.info(f"Spawned isolated process for session {session.session_id} on port {process_info.port}")
            
            # Create viewer client that connects to the isolated process
            client = viewer_manager.get_client(session.session_id)
            if not client:
                success = viewer_manager.create_client(session.session_id, process_info.port)
                if success:
                    client = viewer_manager.get_client(session.session_id)
                    logger.info(f"Created viewer client for isolated session {session.session_id}")
                else:
                    logger.error(f"Failed to create viewer client for isolated session {session.session_id}")
                    return None
        else:
            # Use traditional viewer client approach
            client = viewer_manager.get_client(session.session_id)
            
            if not client:
                # Create new viewer client for this session
                success = viewer_manager.create_client(session.session_id, session.viewer_port)
                if success:
                    client = viewer_manager.get_client(session.session_id)
                    logger.info(f"Created traditional viewer client for session {session.session_id}")
                else:
                    logger.error(f"Failed to create traditional viewer client for session {session.session_id}")
                    return None
        
        return client
    
    def cleanup_session(self, context: Any = None):
        """Clean up session and its resources"""
        session_id = self._extract_session_id(context)
        
        with self._lock:
            if session_id in self.sessions:
                session = self.sessions[session_id]
                
                # Clean up isolated process if applicable
                if session.use_isolated_process:
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
        """Get statistics about active sessions"""
        with self._lock:
            stats = {
                "active_sessions": len(self.sessions),
                "isolated_process_mode": self.use_isolated_processes,
                "sessions": {},
                "process_stats": process_manager.get_stats()
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
                    "use_isolated_process": session.use_isolated_process
                }
                
                # Add process information if using isolated processes
                if session.use_isolated_process:
                    process_info = process_manager.get_process_info(session_id)
                    if process_info:
                        session_info["process"] = {
                            "pid": process_info.pid,
                            "port": process_info.port,
                            "url": process_info.url,
                            "is_running": process_info.is_running,
                            "age_seconds": process_info.age_seconds
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
        if context and hasattr(context, 'session_id'):
            return context.session_id
        elif context and isinstance(context, dict) and 'session_id' in context:
            return context['session_id']
        else:
            # Generate thread-based session ID for testing/development
            # In production MCP server, this would come from the connection context
            thread_id = threading.current_thread().ident
            return f"session_{thread_id}"
    
    def _allocate_port(self) -> int:
        """Allocate a unique port for session-based viewer client
        
        Used only when use_isolated_processes=False.
        Process-based isolation uses ProcessManager for dynamic port allocation.
        
        Returns:
            Sequential port number starting from base_port + 1
        """
        self._port_counter += 1
        allocated_port = self._base_port + self._port_counter
        logger.debug(f"Allocated session-based port: {allocated_port}")
        return allocated_port
    
    @classmethod
    def get_recommended_isolation_mode(cls, client_count: int = 1, production: bool = False) -> bool:
        """Get recommended isolation mode based on deployment characteristics
        
        Args:
            client_count: Expected number of concurrent clients
            production: Whether this is a production deployment
            
        Returns:
            True for process-based isolation, False for session-based isolation
        """
        if production or client_count > 5:
            return True  # Process-based isolation for production/high-concurrency
        else:
            return False  # Session-based isolation for development/low-concurrency


# Global session manager instance with process-based isolation enabled by default
# Provides enterprise-ready multi-client support with complete process isolation
session_manager = SessionManager(use_isolated_processes=True)

# Alternative session manager for lightweight scenarios
# Uncomment and use this for development or single-user deployments:
# session_manager = SessionManager(use_isolated_processes=False)