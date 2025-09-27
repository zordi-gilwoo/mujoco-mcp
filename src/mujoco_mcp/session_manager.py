"""
Session Manager for Multi-Client MuJoCo MCP Server
Handles client session isolation and resource management
Enhanced with ProcessManager integration for isolated processes
"""

import logging
import time
import threading
import uuid
from typing import Dict, Optional, Any
from dataclasses import dataclass

from .viewer_client import MuJoCoViewerClient, ViewerManager, viewer_manager
from .process_manager import test_process_manager as process_manager

logger = logging.getLogger("mujoco_mcp.session_manager")


@dataclass
class SessionInfo:
    """Information about an active MCP client session"""
    session_id: str
    client_id: str
    created_at: float
    last_activity: float
    viewer_port: int
    active_models: Dict[str, str]  # model_id -> scene_type/name
    use_isolated_process: bool = False  # Whether to use isolated processes
    
    def update_activity(self):
        """Update last activity timestamp"""
        self.last_activity = time.time()
    
    def is_stale(self, timeout: float = 1800.0) -> bool:
        """Check if session is stale (inactive for too long)"""
        return (time.time() - self.last_activity) > timeout


class SessionManager:
    """Manages MCP client sessions and their associated resources"""
    
    def __init__(self, use_isolated_processes: bool = True):
        self.sessions: Dict[str, SessionInfo] = {}
        self._lock = threading.RLock()
        self._base_port = 8888
        self._port_counter = 0
        self.use_isolated_processes = use_isolated_processes
        
    def get_or_create_session(self, context: Any = None, use_isolated_process: bool = None) -> SessionInfo:
        """Get existing session or create new one for the MCP client"""
        session_id = self._extract_session_id(context)
        
        with self._lock:
            if session_id in self.sessions:
                session = self.sessions[session_id]
                session.update_activity()
                return session
            
            # Create new session
            client_id = f"client_{len(self.sessions) + 1}_{uuid.uuid4().hex[:8]}"
            
            # Determine if we should use isolated processes
            if use_isolated_process is None:
                use_isolated_process = self.use_isolated_processes
            
            if use_isolated_process:
                # ProcessManager will allocate its own port
                viewer_port = 0  # Will be set when process is spawned
            else:
                # Use traditional port allocation
                viewer_port = self._allocate_port()
            
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
            logger.info(f"Created new session {session_id} for client {client_id} "
                       f"{'with isolated process' if use_isolated_process else 'on port ' + str(viewer_port)}")
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
        """Extract session ID from MCP context or generate one"""
        if context and hasattr(context, 'session_id'):
            return context.session_id
        elif context and isinstance(context, dict) and 'session_id' in context:
            return context['session_id']
        else:
            # For now, create a simple session ID based on thread
            # In a real MCP server, this would come from the connection context
            thread_id = threading.current_thread().ident
            return f"session_{thread_id}"
    
    def _allocate_port(self) -> int:
        """Allocate a unique port for viewer client"""
        self._port_counter += 1
        return self._base_port + self._port_counter


# Global session manager instance
session_manager = SessionManager()