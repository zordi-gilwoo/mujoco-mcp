"""
Process Manager for Multi-Client MuJoCo MCP Server
Spawns isolated Python processes for each client with dedicated ports
"""

import subprocess
import socket
import os
import time
import logging
import signal
import threading
from multiprocessing import Process
from typing import Dict, Any, Optional, List
from dataclasses import dataclass

logger = logging.getLogger("mujoco_mcp.process_manager")


@dataclass
class ProcessInfo:
    """Information about a spawned viewer process"""
    process: subprocess.Popen
    port: int
    pid: int
    url: str
    session_id: str
    created_at: float
    last_health_check: float
    
    @property
    def is_running(self) -> bool:
        """Check if process is still running"""
        return self.process.poll() is None
    
    @property
    def age_seconds(self) -> float:
        """Get process age in seconds"""
        return time.time() - self.created_at


class ProcessManager:
    """Manages isolated MuJoCo viewer processes for multiple clients"""
    
    def __init__(self, port_range_start: int = 8001, port_range_end: int = 9000, use_mock_server: bool = False):
        self.processes: Dict[str, ProcessInfo] = {}
        self.port_range = range(port_range_start, port_range_end + 1)
        self.used_ports = set()
        self._lock = threading.RLock()
        self._health_check_interval = 30.0  # seconds
        self._health_check_thread = None
        self._shutdown_event = threading.Event()
        self.use_mock_server = use_mock_server
        
        # Start health monitoring thread
        self._start_health_monitoring()
        
        logger.info(f"ProcessManager initialized with port range {port_range_start}-{port_range_end}"
                   f"{' (using mock server)' if use_mock_server else ''}")
    
    def spawn_viewer_process(self, session_id: str, isolated: bool = True) -> Optional[ProcessInfo]:
        """Spawn isolated MuJoCo viewer process for a session"""
        with self._lock:
            # Check if process already exists for this session
            if session_id in self.processes:
                existing = self.processes[session_id]
                if existing.is_running:
                    logger.info(f"Process already exists for session {session_id}")
                    return existing
                else:
                    # Clean up dead process
                    self._cleanup_process(session_id)
            
            # Get free port
            port = self._get_free_port()
            if port is None:
                logger.error("No free ports available")
                return None
            
            try:
                # Prepare environment variables
                env = os.environ.copy()
                env['MUJOCO_SESSION_ID'] = session_id
                env['MUJOCO_VIEWER_PORT'] = str(port)
                
                # Command to start isolated viewer server
                if self.use_mock_server:
                    cmd = [
                        'python', '-m', 'mujoco_mcp.mock_viewer_server',
                        '--port', str(port),
                        '--session-id', session_id
                    ]
                else:
                    cmd = [
                        'python', '-m', 'mujoco_mcp.viewer_server',
                        '--port', str(port),
                        '--session-id', session_id
                    ]
                
                if isolated:
                    cmd.append('--isolated')
                
                # Start the process
                project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                env['PYTHONPATH'] = os.path.join(project_root, 'src')
                
                process = subprocess.Popen(
                    cmd,
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    cwd=project_root
                )
                
                # Create process info
                process_info = ProcessInfo(
                    process=process,
                    port=port,
                    pid=process.pid,
                    url=f"http://localhost:{port}",
                    session_id=session_id,
                    created_at=time.time(),
                    last_health_check=time.time()
                )
                
                self.processes[session_id] = process_info
                self.used_ports.add(port)
                
                logger.info(f"Spawned viewer process for session {session_id} on port {port} (PID: {process.pid})")
                
                # Give process time to start
                time.sleep(2)
                
                # Verify process is still running
                if not process_info.is_running:
                    # Get error output if process died
                    try:
                        stdout, stderr = process.communicate(timeout=1)
                        logger.error(f"Process for session {session_id} died immediately")
                        logger.error(f"STDOUT: {stdout.decode() if stdout else 'None'}")
                        logger.error(f"STDERR: {stderr.decode() if stderr else 'None'}")
                    except subprocess.TimeoutExpired:
                        logger.error(f"Process for session {session_id} died immediately (timeout getting output)")
                    
                    self._cleanup_process(session_id)
                    return None
                
                return process_info
                
            except Exception as e:
                logger.error(f"Failed to spawn process for session {session_id}: {e}")
                if port in self.used_ports:
                    self.used_ports.remove(port)
                return None
    
    def terminate_process(self, session_id: str) -> bool:
        """Terminate a specific viewer process"""
        with self._lock:
            if session_id not in self.processes:
                logger.warning(f"No process found for session {session_id}")
                return False
            
            process_info = self.processes[session_id]
            
            try:
                # Try graceful shutdown first
                if process_info.is_running:
                    process_info.process.terminate()
                    
                    # Wait up to 5 seconds for graceful shutdown
                    try:
                        process_info.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        # Force kill if graceful shutdown failed
                        logger.warning(f"Forcefully killing process for session {session_id}")
                        process_info.process.kill()
                        process_info.process.wait()
                
                self._cleanup_process(session_id)
                logger.info(f"Terminated process for session {session_id}")
                return True
                
            except Exception as e:
                logger.error(f"Error terminating process for session {session_id}: {e}")
                return False
    
    def get_process_info(self, session_id: str) -> Optional[ProcessInfo]:
        """Get process information for a session"""
        with self._lock:
            return self.processes.get(session_id)
    
    def list_processes(self) -> Dict[str, Dict[str, Any]]:
        """List all active processes"""
        with self._lock:
            result = {}
            for session_id, process_info in self.processes.items():
                result[session_id] = {
                    'port': process_info.port,
                    'pid': process_info.pid,
                    'url': process_info.url,
                    'is_running': process_info.is_running,
                    'age_seconds': process_info.age_seconds,
                    'created_at': process_info.created_at,
                    'last_health_check': process_info.last_health_check
                }
            return result
    
    def cleanup_all(self):
        """Clean up all processes"""
        with self._lock:
            session_ids = list(self.processes.keys())
            for session_id in session_ids:
                self.terminate_process(session_id)
        
        # Stop health monitoring
        self._shutdown_event.set()
        if self._health_check_thread and self._health_check_thread.is_alive():
            self._health_check_thread.join(timeout=5)
    
    def _get_free_port(self) -> Optional[int]:
        """Find a free port in the configured range"""
        for port in self.port_range:
            if port not in self.used_ports:
                # Test if port is actually free
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    result = sock.connect_ex(('localhost', port))
                    if result != 0:  # Port is free
                        return port
                finally:
                    sock.close()
        
        logger.error("No free ports available in range")
        return None
    
    def _cleanup_process(self, session_id: str):
        """Internal cleanup of process resources"""
        if session_id in self.processes:
            process_info = self.processes[session_id]
            
            # Remove port from used ports
            if process_info.port in self.used_ports:
                self.used_ports.remove(process_info.port)
            
            # Remove from processes dict
            del self.processes[session_id]
    
    def _start_health_monitoring(self):
        """Start background thread for health monitoring"""
        def health_monitor():
            while not self._shutdown_event.is_set():
                try:
                    self._perform_health_checks()
                    time.sleep(self._health_check_interval)
                except Exception as e:
                    logger.error(f"Health monitoring error: {e}")
        
        self._health_check_thread = threading.Thread(target=health_monitor, daemon=True)
        self._health_check_thread.start()
        logger.info("Started health monitoring thread")
    
    def _perform_health_checks(self):
        """Perform health checks on all processes"""
        with self._lock:
            dead_sessions = []
            
            for session_id, process_info in self.processes.items():
                current_time = time.time()
                
                # Check if process is still running
                if not process_info.is_running:
                    logger.warning(f"Process for session {session_id} has died")
                    dead_sessions.append(session_id)
                    continue
                
                # Update health check timestamp
                process_info.last_health_check = current_time
                
                # Optional: Check if process is responsive via port
                # This could be enhanced to do actual HTTP/socket health checks
                
            # Clean up dead processes
            for session_id in dead_sessions:
                self._cleanup_process(session_id)
    
    def get_stats(self) -> Dict[str, Any]:
        """Get manager statistics"""
        with self._lock:
            running_count = sum(1 for p in self.processes.values() if p.is_running)
            return {
                'total_processes': len(self.processes),
                'running_processes': running_count,
                'used_ports': len(self.used_ports),
                'available_ports': len(self.port_range) - len(self.used_ports),
                'port_range': f"{self.port_range.start}-{self.port_range.stop-1}",
                'health_check_interval': self._health_check_interval
            }


# Global process manager instance
process_manager = ProcessManager()

# Test process manager with mock server
test_process_manager = ProcessManager(use_mock_server=True)