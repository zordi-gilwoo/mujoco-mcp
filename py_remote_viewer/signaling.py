"""WebRTC signaling server using WebSocket endpoints."""

import json
import asyncio
import logging
from typing import Dict, Set, Optional
from fastapi import WebSocket, WebSocketDisconnect
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaPlayer

from .config import ViewerConfig
from .webrtc_track import MuJoCoVideoTrack
from .events import EventProtocol
from .mujoco_simulation import MuJoCoSimulation

logger = logging.getLogger(__name__)


class SignalingServer:
    """WebRTC signaling server for remote viewer connections."""
    
    def __init__(self, config: ViewerConfig):
        self.config = config
        
        # Always use real MuJoCo simulation with graceful error handling
        print("[SignalingServer] Initializing MuJoCo simulation")
        try:
            self.simulation = MuJoCoSimulation()
            print("[SignalingServer] MuJoCo simulation initialized successfully")
        except Exception as e:
            print(f"[SignalingServer] WARNING: MuJoCo simulation failed to initialize: {e}")
            print("[SignalingServer] Server will continue but video frames may show errors")
            # Still create the simulation object but it will handle errors gracefully
            self.simulation = MuJoCoSimulation()
        
        # Connection management
        self.peer_connections: Dict[str, RTCPeerConnection] = {}
        self.websockets: Dict[str, WebSocket] = {}
        self.client_counter = 0
        
        # Metrics
        self.stats = {
            "clients_connected": 0,
            "total_connections": 0,
            "frames_sent": 0,
            "events_received": 0,
        }
        
        # Scene management
        self.current_scene_xml = None
        
        # Metrics reporter will be started when needed
        self._metrics_task = None
    
    async def handle_websocket(self, websocket: WebSocket):
        """Handle a new WebSocket connection for signaling.
        
        Args:
            websocket: FastAPI WebSocket connection
        """
        client_id = f"client_{self.client_counter}"
        self.client_counter += 1
        
        await websocket.accept()
        self.websockets[client_id] = websocket
        self.stats["clients_connected"] += 1
        self.stats["total_connections"] += 1
        
        # Start metrics reporter on first connection
        if self._metrics_task is None:
            self._metrics_task = asyncio.create_task(self._metrics_reporter())
        
        logger.info(f"Client {client_id} connected via WebSocket")
        
        try:
            while True:
                # Receive message from client
                data = await websocket.receive_text()
                message = json.loads(data)
                
                # Handle different message types
                await self._handle_signaling_message(client_id, message)
                
        except WebSocketDisconnect:
            logger.info(f"Client {client_id} disconnected")
        except Exception as e:
            logger.error(f"Error handling client {client_id}: {e}")
        finally:
            await self._cleanup_client(client_id)
    
    async def _handle_signaling_message(self, client_id: str, message: Dict):
        """Handle signaling messages from clients.
        
        Args:
            client_id: Unique client identifier
            message: Parsed JSON message from client
        """
        msg_type = message.get("type")
        
        if msg_type == "offer":
            await self._handle_offer(client_id, message)
        elif msg_type == "ice-candidate":
            await self._handle_ice_candidate(client_id, message)
        elif msg_type == "event":
            await self._handle_event(client_id, message)
        else:
            logger.warning(f"Unknown message type from {client_id}: {msg_type}")
    
    async def _handle_offer(self, client_id: str, message: Dict):
        """Handle WebRTC offer from client.
        
        Args:
            client_id: Client identifier
            message: Offer message containing SDP
        """
        logger.info(f"Handling offer from {client_id}")
        
        # Create peer connection with explicit configuration object
        configuration = RTCConfiguration(
            iceServers=[RTCIceServer(urls=[self.config.stun_server])]
        )
        pc = RTCPeerConnection(configuration=configuration)
        
        # Store the peer connection
        self.peer_connections[client_id] = pc
        
        # Set up event handlers
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"Connection state for {client_id}: {pc.connectionState}")
            if pc.connectionState == "closed":
                await self._cleanup_client(client_id)
        
        @pc.on("icecandidate")
        async def on_icecandidate(candidate):
            if candidate:
                await self._send_to_client(client_id, {
                    "type": "ice-candidate",
                    "candidate": candidate.to_dict()
                })
        
        # Add video track - always use MuJoCo
        video_track = MuJoCoVideoTrack(self.config, self.simulation)
        
        pc.addTrack(video_track)
        
        # Set remote description (offer)
        offer = RTCSessionDescription(
            sdp=message["sdp"],
            type=message["type"]
        )
        await pc.setRemoteDescription(offer)
        
        # Create answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        # Send answer to client
        await self._send_to_client(client_id, {
            "type": "answer",
            "sdp": pc.localDescription.sdp
        })
        
        logger.info(f"Sent answer to {client_id}")
    
    async def _handle_ice_candidate(self, client_id: str, message: Dict):
        """Handle ICE candidate from client.
        
        Args:
            client_id: Client identifier
            message: ICE candidate message
        """
        pc = self.peer_connections.get(client_id)
        if pc:
            candidate = message.get("candidate")
            if candidate:
                await pc.addIceCandidate(candidate)
    
    async def _handle_event(self, client_id: str, message: Dict):
        """Handle input event from client.
        
        Args:
            client_id: Client identifier  
            message: Event message containing event data
        """
        event_data = message.get("data")
        if event_data:
            # Parse event using protocol
            event = EventProtocol.parse_event(event_data)
            if event:
                # Forward to simulation
                handled = self.simulation.handle_event(event)
                logger.info(f"Event from {client_id}: {event.type.value} (handled: {handled})")
                self.stats["events_received"] += 1
            else:
                logger.warning(f"Failed to parse event from {client_id}: {event_data}")
    
    async def _send_to_client(self, client_id: str, message: Dict):
        """Send message to specific client.
        
        Args:
            client_id: Client identifier
            message: Message to send
        """
        websocket = self.websockets.get(client_id)
        if websocket:
            try:
                await websocket.send_text(json.dumps(message))
            except Exception as e:
                logger.error(f"Failed to send message to {client_id}: {e}")
                await self._cleanup_client(client_id)
    
    async def _cleanup_client(self, client_id: str):
        """Clean up client connection and resources.
        
        Args:
            client_id: Client identifier
        """
        # Close peer connection
        pc = self.peer_connections.pop(client_id, None)
        if pc:
            await pc.close()
        
        # Remove websocket
        self.websockets.pop(client_id, None)
        
        # Update stats
        if self.stats["clients_connected"] > 0:
            self.stats["clients_connected"] -= 1
        
        logger.info(f"Cleaned up client {client_id}")
    
    async def _metrics_reporter(self):
        """Periodically report server metrics."""
        while True:
            await asyncio.sleep(5.0)  # Report every 5 seconds
            
            logger.info(f"[Metrics] Connected clients: {self.stats['clients_connected']}, "
                       f"Total connections: {self.stats['total_connections']}, "
                       f"Events received: {self.stats['events_received']}")
    
    def get_stats(self) -> Dict:
        """Get current server statistics.
        
        Returns:
            Dict containing server metrics
        """
        return {
            **self.stats,
            "simulation_state": self.simulation.get_state(),
        }
    
    async def broadcast_scene_update(self, scene_xml: str):
        """Broadcast scene update to all connected clients.
        
        Args:
            scene_xml: MuJoCo XML content for the scene
        """
        self.current_scene_xml = scene_xml
        
        # Try to load the scene in the simulation
        try:
            if hasattr(self.simulation, 'load_model'):
                success = self.simulation.load_model(scene_xml)
                if success:
                    logger.info("Scene loaded into simulation successfully")
                else:
                    logger.warning("Scene loading returned False - may have failed")
            else:
                logger.warning("Simulation does not support model loading")
        except Exception as e:
            logger.error(f"Failed to load scene into simulation: {e}")
        
        # Broadcast to all connected WebSocket clients
        message = {
            "type": "scene_updated",
            "xml": scene_xml
        }
        
        disconnected_clients = []
        for client_id, websocket in self.websockets.items():
            try:
                await self._send_to_client(client_id, message)
            except Exception as e:
                logger.warning(f"Failed to send scene update to client {client_id}: {e}")
                disconnected_clients.append(client_id)
        
        # Clean up disconnected clients
        for client_id in disconnected_clients:
            await self._cleanup_client(client_id)
