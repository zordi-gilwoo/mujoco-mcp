"""WebRTC signaling server using WebSocket endpoints."""

import json
import re
import asyncio
import logging
from typing import Any, Dict, Set, Optional, Tuple
from fastapi import WebSocket, WebSocketDisconnect
from aiortc import (
    RTCPeerConnection,
    RTCSessionDescription,
    RTCConfiguration,
    RTCIceServer,
)
from aiortc.sdp import candidate_from_sdp
from aiortc.contrib.media import MediaPlayer

from .config import ViewerConfig
from .webrtc_track import MuJoCoVideoTrack
from .events import EventProtocol
from .mujoco_simulation import MuJoCoSimulation
from .builtin_scenes import get_builtin_scene
from src.mujoco_mcp.menagerie_loader import MenagerieLoader

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
        self.menagerie_loader = MenagerieLoader()

        # LLM Integration
        self.llm_config = {}
        self.llm_generator = None
        self._setup_llm_components()

        # Metrics reporter will be started when needed
        self._metrics_task = None
        self._command_lock = asyncio.Lock()
    
    def _setup_llm_components(self):
        """Setup LLM integration components."""
        try:
            # Import LLM components
            import sys
            from pathlib import Path
            project_root = Path(__file__).parent.parent.parent
            sys.path.insert(0, str(project_root / "src"))
            
            from mujoco_mcp.scene_gen.llm_scene_generator import LLMSceneGenerator
            from mujoco_mcp.scene_gen.metadata_extractor import MetadataExtractor
            
            # Initialize metadata extractor and LLM generator
            metadata_extractor = MetadataExtractor()
            self.llm_generator = LLMSceneGenerator(metadata_extractor)
            print("[SignalingServer] LLM integration components initialized")
            
        except Exception as e:
            print(f"[SignalingServer] LLM integration not available: {e}")
            self.llm_generator = None
    
    async def setup_llm_integration(self, provider: str, api_key: str):
        """Setup LLM integration with specific provider and API key."""
        if not self.llm_generator:
            print("[SignalingServer] LLM generator not available")
            return False
        
        try:
            self.llm_generator.set_provider_config(provider, api_key)
            print(f"[SignalingServer] LLM integration configured for {provider}")
            return True
        except Exception as e:
            print(f"[SignalingServer] Failed to configure LLM integration: {e}")
            return False
    
    async def generate_scene_from_prompt(self, prompt: str) -> Dict[str, Any]:
        """Generate scene XML from natural language prompt using LLM."""
        if not self.llm_generator:
            return {
                "success": False,
                "error": "LLM integration not available. Using basic scene templates instead.",
                "scene_xml": self._generate_basic_scene_from_prompt(prompt)
            }
        
        try:
            # Use LLM to generate scene
            scene_description = await asyncio.get_event_loop().run_in_executor(
                None, self.llm_generator.generate_scene_description, prompt
            )
            
            if scene_description and hasattr(scene_description, 'to_xml'):
                scene_xml = scene_description.to_xml()
                
                # Store and broadcast the generated scene
                self.current_scene_xml = scene_xml
                await self.broadcast_scene_update(scene_xml)
                
                return {
                    "success": True,
                    "scene_xml": scene_xml,
                    "message": "Scene generated successfully using LLM"
                }
            else:
                # Fallback to basic scene generation
                scene_xml = self._generate_basic_scene_from_prompt(prompt)
                return {
                    "success": True,
                    "scene_xml": scene_xml,
                    "message": "Scene generated using fallback method"
                }
                
        except Exception as e:
            print(f"[SignalingServer] LLM scene generation failed: {e}")
            # Fallback to basic scene generation
            scene_xml = self._generate_basic_scene_from_prompt(prompt)
            return {
                "success": True,
                "scene_xml": scene_xml,
                "message": f"LLM generation failed, using fallback: {str(e)}"
            }
    
    def _generate_basic_scene_from_prompt(self, prompt: str) -> str:
        """Generate basic scene XML from prompt using simple keyword matching."""
        prompt_lower = prompt.lower()
        
        if "pendulum" in prompt_lower:
            if "double" in prompt_lower:
                return '''<mujoco>
    <worldbody>
        <body name="pole1" pos="0 0 1">
            <joint name="hinge1" type="hinge" axis="1 0 0"/>
            <geom name="pole1" type="capsule" size="0.02 0.4" rgba="0.8 0.2 0.2 1"/>
            <body name="pole2" pos="0 0 -0.4">
                <joint name="hinge2" type="hinge" axis="1 0 0"/>
                <geom name="pole2" type="capsule" size="0.02 0.4" rgba="0.2 0.8 0.2 1"/>
                <body name="mass" pos="0 0 -0.4">
                    <geom name="mass" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                </body>
            </body>
        </body>
    </worldbody>
</mujoco>'''
            else:
                return '''<mujoco>
    <worldbody>
        <body name="pendulum" pos="0 0 1">
            <joint name="hinge" type="hinge" axis="1 0 0"/>
            <geom name="rod" type="capsule" size="0.02 0.5" rgba="0.8 0.2 0.2 1"/>
            <body name="bob" pos="0 0 -0.5">
                <geom name="bob" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>'''
        elif "cart" in prompt_lower and "pole" in prompt_lower:
            return '''<mujoco>
    <worldbody>
        <body name="cart" pos="0 0 0.1">
            <joint name="slider" type="slide" axis="1 0 0"/>
            <geom name="cart" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
            <body name="pole" pos="0 0 0.1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom name="pole" type="capsule" size="0.02 0.5" rgba="0.2 0.8 0.2 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>'''
        elif "box" in prompt_lower or "cube" in prompt_lower:
            return '''<mujoco>
    <worldbody>
        <body name="box" pos="0 0 0.5">
            <freejoint/>
            <geom name="box" type="box" size="0.1 0.1 0.1" rgba="0.8 0.2 0.2 1"/>
        </body>
    </worldbody>
</mujoco>'''
        else:
            # Default to simple pendulum
            return '''<mujoco>
    <worldbody>
        <body name="pendulum" pos="0 0 1">
            <joint name="hinge" type="hinge" axis="1 0 0"/>
            <geom name="rod" type="capsule" size="0.02 0.5" rgba="0.8 0.2 0.2 1"/>
            <body name="bob" pos="0 0 -0.5">
                <geom name="bob" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
            </body>
        </body>
    </worldbody>
</mujoco>'''
    
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
        except Exception:
            logger.exception(f"Error handling client {client_id}")
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
            iceServers=[RTCIceServer(urls=[self.config.stun_server])],
            bundlePolicy="balanced",
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

        # Set remote description (offer)
        offer = RTCSessionDescription(
            sdp=message["sdp"],
            type=message["type"]
        )
        logger.info(f"Setting remote description for {client_id}")
        await pc.setRemoteDescription(offer)

        # Inspect transceivers contributed by the offer
        for idx, transceiver in enumerate(pc.getTransceivers()):
            logger.info(
                "Offer transceiver %s: kind=%s direction=%s offer_direction=%s sender=%s",
                idx,
                transceiver.kind,
                getattr(transceiver, "direction", None),
                getattr(transceiver, "_offerDirection", None),
                transceiver.sender,
            )

        # Select the video transceiver provided by the client's offer
        video_transceiver = None
        for transceiver in pc.getTransceivers():
            if transceiver.kind == "video":
                video_transceiver = transceiver
                break

        if video_transceiver is None:
            logger.error(
                "No video transceiver present in client offer for %s; SDP:\n%s",
                client_id,
                offer.sdp,
            )
            await self._cleanup_client(client_id)
            return

        # Attach MuJoCo video track
        video_track = MuJoCoVideoTrack(self.config, self.simulation)
        logger.info(f"Attaching MuJoCo video track for {client_id}")
        video_transceiver.direction = "sendonly"

        if video_transceiver.sender:
            video_transceiver.sender.replaceTrack(video_track)
        else:
            logger.info("Video transceiver missing sender; adding track explicitly")
            pc.addTrack(video_track)

        # Inspect transceivers after configuring track
        for idx, transceiver in enumerate(pc.getTransceivers()):
            logger.info(
                "Post-config transceiver %s: kind=%s direction=%s offer_direction=%s sender=%s",
                idx,
                transceiver.kind,
                getattr(transceiver, "direction", None),
                getattr(transceiver, "_offerDirection", None),
                transceiver.sender,
            )

        # Create answer
        logger.info(f"Creating answer for {client_id}")
        answer = await pc.createAnswer()
        logger.info(f"Setting local description for {client_id}")
        await pc.setLocalDescription(answer)

        # Send answer to client
        logger.info(f"Sending answer to {client_id}")
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
            candidate_payload = message.get("candidate")
            if candidate_payload is None:
                await pc.addIceCandidate(None)
                return

            candidate_sdp = candidate_payload.get("candidate")
            if not candidate_sdp:
                await pc.addIceCandidate(None)
                return

            if candidate_sdp.startswith("candidate:"):
                candidate_sdp = candidate_sdp.split(":", 1)[1]

            rtc_candidate = candidate_from_sdp(candidate_sdp)
            rtc_candidate.sdpMid = candidate_payload.get("sdpMid")
            rtc_candidate.sdpMLineIndex = candidate_payload.get("sdpMLineIndex")
            rtc_candidate.usernameFragment = candidate_payload.get("usernameFragment")

            await pc.addIceCandidate(rtc_candidate)
    
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

    async def process_text_command(self, command: str) -> Dict[str, Any]:
        """Execute simple text commands against the simulation."""
        trimmed = (command or "").strip()
        if not trimmed:
            return {"success": False, "error": "Command is required"}

        command_lower = trimmed.lower()

        async with self._command_lock:
            actions = []
            response_payload: Dict[str, Any] = {}

            try:
                if any(keyword in command_lower for keyword in ("play", "start", "resume")):
                    self.simulation.play()
                    actions.append("play_simulation")
                    message = "Simulation is now playing."

                elif "pause" in command_lower:
                    self.simulation.pause()
                    actions.append("pause_simulation")
                    message = "Simulation paused."

                elif "reset" in command_lower:
                    self.simulation.reset()
                    actions.append("reset_simulation")
                    message = "Simulation reset to its initial state."

                elif "stop" in command_lower:
                    self.simulation.stop()
                    actions.append("stop_simulation")
                    message = "Simulation stopped."

                elif any(keyword in command_lower for keyword in ("step", "advance", "tick")):
                    match = re.search(r"(\d+)", command_lower)
                    steps = int(match.group(1)) if match else 1
                    steps = max(1, steps)
                    self.simulation.step(steps, force=True)
                    actions.append("stepped_simulation")
                    message = f"Stepped simulation {steps} step{'s' if steps != 1 else ''}."

                elif any(keyword in command_lower for keyword in ("state", "status", "info")):
                    response_payload["state"] = self.simulation.get_state()
                    actions.append("reported_state")
                    message = "Retrieved current simulation state."

                else:
                    # Try LLM-based scene generation first
                    if self.llm_generator and any(keyword in command_lower for keyword in ("create", "generate", "make", "build")):
                        try:
                            scene_result = await self.generate_scene_from_prompt(trimmed)
                            if scene_result.get('success') and scene_result.get('scene_xml'):
                                response_payload["scene_xml"] = scene_result['scene_xml']
                                actions.append("generated_llm_scene")
                                message = scene_result.get('message', 'Scene generated using LLM')
                            else:
                                return {"success": False, "error": scene_result.get('error', 'Failed to generate scene')}
                        except Exception as e:
                            logger.error(f"LLM scene generation failed: {e}")
                            # Continue to fallback logic below
                    
                    # Fallback to builtin scene matching
                    builtin_scene = self._match_builtin_scene(command_lower)
                    if builtin_scene:
                        if builtin_scene == "franka_panda":
                            success, scene_xml, scene_path = self._load_menagerie_scene("franka_emika_panda")
                            if not success:
                                return {"success": False, "error": scene_path}

                            if scene_xml:
                                await self.broadcast_scene_update(scene_xml, load_into_simulation=False)
                                response_payload["scene_xml"] = scene_xml
                            response_payload["scene_name"] = "franka_emika_panda"
                            response_payload["scene_path"] = scene_path

                            actions.append("created_menagerie_scene")
                            message = "Loaded Franka Panda menagerie scene."
                        else:
                            success = self.simulation.load_builtin_scene(builtin_scene)
                            if not success:
                                return {
                                    "success": False,
                                    "error": f"Failed to load built-in scene '{builtin_scene}'",
                                }

                            scene_xml = get_builtin_scene(builtin_scene)
                            if scene_xml:
                                await self.broadcast_scene_update(scene_xml, load_into_simulation=False)
                                response_payload["scene_xml"] = scene_xml
                                response_payload["scene_name"] = builtin_scene

                            actions.append("created_scene")
                            pretty_name = builtin_scene.replace("_", " ")
                            message = f"Loaded built-in {pretty_name} scene."
                    elif not actions:  # Only if no action was taken above
                        # If LLM is available, try to generate scene from any unrecognized command
                        if self.llm_generator:
                            try:
                                scene_result = await self.generate_scene_from_prompt(trimmed)
                                if scene_result.get('success') and scene_result.get('scene_xml'):
                                    response_payload["scene_xml"] = scene_result['scene_xml']
                                    actions.append("generated_llm_scene")
                                    message = scene_result.get('message', 'Scene generated using LLM')
                                else:
                                    return {"success": False, "error": scene_result.get('error', 'Failed to generate scene')}
                            except Exception as e:
                                return {"success": False, "error": f"Failed to process command: {str(e)}"}
                        else:
                            return {
                                "success": False,
                                "error": f"Unsupported command: {trimmed}. LLM integration not available.",
                            }

                self.stats["events_received"] += 1
                logger.info("Processed text command '%s' -> %s", trimmed, message)

                return {
                    "success": True,
                    "result": message,
                    "actions_taken": actions,
                    **response_payload,
                }

            except Exception as exc:  # pragma: no cover - defensive logging
                logger.exception("Failed to process command '%s'", trimmed)
                return {"success": False, "error": str(exc)}

    def _match_builtin_scene(self, command_lower: str) -> Optional[str]:
        """Best-effort mapping from natural language command to a scene key."""
        if any(keyword in command_lower for keyword in ("cart pole", "cart-pole", "cartpole")):
            return "cartpole"
        if "pendulum" in command_lower:
            return "pendulum"
        if any(keyword in command_lower for keyword in ("franka", "panda")):
            return "franka_panda"
        return None

    def _load_menagerie_scene(self, model_name: str) -> Tuple[bool, Optional[str], str]:
        """Load a Menagerie model by name into the simulation."""

        try:
            scene_path = self.menagerie_loader.get_model_file(model_name)
        except FileNotFoundError as exc:
            return False, None, str(exc)
        except Exception as exc:  # pragma: no cover - defensive
            logger.exception("Failed to locate menagerie model '%s'", model_name)
            return False, None, str(exc)

        if not self.simulation.load_model(str(scene_path)):
            hint = ""
            if self.menagerie_loader.menagerie_root is None:
                hint = (
                    " Ensure MUJOCO_MENAGERIE_PATH points to a local mujoco_menagerie checkout "
                    "so meshes and textures are available."
                )
            return False, None, f"Failed to load menagerie scene from {scene_path}.{hint}"

        try:
            xml_text = scene_path.read_text()
        except Exception as exc:
            logger.warning("Loaded scene from %s but failed to read XML: %s", scene_path, exc)
            xml_text = None

        return True, xml_text, str(scene_path)

    async def broadcast_scene_update(self, scene_xml: str, *, load_into_simulation: bool = True):
        """Broadcast scene update to all connected clients.

        Args:
            scene_xml: MuJoCo XML content for the scene
            load_into_simulation: Whether to reload the scene in the running simulation
        """
        self.current_scene_xml = scene_xml

        # Try to load the scene in the simulation
        if load_into_simulation:
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
