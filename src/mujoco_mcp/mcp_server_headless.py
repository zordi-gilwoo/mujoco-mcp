#!/usr/bin/env python3
"""
MuJoCo MCP Server - Headless Mode
Works without GUI/display requirements with EGL GPU rendering support
"""

import asyncio
import json
import base64
import io
from typing import Dict, Any, List, Optional
import logging

from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

import mujoco
import numpy as np

from .version import __version__
from .egl_renderer import EGLRenderer, create_egl_renderer, check_egl_support
from .h264_encoder import H264Encoder, StreamingEncoder, check_h264_support

try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("mujoco-mcp-headless")

# Create server instance
server = Server("mujoco-mcp-headless")

# Global simulation storage (no viewer needed)
simulations = {}


class HeadlessSimulation:
    """Headless MuJoCo simulation with EGL GPU rendering support"""

    def __init__(self, model_id: str, xml_string: str):
        self.model_id = model_id
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)
        self.viewer = None  # No viewer in headless mode
        
        # EGL rendering support
        self._egl_renderer = None
        self._software_renderer = None
        self._use_egl = False
        
        # Video encoding support
        self._h264_encoder = None
        self._streaming_encoder = None
        
    def enable_egl_rendering(self, width: int = 640, height: int = 480) -> bool:
        """Enable EGL-based GPU rendering"""
        try:
            self._egl_renderer = create_egl_renderer(self.model, width, height)
            if self._egl_renderer:
                self._use_egl = True
                logger.info(f"EGL rendering enabled for model {self.model_id}")
                return True
        except Exception as e:
            logger.warning(f"Failed to enable EGL rendering: {e}")
            
        self._use_egl = False
        return False
        
    def render_frame(self, width: int = 640, height: int = 480, camera_id: int = -1) -> np.ndarray:
        """Render frame using EGL or fallback to software rendering"""
        if self._use_egl and self._egl_renderer:
            try:
                with self._egl_renderer:
                    return self._egl_renderer.render(self.data, camera_id)
            except Exception as e:
                logger.warning(f"EGL rendering failed, falling back to software: {e}")
                self._use_egl = False
                
        # Fallback to software rendering
        return self._render_software(width, height, camera_id)
        
    def _render_software(self, width: int = 640, height: int = 480, camera_id: int = -1) -> np.ndarray:
        """Software rendering fallback"""
        try:
            if not self._software_renderer:
                self._software_renderer = mujoco.Renderer(self.model, height, width)
                
            self._software_renderer.update_scene(self.data, camera=camera_id)
            return self._software_renderer.render()
            
        except Exception as e:
            logger.warning(f"Software rendering failed: {e}")
            # Create a placeholder image
            return np.ones((height, width, 3), dtype=np.uint8) * 128
            
    def setup_h264_encoder(self, width: int = 640, height: int = 480, fps: int = 30, bitrate: str = "2M"):
        """Setup H.264 encoder for video recording"""
        try:
            self._h264_encoder = H264Encoder(width, height, fps, bitrate)
            logger.info(f"H.264 encoder setup for model {self.model_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to setup H.264 encoder: {e}")
            return False
            
    def setup_streaming_encoder(self, width: int = 640, height: int = 480, fps: int = 30, bitrate: str = "1M"):
        """Setup streaming encoder for real-time video"""
        try:
            self._streaming_encoder = StreamingEncoder(width, height, fps, bitrate)
            logger.info(f"Streaming encoder setup for model {self.model_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to setup streaming encoder: {e}")
            return False
            
    def add_frame_to_encoder(self, frame: np.ndarray) -> Optional[bytes]:
        """Add frame to H.264 encoder and return chunk if ready (streaming mode)"""
        if self._streaming_encoder:
            return self._streaming_encoder.add_frame(frame)
        elif self._h264_encoder:
            self._h264_encoder.add_frame(frame)
        return None
        
    def encode_video(self) -> Optional[bytes]:
        """Encode buffered frames to H.264 bytes"""
        if self._h264_encoder:
            try:
                return self._h264_encoder.encode_to_bytes()
            except Exception as e:
                logger.error(f"Video encoding failed: {e}")
        return None

    def close(self):
        """Clean up simulation resources"""
        if self._egl_renderer:
            try:
                self._egl_renderer.__exit__(None, None, None)
            except:
                pass
            self._egl_renderer = None
            
        if self._software_renderer:
            self._software_renderer = None
            
        if self._h264_encoder:
            self._h264_encoder = None
            
        if self._streaming_encoder:
            self._streaming_encoder = None
            
        logger.info(f"Closed simulation {self.model_id}")

    def step(self, steps: int = 1):
        """Step simulation forward"""
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)

    def get_state(self) -> Dict[str, Any]:
        """Get current simulation state"""
        return {
            "time": self.data.time,
            "qpos": self.data.qpos.tolist(),
            "qvel": self.data.qvel.tolist(),
            "ctrl": self.data.ctrl.tolist() if self.model.nu > 0 else [],
            "xpos": self.data.xpos.tolist(),
            "xquat": self.data.xquat.tolist(),
            "nq": self.model.nq,
            "nv": self.model.nv,
            "nu": self.model.nu,
            "nbody": self.model.nbody,
        }

    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)

    def close(self):
        """Clean up (no viewer to close in headless mode)"""


@server.list_tools()
async def handle_list_tools() -> List[types.Tool]:
    """Return list of available MuJoCo MCP tools"""
    return [
        types.Tool(
            name="get_server_info",
            description="Get information about the MuJoCo MCP server",
            inputSchema={"type": "object", "properties": {}, "required": []},
        ),
        types.Tool(
            name="create_scene",
            description="Create a physics simulation scene (headless mode)",
            inputSchema={
                "type": "object",
                "properties": {
                    "scene_type": {
                        "type": "string",
                        "description": "Type of scene to create",
                        "enum": ["pendulum", "double_pendulum", "cart_pole", "arm"],
                    }
                },
                "required": ["scene_type"],
            },
        ),
        types.Tool(
            name="step_simulation",
            description="Step the physics simulation forward",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model to step"},
                    "steps": {
                        "type": "integer",
                        "description": "Number of simulation steps",
                        "default": 1,
                    },
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="get_state",
            description="Get current state of the simulation",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {
                        "type": "string",
                        "description": "ID of the model to get state from",
                    }
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="reset_simulation",
            description="Reset simulation to initial state",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model to reset"}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="close_simulation",
            description="Close and clean up simulation",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model to close"}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="enable_egl_rendering",
            description="Enable EGL-based GPU rendering for headless environments",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model"},
                    "width": {"type": "integer", "description": "Render width", "default": 640},
                    "height": {"type": "integer", "description": "Render height", "default": 480}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="render_egl_frame",
            description="Render frame using EGL GPU acceleration",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model"},
                    "width": {"type": "integer", "description": "Render width", "default": 640},
                    "height": {"type": "integer", "description": "Render height", "default": 480},
                    "camera_id": {"type": "integer", "description": "Camera ID", "default": -1}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="setup_h264_encoder",
            description="Setup H.264 encoder for optimized video streaming",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model"},
                    "width": {"type": "integer", "description": "Video width", "default": 640},
                    "height": {"type": "integer", "description": "Video height", "default": 480},
                    "fps": {"type": "integer", "description": "Frames per second", "default": 30},
                    "bitrate": {"type": "string", "description": "Bitrate (e.g. '2M')", "default": "2M"},
                    "streaming": {"type": "boolean", "description": "Enable streaming mode", "default": False}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="record_h264_video",
            description="Record H.264 video from simulation frames",
            inputSchema={
                "type": "object",
                "properties": {
                    "model_id": {"type": "string", "description": "ID of the model"},
                    "duration": {"type": "number", "description": "Recording duration in seconds", "default": 5.0},
                    "camera_id": {"type": "integer", "description": "Camera ID", "default": -1},
                    "return_bytes": {"type": "boolean", "description": "Return video as base64 bytes", "default": True}
                },
                "required": ["model_id"],
            },
        ),
        types.Tool(
            name="check_gpu_support",
            description="Check EGL and H.264 hardware support capabilities",
            inputSchema={"type": "object", "properties": {}, "required": []},
        ),
    ]


def get_scene_xml(scene_type: str) -> str:
    """Get XML string for different scene types"""

    if scene_type == "pendulum":
        return """
        <mujoco model="pendulum">
            <option gravity="0 0 -9.81" timestep="0.01"/>
            <worldbody>
                <body name="pendulum" pos="0 0 1">
                    <joint name="pivot" type="hinge" axis="0 1 0"/>
                    <geom name="rod" type="capsule" size="0.02" fromto="0 0 0 0 0 -0.5" rgba="0.8 0.2 0.2 1"/>
                    <geom name="mass" pos="0 0 -0.5" type="sphere" size="0.1" rgba="0.2 0.2 0.8 1"/>
                </body>
            </worldbody>
        </mujoco>
        """

    elif scene_type == "double_pendulum":
        return """
        <mujoco model="double_pendulum">
            <option gravity="0 0 -9.81" timestep="0.005"/>
            <worldbody>
                <body name="upper" pos="0 0 1">
                    <joint name="shoulder" type="hinge" axis="0 1 0"/>
                    <geom name="upper_rod" type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3" rgba="0.8 0.2 0.2 1"/>
                    <body name="lower" pos="0 0 -0.3">
                        <joint name="elbow" type="hinge" axis="0 1 0"/>
                        <geom name="lower_rod" type="capsule" size="0.02" fromto="0 0 0 0 0 -0.3" rgba="0.2 0.8 0.2 1"/>
                        <geom name="mass" pos="0 0 -0.3" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                    </body>
                </body>
            </worldbody>
        </mujoco>
        """

    elif scene_type == "cart_pole":
        return """
        <mujoco model="cart_pole">
            <option gravity="0 0 -9.81" timestep="0.02"/>
            <worldbody>
                <body name="cart" pos="0 0 0.5">
                    <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
                    <geom name="cart" type="box" size="0.2 0.1 0.05" rgba="0.3 0.3 0.8 1"/>
                    <body name="pole" pos="0 0 0.05">
                        <joint name="hinge" type="hinge" axis="0 1 0"/>
                        <geom name="pole" type="capsule" size="0.02" fromto="0 0 0 0 0 0.6" rgba="0.8 0.3 0.3 1"/>
                        <geom name="mass" pos="0 0 0.6" type="sphere" size="0.05" rgba="0.2 0.8 0.2 1"/>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <motor name="slide" joint="slider" gear="10"/>
            </actuator>
        </mujoco>
        """

    elif scene_type == "arm":
        return """
        <mujoco model="simple_arm">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <body name="base">
                    <geom type="cylinder" size="0.1 0.02" rgba="0.5 0.5 0.5 1"/>
                    <body name="link1" pos="0 0 0.05">
                        <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                        <geom type="capsule" size="0.03" fromto="0 0 0 0 0 0.2" rgba="0.8 0.2 0.2 1"/>
                        <body name="link2" pos="0 0 0.2">
                            <joint name="joint2" type="hinge" axis="0 1 0" range="-1.57 1.57"/>
                            <geom type="capsule" size="0.025" fromto="0 0 0 0.15 0 0" rgba="0.2 0.8 0.2 1"/>
                        </body>
                    </body>
                </body>
            </worldbody>
            <actuator>
                <motor name="motor1" joint="joint1" gear="50"/>
                <motor name="motor2" joint="joint2" gear="50"/>
            </actuator>
        </mujoco>
        """

    else:
        raise ValueError(f"Unknown scene type: {scene_type}")


@server.call_tool()
async def handle_call_tool(
    name: str, arguments: Dict[str, Any]
) -> List[types.TextContent | types.ImageContent | types.EmbeddedResource]:
    """Handle tool calls"""

    try:
        if name == "get_server_info":
            egl_info = check_egl_support()
            h264_info = check_h264_support()
            
            result = json.dumps(
                {
                    "name": "MuJoCo MCP Server (Headless)",
                    "version": __version__,
                    "description": "Control MuJoCo physics simulations through MCP with GPU rendering and H.264 encoding",
                    "status": "ready",
                    "mode": "headless",
                    "capabilities": [
                        "create_scene",
                        "step_simulation", 
                        "get_state",
                        "reset",
                        "no_viewer_required",
                        "egl_gpu_rendering",
                        "h264_video_encoding",
                        "streaming_video"
                    ],
                    "gpu_support": {
                        "egl_available": egl_info.get("egl_available", False),
                        "gpu_rendering": egl_info.get("gpu_available", False),
                        "hardware_h264": len(h264_info.get("hardware_encoders", [])) > 0,
                        "software_h264": h264_info.get("software_encoder", False)
                    }
                },
                indent=2,
            )

        elif name == "create_scene":
            scene_type = arguments["scene_type"]
            model_id = scene_type

            # Check if already exists
            if model_id in simulations:
                result = (
                    f"⚠️ Scene '{model_id}' already exists. Use a different ID or close it first."
                )
            else:
                # Create headless simulation
                xml_string = get_scene_xml(scene_type)
                sim = HeadlessSimulation(model_id, xml_string)
                simulations[model_id] = sim

                # Get initial state
                state = sim.get_state()

                result = f"✅ Created {scene_type} scene (headless mode)\n"
                result += f"Model ID: {model_id}\n"
                result += f"Degrees of freedom: {state['nq']}\n"
                result += f"Bodies: {state['nbody']}\n"
                result += f"Actuators: {state['nu']}\n"
                result += "Ready for simulation!"

        elif name == "step_simulation":
            model_id = arguments["model_id"]
            steps = arguments.get("steps", 1)

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found. Create it first."
            else:
                sim = simulations[model_id]
                sim.step(steps)
                result = f"⏩ Stepped {model_id} simulation {steps} time(s)\n"
                result += f"Simulation time: {sim.data.time:.3f}s"

        elif name == "get_state":
            model_id = arguments["model_id"]

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found. Create it first."
            else:
                sim = simulations[model_id]
                state = sim.get_state()
                result = json.dumps(state, indent=2)

        elif name == "reset_simulation":
            model_id = arguments["model_id"]

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found. Create it first."
            else:
                sim = simulations[model_id]
                sim.reset()
                result = f"🔄 Reset {model_id} to initial state"

        elif name == "close_simulation":
            model_id = arguments["model_id"]

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found."
            else:
                simulations[model_id].close()
                del simulations[model_id]
                result = f"🚪 Closed simulation '{model_id}'"

        elif name == "enable_egl_rendering":
            model_id = arguments["model_id"]
            width = arguments.get("width", 640)
            height = arguments.get("height", 480)

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found."
            else:
                sim = simulations[model_id]
                success = sim.enable_egl_rendering(width, height)
                if success:
                    result = f"🚀 EGL rendering enabled for '{model_id}' ({width}x{height})"
                else:
                    result = f"⚠️ EGL rendering not available, using software fallback for '{model_id}'"

        elif name == "render_egl_frame":
            model_id = arguments["model_id"]
            width = arguments.get("width", 640)
            height = arguments.get("height", 480)
            camera_id = arguments.get("camera_id", -1)

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found."
            else:
                sim = simulations[model_id]
                try:
                    frame = sim.render_frame(width, height, camera_id)
                    
                    # Convert to base64 PNG for transport
                    if PIL_AVAILABLE:
                        img = Image.fromarray(frame)
                        buffer = io.BytesIO()
                        img.save(buffer, format='PNG')
                        img_b64 = base64.b64encode(buffer.getvalue()).decode()
                        
                        return [types.ImageContent(
                            type="image",
                            data=img_b64,
                            mimeType="image/png"
                        )]
                    else:
                        result = f"✅ Frame rendered ({width}x{height}) but PIL not available for image encoding"
                        
                except Exception as e:
                    result = f"❌ Rendering failed: {str(e)}"

        elif name == "setup_h264_encoder":
            model_id = arguments["model_id"]
            width = arguments.get("width", 640)
            height = arguments.get("height", 480)
            fps = arguments.get("fps", 30)
            bitrate = arguments.get("bitrate", "2M")
            streaming = arguments.get("streaming", False)

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found."
            else:
                sim = simulations[model_id]
                if streaming:
                    success = sim.setup_streaming_encoder(width, height, fps, bitrate)
                    encoder_type = "streaming"
                else:
                    success = sim.setup_h264_encoder(width, height, fps, bitrate)
                    encoder_type = "batch"
                
                if success:
                    result = f"🎥 H.264 {encoder_type} encoder setup for '{model_id}' ({width}x{height}@{fps}fps, {bitrate})"
                else:
                    result = f"❌ Failed to setup H.264 encoder for '{model_id}'"

        elif name == "record_h264_video":
            model_id = arguments["model_id"]
            duration = arguments.get("duration", 5.0)
            camera_id = arguments.get("camera_id", -1)
            return_bytes = arguments.get("return_bytes", True)

            if model_id not in simulations:
                result = f"❌ Model '{model_id}' not found."
            else:
                sim = simulations[model_id]
                
                # Setup encoder if not already done
                if not sim._h264_encoder and not sim._streaming_encoder:
                    sim.setup_h264_encoder()
                
                try:
                    # Record frames
                    fps = 30
                    total_frames = int(duration * fps)
                    
                    for i in range(total_frames):
                        # Step simulation
                        sim.step(1)
                        
                        # Render and add frame
                        frame = sim.render_frame(camera_id=camera_id)
                        chunk = sim.add_frame_to_encoder(frame)
                        
                        # For streaming encoder, handle chunks as they're ready
                        if chunk:
                            # In a real streaming scenario, you'd send this chunk
                            pass
                    
                    # Get final encoded video
                    if sim._streaming_encoder:
                        video_data = sim._streaming_encoder.flush()
                    else:
                        video_data = sim.encode_video()
                    
                    if video_data and return_bytes:
                        video_b64 = base64.b64encode(video_data).decode()
                        return [types.EmbeddedResource(
                            type="resource",
                            resource={
                                "uri": f"data:video/mp4;base64,{video_b64}",
                                "mimeType": "video/mp4",
                                "text": f"H.264 video recording from {model_id} ({duration}s)"
                            }
                        )]
                    elif video_data:
                        result = f"✅ Video recorded for '{model_id}' ({duration}s, {total_frames} frames, {len(video_data)} bytes)"
                    else:
                        result = f"⚠️ Video recording completed but no data generated (frames: {total_frames})"
                        
                except Exception as e:
                    result = f"❌ Video recording failed: {str(e)}"

        elif name == "check_gpu_support":
            egl_info = check_egl_support()
            h264_info = check_h264_support()
            
            support_info = {
                "egl_rendering": egl_info,
                "h264_encoding": h264_info,
                "summary": {
                    "egl_available": egl_info.get("egl_available", False),
                    "gpu_rendering": egl_info.get("gpu_available", False),
                    "hardware_h264": len(h264_info.get("hardware_encoders", [])) > 0,
                    "software_h264": h264_info.get("software_encoder", False)
                }
            }
            
            result = json.dumps(support_info, indent=2)

        else:
            result = f"❌ Unknown tool: {name}"

        return [types.TextContent(type="text", text=str(result))]

    except Exception as e:
        logger.exception(f"Error in tool {name}: {e}")
        return [types.TextContent(type="text", text=f"❌ Error: {str(e)}")]


async def main():
    """Main entry point for MCP server"""
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            InitializationOptions(
                server_name="mujoco-mcp-headless",
                server_version=__version__,
                capabilities=server.get_capabilities(
                    notification_options=NotificationOptions(),
                    experimental_capabilities={},
                ),
            ),
        )


if __name__ == "__main__":
    asyncio.run(main())
