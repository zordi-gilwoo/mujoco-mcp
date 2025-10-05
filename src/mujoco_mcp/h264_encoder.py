"""
H.264 Video Encoder for optimized streaming performance
Provides efficient video encoding with hardware acceleration when available
"""

import os
import time
import logging
import tempfile
import subprocess
from typing import List, Optional, Dict, Any, Union
from pathlib import Path
import numpy as np

logger = logging.getLogger(__name__)

try:
    import ffmpeg

    FFMPEG_AVAILABLE = True
except ImportError:
    FFMPEG_AVAILABLE = False

try:
    from PIL import Image

    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


class H264Encoder:
    """Hardware-accelerated H.264 encoder for video streaming"""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        bitrate: str = "2M",
        preset: str = "medium",
        use_hardware: bool = True,
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.preset = preset
        self.use_hardware = use_hardware

        self._frames_buffer = []
        self._temp_dir = None
        self._encoding_process = None
        self._hardware_encoder = self._detect_hardware_encoder()

    def _detect_hardware_encoder(self) -> Optional[str]:
        """Detect available hardware encoders"""
        if not self.use_hardware or not FFMPEG_AVAILABLE:
            return None

        # Test hardware encoders in order of preference
        encoders = [
            ("h264_nvenc", "NVIDIA NVENC"),
            ("h264_qsv", "Intel QuickSync"),
            ("h264_videotoolbox", "Apple VideoToolbox"),
            ("h264_vaapi", "VAAPI"),
            ("h264_amf", "AMD AMF"),
        ]

        for encoder, name in encoders:
            if self._test_encoder(encoder):
                logger.info(f"Using hardware encoder: {name} ({encoder})")
                return encoder

        logger.info("No hardware encoder available, using software encoding")
        return None

    def _test_encoder(self, encoder: str) -> bool:
        """Test if an encoder is available"""
        try:
            cmd = [
                "ffmpeg",
                "-f",
                "lavfi",
                "-i",
                "testsrc=duration=1:size=64x64:rate=1",
                "-c:v",
                encoder,
                "-preset",
                "ultrafast",
                "-f",
                "null",
                "-",
            ]
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False

    def add_frame(self, frame: np.ndarray):
        """Add frame to encoding buffer"""
        if frame.shape[:2] != (self.height, self.width):
            frame = self._resize_frame(frame)

        self._frames_buffer.append(frame.copy())

    def _resize_frame(self, frame: np.ndarray) -> np.ndarray:
        """Resize frame to target dimensions"""
        if not PIL_AVAILABLE:
            # Simple nearest neighbor interpolation
            h, w = frame.shape[:2]
            scale_h = self.height / h
            scale_w = self.width / w

            new_frame = np.zeros((self.height, self.width, 3), dtype=frame.dtype)
            for i in range(self.height):
                for j in range(self.width):
                    src_i = min(int(i / scale_h), h - 1)
                    src_j = min(int(j / scale_w), w - 1)
                    new_frame[i, j] = frame[src_i, src_j]
            return new_frame
        else:
            # Use PIL for better quality resizing
            img = Image.fromarray(frame)
            img_resized = img.resize((self.width, self.height), Image.Resampling.LANCZOS)
            return np.array(img_resized)

    def encode_to_bytes(self) -> bytes:
        """Encode buffered frames to H.264 bytes"""
        if not self._frames_buffer:
            return b""

        if not FFMPEG_AVAILABLE:
            raise RuntimeError("FFmpeg not available for H.264 encoding")

        with tempfile.TemporaryDirectory() as temp_dir:
            # Write frames to temporary images
            frame_paths = []
            for i, frame in enumerate(self._frames_buffer):
                frame_path = os.path.join(temp_dir, f"frame_{i:06d}.png")
                if PIL_AVAILABLE:
                    Image.fromarray(frame).save(frame_path)
                else:
                    self._save_frame_simple(frame, frame_path)
                frame_paths.append(frame_path)

            # Encode to H.264
            output_path = os.path.join(temp_dir, "output.mp4")
            self._encode_frames_to_file(frame_paths, output_path)

            # Read encoded bytes
            with open(output_path, "rb") as f:
                return f.read()

    def _save_frame_simple(self, frame: np.ndarray, path: str):
        """Simple PNG saving without PIL"""
        # This is a fallback - in practice, PIL should be available
        # For now, create a simple PPM file that FFmpeg can read
        with open(path.replace(".png", ".ppm"), "w") as f:
            f.write(f"P3\n{self.width} {self.height}\n255\n")
            for row in frame:
                for pixel in row:
                    f.write(f"{pixel[0]} {pixel[1]} {pixel[2]} ")
                f.write("\n")

    def _encode_frames_to_file(self, frame_paths: List[str], output_path: str):
        """Encode frames to H.264 file using FFmpeg"""
        # Determine input pattern
        if frame_paths[0].endswith(".ppm"):
            input_pattern = os.path.join(os.path.dirname(frame_paths[0]), "frame_%06d.ppm")
        else:
            input_pattern = os.path.join(os.path.dirname(frame_paths[0]), "frame_%06d.png")

        # Build FFmpeg command
        cmd = ["ffmpeg", "-y", "-r", str(self.fps), "-i", input_pattern]

        # Add codec and settings
        if self._hardware_encoder:
            cmd.extend(["-c:v", self._hardware_encoder])

            # Hardware-specific settings
            if "nvenc" in self._hardware_encoder:
                cmd.extend(["-preset", "p4", "-tune", "ll"])  # Low latency
            elif "qsv" in self._hardware_encoder:
                cmd.extend(["-preset", "veryfast"])
            else:
                cmd.extend(["-preset", self.preset])
        else:
            cmd.extend(["-c:v", "libx264", "-preset", self.preset])

        # Add bitrate and other settings
        cmd.extend(
            [
                "-b:v",
                self.bitrate,
                "-maxrate",
                self.bitrate,
                "-bufsize",
                f"{int(self.bitrate[:-1]) * 2}M",
                "-g",
                str(self.fps * 2),  # GOP size
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                output_path,
            ]
        )

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                raise RuntimeError(f"FFmpeg encoding failed: {result.stderr}")
        except subprocess.TimeoutExpired:
            raise RuntimeError("FFmpeg encoding timed out")

    def encode_to_file(self, output_path: str):
        """Encode buffered frames to H.264 file"""
        if not self._frames_buffer:
            raise ValueError("No frames to encode")

        with tempfile.TemporaryDirectory() as temp_dir:
            # Save frames
            frame_paths = []
            for i, frame in enumerate(self._frames_buffer):
                frame_path = os.path.join(temp_dir, f"frame_{i:06d}.png")
                if PIL_AVAILABLE:
                    Image.fromarray(frame).save(frame_path)
                else:
                    self._save_frame_simple(frame, frame_path)
                frame_paths.append(frame_path)

            # Encode
            self._encode_frames_to_file(frame_paths, output_path)

    def clear_buffer(self):
        """Clear frame buffer"""
        self._frames_buffer.clear()

    def get_buffer_size(self) -> int:
        """Get number of frames in buffer"""
        return len(self._frames_buffer)

    def get_info(self) -> Dict[str, Any]:
        """Get encoder information"""
        return {
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "bitrate": self.bitrate,
            "preset": self.preset,
            "hardware_encoder": self._hardware_encoder,
            "frames_buffered": len(self._frames_buffer),
            "ffmpeg_available": FFMPEG_AVAILABLE,
            "pil_available": PIL_AVAILABLE,
        }


class StreamingEncoder:
    """Real-time streaming encoder for continuous video encoding"""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        bitrate: str = "1M",
        chunk_duration: float = 2.0,  # seconds per chunk
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.chunk_duration = chunk_duration
        self.frames_per_chunk = int(fps * chunk_duration)

        self._encoder = H264Encoder(width, height, fps, bitrate, preset="ultrafast")
        self._chunk_buffer = []
        self._total_frames = 0

    def add_frame(self, frame: np.ndarray) -> Optional[bytes]:
        """Add frame and return encoded chunk if ready"""
        self._encoder.add_frame(frame)
        self._total_frames += 1

        if self._encoder.get_buffer_size() >= self.frames_per_chunk:
            # Encode current chunk
            chunk_data = self._encoder.encode_to_bytes()
            self._encoder.clear_buffer()

            return chunk_data

        return None

    def flush(self) -> Optional[bytes]:
        """Encode remaining frames"""
        if self._encoder.get_buffer_size() > 0:
            chunk_data = self._encoder.encode_to_bytes()
            self._encoder.clear_buffer()
            return chunk_data
        return None

    def get_stats(self) -> Dict[str, Any]:
        """Get streaming statistics"""
        return {
            "total_frames": self._total_frames,
            "current_buffer": self._encoder.get_buffer_size(),
            "frames_per_chunk": self.frames_per_chunk,
            "encoder_info": self._encoder.get_info(),
        }


def check_h264_support() -> Dict[str, Any]:
    """Check H.264 encoding support and capabilities"""
    info = {
        "ffmpeg_available": FFMPEG_AVAILABLE,
        "pil_available": PIL_AVAILABLE,
        "hardware_encoders": [],
        "software_encoder": False,
    }

    if not FFMPEG_AVAILABLE:
        return info

    # Test hardware encoders
    encoders = [
        ("h264_nvenc", "NVIDIA NVENC"),
        ("h264_qsv", "Intel QuickSync"),
        ("h264_videotoolbox", "Apple VideoToolbox"),
        ("h264_vaapi", "VAAPI"),
        ("h264_amf", "AMD AMF"),
    ]

    dummy_encoder = H264Encoder(64, 64, use_hardware=True)
    for encoder, name in encoders:
        if dummy_encoder._test_encoder(encoder):
            info["hardware_encoders"].append({"name": name, "codec": encoder})

    # Test software encoder
    info["software_encoder"] = dummy_encoder._test_encoder("libx264")

    return info
