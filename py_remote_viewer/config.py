"""Configuration settings for the remote viewer."""

import os
from typing import Dict, Any
from dataclasses import dataclass


@dataclass
class ViewerConfig:
    """Configuration for the remote viewer server."""
    
    # Server settings
    host: str = "localhost"
    port: int = 8000
    
    # Frame generation settings
    frame_width: int = 640
    frame_height: int = 480
    frame_rate: int = 30
    
    # WebRTC settings
    stun_server: str = "stun:stun.l.google.com:19302"
    
    # Development settings
    use_synthetic_frames: bool = False  # Changed default to use real MuJoCo
    debug_mode: bool = False
    
    # Logging settings
    log_level: str = "INFO"
    
    @classmethod
    def from_env(cls) -> "ViewerConfig":
        """Create configuration from environment variables."""
        return cls(
            host=os.getenv("VIEWER_HOST", cls.host),
            port=int(os.getenv("VIEWER_PORT", str(cls.port))),
            frame_width=int(os.getenv("FRAME_WIDTH", str(cls.frame_width))),
            frame_height=int(os.getenv("FRAME_HEIGHT", str(cls.frame_height))),
            frame_rate=int(os.getenv("FRAME_RATE", str(cls.frame_rate))),
            stun_server=os.getenv("STUN_SERVER", cls.stun_server),
            use_synthetic_frames=os.getenv("USE_SYNTHETIC_FRAMES", "0") == "1",
            debug_mode=os.getenv("DEBUG_MODE", "0") == "1",
            log_level=os.getenv("LOG_LEVEL", cls.log_level),
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary."""
        return {
            "host": self.host,
            "port": self.port,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "frame_rate": self.frame_rate,
            "stun_server": self.stun_server,
            "use_synthetic_frames": self.use_synthetic_frames,
            "debug_mode": self.debug_mode,
            "log_level": self.log_level,
        }