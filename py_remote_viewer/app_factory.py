"""Application factory for the remote viewer server."""

from .server import create_app
from .config import ViewerConfig


def create_app_from_config(config_dict: dict = None):
    """Create FastAPI app from configuration dictionary.
    
    Args:
        config_dict: Configuration dictionary (uses env defaults if None)
        
    Returns:
        Configured FastAPI application
    """
    if config_dict:
        # Create config from dictionary
        config = ViewerConfig(**config_dict)
    else:
        # Use environment defaults
        config = ViewerConfig.from_env()
    
    return create_app(config)


def create_development_app():
    """Create FastAPI app with development settings."""
    config = ViewerConfig(
        host="localhost",
        port=8000,
        debug_mode=True,
        log_level="DEBUG",
    )
    
    return create_app(config)


def create_production_app():
    """Create FastAPI app with production settings."""
    config = ViewerConfig(
        host="0.0.0.0",
        port=8000,
        debug_mode=False,
        log_level="INFO",
    )
    
    return create_app(config)