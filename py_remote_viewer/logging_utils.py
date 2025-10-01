"""Logging utilities for the remote viewer."""

import logging
import sys
from typing import Optional


def setup_logging(level: str = "INFO", format_str: Optional[str] = None):
    """Setup logging configuration for the remote viewer.
    
    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        format_str: Custom log format string
    """
    if format_str is None:
        format_str = "[%(asctime)s] %(name)s - %(levelname)s - %(message)s"
    
    # Convert string level to logging constant
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    
    # Configure root logger
    logging.basicConfig(
        level=numeric_level,
        format=format_str,
        stream=sys.stdout,
        force=True  # Override any existing configuration
    )
    
    # Configure specific loggers
    
    # Our application loggers
    logging.getLogger("py_remote_viewer").setLevel(numeric_level)
    
    # Third-party loggers - reduce verbosity unless in debug mode
    if numeric_level <= logging.DEBUG:
        # In debug mode, show more details
        logging.getLogger("aiortc").setLevel(logging.INFO)
        logging.getLogger("fastapi").setLevel(logging.INFO)
        logging.getLogger("uvicorn").setLevel(logging.INFO)
    else:
        # In normal mode, reduce third-party noise
        logging.getLogger("aiortc").setLevel(logging.WARNING)
        logging.getLogger("fastapi").setLevel(logging.WARNING)
        logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    
    logger = logging.getLogger(__name__)
    logger.info(f"Logging configured at level: {level}")


def get_logger(name: str) -> logging.Logger:
    """Get a logger with the specified name.
    
    Args:
        name: Logger name
        
    Returns:
        Logger instance
    """
    return logging.getLogger(f"py_remote_viewer.{name}")