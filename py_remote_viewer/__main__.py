"""Main entry point for the py_remote_viewer package."""

import sys
import argparse
from .server import run_server
from .config import ViewerConfig


def main():
    """Main entry point for the remote viewer server."""
    parser = argparse.ArgumentParser(
        description="MuJoCo Remote Viewer - Python-based headless WebRTC viewer"
    )
    
    parser.add_argument(
        "--host",
        default=None,
        help="Server host (default: from env or localhost)"
    )
    
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Server port (default: from env or 8000)"
    )
    
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        default=None,
        help="Logging level (default: from env or INFO)"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug mode"
    )
    
    args = parser.parse_args()
    
    # Create configuration
    config = ViewerConfig.from_env()
    
    # Override with command line arguments
    if args.host:
        config.host = args.host
    if args.port:
        config.port = args.port
    if args.log_level:
        config.log_level = args.log_level
    if args.debug:
        config.debug_mode = True
    
    print(f"🚀 Starting MuJoCo Remote Viewer")
    print(f"📍 Server: http://{config.host}:{config.port}")
    print(f"🐛 Debug mode: {config.debug_mode}")
    print()
    
    try:
        run_server(config=config)
    except KeyboardInterrupt:
        print("\n👋 Server stopped by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Server error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()