#!/usr/bin/env python3
"""
Launch script for MuJoCo MCP Client Monitor
Simple launcher to start the web-based monitoring interface
"""

import asyncio
import argparse
import logging
import sys

from src.mujoco_mcp.client_monitor import start_monitor_server


def main():
    parser = argparse.ArgumentParser(description="MuJoCo MCP Client Monitor")
    parser.add_argument("--host", default="localhost", help="Host to bind to (default: localhost)")
    parser.add_argument("--port", type=int, default=8080, help="Port to bind to (default: 8080)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    # Set up logging
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    logger = logging.getLogger("mujoco_mcp.monitor_launcher")
    
    async def run_monitor():
        try:
            logger.info(f"Starting MuJoCo MCP Client Monitor...")
            logger.info(f"Web interface will be available at: http://{args.host}:{args.port}")
            
            runner = await start_monitor_server(args.host, args.port)
            
            print("\n" + "="*60)
            print("🎮 MuJoCo MCP Client Monitor Started")
            print("="*60)
            print(f"📊 Web Dashboard: http://{args.host}:{args.port}")
            print(f"🔗 API Endpoint: http://{args.host}:{args.port}/api/monitor")
            print(f"🔌 WebSocket: ws://{args.host}:{args.port}/ws")
            print("\n📝 Features:")
            print("  • Real-time client session monitoring")
            print("  • System resource tracking")
            print("  • Conservative capacity estimation")
            print("  • Auto-refreshing dashboard")
            print("\n⚡ Quick Tips:")
            print("  • The dashboard updates every 2 seconds")
            print("  • Check 'System Health' for capacity warnings")
            print("  • Use Ctrl+C to stop the monitor")
            print("="*60)
            
            # Keep running
            try:
                while True:
                    await asyncio.sleep(1)
            except KeyboardInterrupt:
                logger.info("Shutting down monitor...")
                await runner.cleanup()
                print("\n👋 Monitor stopped gracefully")
            
        except Exception as e:
            logger.error(f"Error starting monitor: {e}")
            sys.exit(1)
    
    try:
        asyncio.run(run_monitor())
    except KeyboardInterrupt:
        print("\n👋 Monitor stopped")


if __name__ == "__main__":
    main()