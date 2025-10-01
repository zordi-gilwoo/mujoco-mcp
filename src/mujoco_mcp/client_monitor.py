"""
Client Monitor Web Interface
Simple web-based monitoring for MuJoCo MCP multi-client sessions
"""

import asyncio
import json
import logging
import time
from typing import Dict, Any
from aiohttp import web, WSMsgType
import aiohttp_cors
import psutil
import os

from .session_manager import session_manager

logger = logging.getLogger("mujoco_mcp.client_monitor")


class ClientMonitor:
    """Web-based client monitoring interface"""
    
    def __init__(self, host: str = "localhost", port: int = 8080):
        self.host = host
        self.port = port
        self.app = None
        self.websockets = set()
        
    async def get_system_stats(self) -> Dict[str, Any]:
        """Get system resource statistics"""
        try:
            # Get system resources
            memory = psutil.virtual_memory()
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            # Get process-specific info
            process = psutil.Process(os.getpid())
            process_memory = process.memory_info()
            
            return {
                "system": {
                    "cpu_percent": cpu_percent,
                    "memory_total_gb": round(memory.total / (1024**3), 2),
                    "memory_used_gb": round(memory.used / (1024**3), 2),
                    "memory_percent": memory.percent,
                    "memory_available_gb": round(memory.available / (1024**3), 2)
                },
                "process": {
                    "memory_rss_mb": round(process_memory.rss / (1024**2), 2),
                    "memory_vms_mb": round(process_memory.vms / (1024**2), 2),
                    "cpu_percent": process.cpu_percent(),
                    "num_threads": process.num_threads()
                }
            }
        except Exception as e:
            logger.error(f"Error getting system stats: {e}")
            return {"error": str(e)}
    
    def estimate_max_clients(self) -> Dict[str, Any]:
        """Conservative estimation of maximum supported clients"""
        try:
            memory = psutil.virtual_memory()
            
            # Conservative estimates based on typical MuJoCo usage
            estimated_memory_per_client_mb = 170  # Conservative estimate
            
            # Reserve memory for system (leave 25% for OS and other processes)
            available_memory_gb = memory.available / (1024**3)
            usable_memory_gb = available_memory_gb * 0.75
            usable_memory_mb = usable_memory_gb * 1024
            
            # Calculate max clients based on memory
            max_clients_memory = int(usable_memory_mb / estimated_memory_per_client_mb)
            
            # Network port constraints (assuming port range 8889-9888 = 1000 ports)
            max_clients_ports = 1000
            
            # CPU constraint (assume each client needs ~5% CPU, leave 25% for system)
            cpu_count = psutil.cpu_count()
            max_clients_cpu = int((cpu_count * 0.75) / 0.05)
            
            # Take the most conservative estimate
            max_clients = min(max_clients_memory, max_clients_ports, max_clients_cpu, 50)  # Hard cap at 50
            
            return {
                "estimated_max_clients": max_clients,
                "constraints": {
                    "memory_limit": max_clients_memory,
                    "port_limit": max_clients_ports,
                    "cpu_limit": max_clients_cpu,
                    "hard_cap": 50
                },
                "assumptions": {
                    "memory_per_client_mb": estimated_memory_per_client_mb,
                    "cpu_per_client_percent": 5,
                    "system_reserve_percent": 25
                }
            }
        except Exception as e:
            logger.error(f"Error estimating max clients: {e}")
            return {
                "estimated_max_clients": 10,  # Fallback conservative estimate
                "error": str(e)
            }
    
    async def get_monitoring_data(self) -> Dict[str, Any]:
        """Get comprehensive monitoring data"""
        session_stats = session_manager.get_session_stats()
        system_stats = await self.get_system_stats()
        capacity_estimate = self.estimate_max_clients()
        
        return {
            "timestamp": time.time(),
            "sessions": session_stats,
            "system": system_stats,
            "capacity": capacity_estimate,
            "health": {
                "status": "healthy" if session_stats["active_sessions"] < capacity_estimate["estimated_max_clients"] * 0.8 else "warning",
                "utilization_percent": round((session_stats["active_sessions"] / capacity_estimate["estimated_max_clients"]) * 100, 1)
            }
        }
    
    async def websocket_handler(self, request):
        """WebSocket handler for real-time updates"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.websockets.add(ws)
        logger.info(f"WebSocket client connected. Total: {len(self.websockets)}")
        
        try:
            # Send initial data
            data = await self.get_monitoring_data()
            await ws.send_str(json.dumps(data))
            
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    if msg.data == 'ping':
                        await ws.send_str('pong')
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f'WebSocket error: {ws.exception()}')
                    break
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.websockets.discard(ws)
            logger.info(f"WebSocket client disconnected. Total: {len(self.websockets)}")
        
        return ws
    
    async def api_handler(self, request):
        """REST API handler for monitoring data"""
        data = await self.get_monitoring_data()
        return web.json_response(data)
    
    async def index_handler(self, request):
        """Serve the monitoring dashboard HTML"""
        html = """
<!DOCTYPE html>
<html>
<head>
    <title>MuJoCo MCP Client Monitor</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background-color: #f5f5f5; 
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
            background: white; 
            padding: 20px; 
            border-radius: 8px; 
            box-shadow: 0 2px 4px rgba(0,0,0,0.1); 
        }
        .header { 
            text-align: center; 
            color: #333; 
            margin-bottom: 30px; 
        }
        .stats-grid { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); 
            gap: 20px; 
            margin-bottom: 30px; 
        }
        .stat-card { 
            background: #f8f9fa; 
            padding: 20px; 
            border-radius: 6px; 
            border-left: 4px solid #007bff; 
        }
        .stat-card h3 { 
            margin: 0 0 10px 0; 
            color: #333; 
        }
        .stat-value { 
            font-size: 2em; 
            font-weight: bold; 
            margin: 10px 0; 
        }
        .healthy { color: #28a745; }
        .warning { color: #ffc107; }
        .danger { color: #dc3545; }
        .sessions-table { 
            width: 100%; 
            border-collapse: collapse; 
            margin-top: 20px; 
        }
        .sessions-table th, .sessions-table td { 
            padding: 12px; 
            text-align: left; 
            border-bottom: 1px solid #ddd; 
        }
        .sessions-table th { 
            background-color: #f8f9fa; 
            font-weight: bold; 
        }
        .progress-bar { 
            width: 100%; 
            height: 20px; 
            background-color: #e9ecef; 
            border-radius: 10px; 
            overflow: hidden; 
        }
        .progress-fill { 
            height: 100%; 
            transition: width 0.3s ease; 
        }
        .status-indicator { 
            display: inline-block; 
            width: 12px; 
            height: 12px; 
            border-radius: 50%; 
            margin-right: 8px; 
        }
        .footer { 
            text-align: center; 
            margin-top: 30px; 
            color: #666; 
            font-size: 0.9em; 
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎮 MuJoCo MCP Client Monitor</h1>
            <p>Real-time monitoring of multi-client sessions</p>
        </div>
        
        <div class="stats-grid">
            <div class="stat-card">
                <h3>Active Clients</h3>
                <div class="stat-value" id="active-clients">-</div>
                <div>Connected sessions</div>
            </div>
            
            <div class="stat-card">
                <h3>Max Capacity</h3>
                <div class="stat-value" id="max-capacity">-</div>
                <div>Estimated maximum clients</div>
            </div>
            
            <div class="stat-card">
                <h3>System Health</h3>
                <div class="stat-value" id="health-status">-</div>
                <div>
                    <div class="progress-bar">
                        <div class="progress-fill" id="utilization-bar"></div>
                    </div>
                    <div id="utilization-text">- % utilization</div>
                </div>
            </div>
            
            <div class="stat-card">
                <h3>Memory Usage</h3>
                <div class="stat-value" id="memory-usage">-</div>
                <div id="memory-details">System memory</div>
            </div>
        </div>
        
        <div class="stat-card">
            <h3>Active Sessions</h3>
            <table class="sessions-table" id="sessions-table">
                <thead>
                    <tr>
                        <th>Client ID</th>
                        <th>Session Age</th>
                        <th>Last Activity</th>
                        <th>Viewer Port</th>
                        <th>Active Models</th>
                    </tr>
                </thead>
                <tbody id="sessions-tbody">
                    <tr><td colspan="5" style="text-align: center; color: #666;">No active sessions</td></tr>
                </tbody>
            </table>
        </div>
        
        <div class="footer">
            <p>Last updated: <span id="last-updated">Never</span></p>
            <p>WebSocket Status: <span class="status-indicator" id="ws-indicator"></span><span id="ws-status">Connecting...</span></p>
        </div>
    </div>

    <script>
        let ws = null;
        let reconnectInterval = null;
        
        function connect() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                updateConnectionStatus('connected');
                if (reconnectInterval) {
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                }
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateDashboard(data);
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                updateConnectionStatus('disconnected');
                if (!reconnectInterval) {
                    reconnectInterval = setInterval(connect, 5000);
                }
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
                updateConnectionStatus('error');
            };
        }
        
        function updateConnectionStatus(status) {
            const indicator = document.getElementById('ws-indicator');
            const statusText = document.getElementById('ws-status');
            
            switch(status) {
                case 'connected':
                    indicator.style.backgroundColor = '#28a745';
                    statusText.textContent = 'Connected';
                    break;
                case 'disconnected':
                    indicator.style.backgroundColor = '#ffc107';
                    statusText.textContent = 'Reconnecting...';
                    break;
                case 'error':
                    indicator.style.backgroundColor = '#dc3545';
                    statusText.textContent = 'Connection Error';
                    break;
                default:
                    indicator.style.backgroundColor = '#6c757d';
                    statusText.textContent = 'Connecting...';
            }
        }
        
        function updateDashboard(data) {
            // Update main stats
            document.getElementById('active-clients').textContent = data.sessions.active_sessions;
            document.getElementById('max-capacity').textContent = data.capacity.estimated_max_clients;
            
            // Update health status
            const healthStatus = document.getElementById('health-status');
            const healthClass = data.health.status === 'healthy' ? 'healthy' : 
                               data.health.status === 'warning' ? 'warning' : 'danger';
            healthStatus.textContent = data.health.status.toUpperCase();
            healthStatus.className = `stat-value ${healthClass}`;
            
            // Update utilization bar
            const utilizationBar = document.getElementById('utilization-bar');
            const utilization = data.health.utilization_percent;
            utilizationBar.style.width = `${utilization}%`;
            utilizationBar.style.backgroundColor = utilization < 60 ? '#28a745' : 
                                                  utilization < 80 ? '#ffc107' : '#dc3545';
            document.getElementById('utilization-text').textContent = `${utilization}% utilization`;
            
            // Update memory usage
            if (data.system.system) {
                document.getElementById('memory-usage').textContent = 
                    `${data.system.system.memory_percent.toFixed(1)}%`;
                document.getElementById('memory-details').textContent = 
                    `${data.system.system.memory_used_gb}GB / ${data.system.system.memory_total_gb}GB`;
            }
            
            // Update sessions table
            const tbody = document.getElementById('sessions-tbody');
            if (data.sessions.active_sessions === 0) {
                tbody.innerHTML = '<tr><td colspan="5" style="text-align: center; color: #666;">No active sessions</td></tr>';
            } else {
                tbody.innerHTML = '';
                for (const [sessionId, session] of Object.entries(data.sessions.sessions)) {
                    const row = document.createElement('tr');
                    row.innerHTML = `
                        <td>${session.client_id}</td>
                        <td>${Math.floor(session.age_seconds / 60)}m ${Math.floor(session.age_seconds % 60)}s</td>
                        <td>${Math.floor(session.idle_seconds / 60)}m ${Math.floor(session.idle_seconds % 60)}s ago</td>
                        <td>${session.viewer_port}</td>
                        <td>${session.active_models.join(', ') || 'None'}</td>
                    `;
                    tbody.appendChild(row);
                }
            }
            
            // Update timestamp
            document.getElementById('last-updated').textContent = new Date().toLocaleTimeString();
        }
        
        // Start connection
        connect();
        
        // Send ping every 30 seconds to keep connection alive
        setInterval(function() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('ping');
            }
        }, 30000);
    </script>
</body>
</html>
        """
        return web.Response(text=html, content_type='text/html')
    
    async def broadcast_updates(self):
        """Broadcast updates to all connected WebSocket clients"""
        if not self.websockets:
            return
            
        try:
            data = await self.get_monitoring_data()
            message = json.dumps(data)
            
            # Send to all connected websockets
            disconnected = set()
            for ws in self.websockets:
                try:
                    await ws.send_str(message)
                except Exception as e:
                    logger.error(f"Error sending to websocket: {e}")
                    disconnected.add(ws)
            
            # Remove disconnected websockets
            self.websockets -= disconnected
            
        except Exception as e:
            logger.error(f"Error broadcasting updates: {e}")
    
    async def periodic_updates(self):
        """Send periodic updates to WebSocket clients"""
        while True:
            await asyncio.sleep(2)  # Update every 2 seconds
            await self.broadcast_updates()
    
    def create_app(self):
        """Create the web application"""
        self.app = web.Application()
        
        # Add CORS support
        cors = aiohttp_cors.setup(self.app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods="*"
            )
        })
        
        # Routes
        self.app.router.add_get('/', self.index_handler)
        self.app.router.add_get('/api/monitor', self.api_handler)
        self.app.router.add_get('/ws', self.websocket_handler)
        
        # Add CORS to all routes
        for route in list(self.app.router.routes()):
            cors.add(route)
        
        return self.app
    
    async def start_server(self):
        """Start the monitoring web server"""
        self.create_app()
        
        # Start periodic updates task
        asyncio.create_task(self.periodic_updates())
        
        runner = web.AppRunner(self.app)
        await runner.setup()
        
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        
        logger.info(f"Client monitor server started at http://{self.host}:{self.port}")
        return runner


# Global client monitor instance
client_monitor = ClientMonitor()


async def start_monitor_server(host: str = "localhost", port: int = 8080):
    """Start the client monitoring server"""
    client_monitor.host = host
    client_monitor.port = port
    return await client_monitor.start_server()


if __name__ == "__main__":
    async def main():
        runner = await start_monitor_server()
        try:
            # Keep the server running
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            await runner.cleanup()
    
    asyncio.run(main())