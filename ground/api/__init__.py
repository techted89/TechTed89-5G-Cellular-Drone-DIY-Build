#!/usr/bin/env python3
"""
Ground Station API for Flight Controller
Provides REST API endpoints for telemetry, commands, and status
"""

import asyncio
import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class CommandType(Enum):
    TAKEOFF = "takeoff"
    LAND = "land"
    RETURN_HOME = "return_home"
    HOVER = "hover"
    TRACK = "track"
    FLY_TO = "fly_to"
    LANDING_ZONE = "landing_zone"
    LANDING_TARGET = "landing_target"
    LANDING_ALTITUDE = "landing_altitude"
    LANDING_SPEED = "landing_speed"
    LANDING_DURATION = "landing_duration"


@dataclass
class TelemetryData:
    """Telemetry data structure"""
    timestamp: float
    altitude: float  # meters AGL
    distance: float  # meters from home point
    battery_voltage: float  # V
    battery_remaining: int  # %
    gps_lat: float
    gps_lon: float
    gps_alt: float  # meters AMSL
    imu_roll: float
    imu_pitch: float
    imu_yaw: float
    rangefinder_distance: float  # meters
    signal_strength: int  # 0-255
    temperature: float  # C
    command_ack: bool = False


class ConnectionStatus(Enum):
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    ERROR = "error"


class GroundStationAPI:
    """Ground Station API for flight controller communication"""

    def __init__(self, config: dict):
        self.config = config.get('ground_station', {})
        self.host = self.config.get('host', '0.0.0.0')
        self.port = self.config.get('port', 8081)
        self.websocket_url = self.config.get('websocket_url', 'ws://localhost:8082')
        self.api_key = self.config.get('api_key', 'ground_station_secret')
        
        # Connection state
        self.status = ConnectionStatus.DISCONNECTED
        self.last_update = 0.0
        self.telemetry_buffer = []
        self.clients = set()
        self.is_running = False
        
        # Command queue
        self.command_queue = asyncio.Queue()
        self._run_task = None
        self._server_task = None

    async def start(self):
        """Start the ground station API server"""
        logger.info(f"Starting Ground Station API on {self.host}:{self.port}")
        self.is_running = True
        
        # Start command processor
        self._run_task = asyncio.create_task(self._command_processor())
        # Start server
        self._server_task = asyncio.create_task(self._run_server())
        logger.info("Ground Station API started")

    async def stop(self):
        """Stop the ground station API server"""
        self.is_running = False
        if self._run_task:
            self._run_task.cancel()
            try:
                await self._run_task
            except asyncio.CancelledError:
                pass
        if self._server_task:
            self._server_task.cancel()
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass
        logger.info("Ground Station API stopped")

    async def _command_processor(self):
        """Process commands from queue"""
        while self.is_running:
            try:
                cmd = await self.command_queue.get()
                await self._handle_command(cmd)
                self.command_queue.task_done()
            except Exception as e:
                logger.error(f"Command processing error: {e}")
                await asyncio.sleep(0.1)

    async def _handle_command(self, cmd: Dict[str, Any]):
        """Handle a command"""
        cmd_type = cmd.get('type')
        logger.info(f"Handling command: {cmd_type}")
        
        # Simulate command execution
        await asyncio.sleep(0.1)
        
        # Acknowledge command
        ack = {
            'type': 'command_ack',
            'command_id': cmd.get('id'),
            'status': 'success',
            'timestamp': time.time()
        }
        # Broadcast to clients
        for client in self.clients:
            try:
                await client.send_json(ack)
            except Exception as e:
                logger.error(f"Failed to send ack to client: {e}")

    async def _run_server(self):
        """Run the HTTP server"""
        from aiohttp import web
        
        async def index(request):
            return web.json_response({
                'status': self.status.value,
                'api_key': self.api_key,
                'commands': [cmd['type'] for cmd in self.config.get('commands', [])]
            })
            
        async def telemetry(request):
            data = {
                'status': self.status.value,
                'telemetry': self.get_latest_telemetry()
            }
            return web.json_response(data)
            
        async def command(request):
            payload = await request.json()
            await self.command_queue.put(payload)
            return web.json_response({'status': 'command_queued'})
            
        app = web.Application()
        app.add_routes([
            web.get('/', index),
            web.get('/telemetry', telemetry),
            web.post('/command', command),
        ])
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        logger.info(f"HTTP API server running on {self.host}:{self.port}")
        
        while self.is_running:
            await asyncio.sleep(0.1)

    async def get_latest_telemetry(self) -> Dict[str, Any]:
        """Get latest telemetry data"""
        if not self.is_running or not self.clients:
            return {
                'timestamp': time.time(),
                'altitude': 0.0,
                'distance': 0.0,
                'battery_voltage': 0.0,
                'battery_remaining': 0,
                'gps_lat': 0.0,
                'gps_lon': 0.0,
                'gps_alt': 0.0,
                'imu_roll': 0.0,
                'imu_pitch': 0.0,
                'imu_yaw': 0.0,
                'rangefinder_distance': 0.0,
                'signal_strength': 0,
                'temperature': 0.0,
                'command_ack': False
            }
            
        # Return the most recent telemetry
        if self.telemetry_buffer:
            return self.telemetry_buffer[-1]
        else:
            return {
                'timestamp': time.time(),
                'altitude': 0.0,
                'distance': 0.0,
                'battery_voltage': 0.0,
                'battery_remaining': 0,
                'gps_lat': 0.0,
                'gps_lon': 0.0,
                'gps_alt': 0.0,
                'imu_roll': 0.0,
                'imu_pitch': 0.0,
                'imu_yaw': 0.0,
                'rangefinder_distance': 0.0,
                'signal_strength': 0,
                'temperature': 0.0,
                'command_ack': False
            }

    def get_status(self) -> str:
        """Get current connection status"""
        return self.status.value

    def add_client(self, client):
        """Add a client connection"""
        self.clients.add(client)

    def remove_client(self, client):
        """Remove a client connection"""
        self.clients.discard(client)