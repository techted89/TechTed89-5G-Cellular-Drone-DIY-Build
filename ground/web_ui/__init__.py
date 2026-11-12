#!/usr/bin/env python3
"""
Ground Station Web UI
Provides a web interface for ground station monitoring and control
"""

import asyncio
import logging
import json
import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)


class UIState(Enum):
    IDLE = "idle"
    CONNECTED = "connected"
    FLYING = "flying"
    LANDING = "landing"
    ERROR = "error"


@dataclass
class UISettings:
    """Web UI settings"""
    refresh_interval: float = 1.0  # seconds
    show_telemetry: bool = True
    show_map: bool = True
    show_console: bool = True
    theme: str = "dark"
    auto_connect: bool = True


@dataclass
class UICommand:
    """UI command from web interface"""
    type: str
    payload: Dict[str, Any] = field(default_factory=dict)


class GroundWebUI:
    """Ground Station Web User Interface"""

    def __init__(self, config: dict = None):
        self.config = config or {}
        self.state = UIState.IDLE
        self.last_update = 0.0
        self.settings = UISettings()
        self.commands = asyncio.Queue()
        self._server = None
        self._port = self.config.get('port', 8082)
        self._clients = set()
        self._running = False
        self._latest_telemetry: Dict[str, Any] = {}

    async def start(self):
        """Start the web UI server"""
        logger.info(f"Starting Ground Web UI on port {self._port}")
        self._running = True
        self._clients = set()
        self.state = UIState.CONNECTED
        self._server_task = asyncio.create_task(self._run_server())

    async def stop(self):
        """Stop the web UI server"""
        logger.info("Stopping Ground Web UI")
        self._running = False
        if self._server_task:
            self._server_task.cancel()
            try:
                await self._server_task
            except asyncio.CancelledError:
                pass

    async def _run_server(self):
        """Run the HTTP server for web UI"""
        from aiohttp import web

        async def ui_page(request):
            return web.FileResponse('web_ui/templates/index.html')

        async def telemetry_api(request):
            return web.json_response(self._latest_telemetry)

        async def state_api(request):
            return web.json_response({'state': self.state.value})

        async def commands_api(request):
            payload = await request.json()
            await self.commands.put(payload)
            return web.json_response({'status': 'command_queued'})

        async def settings_api(request):
            payload = await request.json()
            self.settings = UISettings(**payload)
            return web.json_response({'status': 'settings_updated'})

        app = web.Application()
        app.add_routes([
            web.get('/', ui_page),
            web.get('/api/telemetry', telemetry_api),
            web.get('/api/state', state_api),
            web.post('/api/command', commands_api),
            web.post('/api/settings', settings_api),
            web.static('/static', 'web_ui/static'),
        ])

        # Create templates directory
        import os
        os.makedirs('web_ui/templates', exist_ok=True)
        os.makedirs('web_ui/static', exist_ok=True)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self._port)
        await site.start()

        logger.info(f"Web UI server running on 0.0.0.0:{self._port}")

        while self._running:
            await asyncio.sleep(0.1)

    async def update_telemetry(self, telemetry: Dict[str, Any]):
        """Update displayed telemetry"""
        self._latest_telemetry = telemetry
        self.last_update = time.time()

        # Update state based on telemetry
        if telemetry.get('command_ack'):
            self.state = UIState.CONNECTED
        elif telemetry.get('battery_remaining', 100) < 20:
            self.state = UIState.IDLE
        else:
            self.state = UIState.FLYING

    async def add_client(self, client):
        """Add a web client"""
        self._clients.add(client)

    async def remove_client(self, client):
        """Remove a web client"""
        self._clients.discard(client)

    def get_state(self) -> str:
        """Get current UI state"""
        return self.state.value

    def get_settings(self) -> UISettings:
        """Get current UI settings"""
        return self.settings