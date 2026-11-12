#!/usr/bin/env python3
"""
SBC Main Entry Point for 5G Drone System
Runs on Linux companion computer (Jetson, Raspberry Pi, etc.)
Handles: WebRTC video streaming, MAVLink bridge, Tailscale mesh, SMS beacons, REST API, Game Controller
"""

import asyncio
import logging
import signal
import sys
import os
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sbc.mavlink.mavlink_bridge import MavlinkBridge
from sbc.webrtc.webrtc_server import WebRTCServer
from sbc.tailscale.tailscale_manager import TailscaleManager
from sbc.sms.sms_beacon import SMSBeacon
from sbc.controller.game_controller import GameControllerManager
from sbc.api.rest_api import RestAPI
from shared.config.config_loader import load_config

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SBCMain:
    def __init__(self, config_path: str = None):
        self.config = load_config(config_path or "shared/config/drone_config.yaml")
        self.running = False
        self.tasks = []
        
        # Core components
        self.mavlink_bridge = None
        self.webrtc_server = None
        self.tailscale_manager = None
        self.sms_beacon = None
        self.rest_api = None
        self.controller_manager = None
    
    async def initialize(self):
        """Initialize all subsystems"""
        logger.info("Initializing 5G Drone SBC...")
        
        # Initialize Tailscale first (network foundation)
        self.tailscale_manager = TailscaleManager(self.config)
        await self.tailscale_manager.start()
        logger.info("Tailscale mesh established")
        
        # Initialize MAVLink bridge to flight controller
        self.mavlink_bridge = MavlinkBridge(self.config, self.tailscale_manager)
        await self.mavlink_bridge.connect()
        logger.info("MAVLink bridge connected to FC")
        
        # Initialize WebRTC video server
        self.webrtc_server = WebRTCServer(self.config, self.mavlink_bridge)
        await self.webrtc_server.start()
        logger.info("WebRTC video server started")
        
        # Initialize SMS emergency beacon
        self.sms_beacon = SMSBeacon(self.config, self.mavlink_bridge)
        await self.sms_beacon.start()
        logger.info("SMS emergency beacon active")
        
        # Initialize Game Controller
        self.controller_manager = GameControllerManager(self.config, self.mavlink_bridge)
        await self.controller_manager.start()
        logger.info("Game controller manager started")
        
        # Initialize REST API
        self.rest_api = RestAPI(self.config, self.mavlink_bridge, self.webrtc_server, self.sms_beacon, self.controller_manager)
        await self.rest_api.start()
        logger.info("REST API server started")
        
        # Register telemetry callback
        self.mavlink_bridge.register_telemetry_callback(self._on_telemetry)
        
        logger.info("SBC initialization complete")
    
    async def _on_telemetry(self, telemetry_data: dict):
        """Handle telemetry updates from FC"""
        # Forward to WebRTC data channel for ground station
        if self.webrtc_server:
            await self.webrtc_server.broadcast_telemetry(telemetry_data)
        
        # Check failsafe conditions
        await self._check_failsafes(telemetry_data)
    
    async def _check_failsafes(self, telemetry: dict):
        """Check and trigger failsafes based on telemetry"""
        failsafe_config = self.config.get('failsafe', {})
        
        # Battery failsafe
        battery = telemetry.get('battery', {})
        if battery:
            voltage = battery.get('voltage', 0) / 1000.0  # mV to V
            rth_voltage = failsafe_config.get('battery', {}).get('rth_voltage', 15.2)
            critical_voltage = failsafe_config.get('battery', {}).get('critical_voltage', 14.8)
            
            if voltage <= critical_voltage:
                await self._trigger_failsafe("critical_battery", f"Battery critical: {voltage:.1f}V")
            elif voltage <= rth_voltage:
                await self._trigger_failsafe("low_battery", f"Low battery: {voltage:.1f}V", rth=True)
        
        # Signal failsafe
        signal_info = telemetry.get('signal', {})
        if signal_info:
            last_telemetry_age = signal_info.get('last_telemetry_age', 0)
            telemetry_timeout = failsafe_config.get('signal', {}).get('telemetry_timeout', 3.0)
            
            if last_telemetry_age > telemetry_timeout:
                await self._trigger_failsafe("signal_loss", f"Telemetry timeout: {last_telemetry_age:.1f}s")
    
    async def _trigger_failsafe(self, reason: str, message: str, rth: bool = False):
        """Trigger failsafe action"""
        logger.warning(f"FAILSAFE TRIGGERED: {reason} - {message}")
        
        # Send SMS beacon
        if self.sms_beacon:
            await self.sms_beacon.send_emergency_beacon(reason, message, severity=3)
        
        # Trigger RTH if needed
        if rth and self.mavlink_bridge:
            await self.mavlink_bridge.trigger_rtl()
    
    async def run(self):
        """Main run loop"""
        self.running = True
        logger.info("SBC running - press Ctrl+C to stop")
        
        try:
            while self.running:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
    
    async def shutdown(self):
        """Graceful shutdown"""
        logger.info("Shutting down SBC...")
        self.running = False
        
        # Stop components in reverse order
        if self.rest_api:
            await self.rest_api.stop()
        if self.controller_manager:
            await self.controller_manager.stop()
        if self.sms_beacon:
            await self.sms_beacon.stop()
        if self.webrtc_server:
            await self.webrtc_server.stop()
        if self.mavlink_bridge:
            await self.mavlink_bridge.disconnect()
        if self.tailscale_manager:
            await self.tailscale_manager.stop()
        
        logger.info("SBC shutdown complete")


async def main():
    config_path = sys.argv[1] if len(sys.argv) > 1 else None
    sbc = SBCMain(config_path)
    
    # Setup signal handlers
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(sbc.shutdown()))
    
    try:
        await sbc.initialize()
        await sbc.run()
    except Exception as e:
        logger.error(f"SBC error: {e}", exc_info=True)
        await sbc.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())