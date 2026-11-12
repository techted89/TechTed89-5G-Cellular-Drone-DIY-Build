#!/usr/bin/env python3
"""
Telemetry Processor for Ground Station
Processes and manages telemetry data stream
"""

import asyncio
import logging
import time
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class TelemetryStatus(Enum):
    STREAMING = "streaming"
    PAUSED = "paused"
    ERROR = "error"


@dataclass
class TelemetryPoint:
    """Single telemetry point in time series"""
    timestamp: float
    altitude: float  # meters AGL
    distance: float  # meters from home
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


@dataclass
class TelemetryConfig:
    """Telemetry processing configuration"""
    enabled: bool = True
    broadcast_interval: float = 1.0  # seconds
    max_history: int = 1000  # number of points to keep
    compression: bool = True


class TelemetryProcessor:
    """Telemetry data processing pipeline"""

    def __init__(self, config: dict = None):
        self.config = config or {}
        self.status = TelemetryStatus.STOPPED
        self._history: List[TelemetryPoint] = []
        self._history_max = self.config.get('max_history', 1000)
        self._broadcast_interval = self.config.get('broadcast_interval', 1.0)
        self._last_broadcast = 0.0
        self._broadcast_task = None
        self._clients = set()
        self._latest: Optional[TelemetryPoint] = None

    async def start(self):
        """Start telemetry processor"""
        logger.info("Starting telemetry processor")
        self.status = TelemetryStatus.STREAMING
        self._last_broadcast = time.time()
        self._broadcast_task = asyncio.create_task(self._broadcast_loop())

    async def stop(self):
        """Stop telemetry processor"""
        logger.info("Stopping telemetry processor")
        self.status = TelemetryStatus.STOPPED
        if self._broadcast_task:
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass

    def add_client(self, client):
        """Add a broadcast client"""
        self._clients.add(client)

    def remove_client(self, client):
        """Remove a broadcast client"""
        self._clients.discard(client)

    def update(self, point: TelemetryPoint):
        """Add a new telemetry point to history"""
        self._latest = point
        
        # Add to history, maintaining max size
        if len(self._history) >= self._history_max:
            self._history.pop(0)
        self._history.append(point)
        
        # Broadcast if enough time elapsed
        now = time.time()
        if now - self._last_broadcast >= self._broadcast_interval:
            self._last_broadcast = now
            self._broadcast()

    def _broadcast(self):
        """Broadcast latest telemetry to all clients"""
        if self._latest:
            data = self._latest.__dict__.copy()
            for client in self._clients.copy():
                try:
                    # Send data to client
                    pass  # Will be implemented per transport
                except Exception as e:
                    logger.error(f"Broadcast error: {e}")

    async def _broadcast_loop(self):
        """Background broadcast loop"""
        while self.status == TelemetryStatus.STREAMING:
            try:
                await asyncio.sleep(self._broadcast_interval)
                if self._latest:
                    self._broadcast()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Broadcast loop error: {e}")
                await asyncio.sleep(0.1)

    def get_latest(self) -> Optional[TelemetryPoint]:
        """Get latest telemetry point"""
        return self._latest

    def get_history(self, count: int = None) -> List[TelemetryPoint]:
        """Get telemetry history"""
        if count is None:
            return self._history
        return self._history[-count:] if count < len(self._history) else self._history

    def get_status(self) -> str:
        """Get telemetry status"""
        return self.status.value

    def clear_history(self):
        """Clear telemetry history"""
        self._history.clear()
        self._latest = None
        logger.info("Telemetry history cleared")