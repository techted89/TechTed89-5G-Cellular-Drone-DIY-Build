#!/usr/bin/env python3
"""
Ground Station Configuration Manager
Manages ground station configuration for telemetry, commands, and settings
"""

import asyncio
import logging
import json
import os
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)


class ConfigCategory(Enum):
    TELEMETRY = "telemetry"
    COMMUNICATION = "communication"
    SAFETY = "safety"
    NAVIGATION = "navigation"
    UI = "ui"


@dataclass
class TelemetryConfig:
    """Telemetry configuration"""
    enabled: bool = True
    broadcast_interval: float = 1.0  # seconds
    data_fields: List[str] = field(default_factory=lambda: [
        'altitude', 'distance', 'battery_voltage', 'battery_remaining',
        'gps_lat', 'gps_lon', 'imu_roll', 'imu_pitch', 'imu_yaw'
    ])
    max_buffer_size: int = 1000
    compression_enabled: bool = True


@dataclass
class CommunicationConfig:
    """Communication configuration"""
    websocket_url: str = "ws://localhost:8082"
    http_host: str = "0.0.0.0"
    http_port: int = 8081
    http_allowed_origins: List[str] = field(default_factory=lambda: ["*"])
    api_key: str = "ground_station_secret"
    reconnect_interval: float = 3.0  # seconds
    max_reconnect_attempts: int = 10


@dataclass
class SafetyConfig:
    """Safety configuration"""
    enabled: bool = True
    rssi_threshold: float = 20.0  # dBm
    voltage_low_threshold: float = 3.5  # V per cell
    altitude_max: float = 120.0  # meters
    distance_max: float = 500.0  # meters from home
    geofence_enabled: bool = True
    geofence_radius: float = 100.0  # meters


@dataclass
class NavigationConfig:
    """Navigation configuration"""
    default_airspeed: float = 10.0  # m/s
    hover_altitude: float = 5.0  # meters
    return_home_altitude: float = 10.0  # meters
    return_home_speed: float = 5.0  # m/s
    loiter_speed: float = 2.0  # m/s


@dataclass
class GroundStationConfig:
    """Complete ground station configuration"""
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)


class ConfigManager:
    """Ground Station Configuration Manager"""

    def __init__(self, config_path: str = None):
        self.config_path = config_path or os.path.join(
            os.path.dirname(__file__), 'ground_config.yaml')
        self.config = GroundStationConfig()
        self._last_loaded = 0.0

    async def load(self) -> GroundStationConfig:
        """Load configuration from file"""
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    data = json.load(f)
                await self._parse_config(data)
                self._last_loaded = time.time()
                logger.info(f"Ground station config loaded from {self.config_path}")
            else:
                logger.warning(f"Config file not found: {self.config_path}, using defaults")
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
        
        return self.config

    async def _parse_config(self, data: Dict[str, Any]):
        """Parse configuration data into dataclasses"""
        try:
            # Parse telemetry
            t_data = data.get('telemetry', {})
            self.config.telemetry = TelemetryConfig(**{
                k: t_data.get(k, v) 
                for k, v in TelemetryConfig.__dataclass_fields__.items()
            })

            # Parse communication
            c_data = data.get('communication', {})
            self.config.communication = CommunicationConfig(**{
                k: c_data.get(k, v) 
                for k, v in CommunicationConfig.__dataclass_fields__.items()
            })

            # Parse safety
            s_data = data.get('safety', {})
            self.config.safety = SafetyConfig(**{
                k: s_data.get(k, v) 
                for k, v in SafetyConfig.__dataclass_fields__.items()
            })

            # Parse navigation
            n_data = data.get('navigation', {})
            self.config.navigation = NavigationConfig(**{
                k: n_data.get(k, v) 
                for k, v in NavigationConfig.__dataclass_fields__.items()
            })
        except Exception as e:
            logger.error(f"Config parse error: {e}")

    def get_config(self) -> GroundStationConfig:
        """Get current configuration"""
        return self.config

    async def save(self) -> bool:
        """Save configuration to file"""
        try:
            data = {
                'telemetry': {
                    'enabled': self.config.telemetry.enabled,
                    'broadcast_interval': self.config.telemetry.broadcast_interval,
                    'data_fields': self.config.telemetry.data_fields,
                    'max_buffer_size': self.config.telemetry.max_buffer_size,
                    'compression_enabled': self.config.telemetry.compression_enabled,
                },
                'communication': {
                    'websocket_url': self.config.communication.websocket_url,
                    'http_host': self.config.communication.http_host,
                    'http_port': self.config.communication.http_port,
                    'http_allowed_origins': self.config.communication.http_allowed_origins,
                    'api_key': self.config.communication.api_key,
                    'reconnect_interval': self.config.communication.reconnect_interval,
                    'max_reconnect_attempts': self.config.communication.max_reconnect_attempts,
                },
                'safety': {
                    'enabled': self.config.safety.enabled,
                    'rssi_threshold': self.config.safety.rssi_threshold,
                    'voltage_low_threshold': self.config.safety.voltage_low_threshold,
                    'altitude_max': self.config.safety.altitude_max,
                    'distance_max': self.config.safety.distance_max,
                    'geofence_enabled': self.config.safety.geofence_enabled,
                    'geofence_radius': self.config.safety.geofence_radius,
                },
                'navigation': {
                    'default_airspeed': self.config.navigation.default_airspeed,
                    'hover_altitude': self.config.navigation.hover_altitude,
                    'return_home_altitude': self.config.navigation.return_home_altitude,
                    'return_home_speed': self.config.navigation.return_home_speed,
                    'loiter_speed': self.config.navigation.loiter_speed,
                }
            }
            
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            with open(self.config_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            logger.info(f"Ground station config saved to {self.config_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to save config: {e}")
            return False

    def reload(self):
        """Reload configuration from file"""
        asyncio.create_task(self.load())


# Default configuration
DEFAULT_CONFIG = GroundStationConfig()

__all__ = ['ConfigManager', 'GroundStationConfig', 'TelemetryConfig', 'CommunicationConfig',
           'SafetyConfig', 'NavigationConfig', 'DEFAULT_CONFIG']