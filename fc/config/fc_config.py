#!/usr/bin/env python3
"""
FC Configuration Manager
Loads and saves flight controller configuration
"""

import json
import os
from typing import Dict, Any, Optional

DEFAULT_FC_CONFIG = {
    'mavlink': {
        'device': '/dev/ttyTHS1',
        'baudrate': 921600,
        'source_system': 1,
        'source_component': 191
    },
    'gps': {
        'device': '/dev/ttyGPS',
        'baudrate': 115200,
        'protocol': 'ublox',
        'update_rate': 10
    },
    'imu': {
        'type': 'icm20689',
        'bus': 1,
        'address': 0x68,
        'sample_rate': 400,
        'accel_range': 16,
        'gyro_range': 2000
    },
    'barometer': {
        'type': 'bmp388',
        'bus': 1,
        'address': 0x77,
        'sample_rate': 100,
        'oversampling': 'ultra_high'
    },
    'battery': {
        'type': 'adc',
        'cell_count': 4,
        'capacity': 5200,
        'cell_min': 3.2,
        'cell_max': 4.2,
        'voltage_divider': 11.0,
        'current_shunt': 0.001,
        'current_gain': 50.0
    },
    'rangefinder': {
        'type': 'tf_luna',
        'bus': 1,
        'address': 0x10,
        'orientation': 'down',
        'min_range': 0.2,
        'max_range': 8.0,
        'sample_rate': 100
    },
    'failsafe': {
        'enabled': True,
        'battery_low_voltage': 3.3,
        'battery_critical_voltage': 3.0,
        'rssi_threshold': 20,
        'geofence_enabled': True,
        'geofence_radius': 100,
        'return_home_altitude': 50,
        'land_on_failsafe': True
    },
    'flight_modes': {
        'default': 'GUIDED',
        'rth_mode': 'RTL',
        'auto_mode': 'AUTO'
    }
}


def load_fc_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load flight controller configuration from file or return defaults"""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), 'fc_config.json')
    
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
            # Merge with defaults
            return _merge_config(DEFAULT_FC_CONFIG, config)
    except Exception as e:
        print(f"Error loading FC config: {e}")
    
    return DEFAULT_FC_CONFIG


def save_fc_config(config: Dict[str, Any], config_path: Optional[str] = None) -> bool:
    """Save flight controller configuration to file"""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), 'fc_config.json')
    
    try:
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        return True
    except Exception as e:
        print(f"Error saving FC config: {e}")
        return False


def _merge_config(default: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge override config into default"""
    result = default.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _merge_config(result[key], value)
        else:
            result[key] = value
    return result