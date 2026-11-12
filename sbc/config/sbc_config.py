#!/usr/bin/env python3
"""
SBC Configuration Manager
Loads and saves SBC configuration
"""

import json
import os
from typing import Dict, Any, Optional

DEFAULT_SBC_CONFIG = {
    'mavlink': {
        'device': '/dev/ttyTHS1',
        'baudrate': 921600,
        'source_system': 1,
        'source_component': 191
    },
    'webrtc': {
        'stun_servers': ['stun.l.google.com:19302'],
        'turn_servers': [],
        'video_resolution': '1280x720',
        'video_framerate': 30,
        'video_bitrate': 4000000
    },
    'tailscale': {
        'auth_key_env': 'TAILSCALE_AUTH_KEY',
        'vps_mesh_enabled': True
    },
    'sms': {
        'provider': 'twilio',
        'enabled': True
    },
    'controller': {
        'enabled': True,
        'backend': 'auto',
        'device_path': '/dev/input/js0',
        'poll_rate': 50
    },
    'api': {
        'host': '0.0.0.0',
        'port': 8081
    },
    'logging': {
        'level': 'INFO',
        'file': '/var/log/hermes-sbc.log'
    }
}


def load_sbc_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load SBC configuration from file or return defaults"""
    if config_path is None:
        config_path = os.path.join(os.path.dirname(__file__), 'sbc_config.json')

    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
            return _merge_config(DEFAULT_SBC_CONFIG, config)
    except Exception as e:
        print(f"Error loading SBC config: {e}")

    return DEFAULT_SBC_CONFIG


def _merge_config(default: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge override config into default"""
    result = default.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _merge_config(result[key], value)
        else:
            result[key] = value
    return result