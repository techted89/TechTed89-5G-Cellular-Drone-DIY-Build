#!/usr/bin/env python3
"""
Configuration Loader for 5G Drone System
Loads YAML config with environment variable substitution
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path


def load_config(config_path: str = None) -> Dict[str, Any]:
    """
    Load configuration from YAML file with environment variable substitution.
    
    Args:
        config_path: Path to config file. If None, searches default locations.
    
    Returns:
        Configuration dictionary with env vars substituted.
    """
    if config_path is None:
        # Search default locations
        search_paths = [
            Path.cwd() / "shared/config/drone_config.yaml",
            Path.cwd() / "config/drone_config.yaml",
            Path(__file__).parent.parent / "shared/config/drone_config.yaml",
            Path("/etc/drone/drone_config.yaml"),
            Path.home() / ".drone/drone_config.yaml",
        ]
        
        for path in search_paths:
            if path.exists():
                config_path = str(path)
                break
        else:
            raise FileNotFoundError("No config file found. Tried: " + ", ".join(str(p) for p in search_paths))
    
    config_path = Path(config_path)
    
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    
    with open(config_path, 'r') as f:
        content = f.read()
    
    # Substitute environment variables: ${VAR} or ${VAR:-default}
    def substitute_env(match):
        var_expr = match.group(1)
        if ':-' in var_expr:
            var_name, default = var_expr.split(':-', 1)
            return os.environ.get(var_name, default)
        else:
            return os.environ.get(var_expr, '')
    
    import re
    content = re.sub(r'\$\{([^}]+)\}', substitute_env, content)
    
    config = yaml.safe_load(content)
    
    # Add config file path for reference
    config['_config_path'] = str(config_path)
    config['_config_dir'] = str(config_path.parent)
    
    return config


def save_config(config: Dict[str, Any], config_path: str):
    """Save configuration to YAML file"""
    config_path = Path(config_path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Remove internal keys
    save_config = {k: v for k, v in config.items() if not k.startswith('_')}
    
    with open(config_path, 'w') as f:
        yaml.dump(save_config, f, default_flow_style=False, sort_keys=False)


def merge_configs(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge two config dictionaries"""
    result = base.copy()
    
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = merge_configs(result[key], value)
        else:
            result[key] = value
    
    return result


def get_config_value(config: Dict[str, Any], path: str, default: Any = None) -> Any:
    """Get nested config value using dot notation (e.g., 'network.tailscale.hostname')"""
    keys = path.split('.')
    value = config
    
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            return default
    
    return value