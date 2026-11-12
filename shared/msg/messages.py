#!/usr/bin/env python3
"""
Shared Message Types
Common message definitions used across the system
"""

from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any, AnyStr


class MessageType(Enum):
    CONTROL = "control"
    TELEMETRY = "telemetry"
    FAILSAFE = "failsafe"
    CONFIG = "config"
    HEARTBEAT = "heartbeat"
    COMMAND_ACK = "command_ack"


@dataclass
class ControlMessage:
    """Control command message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class TelemetryMessage:
    """Telemetry data message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class FailsafeMessage:
    """Failsafe notification message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class ConfigMessage:
    """Configuration change message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class HeartbeatMessage:
    """Heartbeat notification message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class CommandAckMessage:
    """Command acknowledgment message"""
    type: MessageType
    payload: Dict[str, Any]
    timestamp: float = 0.0


@dataclass
class MessagePriority:
    """Priority levels for message handling"""
    HIGH = 1
    NORMAL = 2
    LOW = 3
    BACKGROUND = 4


class MessagePriorityLevel:
    """Convenience class for message priorities"""
    @staticmethod
    def HIGH(priority: int = MessagePriority.HIGH) -> int:
        return priority

    @staticmethod
    def NORMAL(priority: int = MessagePriority.NORMAL) -> int:
        return priority

    @staticmethod
    def LOW(priority: int = MessagePriority.LOW) -> int:
        return priority

    @staticmethod
    def BACKGROUND(priority: int = MessagePriority.BACKGROUND) -> int:
        return priority