#!/usr/bin/env python3
"""
LoRa Protocol for Drone Control
Defines message framing and protocol for LoRa-based control and telemetry
"""

import struct
import time
import logging
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum

from .lora_security import (
    LoRaSecurity, LoRaSecurityConfig, LoRaSecurityLevel,
    LoRaAuthError, LoRaReplayError, LoRaTimestampError
)

logger = logging.getLogger(__name__)


class LoRaMessageType(Enum):
    """LoRa message types"""
    # SBC -> FC commands
    CMD_TAKEOFF = 0x01
    CMD_LAND = 0x02
    CMD_RTH = 0x03
    CMD_SET_MODE = 0x04
    CMD_SET_ATTITUDE = 0x05
    CMD_SET_POSITION = 0x06
    CMD_SET_VELOCITY = 0x07
    CMD_EMERGENCY_STOP = 0x08
    CMD_PING = 0x09

    # FC -> SBC telemetry
    TELEMETRY_POSITION = 0x10
    TELEMETRY_ATTITUDE = 0x11
    TELEMETRY_VELOCITY = 0x12
    TELEMETRY_STATUS = 0x13
    TELEMETRY_SENSOR = 0x14

    # ACK/NAK
    ACK = 0x20
    NAK = 0x21

    # System
    HEARTBEAT = 0x30
    CONFIG = 0x31


class LoRaPriority(Enum):
    """Message priority for LoRa transmission"""
    CRITICAL = 0  # Emergency stop, failsafe
    HIGH = 1      # Flight commands
    NORMAL = 2    # Regular telemetry
    LOW = 3       # Logging/status


@dataclass
class LoRaFrame:
    """LoRa frame structure"""
    version: int = 1
    msg_type: LoRaMessageType = LoRaMessageType.HEARTBEAT
    priority: LoRaPriority = LoRaPriority.NORMAL
    sequence: int = 0
    payload: bytes = b''
    timestamp: float = 0
    device_id: int = 0

    def to_bytes(self) -> bytes:
        """Serialize frame to bytes for LoRa transmission"""
        # Header: version(1) + type(1) + priority(1) + seq(2) + ts(4) + payload_len(1) + devid(4) = 13 bytes
        header = struct.pack(
            '>BBHBI',  # version, msg_type, priority, seq, ts (8 bytes)
            self.version,
            self.msg_type.value,
            self.priority.value,
            self.sequence & 0xFFFF,
            int(self.timestamp) & 0xFFFFFFFF,
        )
        # Add payload length separately
        frame = header + bytes([len(self.payload) & 0xFF]) + self.payload
        # Add device ID (4 bytes, big endian)
        frame += struct.pack('>I', self.device_id & 0xFFFFFFFF)
        return frame

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['LoRaFrame']:
        """Deserialize frame from bytes"""
        if len(data) < 11:
            return None

        try:
            version = data[0]
            msg_type_val = data[1]
            priority_val = data[2]
            sequence = struct.unpack('>H', data[3:5])[0]
            timestamp = struct.unpack('>I', data[5:9])[0]
            payload_len = data[9]
            payload = data[10:10 + payload_len]

            # Validate message type
            try:
                msg_type = LoRaMessageType(msg_type_val)
            except ValueError:
                logger.warning(f"Unknown LoRa message type: {msg_type_val}")
                return None

            # Extract device ID from remaining bytes after payload
            rx_seq_offset = 10 + payload_len
            if len(data) >= rx_seq_offset + 4:
                device_id = struct.unpack('>I', data[rx_seq_offset:rx_seq_offset+4])[0]
            else:
                device_id = 0

            return cls(
                version=version,
                msg_type=msg_type,
                priority=LoRaPriority(priority_val),
                sequence=sequence,
                payload=payload,
                timestamp=timestamp,
                device_id=device_id,
            )
        except Exception as e:
            logger.error(f"Frame parse error: {e}")
            return None


@dataclass
class PositionCommand:
    """Position command payload"""
    lat: float
    lon: float
    alt: float
    heading: float = 0.0
    speed: float = 5.0  # m/s

    def to_bytes(self) -> bytes:
        return struct.pack('<ddddf', self.lat, self.lon, self.alt, self.heading, self.speed)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['PositionCommand']:
        if len(data) < 32:
            return None
        lat, lon, alt, heading, speed = struct.unpack('<ddddf', data[:32])
        return cls(lat=lat, lon=lon, alt=alt, heading=heading, speed=speed)


@dataclass
class VelocityCommand:
    """Velocity command payload"""
    vx: float  # m/s
    vy: float  # m/s
    vz: float  # m/s
    yaw_rate: float = 0.0  # rad/s

    def to_bytes(self) -> bytes:
        return struct.pack('<ffff', self.vx, self.vy, self.vz, self.yaw_rate)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['VelocityCommand']:
        if len(data) < 16:
            return None
        vx, vy, vz, yaw_rate = struct.unpack('<ffff', data[:16])
        return cls(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)


@dataclass
class TelemetryPosition:
    """Position telemetry payload"""
    lat: float
    lon: float
    alt: float
    alt_amsl: float
    heading: float
    speed: float
    timestamp: float = 0

    def to_bytes(self) -> bytes:
        return struct.pack('<ddddffI', self.lat, self.lon, self.alt, self.alt_amsl,
                           self.heading, self.speed, int(self.timestamp) & 0xFFFFFFFF)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['TelemetryPosition']:
        if len(data) < 44:
            return None
        lat, lon, alt, alt_amsl, heading, speed, ts = struct.unpack('<ddddffI', data[:44])
        return cls(lat=lat, lon=lon, alt=alt, alt_amsl=alt_amsl, heading=heading,
                   speed=speed, timestamp=ts)


@dataclass
class TelemetryAttitude:
    """Attitude telemetry payload"""
    roll: float
    pitch: float
    yaw: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float

    def to_bytes(self) -> bytes:
        return struct.pack('<fffffffff', self.roll, self.pitch, self.yaw,
                           self.accel_x, self.accel_y, self.accel_z,
                           self.gyro_x, self.gyro_y, self.gyro_z)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['TelemetryAttitude']:
        if len(data) < 36:
            return None
        vals = struct.unpack('<fffffffff', data[:36])
        return cls(roll=vals[0], pitch=vals[1], yaw=vals[2],
                   accel_x=vals[3], accel_y=vals[4], accel_z=vals[5],
                   gyro_x=vals[6], gyro_y=vals[7], gyro_z=vals[8])


@dataclass
class TelemetryStatus:
    """Flight status telemetry payload"""
    armed: bool
    flying: bool
    battery_voltage: float
    battery_remaining: int
    mode: str
    error_flags: int

    def to_bytes(self) -> bytes:
        return struct.pack('>??fBiH',
                           self.armed, self.flying,
                           self.battery_voltage,
                           self.battery_remaining,
                           len(self.mode), self.mode.encode('utf-8')[:255],
                           self.error_flags)

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['TelemetryStatus']:
        if len(data) < 20:
            return None
        try:
            armed, flying, voltage, remaining, mode_len, error_flags = struct.unpack('>??fBiH', data[:12])
            mode = data[12:12 + mode_len].decode('utf-8', errors='ignore')
            return cls(armed=armed, flying=flying, battery_voltage=voltage,
                       battery_remaining=remaining, mode=mode, error_flags=error_flags)
        except Exception as e:
            logger.error(f"Status parse error: {e}")
            return None


# Protocol handler
class LoRaProtocol:
    """LoRa protocol handler for framing and encoding/decoding"""

    def __init__(self, security_config: Optional[LoRaSecurityConfig] = None, device_id: int = 0):
        self._tx_sequence = 0
        self._rx_sequence = 0
        self.device_id = device_id
        self.security = LoRaSecurity(security_config) if security_config else None

    def next_sequence(self) -> int:
        """Get next TX sequence number"""
        self._tx_sequence = (self._tx_sequence + 1) & 0xFFFF
        return self._tx_sequence

    def encode_command(self, cmd_type: LoRaMessageType, payload: bytes,
                       priority: LoRaPriority = LoRaPriority.NORMAL,
                       device_id: int = 0) -> bytes:
        """Encode a command frame with optional security"""
        sequence = self.next_sequence()
        frame = LoRaFrame(
            msg_type=cmd_type,
            priority=priority,
            sequence=sequence,
            payload=payload,
            timestamp=time.time(),
            device_id=device_id or self.device_id
        )
        frame_bytes = frame.to_bytes()

        # Apply security if configured
        if self.security:
            frame_bytes = self.security.secure_encode(
                frame_bytes,
                device_id=device_id or self.device_id,
                sequence=sequence
            )

        return frame_bytes

    def decode_frame(self, data: bytes, device_id: int = 0) -> Optional[LoRaFrame]:
        """Decode a received frame with optional security"""
        # Apply security if configured
        if self.security:
            try:
                payload, sequence, _ = self.security.secure_decode(data, device_id)
                frame = LoRaFrame.from_bytes(payload)
                if frame:
                    # Validate security policies
                    self.security.validate_received(
                        frame.device_id,
                        sequence or frame.sequence,
                        int(frame.timestamp)
                    )
                    return frame
            except (LoRaAuthError, LoRaReplayError, LoRaTimestampError) as e:
                logger.warning(f"LoRa security validation failed: {e}")
                return None

        return LoRaFrame.from_bytes(data)

    def encode_takeoff(self, altitude: float = 10.0, device_id: int = 0) -> bytes:
        """Encode takeoff command"""
        payload = struct.pack('<f', altitude)
        return self.encode_command(LoRaMessageType.CMD_TAKEOFF, payload, LoRaPriority.HIGH)

    def encode_land(self) -> bytes:
        """Encode land command"""
        return self.encode_command(LoRaMessageType.CMD_LAND, b'', LoRaPriority.CRITICAL)

    def encode_rth(self) -> bytes:
        """Encode return-to-home command"""
        return self.encode_command(LoRaMessageType.CMD_RTH, b'', LoRaPriority.CRITICAL)

    def encode_position(self, lat: float, lon: float, alt: float,
                        heading: float = 0.0, speed: float = 5.0) -> bytes:
        """Encode position command"""
        cmd = PositionCommand(lat=lat, lon=lon, alt=alt, heading=heading, speed=speed)
        return self.encode_command(LoRaMessageType.CMD_SET_POSITION, cmd.to_bytes(), LoRaPriority.HIGH)

    def encode_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bytes:
        """Encode velocity command"""
        cmd = VelocityCommand(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)
        return self.encode_command(LoRaMessageType.CMD_SET_VELOCITY, cmd.to_bytes(), LoRaPriority.HIGH)

    def encode_emergency_stop(self) -> bytes:
        """Encode emergency stop command"""
        return self.encode_command(LoRaMessageType.CMD_EMERGENCY_STOP, b'', LoRaPriority.CRITICAL)

    def encode_heartbeat(self) -> bytes:
        """Encode heartbeat"""
        return self.encode_command(LoRaMessageType.HEARTBEAT, b'', LoRaPriority.NORMAL)

    def decode_position_command(self, payload: bytes) -> Optional[PositionCommand]:
        return PositionCommand.from_bytes(payload)

    def decode_velocity_command(self, payload: bytes) -> Optional[VelocityCommand]:
        return VelocityCommand.from_bytes(payload)

    def decode_telemetry_position(self, payload: bytes) -> Optional[TelemetryPosition]:
        return TelemetryPosition.from_bytes(payload)

    def decode_telemetry_attitude(self, payload: bytes) -> Optional[TelemetryAttitude]:
        return TelemetryAttitude.from_bytes(payload)

    def decode_telemetry_status(self, payload: bytes) -> Optional[TelemetryStatus]:
        return TelemetryStatus.from_bytes(payload)