#!/usr/bin/env python3
"""
MAVLink Bridge for SBC
Handles bidirectional MAVLink communication between Flight Controller and Ground Station
via serial (FC) and WebRTC data channels (Ground)
"""

import asyncio
import logging
import struct
import time
from typing import Callable, Dict, Optional, Any
from dataclasses import dataclass
from collections import deque

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    serial_tools_list_ports = None
    SERIAL_AVAILABLE = False

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)

# MAVLink constants
MAVLINK_STX = 0xFD
MAVLINK_MAX_PAYLOAD_LEN = 255

# Common MAVLink message IDs
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_PARAM_REQUEST_READ = 20
MAVLINK_MSG_ID_PARAM_VALUE = 22
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_MANUAL_CONTROL = 69
MAVLINK_MSG_ID_RC_CHANNELS = 65
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33
MAVLINK_MSG_ID_VFR_HUD = 74
MAVLINK_MSG_ID_BATTERY_STATUS = 147
MAVLINK_MSG_ID_MISSION_ITEM = 39
MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_SET_MODE = 11
MAVLINK_MSG_ID_COMPONENT_INFORMATION = 390

# MAVLink command IDs
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_MISSION_START = 300

# MAV_MODE flags
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# MAV_STATE
MAV_STATE_STANDBY = 0
MAV_STATE_ACTIVE = 4
MAV_STATE_CRITICAL = 6
MAV_STATE_EMERGENCY = 7

@dataclass
class MavlinkMessage:
    msgid: int
    payload: bytes
    seq: int
    sysid: int
    compid: int
    timestamp_us: int = 0
    
    def __post_init__(self):
        if self.timestamp_us == 0:
            self.timestamp_us = int(time.time() * 1_000_000)


class MavlinkBridge:
    def __init__(self, config: dict, tailscale_manager=None):
        self.config = config
        self.tailscale_manager = tailscale_manager
        self.mavlink_config = config.get('network', {}).get('mavlink', {})
        
        # Serial connection
        self.serial_port = None
        self.serial_task = None
        self.read_buffer = bytearray()
        
        # Message queues
        self.tx_queue = asyncio.Queue(maxsize=1000)
        self.rx_queue = asyncio.Queue(maxsize=1000)
        
        # Ground station data channels (WebRTC)
        self.ground_channels: Dict[str, Any] = {}  # channel_id -> DataChannel
        
        # Telemetry callback
        self.telemetry_callback: Optional[Callable] = None
        
        # State
        self.connected = False
        self.fc_heartbeat = None
        self.last_heartbeat_time = 0
        self.message_seq = 0
        self.source_system = self.mavlink_config.get('source_system', 1)
        self.source_component = self.mavlink_config.get('source_component', 191)
        self.target_system = self.mavlink_config.get('target_system', 1)
        self.target_component = self.mavlink_config.get('target_component', 1)
        
        # Pending commands
        self.pending_commands: Dict[int, asyncio.Future] = {}
        self.command_seq = 0
    
    async def connect(self):
        """Connect to flight controller via serial"""
        device = self.mavlink_config.get('device', '/dev/ttyTHS1')
        baudrate = self.mavlink_config.get('baudrate', 921600)
        
        logger.info(f"Connecting to FC on {device} @ {baudrate}")
        
        try:
            self.serial_port = serial.Serial(
                port=device,
                baudrate=baudrate,
                timeout=0.1,
                write_timeout=1
            )
            self.connected = True
            
            # Start read/write tasks
            self.serial_task = asyncio.create_task(self._serial_loop())
            
            # Start heartbeat sender
            asyncio.create_task(self._heartbeat_loop())
            
            # Request data streams
            await self._request_data_streams()
            
            logger.info("MAVLink bridge connected")
        except Exception as e:
            logger.error(f"Failed to connect to FC: {e}")
            raise
    
    async def disconnect(self):
        """Disconnect from flight controller"""
        self.connected = False
        
        if self.serial_task:
            self.serial_task.cancel()
            try:
                await self.serial_task
            except asyncio.CancelledError:
                pass
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        # Close ground channels
        for channel in self.ground_channels.values():
            try:
                channel.close()
            except:
                pass
        self.ground_channels.clear()
        
        logger.info("MAVLink bridge disconnected")
    
    def register_telemetry_callback(self, callback: Callable):
        """Register callback for telemetry updates"""
        self.telemetry_callback = callback
    
    def register_ground_channel(self, channel_id: str, data_channel):
        """Register a WebRTC data channel from ground station"""
        self.ground_channels[channel_id] = data_channel
        logger.info(f"Ground channel registered: {channel_id}")
    
    def unregister_ground_channel(self, channel_id: str):
        """Unregister a ground station data channel"""
        if channel_id in self.ground_channels:
            del self.ground_channels[channel_id]
            logger.info(f"Ground channel unregistered: {channel_id}")
    
    async def _serial_loop(self):
        """Main serial read/write loop"""
        while self.connected:
            try:
                # Write pending messages
                try:
                    msg = await asyncio.wait_for(self.tx_queue.get(), timeout=0.01)
                    await self._send_mavlink_message(msg)
                except asyncio.TimeoutError:
                    pass
                
                # Read incoming data
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.read_buffer.extend(data)
                    await self._parse_buffer()
                
                await asyncio.sleep(0.001)  # 1ms loop
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Serial loop error: {e}")
                await asyncio.sleep(0.1)
    
    async def _parse_buffer(self):
        """Parse MAVLink messages from buffer"""
        while len(self.read_buffer) >= 8:  # Minimum header size
            # Find STX
            stx_idx = self.read_buffer.find(MAVLINK_STX)
            if stx_idx == -1:
                self.read_buffer.clear()
                return
            
            if stx_idx > 0:
                self.read_buffer = self.read_buffer[stx_idx:]
            
            if len(self.read_buffer) < 10:
                return
            
            # Parse header
            payload_len = self.read_buffer[1]
            if payload_len > MAVLINK_MAX_PAYLOAD_LEN:
                self.read_buffer = self.read_buffer[1:]
                continue
            
            total_len = 10 + payload_len + 2  # header + payload + CRC
            if len(self.read_buffer) < total_len:
                return
            
            # Extract message
            msg_data = self.read_buffer[:total_len]
            self.read_buffer = self.read_buffer[total_len:]
            
            # Verify CRC (simplified - full CRC requires X.25 implementation)
            msg = self._decode_mavlink(msg_data)
            if msg:
                await self._handle_mavlink_message(msg)
    
    def _decode_mavlink(self, data: bytes) -> Optional[MavlinkMessage]:
        """Decode MAVLink v2 packet"""
        if len(data) < 10:
            return None
        
        if data[0] != MAVLINK_STX:
            return None
        
        payload_len = data[1]
        seq = data[2]
        sysid = data[3]
        compid = data[4]
        msgid = (data[7] << 16) | (data[6] << 8) | data[5]
        
        payload = data[10:10+payload_len]
        
        return MavlinkMessage(
            msgid=msgid,
            payload=payload,
            seq=seq,
            sysid=sysid,
            compid=compid
        )
    
    async def _handle_mavlink_message(self, msg: MavlinkMessage):
        """Handle received MAVLink message"""
        # Update last heartbeat time
        self.last_heartbeat_time = time.time()
        
        # Forward to ground station data channels
        await self._forward_to_ground(msg)
        
        # Handle specific message types
        if msg.msgid == MAVLINK_MSG_ID_HEARTBEAT:
            self.fc_heartbeat = msg
        elif msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK:
            await self._handle_command_ack(msg)
        
        # Parse telemetry and trigger callback
        telemetry = self._parse_telemetry(msg)
        if telemetry and self.telemetry_callback:
            await self.telemetry_callback(telemetry)
    
    async def _forward_to_ground(self, msg: MavlinkMessage):
        """Forward MAVLink message to all ground station data channels"""
        if not self.ground_channels:
            return
        
        # Encode for WebRTC data channel
        encoded = self._encode_mavlink_v2(msg)
        
        dead_channels = []
        for channel_id, channel in self.ground_channels.items():
            try:
                if hasattr(channel, 'send'):
                    channel.send(encoded)
                elif hasattr(channel, 'write'):
                    await channel.write(encoded)
            except Exception as e:
                logger.warning(f"Failed to send to ground channel {channel_id}: {e}")
                dead_channels.append(channel_id)
        
        for cid in dead_channels:
            self.unregister_ground_channel(cid)
    
    async def _handle_command_ack(self, msg: MavlinkMessage):
        """Handle command acknowledgment"""
        if len(msg.payload) < 4:
            return
        
        command = struct.unpack('<H', msg.payload[0:2])[0]
        result = msg.payload[2]
        
        # Resolve pending command future
        for seq, future in list(self.pending_commands.items()):
            if not future.done():
                future.set_result({'command': command, 'result': result})
                del self.pending_commands[seq]
                break
    
    def _parse_telemetry(self, msg: MavlinkMessage) -> Optional[dict]:
        """Parse telemetry data from MAVLink message"""
        telemetry = {'timestamp_us': msg.timestamp_us}
        
        try:
            if msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                # lat, lon, alt, relative_alt, vx, vy, vz, hdg
                vals = struct.unpack('<iiiihhhh', msg.payload[:28])
                telemetry['position'] = {
                    'lat': vals[0] / 1e7,
                    'lon': vals[1] / 1e7,
                    'alt': vals[2] / 1000.0,
                    'relative_alt': vals[3] / 1000.0,
                    'vx': vals[4] / 100.0,
                    'vy': vals[5] / 100.0,
                    'vz': vals[6] / 100.0,
                    'hdg': vals[7] / 100.0
                }
            
            elif msg.msgid == MAVLINK_MSG_ID_ATTITUDE:
                vals = struct.unpack('<fffffff', msg.payload[:28])
                telemetry['attitude'] = {
                    'roll': vals[0],
                    'pitch': vals[1],
                    'yaw': vals[2],
                    'rollspeed': vals[3],
                    'pitchspeed': vals[4],
                    'yawspeed': vals[6]
                }
            
            elif msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS:
                if len(msg.payload) >= 32:
                    vals = struct.unpack('<BBBhhhhhhb', msg.payload[:32])
                    voltages = list(struct.unpack('<10H', msg.payload[32:52])) if len(msg.payload) >= 52 else []
                    telemetry['battery'] = {
                        'id': vals[0],
                        'function': vals[1],
                        'type': vals[2],
                        'temperature': vals[3] / 100.0,
                        'voltages': [v / 1000.0 for v in voltages[:vals[0]]] if voltages else [],
                        'current': vals[4] / 100.0,
                        'consumed': vals[5] / 1000.0,
                        'energy_consumed': vals[6] / 1000.0,
                        'remaining': vals[7]
                    }
            
            elif msg.msgid == MAVLINK_MSG_ID_VFR_HUD:
                vals = struct.unpack('<ffffhh', msg.payload[:20])
                telemetry['vfr'] = {
                    'airspeed': vals[0],
                    'groundspeed': vals[1],
                    'heading': vals[2],
                    'throttle': vals[3],
                    'alt': vals[4],
                    'climb': vals[5]
                }
            
            elif msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS:
                chancount = msg.payload[0]
                channels = list(struct.unpack(f'<{chancount}H', msg.payload[1:1+chancount*2]))
                rssi = struct.unpack('<H', msg.payload[1+chancount*2:3+chancount*2])[0] if len(msg.payload) >= 3+chancount*2 else 0
                telemetry['rc_channels'] = {
                    'channels': channels,
                    'rssi': rssi
                }
            
            elif msg.msgid == MAVLINK_MSG_ID_HEARTBEAT:
                vals = struct.unpack('<IBBBBB', msg.payload[:9])
                telemetry['heartbeat'] = {
                    'custom_mode': vals[0],
                    'type': vals[1],
                    'autopilot': vals[2],
                    'base_mode': vals[3],
                    'system_status': vals[4],
                    'mavlink_version': vals[5],
                    'armed': bool(vals[3] & MAV_MODE_FLAG_SAFETY_ARMED)
                }
                telemetry['armed'] = bool(vals[3] & MAV_MODE_FLAG_SAFETY_ARMED)
                telemetry['flight_mode'] = vals[0] & 0xFF
            
            return telemetry if len(telemetry) > 1 else None
            
        except Exception as e:
            logger.debug(f"Telemetry parse error for msg {msg.msgid}: {e}")
            return None
    
    async def _heartbeat_loop(self):
        """Send periodic heartbeat to FC"""
        interval = self.mavlink_config.get('heartbeat_interval', 1000) / 1000.0
        
        while self.connected:
            try:
                await self.send_heartbeat()
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                await asyncio.sleep(1)
    
    async def send_heartbeat(self):
        """Send heartbeat to FC"""
        # Base mode: custom mode enabled
        base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = 0
        system_status = MAV_STATE_ACTIVE
        
        payload = struct.pack('<IBBBBB', custom_mode, 2, 3, base_mode, system_status, 3)
        msg = MavlinkMessage(
            msgid=MAVLINK_MSG_ID_HEARTBEAT,
            payload=payload,
            seq=self._next_seq(),
            sysid=self.source_system,
            compid=self.source_component
        )
        await self.tx_queue.put(msg)
    
    async def _request_data_streams(self):
        """Request data streams from FC"""
        streams = self.mavlink_config.get('stream_rates', [])
        
        for stream in streams:
            msg_id = stream.get('msg_id', 0)
            rate = stream.get('rate', 10)
            
            # MAV_CMD_SET_MESSAGE_INTERVAL (511)
            await self.send_command_long(
                command=511,
                param1=float(msg_id),
                param2=1e6 / rate if rate > 0 else -1,  # interval in microseconds
                param3=0, param4=0, param5=0, param6=0, param7=0
            )
    
    def _next_seq(self) -> int:
        self.message_seq = (self.message_seq + 1) % 256
        return self.message_seq
    
    def _next_command_seq(self) -> int:
        self.command_seq = (self.command_seq + 1) % 256
        return self.command_seq
    
    def _encode_mavlink_v2(self, msg: MavlinkMessage) -> bytes:
        """Encode MAVLink v2 packet"""
        payload = msg.payload
        payload_len = len(payload)
        
        # Header
        header = bytearray([MAVLINK_STX, payload_len, msg.seq, msg.sysid, msg.compid])
        
        # Message ID (24-bit, little endian)
        header.extend([
            msg.msgid & 0xFF,
            (msg.msgid >> 8) & 0xFF,
            (msg.msgid >> 16) & 0xFF
        ])
        
        # Payload
        header.extend(payload)
        
        # CRC (simplified - using 0xFFFF for now, real implementation needs X.25 CRC)
        crc = 0xFFFF
        header.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        
        return bytes(header)
    
    async def _send_mavlink_message(self, msg: MavlinkMessage):
        """Send MAVLink message over serial"""
        if not self.serial_port or not self.serial_port.is_open:
            return
        
        encoded = self._encode_mavlink_v2(msg)
        try:
            self.serial_port.write(encoded)
        except Exception as e:
            logger.error(f"Serial write error: {e}")
    
    async def send_command_long(self, command: int, param1=0, param2=0, param3=0, 
                                 param4=0, param5=0, param6=0, param7=0,
                                 confirmation=0, target_system=None, target_component=None) -> dict:
        """Send COMMAND_LONG and wait for ACK"""
        target_system = target_system or self.target_system
        target_component = target_component or self.target_component
        
        payload = struct.pack('<BBBBBfffff',
            target_system, target_component, command, confirmation, 0,
            param1, param2, param3, param4, param5, param6, param7
        )
        
        seq = self._next_command_seq()
        msg = MavlinkMessage(
            msgid=MAVLINK_MSG_ID_COMMAND_LONG,
            payload=payload,
            seq=seq,
            sysid=self.source_system,
            compid=self.source_component
        )
        
        # Create future for ACK
        future = asyncio.get_event_loop().create_future()
        self.pending_commands[seq] = future
        
        await self.tx_queue.put(msg)
        
        try:
            result = await asyncio.wait_for(future, timeout=5.0)
            return result
        except asyncio.TimeoutError:
            return {'error': 'timeout', 'command': command}
        finally:
            self.pending_commands.pop(seq, None)
    
    async def arm(self, arm: bool = True) -> dict:
        """Arm/disarm the vehicle"""
        return await self.send_command_long(
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 if arm else 0.0,
            param2=21196.0 if arm else 0.0  # Force arm if needed
        )
    
    async def trigger_rtl(self) -> dict:
        """Trigger Return to Launch"""
        return await self.send_command_long(
            command=MAV_CMD_NAV_RETURN_TO_LAUNCH
        )
    
    async def land(self) -> dict:
        """Trigger Land"""
        return await self.send_command_long(
            command=MAV_CMD_NAV_LAND
        )
    
    async def takeoff(self, altitude: float) -> dict:
        """Trigger Takeoff"""
        return await self.send_command_long(
            command=MAV_CMD_NAV_TAKEOFF,
            param7=altitude
        )
    
    async def set_mode(self, mode: int, custom_mode: int = 0) -> dict:
        """Set flight mode"""
        return await self.send_command_long(
            command=MAV_CMD_DO_SET_MODE,
            param1=float(mode),
            param2=float(custom_mode)
        )
    
    async def send_manual_control(self, x: int, y: int, z: int, r: int, buttons: int = 0):
        """Send manual control (joystick) command"""
        payload = struct.pack('<hhhhH', x, y, z, r, buttons)
        msg = MavlinkMessage(
            msgid=MAVLINK_MSG_ID_MANUAL_CONTROL,
            payload=payload,
            seq=self._next_seq(),
            sysid=self.source_system,
            compid=self.source_component
        )
        await self.tx_queue.put(msg)
    
    async def send_rc_channels_override(self, channels: list):
        """Send RC channels override"""
        # MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE (70)
        payload = struct.pack('<BBB' + 'H' * 18,
            self.target_system, self.target_component, len(channels), *channels[:18]
        )
        msg = MavlinkMessage(
            msgid=70,
            payload=payload,
            seq=self._next_seq(),
            sysid=self.source_system,
            compid=self.source_component
        )
        await self.tx_queue.put(msg)
    
    def get_connection_status(self) -> dict:
        """Get connection status"""
        return {
            'connected': self.connected,
            'fc_heartbeat': self.fc_heartbeat is not None,
            'last_heartbeat_age': time.time() - self.last_heartbeat_time if self.last_heartbeat_time else None,
            'ground_channels': len(self.ground_channels),
            'serial_port': self.mavlink_config.get('device') if self.serial_port else None
        }