#!/usr/bin/env python3
"""
LoRa Bridge for Drone Control
Bridge LoRa radio to MAVLink for ground station control via LoRa
"""

import asyncio
import logging
import time
import struct
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

from fc.sensors.lora import LoRaRadio, LoRaPacket, LoRaMode, LoRaBandwidth, LoRaSpreadingFactor
from fc.mavlink.lora_protocol import LoRaProtocol, LoRaMessageType, LoRaPriority, LoRaFrame
from fc.mavlink.lora_security import LoRaSecurityLevel, LoRaSecurityConfig

logger = logging.getLogger(__name__)


class LoRaBridgeState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    LISTENING = "listening"
    TRANSMITTING = "transmitting"
    ERROR = "error"


@dataclass
class LoRaBridgeConfig:
    """LoRa bridge configuration"""
    frequency: float = 915.0  # MHz
    tx_power: int = 20  # dBm
    spreading_factor: LoRaSpreadingFactor = LoRaSpreadingFactor.SF_10
    bandwidth: LoRaBandwidth = LoRaBandwidth.BW_125_KHZ
    coding_rate: int = 1  # 4/5
    preamble_length: int = 8
    sync_word: int = 0x12
    crc_enabled: bool = True
    max_payload_length: int = 255
    enable_received_callback: bool = True
    enable_sent_callback: bool = True
    # Security configuration
    security_level: int = 0  # 0=NONE, 1=AUTH_ONLY, 2=ENCRYPTED, 3=FULL
    pre_shared_key: bytes = b''   # 32 bytes for HMAC-SHA256
    encryption_key: bytes = b''   # 16 bytes for AES-128
    device_id: int = 0            # Unique device identifier (0 = any)
    allowed_device_ids: tuple = ()  # Whitelist of allowed device IDs


class LoRaBridge:
    """LoRa bridge connecting LoRa radio to control system"""

    def __init__(self, config: dict = None, mavlink_link=None):
        self.config = config or {}
        lora_dict = self.config.get('lora', {}) if isinstance(self.config.get('lora'), dict) else {}
        self.lora_config = LoRaBridgeConfig(
            frequency=lora_dict.get('frequency', 915.0),
            tx_power=lora_dict.get('tx_power', 20),
            spreading_factor=LoRaSpreadingFactor(lora_dict.get('spreading_factor', 10)),
            bandwidth=LoRaBandwidth(lora_dict.get('bandwidth', 7)),
            coding_rate=lora_dict.get('coding_rate', 1),
            preamble_length=lora_dict.get('preamble_length', 8),
            sync_word=lora_dict.get('sync_word', 0x12),
            crc_enabled=lora_dict.get('crc_enabled', True),
        )

        self.state = LoRaBridgeState.DISCONNECTED
        self.radio: Optional[LoRaRadio] = None
        self.protocol = LoRaProtocol()
        self.mavlink_link = mavlink_link

        self._received_callback: Optional[Callable[[LoRaFrame], None]] = None
        self._sent_callback: Optional[Callable[[LoRaFrame], None]] = None
        self._running = False
        self._rx_task = None

        self.packets_received = 0
        self.packets_sent = 0
        self.bytes_sent = 0
        self.bytes_received = 0

    async def start(self, received_callback: Optional[Callable[[LoRaFrame], None]] = None,
                   mavlink_send_callback: Optional[Callable[[bytes], None]] = None) -> bool:
        """Start the LoRa bridge"""
        logger.info(f"Starting LoRa bridge: freq={self.lora_config.frequency}MHz, "
                    f"SF{self.lora_config.spreading_factor.value}, "
                    f"BW{self.lora_config.bandwidth.name}")

        self.state = LoRaBridgeState.CONNECTED
        self._received_callback = received_callback
        self._sent_callback = mavlink_send_callback  # Use mavlink callback as sent callback
        self.mavlink_send_callback = mavlink_send_callback

        try:
            self.radio = LoRaRadio({
                'lora': {
                    'frequency': self.lora_config.frequency,
                    'tx_power': self.lora_config.tx_power,
                    'spreading_factor': self.lora_config.spreading_factor.value,
                    'bandwidth': self.lora_config.bandwidth.value,
                    'coding_rate': self.lora_config.coding_rate,
                    'preamble_length': self.lora_config.preamble_length,
                    'sync_word': self.lora_config.sync_word,
                    'crc_enabled': self.lora_config.crc_enabled,
                }
            })

            await self.radio.start(lambda packet: self._handle_rx_packet(packet))
            self._running = True
            self._rx_task = asyncio.create_task(self._rx_loop())
            self.state = LoRaBridgeState.LISTENING
            logger.info("LoRa bridge started and listening")
            return True
        except Exception as e:
            logger.error(f"LoRa bridge failed to start: {e}")
            self.state = LoRaBridgeState.ERROR
            return False

    def set_received_callback(self, callback: Callable[[LoRaFrame], None]):
        """Set received message callback"""
        self._received_callback = callback

    def set_sent_callback(self, callback: Callable[[LoRaFrame], None]):
        """Set sent message callback"""
        self._sent_callback = callback

    async def stop(self):
        """Stop the LoRa bridge"""
        self._running = False
        if self._rx_task:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
        if self.radio:
            await self.radio.stop()
        self.state = LoRaBridgeState.DISCONNECTED
        logger.info("LoRa bridge stopped")

    async def _rx_loop(self):
        """Background RX loop"""
        while self._running:
            try:
                await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"LoRa bridge RX loop error: {e}")
                await asyncio.sleep(0.1)

    async def _handle_rx_packet(self, packet: LoRaPacket):
        """Handle received LoRa packet"""
        try:
            frame = self.protocol.decode_frame(packet.data)
            if not frame:
                logger.debug(f"Failed to decode LoRa packet: {packet.data}")
                return

            self.packets_received += 1
            self.bytes_received += len(packet.data)

            logger.debug(f"LoRa RX: type={frame.msg_type.name}, seq={frame.sequence}, "
                        f"rssi={packet.rssi:.1f}dBm, snr={packet.snr:.1f}dB")

            if self._received_callback:
                try:
                    result = self._received_callback(frame)
                    if asyncio.iscoroutine(result):
                        await result
                except Exception as e:
                    logger.error(f"LoRa received callback error: {e}")

        except Exception as e:
            logger.error(f"LoRa packet handling error: {e}")

    async def send_command(self, msg_type: LoRaMessageType, payload: bytes = b'',
                            priority: LoRaPriority = LoRaPriority.NORMAL) -> bool:
        """Send a LoRa command"""
        if not self._running or not self.radio:
            return False

        frame = self.protocol.encode_command(msg_type, payload, priority)
        sent = await self.radio.send(frame.to_bytes())

        if sent:
            self.packets_sent += 1
            self.bytes_sent += len(frame.to_bytes())
            logger.info(f"LoRa command sent: {msg_type.name}")

            if self._sent_callback:
                try:
                    result = self._sent_callback(frame)
                    if asyncio.iscoroutine(result):
                        await result
                except Exception as e:
                    logger.error(f"LoRa sent callback error: {e}")

        return sent

    async def send_takeoff(self, altitude: float = 10.0) -> bool:
        """Send takeoff command"""
        return await self.send_command(LoRaMessageType.CMD_TAKEOFF, 
                                     struct.pack('<f', altitude),
                                     LoRaPriority.HIGH)

    async def send_land(self) -> bool:
        """Send land command"""
        return await self.send_command(LoRaMessageType.CMD_LAND, b'', LoRaPriority.CRITICAL)

    async def send_rth(self) -> bool:
        """Send return-to-home command"""
        return await self.send_command(LoRaMessageType.CMD_RTH, b'', LoRaPriority.CRITICAL)

    async def send_emergency_stop(self) -> bool:
        """Send emergency stop command"""
        return await self.send_command(LoRaMessageType.CMD_EMERGENCY_STOP, b'', LoRaPriority.CRITICAL)

    async def send_position(self, lat: float, lon: float, alt: float,
                            heading: float = 0.0, speed: float = 5.0) -> bool:
        """Send position command"""
        cmd = struct.pack('<ddddf', lat, lon, alt, heading, speed)
        return await self.send_command(LoRaMessageType.CMD_SET_POSITION, cmd, LoRaPriority.HIGH)

    async def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0) -> bool:
        """Send velocity command"""
        cmd = struct.pack('<ffff', vx, vy, vz, yaw_rate)
        return await self.send_command(LoRaMessageType.CMD_SET_VELOCITY, cmd, LoRaPriority.HIGH)

    async def get_stats(self) -> Dict[str, Any]:
        """Get LoRa bridge statistics"""
        return {
            'state': self.state.value,
            'packets_received': self.packets_received,
            'packets_sent': self.packets_sent,
            'bytes_sent': self.bytes_sent,
            'bytes_received': self.bytes_received,
            'frequency': self.lora_config.frequency,
            'spreading_factor': self.lora_config.spreading_factor.value,
            'tx_power': self.lora_config.tx_power,
        }

    async def check_connection(self) -> bool:
        """Check LoRa connection status"""
        return self.state == LoRaBridgeState.CONNECTED and self._running


async def create_lora_bridge(config: dict, mavlink_send_callback) -> LoRaBridge:
    """Create and start LoRa bridge"""
    # Parse security settings from config if present
    security_config = None
    if 'security' in config:
        sec = config['security']
        if 'level' in sec:
            level_map = {
                'none': LoRaSecurityLevel.NONE,
                'auth_only': LoRaSecurityLevel.AUTH_ONLY,
                'encrypted': LoRaSecurityLevel.ENCRYPTED,
                'full': LoRaSecurityLevel.FULL
            }
            sec_level = level_map.get(sec['level'], LoRaSecurityLevel.NONE)
            security_config = LoRaSecurityConfig(
                level=sec_level,
                pre_shared_key=bytes.fromhex(sec.get('pre_shared_key', '')),
                encryption_key=bytes.fromhex(sec.get('encryption_key', '')),
                device_id=sec.get('device_id', 0),
                allowed_device_ids=tuple(sec.get('allowed_device_ids', []))
            )

    bridge = LoRaBridge(config)

    def handle_command(frame: LoRaFrame):
        """Handle LoRa command frame"""
        if frame.msg_type in (LoRaMessageType.CMD_TAKEOFF, LoRaMessageType.CMD_LAND,
                               LoRaMessageType.CMD_RTH, LoRaMessageType.CMD_EMERGENCY_STOP,
                               LoRaMessageType.CMD_SET_POSITION, LoRaMessageType.CMD_SET_VELOCITY,
                               LoRaMessageType.CMD_SET_MODE, LoRaMessageType.CMD_SET_ATTITUDE):
            payload = frame.payload
            if frame.msg_type == LoRaMessageType.CMD_TAKEOFF:
                altitude = struct.unpack('<f', payload[:4])[0]
                return mavlink_send_callback(f"command:{frame.msg_type.value}:{altitude}")
            elif frame.msg_type == LoRaMessageType.CMD_SET_POSITION:
                lat, lon, alt, heading, speed = struct.unpack('<ddddf', payload[:32])
                return mavlink_send_callback(f"position:{lat}:{lon}:{alt}:{heading}:{speed}")
            elif frame.msg_type == LoRaMessageType.CMD_SET_VELOCITY:
                vx, vy, vz, yaw_rate = struct.unpack('<ffff', payload[:16])
                return mavlink_send_callback(f"velocity:{vx}:{vy}:{vz}:{yaw_rate}")
            elif frame.msg_type == LoRaMessageType.CMD_LAND:
                return mavlink_send_callback("command:2")
            elif frame.msg_type == LoRaMessageType.CMD_RTH:
                return mavlink_send_callback("command:3")
            elif frame.msg_type == LoRaMessageType.CMD_EMERGENCY_STOP:
                return mavlink_send_callback("command:8")

        elif frame.msg_type in (LoRaMessageType.ACK, LoRaMessageType.NAK):
            logger.info(f"LoRa ACK/NAK: {frame}")

        return True

    async def received_handler(frame: LoRaFrame):
        return await asyncio.to_thread(handle_command, frame)

    started = await bridge.start(received_handler, mavlink_send_callback)
    if started:
        logger.info("LoRa bridge started successfully")

    return bridge


class LoRaControlServer:
    """Server that receives LoRa commands and forwards to drone"""

    def __init__(self, port: int = 8080):
        self.port = port
        self.bridge: Optional[LoRaBridge] = None
        self.running = False
        self._task = None

    async def start(self, bridge: LoRaBridge) -> bool:
        """Start LoRa control server"""
        self.bridge = bridge
        self.running = True
        self._task = asyncio.create_task(self._server_loop())
        return True

    async def stop(self):
        """Stop LoRa control server"""
        self.running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    async def _server_loop(self):
        """Main server loop"""
        while self.running:
            try:
                if await self.bridge.check_connection():
                    stats = await self.bridge.get_stats()
                    logger.info(f"LoRa control server: {stats}")
                await asyncio.sleep(1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"LoRa control server error: {e}")
                await asyncio.sleep(1)

    def get_bridge(self) -> Optional[LoRaBridge]:
        """Get connected bridge"""
        return self.bridge