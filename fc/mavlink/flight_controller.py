#!/usr/bin/env python3
"""
FC (Flight Controller) Side - Arduino/STM32/Pixhawk Companion
This runs on the flight controller or a companion microcontroller
Handles: Low-level motor control, sensor fusion, failsafe execution
"""

import asyncio
import logging
import struct
import time
from typing import Dict, Optional, Callable
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)

class FlightMode(Enum):
    """ArduPilot-compatible flight modes"""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    POSHOLD = 11
    LAND = 9
    THROW = 12
    AVOID_ADSB = 13
    GUIDED_NOGPS = 14
    SPORT = 15
    FLIP = 16
    AUTOTUNE = 17
    POSHOLD_BRAKE = 18


@dataclass
class FCState:
    """Flight controller state"""
    armed: bool = False
    mode: FlightMode = FlightMode.STABILIZE
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    relative_alt: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: int = 100
    rssi: int = 0
    snr: int = 0
    gps_fix: int = 0
    gps_satellites: int = 0
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0
    failsafe_active: bool = False
    failsafe_reason: str = ""
    last_heartbeat: float = 0
    last_telemetry: float = 0
    last_control: float = 0


class FlightController:
    """
    Flight Controller Interface
    Communicates with SBC via MAVLink serial
    """
    
    def __init__(self, config: dict):
        self.config = config
        self.fc_config = config.get('fc', {})
        self.state = FCState()
        self.running = False
        
        # MAVLink connection (to SBC)
        self.serial_device = self.fc_config.get('serial_device', '/dev/ttyS0')
        self.baudrate = self.fc_config.get('baudrate', 921600)
        
        # Failsafe configuration
        self.failsafe_config = config.get('failsafe', {})
        
        # Callbacks
        self.on_command: Optional[Callable] = None
        self.on_telemetry_request: Optional[Callable] = None
        
        # Internal
        self._serial = None
        self._read_task = None
        self._heartbeat_task = None
        self._failsafe_task = None
    
    async def start(self):
        """Start flight controller interface"""
        logger.info("Starting Flight Controller interface...")
        
        # Open serial connection to SBC
        import serial
        try:
            self._serial = serial.Serial(
                port=self.serial_device,
                baudrate=self.baudrate,
                timeout=0.1
            )
            logger.info(f"FC serial opened: {self.serial_device} @ {self.baudrate}")
        except Exception as e:
            logger.error(f"Failed to open serial: {e}")
            raise
        
        self.running = True
        
        # Start tasks
        self._read_task = asyncio.create_task(self._read_loop())
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
        self._failsafe_task = asyncio.create_task(self._failsafe_monitor())
        
        logger.info("Flight Controller interface started")
    
    async def stop(self):
        """Stop flight controller interface"""
        self.running = False
        
        for task in [self._read_task, self._heartbeat_task, self._failsafe_task]:
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        
        if self._serial and self._serial.is_open:
            self._serial.close()
        
        logger.info("Flight Controller interface stopped")
    
    async def _read_loop(self):
        """Read MAVLink messages from SBC"""
        buffer = bytearray()
        
        while self.running:
            try:
                if self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting)
                    buffer.extend(data)
                    
                    # Parse MAVLink messages (simplified)
                    messages = self._parse_mavlink(buffer)
                    for msg in messages:
                        await self._handle_message(msg)
                        buffer = buffer[msg['consumed']:]
                
                await asyncio.sleep(0.001)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"FC read error: {e}")
                await asyncio.sleep(0.1)
    
    def _parse_mavlink(self, buffer: bytearray) -> list:
        """Parse MAVLink messages from buffer"""
        messages = []
        MAVLINK_STX = 0xFD
        
        while len(buffer) >= 10:
            # Find STX
            stx_idx = buffer.find(MAVLINK_STX)
            if stx_idx == -1:
                buffer.clear()
                break
            
            if stx_idx > 0:
                buffer = buffer[stx_idx:]
            
            if len(buffer) < 10:
                break
            
            payload_len = buffer[1]
            if payload_len > 255:
                buffer = buffer[1:]
                continue
            
            total_len = 10 + payload_len + 2
            if len(buffer) < total_len:
                break
            
            msg_data = buffer[:total_len]
            msg = self._decode_mavlink(msg_data)
            if msg:
                msg['consumed'] = total_len
                messages.append(msg)
            else:
                buffer = buffer[1:]
        
        return messages
    
    def _decode_mavlink(self, data: bytes) -> Optional[dict]:
        """Decode MAVLink v2 packet"""
        if len(data) < 10 or data[0] != 0xFD:
            return None
        
        payload_len = data[1]
        seq = data[2]
        sysid = data[3]
        compid = data[4]
        msgid = (data[7] << 16) | (data[6] << 8) | data[5]
        payload = data[10:10+payload_len]
        
        return {
            'msgid': msgid,
            'seq': seq,
            'sysid': sysid,
            'compid': compid,
            'payload': payload
        }
    
    async def _handle_message(self, msg: dict):
        """Handle incoming MAVLink message from SBC"""
        msgid = msg['msgid']
        payload = msg['payload']
        
        self.state.last_control = time.time()
        
        # COMMAND_LONG (76)
        if msgid == 76:
            await self._handle_command_long(payload)
        
        # SET_MODE (11)
        elif msgid == 11:
            await self._handle_set_mode(payload)
        
        # MANUAL_CONTROL (69)
        elif msgid == 69:
            await self._handle_manual_control(payload)
        
        # RC_CHANNELS_OVERRIDE (70)
        elif msgid == 70:
            await self._handle_rc_override(payload)
        
        # HEARTBEAT (0) - from SBC
        elif msgid == 0:
            self.state.last_heartbeat = time.time()
        
        # Request telemetry stream
        if self.on_telemetry_request:
            await self.on_telemetry_request()
    
    async def _handle_command_long(self, payload: bytes):
        """Handle COMMAND_LONG message"""
        if len(payload) < 32:
            return
        
        import struct
        values = struct.unpack('<BBBBBfffffff', payload[:32])
        target_system, target_component, command, confirmation = values[:4]
        params = values[4:]
        
        logger.info(f"FC received command: {command} with params: {params}")
        
        result = 0  # MAV_RESULT_ACCEPTED
        
        if command == 400:  # MAV_CMD_COMPONENT_ARM_DISARM
            arm = params[0] > 0.5
            force = params[1] == 21196.0
            if arm:
                await self.arm(force)
            else:
                await self.disarm()
        
        elif command == 20:  # MAV_CMD_NAV_RETURN_TO_LAUNCH
            await self.trigger_rtl()
        
        elif command == 21:  # MAV_CMD_NAV_LAND
            await self.land()
        
        elif command == 22:  # MAV_CMD_NAV_TAKEOFF
            altitude = params[6]
            await self.takeoff(altitude)
        
        elif command == 176:  # MAV_CMD_DO_SET_MODE
            mode = int(params[0])
            custom_mode = int(params[1])
            await self.set_mode(mode, custom_mode)
        
        elif command == 511:  # MAV_CMD_SET_MESSAGE_INTERVAL
            msg_id = int(params[0])
            interval = params[1]
            logger.info(f"Set message interval: {msg_id} = {interval}us")
        
        # Send COMMAND_ACK
        await self._send_command_ack(command, result)
    
    async def _handle_set_mode(self, payload: bytes):
        """Handle SET_MODE message"""
        if len(payload) < 4:
            return
        
        import struct
        target_system, base_mode, custom_mode = struct.unpack('<BBH', payload[:4])
        
        # Extract mode from base_mode and custom_mode
        # This is simplified - real implementation depends on autopilot
        mode = custom_mode & 0xFF
        await self.set_mode(mode, custom_mode)
    
    async def _handle_manual_control(self, payload: bytes):
        """Handle MANUAL_CONTROL message (joystick input)"""
        if len(payload) < 10:
            return
        
        import struct
        x, y, z, r, buttons = struct.unpack('<hhhhH', payload[:10])
        
        # Convert to motor commands
        # x: roll (-1000 to 1000)
        # y: pitch (-1000 to 1000)
        # z: throttle (0 to 1000)
        # r: yaw (-1000 to 1000)
        
        logger.debug(f"Manual control: roll={x}, pitch={y}, throttle={z}, yaw={r}, buttons={buttons}")
        
        # In a real FC, this would go to the mixer
        # For now, just log
        if self.on_command:
            await self.on_command('manual', {'roll': x, 'pitch': y, 'throttle': z, 'yaw': r})
    
    async def _handle_rc_override(self, payload: bytes):
        """Handle RC_CHANNELS_OVERRIDE message"""
        if len(payload) < 3:
            return
        
        import struct
        target_system, target_component, n_channels = struct.unpack('<BBB', payload[:3])
        channels = list(struct.unpack(f'<{n_channels}H', payload[3:3+n_channels*2]))
        
        logger.debug(f"RC override: {channels}")
        
        if self.on_command:
            await self.on_command('rc_override', {'channels': channels})
    
    async def _send_command_ack(self, command: int, result: int):
        """Send COMMAND_ACK back to SBC"""
        import struct
        payload = struct.pack('<HBBi', command, result, 0, 0)
        await self._send_mavlink(77, payload)  # COMMAND_ACK = 77
    
    async def _send_mavlink(self, msgid: int, payload: bytes):
        """Send MAVLink message to SBC"""
        if not self._serial or not self._serial.is_open:
            return
        
        # Build MAVLink v2 packet
        seq = int(time.time() * 1000) % 256
        sysid = 1  # FC system ID
        compid = 1  # Autopilot component ID
        
        header = bytearray([0xFD, len(payload), seq, sysid, compid])
        header.extend([
            msgid & 0xFF,
            (msgid >> 8) & 0xFF,
            (msgid >> 16) & 0xFF
        ])
        header.extend(payload)
        
        # CRC (simplified)
        header.extend([0xFF, 0xFF])
        
        try:
            self._serial.write(header)
        except Exception as e:
            logger.error(f"FC serial write error: {e}")
    
    async def _heartbeat_loop(self):
        """Send periodic heartbeat to SBC"""
        while self.running:
            try:
                await self._send_heartbeat()
                await asyncio.sleep(0.1)  # 10 Hz
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"FC heartbeat error: {e}")
                await asyncio.sleep(1)
    
    async def _send_heartbeat(self):
        """Send HEARTBEAT message"""
        import struct
        
        # Base mode
        base_mode = 0x80 if self.state.armed else 0x00  # MAV_MODE_FLAG_SAFETY_ARMED
        base_mode |= 0x01  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        
        custom_mode = self.state.mode.value
        system_status = 4  # MAV_STATE_ACTIVE
        if self.state.failsafe_active:
            system_status = 6  # MAV_STATE_CRITICAL
        
        payload = struct.pack('<IBBBBB', custom_mode, 2, 3, base_mode, system_status, 3)
        await self._send_mavlink(0, payload)  # HEARTBEAT = 0
    
    async def _send_telemetry(self):
        """Send telemetry data to SBC"""
        # This would send various MAVLink telemetry messages
        # GLOBAL_POSITION_INT (33), ATTITUDE (30), BATTERY_STATUS (147), etc.
        pass
    
    async def _failsafe_monitor(self):
        """Monitor for failsafe conditions"""
        while self.running:
            try:
                await self._check_failsafes()
                await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"FC failsafe error: {e}")
                await asyncio.sleep(1)
    
    async def _check_failsafes(self):
        """Check failsafe conditions"""
        now = time.time()
        failsafe = self.failsafe_config
        
        # Signal loss failsafe
        signal_config = failsafe.get('signal', {})
        telemetry_timeout = signal_config.get('telemetry_timeout', 3.0)
        control_timeout = signal_config.get('control_timeout', 2.0)
        
        if now - self.state.last_telemetry > telemetry_timeout:
            await self._trigger_failsafe("signal_loss", "Telemetry timeout from SBC")
        
        if now - self.state.last_control > control_timeout:
            await self._trigger_failsafe("control_loss", "Control timeout from SBC")
        
        # Battery failsafe
        battery_config = failsafe.get('battery', {})
        critical_voltage = battery_config.get('critical_voltage', 14.8)
        rth_voltage = battery_config.get('rth_voltage', 15.2)
        
        if self.state.battery_voltage > 0:
            if self.state.battery_voltage <= critical_voltage:
                await self._trigger_failsafe("critical_battery", f"Battery critical: {self.state.battery_voltage:.1f}V")
            elif self.state.battery_voltage <= rth_voltage and not self.state.failsafe_active:
                await self._trigger_failsafe("low_battery", f"Low battery: {self.state.battery_voltage:.1f}V", rth=True)
        
        # Geofence failsafe
        geofence_config = failsafe.get('geofence', {})
        if geofence_config.get('enabled', True):
            max_radius = geofence_config.get('max_radius', 5000.0)
            max_alt = geofence_config.get('max_altitude', 120.0)
            
            # Check distance from home
            if self.state.home_lat != 0 and self.state.home_lon != 0:
                import math
                dlat = self.state.lat - self.state.home_lat
                dlon = self.state.lon - self.state.home_lon
                distance = math.sqrt(dlat*dlat + dlon*dlon) * 111319  # rough meters
                
                if distance > max_radius:
                    await self._trigger_failsafe("geofence", f"Exceeded max radius: {distance:.0f}m")
            
            if self.state.relative_alt > max_alt:
                await self._trigger_failsafe("geofence", f"Exceeded max altitude: {self.state.relative_alt:.1f}m")
    
    async def _trigger_failsafe(self, reason: str, message: str, rth: bool = False):
        """Trigger failsafe action"""
        if self.state.failsafe_active:
            return
        
        logger.warning(f"FC FAILSAFE: {reason} - {message}")
        self.state.failsafe_active = True
        self.state.failsafe_reason = reason
        
        if rth:
            await self.trigger_rtl()
        else:
            await self.land()
    
    # Public control methods
    
    async def arm(self, force: bool = False):
        """Arm the vehicle"""
        if self.state.armed:
            return
        
        logger.info("Arming vehicle...")
        self.state.armed = True
        
        # Send armed status in next heartbeat
    
    async def disarm(self):
        """Disarm the vehicle"""
        if not self.state.armed:
            return
        
        logger.info("Disarming vehicle...")
        self.state.armed = False
        self.state.failsafe_active = False
        self.state.failsafe_reason = ""
    
    async def trigger_rtl(self):
        """Trigger Return to Launch"""
        logger.info("RTL triggered")
        self.state.mode = FlightMode.RTL
    
    async def land(self):
        """Trigger Land"""
        logger.info("Land triggered")
        self.state.mode = FlightMode.LAND
    
    async def takeoff(self, altitude: float):
        """Trigger Takeoff"""
        logger.info(f"Takeoff to {altitude}m")
        self.state.mode = FlightMode.GUIDED
        # Would send takeoff command to navigation
    
    async def set_mode(self, mode: int, custom_mode: int = 0):
        """Set flight mode"""
        try:
            self.state.mode = FlightMode(mode)
            logger.info(f"Mode set to {self.state.mode.name}")
        except ValueError:
            logger.warning(f"Unknown mode: {mode}")
    
    def update_state(self, **kwargs):
        """Update FC state from sensors"""
        for key, value in kwargs.items():
            if hasattr(self.state, key):
                setattr(self.state, key, value)
        
        self.state.last_telemetry = time.time()
    
    def get_status(self) -> dict:
        """Get FC status"""
        return {
            'armed': self.state.armed,
            'mode': self.state.mode.name,
            'mode_value': self.state.mode.value,
            'position': {
                'lat': self.state.lat,
                'lon': self.state.lon,
                'alt': self.state.alt,
                'relative_alt': self.state.relative_alt
            },
            'attitude': {
                'roll': self.state.roll,
                'pitch': self.state.pitch,
                'yaw': self.state.yaw
            },
            'velocity': {
                'vx': self.state.vx,
                'vy': self.state.vy,
                'vz': self.state.vz
            },
            'battery': {
                'voltage': self.state.battery_voltage,
                'current': self.state.battery_current,
                'remaining': self.state.battery_remaining
            },
            'gps': {
                'fix': self.state.gps_fix,
                'satellites': self.state.gps_satellites
            },
            'home': {
                'lat': self.state.home_lat,
                'lon': self.state.home_lon,
                'alt': self.state.home_alt
            },
            'failsafe': {
                'active': self.state.failsafe_active,
                'reason': self.state.failsafe_reason
            },
            'signal': {
                'rssi': self.state.rssi,
                'snr': self.state.snr
            },
            'timestamps': {
                'heartbeat': self.state.last_heartbeat,
                'telemetry': self.state.last_telemetry,
                'control': self.state.last_control
            }
        }


# Example usage for testing
async def main():
    logging.basicConfig(level=logging.INFO)
    
    config = {
        'fc': {
            'serial_device': '/dev/ttyUSB0',
            'baudrate': 921600
        },
        'failsafe': {
            'signal': {
                'telemetry_timeout': 3.0,
                'control_timeout': 2.0
            },
            'battery': {
                'critical_voltage': 14.8,
                'rth_voltage': 15.2
            },
            'geofence': {
                'enabled': True,
                'max_radius': 5000.0,
                'max_altitude': 120.0
            }
        }
    }
    
    fc = FlightController(config)
    
    try:
        await fc.start()
        
        # Simulate sensor updates
        while True:
            fc.update_state(
                lat=37.7749 + (time.time() % 100) * 0.0001,
                lon=-122.4194 + (time.time() % 100) * 0.0001,
                alt=100 + (time.time() % 10),
                relative_alt=50 + (time.time() % 10),
                roll=0.1 * (time.time() % 2 - 1),
                pitch=0.05 * (time.time() % 2 - 1),
                yaw=(time.time() * 10) % 360,
                battery_voltage=16.8 - (time.time() % 100) * 0.01,
                battery_current=10 + (time.time() % 5),
                battery_remaining=80 - int(time.time() % 80),
                gps_fix=3,
                gps_satellites=12,
                rssi=-50,
                snr=20
            )
            
            if fc.state.home_lat == 0:
                fc.update_state(home_lat=fc.state.lat, home_lon=fc.state.lon, home_alt=fc.state.alt)
            
            await asyncio.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        await fc.stop()


if __name__ == "__main__":
    asyncio.run(main())