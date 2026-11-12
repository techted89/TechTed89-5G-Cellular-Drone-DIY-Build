#!/usr/bin/env python3
"""
GPS Driver for Flight Controller
Supports u-blox, MTK, and NMEA GPS receivers via UART/I2C
"""

import asyncio
import logging
import math
import time
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class GPSFix(Enum):
    NO_FIX = 0
    DEAD_RECKONING = 1
    FIX_2D = 2
    FIX_3D = 3
    GPS_DEAD_RECKONING = 4
    TIME_ONLY = 5


@dataclass
class GPSData:
    fix: GPSFix = GPSFix.NO_FIX
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    alt_msl: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    hdop: float = 99.9
    vdop: float = 99.9
    satellites: int = 0
    timestamp: float = 0
    eph: float = 0  # Horizontal position error
    epv: float = 0  # Vertical position error


class GPS:
    """GPS receiver interface"""

    def __init__(self, config: dict):
        self.config = config.get('gps', {})
        self.device = self.config.get('device', '/dev/ttyGPS')
        self.baudrate = self.config.get('baudrate', 115200)
        self.protocol = self.config.get('protocol', 'ublox')  # ublox, mtk, nmea
        self.update_rate = self.config.get('update_rate', 10)  # Hz

        self.serial = None
        self.running = False
        self._read_task = None
        self.data = GPSData()

        # u-blox UBX message parsing
        self._ubx_buffer = bytearray()
        self._ubx_state = 0  # 0=sync1, 1=sync2, 2=class, 3=id, 4=length1, 5=length2, 6=payload, 7=checksum_a, 8=checksum_b

    async def start(self):
        """Start GPS receiver"""
        import serial
        try:
            self.serial = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0.1
            )
            logger.info(f"GPS opened: {self.device} @ {self.baudrate}")

            # Configure GPS for desired update rate
            if self.protocol == 'ublox':
                await self._configure_ublox()

            self.running = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info("GPS started")
        except Exception as e:
            logger.error(f"Failed to start GPS: {e}")
            raise

    async def _configure_ublox(self):
        """Configure u-blox GPS for desired settings"""
        if not self.serial:
            return

        # Set update rate (10 Hz = 100ms interval)
        interval_ms = 1000 // self.update_rate
        # CFG-RATE message (class 0x06, id 0x08)
        msg = bytes([0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
                     interval_ms & 0xFF, (interval_ms >> 8) & 0xFF,
                     0x01, 0x00, 0x01, 0x00, 0x00, 0x00])
        # Add checksum
        ck_a = 0
        ck_b = 0
        for byte in msg[2:-2]:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        msg += bytes([ck_a, ck_b])

        self.serial.write(msg)
        await asyncio.sleep(0.1)

    async def stop(self):
        """Stop GPS receiver"""
        self.running = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self.serial and self.serial.is_open:
            self.serial.close()
        logger.info("GPS stopped")

    async def _read_loop(self):
        """Read GPS data continuously"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    if self.protocol == 'ublox':
                        self._parse_ubx(data)
                    elif self.protocol == 'nmea':
                        self._parse_nmea(data)
                await asyncio.sleep(0.01)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"GPS read error: {e}")
                await asyncio.sleep(0.1)

    def _parse_ubx(self, data: bytes):
        """Parse u-blox UBX binary protocol"""
        for byte in data:
            if self._ubx_state == 0:
                if byte == 0xB5:
                    self._ubx_state = 1
                    self._ubx_buffer = bytearray([byte])
            elif self._ubx_state == 1:
                if byte == 0x62:
                    self._ubx_state = 2
                    self._ubx_buffer.append(byte)
                else:
                    self._ubx_state = 0
            elif self._ubx_state == 2:
                self._ubx_buffer.append(byte)
                self._ubx_state = 3
            elif self._ubx_state == 3:
                self._ubx_buffer.append(byte)
                self._ubx_state = 4
            elif self._ubx_state == 4:
                self._ubx_buffer.append(byte)
                self._payload_len = byte | (self._ubx_buffer[4] << 8)
                if self._payload_len == 0:
                    self._ubx_state = 7  # Skip to checksum
                else:
                    self._ubx_state = 5
                    self._payload_count = 0
            elif self._ubx_state == 5:
                self._ubx_buffer.append(byte)
                self._payload_count += 1
                if self._payload_count >= self._payload_len:
                    self._ubx_state = 7
            elif self._ubx_state == 7:
                self._ubx_buffer.append(byte)  # CK_A
                self._ubx_state = 8
            elif self._ubx_state == 8:
                self._ubx_buffer.append(byte)  # CK_B
                self._process_ubx_message()
                self._ubx_state = 0

    def _process_ubx_message(self):
        """Process complete UBX message"""
        if len(self._ubx_buffer) < 8:
            return

        msg_class = self._ubx_buffer[2]
        msg_id = self._ubx_buffer[3]
        payload = self._ubx_buffer[6:-2]

        # NAV-PVT (0x01, 0x07) - Position, Velocity, Time
        if msg_class == 0x01 and msg_id == 0x07:
            self._parse_nav_pvt(payload)

        # NAV-SAT (0x01, 0x35) - Satellite info
        elif msg_class == 0x01 and msg_id == 0x35:
            self._parse_nav_sat(payload)

    def _parse_nav_pvt(self, payload: bytes):
        """Parse NAV-PVT message"""
        import struct
        if len(payload) < 92:
            return

        # Parse NAV-PVT (u-blox 8/9)
        vals = struct.unpack('<IiiiiiiiIHHHHHHHhhhhHH', payload[:80])
        itow = vals[0]          # GPS time of week (ms)
        year = vals[1]
        month = vals[2]
        day = vals[3]
        hour = vals[4]
        minute = vals[5]
        second = vals[6]
        valid = vals[7]         # Validity flags
        tacc = vals[8]          # Time accuracy estimate
        nano = vals[9]          # Fraction of second
        fix_type = vals[10]     # Fix type
        flags = vals[11]        # Fix status flags
        flags2 = vals[12]       # Additional flags
        num_sv = vals[13]       # Number of satellites
        lon = vals[14]          # Longitude (1e-7 deg)
        lat = vals[15]          # Latitude (1e-7 deg)
        height = vals[16]       # Height above ellipsoid (mm)
        h_msl = vals[17]        # Height above MSL (mm)
        h_acc = vals[18]        # Horizontal accuracy (mm)
        v_acc = vals[19]        # Vertical accuracy (mm)
        vel_n = vals[20]        # NED north velocity (mm/s)
        vel_e = vals[21]        # NED east velocity (mm/s)
        vel_d = vals[22]        # NED down velocity (mm/s)
        g_speed = vals[23]      # Ground speed (mm/s)
        heading = vals[24]      # Heading (1e-5 deg)
        s_acc = vals[25]        # Speed accuracy (mm/s)
        h_acc_2 = vals[26]      # Heading accuracy (1e-5 deg)
        p_dop = vals[27]        # Position DOP (0.01)
        # flags3 = vals[28]      # Additional flags (if present)

        self.data.fix = GPSFix(fix_type)
        self.data.lat = lat / 1e7
        self.data.lon = lon / 1e7
        self.data.alt = height / 1000.0
        self.data.alt_msl = h_msl / 1000.0
        self.data.speed = g_speed / 1000.0
        self.data.heading = heading / 1e5
        self.data.hdop = p_dop / 100.0
        self.data.satellites = num_sv
        self.data.eph = h_acc / 1000.0
        self.data.epv = v_acc / 1000.0
        self.data.timestamp = time.time()

    def _parse_nav_sat(self, payload: bytes):
        """Parse NAV-SAT message for satellite info"""
        # Satellite count already in NAV-PVT
        pass

    def _parse_nmea(self, data: bytes):
        """Parse NMEA sentences"""
        lines = data.decode('ascii', errors='ignore').split('\n')
        for line in lines:
            line = line.strip()
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                self._parse_gga(line)
            elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                self._parse_rmc(line)

    def _parse_gga(self, line: str):
        """Parse GGA sentence"""
        parts = line.split(',')
        if len(parts) < 15:
            return
        try:
            fix_quality = int(parts[6])
            if fix_quality == 0:
                self.data.fix = GPSFix.NO_FIX
            elif fix_quality == 1:
                self.data.fix = GPSFix.FIX_3D
            elif fix_quality == 2:
                self.data.fix = GPSFix.FIX_3D

            if parts[2] and parts[4]:
                lat_deg = float(parts[2][:2])
                lat_min = float(parts[2][2:])
                lat = lat_deg + lat_min / 60.0
                if parts[3] == 'S':
                    lat = -lat

                lon_deg = float(parts[4][:3])
                lon_min = float(parts[4][3:])
                lon = lon_deg + lon_min / 60.0
                if parts[5] == 'W':
                    lon = -lon

                self.data.lat = lat
                self.data.lon = lon

            if parts[9]:
                self.data.alt_msl = float(parts[9])
            if parts[7]:
                self.data.satellites = int(parts[7])
            if parts[8]:
                self.data.hdop = float(parts[8])
            self.data.timestamp = time.time()
        except (ValueError, IndexError) as e:
            logger.debug(f"GGA parse error: {e}")

    def _parse_rmc(self, line: str):
        """Parse RMC sentence"""
        parts = line.split(',')
        if len(parts) < 12:
            return
        try:
            if parts[2] == 'A':  # Valid
                if parts[3] and parts[5]:
                    lat_deg = float(parts[3][:2])
                    lat_min = float(parts[3][2:])
                    lat = lat_deg + lat_min / 60.0
                    if parts[4] == 'S':
                        lat = -lat

                    lon_deg = float(parts[5][:3])
                    lon_min = float(parts[5][3:])
                    lon = lon_deg + lon_min / 60.0
                    if parts[6] == 'W':
                        lon = -lon

                    self.data.lat = lat
                    self.data.lon = lon

                if parts[7]:
                    self.data.speed = float(parts[7]) * 0.514444  # knots to m/s
                if parts[8]:
                    self.data.heading = float(parts[8])
                self.data.timestamp = time.time()
        except (ValueError, IndexError) as e:
            logger.debug(f"RMC parse error: {e}")

    def get_data(self) -> GPSData:
        """Get latest GPS data"""
        return self.data

    def get_status(self) -> dict:
        """Get GPS status"""
        return {
            'fix': self.data.fix.name,
            'lat': self.data.lat,
            'lon': self.data.lon,
            'alt': self.data.alt,
            'alt_msl': self.data.alt_msl,
            'speed': self.data.speed,
            'heading': self.data.heading,
            'hdop': self.data.hdop,
            'satellites': self.data.satellites,
            'eph': self.data.eph,
            'epv': self.data.epv,
            'timestamp': self.data.timestamp,
            'age': time.time() - self.data.timestamp if self.data.timestamp > 0 else None
        }


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'gps': {
            'device': '/dev/ttyUSB0',
            'baudrate': 115200,
            'protocol': 'ublox',
            'update_rate': 10
        }
    }
    gps = GPS(config)
    try:
        await gps.start()
        while True:
            status = gps.get_status()
            print(f"GPS: fix={status['fix']} lat={status['lat']:.6f} lon={status['lon']:.6f} "
                  f"alt={status['alt']:.1f} sats={status['satellites']} hdop={status['hdop']:.1f}")
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        await gps.stop()


if __name__ == '__main__':
    asyncio.run(main())