#!/usr/bin/env python3
"""
Rangefinder Driver for Flight Controller
Supports TF-Luna, TFmini, Benewake, LightWare, Garmin Lidar, and ultrasonic sensors
"""

import asyncio
import logging
import time
from typing import Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


@dataclass
class RangeReading:
    """Rangefinder reading"""
    timestamp: float
    distance: float  # meters
    signal_strength: int  # 0-255 or 0-65535
    temperature: float = 0.0  # C
    valid: bool = True


class RangefinderType(Enum):
    TF_LUNA = "tf_luna"
    TFMINI = "tfmini"
    TFMINI_S = "tfmini_s"
    TFMINI_PLUS = "tfmini_plus"
    LIGHTWARE_SF11 = "sf11"
    LIGHTWARE_SF20 = "sf20"
    LIGHTWARE_SF30 = "sf30"
    GARMIN_LIDAR_LITE = "lidar_lite"
    GARMIN_LIDAR_LITE_V3 = "lidar_lite_v3"
    TERARANGER_EVO = "teraranger_evo"
    ULTRASONIC_HCSR04 = "hcsr04"
    ULTRASONIC_MB1240 = "mb1240"
    VL53L0X = "vl53l0x"
    VL53L1X = "vl53l1x"
    MOCK = "mock"


class Rangefinder:
    """Rangefinder interface for altitude/obstacle detection"""

    def __init__(self, config: dict):
        self.config = config.get('rangefinder', {})
        self.rangefinder_type = RangefinderType(self.config.get('type', 'tf_luna'))
        self.bus = self.config.get('bus', 1)
        self.address = self.config.get('address', 0x10)  # I2C address for TF-Luna
        self.serial_device = self.config.get('serial_device', '/dev/ttyRANGE')
        self.serial_baud = self.config.get('serial_baud', 115200)
        self.pin_trigger = self.config.get('pin_trigger', None)  # GPIO for ultrasonic
        self.pin_echo = self.config.get('pin_echo', None)
        self.orientation = self.config.get('orientation', 'down')  # down, forward, up
        self.min_range = self.config.get('min_range', 0.05)  # meters
        self.max_range = self.config.get('max_range', 50.0)  # meters
        self.sample_rate = self.config.get('sample_rate', 100)  # Hz

        self.bus_obj = None
        self.serial = None
        self.running = False
        self._read_task = None
        self._latest = RangeReading(
            timestamp=0,
            distance=0.0,
            signal_strength=0,
            valid=False
        )

    async def start(self):
        """Start rangefinder"""
        try:
            if self.rangefinder_type in (RangefinderType.TF_LUNA, RangefinderType.TFMINI,
                                          RangefinderType.TFMINI_S, RangefinderType.TFMINI_PLUS):
                await self._start_tf_series()
            elif self.rangefinder_type in (RangefinderType.LIGHTWARE_SF11,
                                            RangefinderType.LIGHTWARE_SF20,
                                            RangefinderType.LIGHTWARE_SF30):
                await self._start_lightware()
            elif self.rangefinder_type in (RangefinderType.GARMIN_LIDAR_LITE,
                                            RangefinderType.GARMIN_LIDAR_LITE_V3):
                await self._start_garmin()
            elif self.rangefinder_type in (RangefinderType.TERARANGER_EVO,):
                await self._start_teraranger()
            elif self.rangefinder_type in (RangefinderType.ULTRASONIC_HCSR04,
                                            RangefinderType.ULTRASONIC_MB1240):
                await self._start_ultrasonic()
            elif self.rangefinder_type in (RangefinderType.VL53L0X, RangefinderType.VL53L1X):
                await self._start_vl53()
            else:
                await self._start_generic()

            self.running = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info(f"Rangefinder started: {self.rangefinder_type.value} @ {self.sample_rate}Hz")
        except Exception as e:
            logger.error(f"Failed to start rangefinder: {e}")
            raise

    async def _start_tf_series(self):
        """Initialize Benewake TF series (TF-Luna, TFmini, etc.)"""
        if self.rangefinder_type in (RangefinderType.TF_LUNA, RangefinderType.VL53L0X,
                                      RangefinderType.VL53L1X):
            # I2C interface
            try:
                import smbus2
                self.bus_obj = smbus2.SMBus(self.bus)
                # TF-Luna: 0x10 default address
                # VL53L0X: 0x29 default address
                logger.info(f"TF Series I2C initialized at 0x{self.address:02X}")
            except ImportError:
                logger.warning("smbus2 not available")
                self.bus_obj = None
        else:
            # Serial interface (TFmini, TFmini-S, TFmini-Plus)
            try:
                import serial
                self.serial = serial.Serial(
                    port=self.serial_device,
                    baudrate=self.serial_baud,
                    timeout=0.1
                )
                logger.info(f"TF Series serial initialized: {self.serial_device} @ {self.serial_baud}")
            except Exception as e:
                logger.error(f"TF Series serial init error: {e}")
                self.serial = None

    async def _start_lightware(self):
        """Initialize LightWare SF series"""
        try:
            import serial
            self.serial = serial.Serial(
                port=self.serial_device,
                baudrate=115200,
                timeout=0.1
            )
            logger.info(f"LightWare serial initialized: {self.serial_device}")
        except Exception as e:
            logger.error(f"LightWare init error: {e}")
            self.serial = None

    async def _start_garmin(self):
        """Initialize Garmin Lidar Lite"""
        if self.rangefinder_type == RangefinderType.GARMIN_LIDAR_LITE:
            # I2C
            try:
                import smbus2
                self.bus_obj = smbus2.SMBus(self.bus)
                # Garmin Lidar Lite v1/v2: 0x62
                self.address = 0x62
                logger.info("Garmin Lidar Lite I2C initialized")
            except ImportError:
                logger.warning("smbus2 not available")
                self.bus_obj = None
        else:
            # v3 uses I2C at 0x62 or PWM
            try:
                import smbus2
                self.bus_obj = smbus2.SMBus(self.bus)
                self.address = 0x62
                logger.info("Garmin Lidar Lite v3 I2C initialized")
            except ImportError:
                logger.warning("smbus2 not available")
                self.bus_obj = None

    async def _start_teraranger(self):
        """Initialize Teraranger Evo"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # Teraranger Evo: 0x01 (can be changed)
            self.address = 0x01
            logger.info("Teraranger Evo I2C initialized")
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None

    async def _start_ultrasonic(self):
        """Initialize ultrasonic (HC-SR04, MB1240)"""
        # For HC-SR04, use GPIO
        # For MB1240, can use serial, I2C, or PWM
        logger.info(f"Ultrasonic initialized: {self.rangefinder_type.value}")

    async def _start_vl53(self):
        """Initialize ST VL53L0X/VL53L1X"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # VL53L0X: 0x29, VL53L1X: 0x29
            self.address = 0x29
            # Basic init
            self.bus_obj.write_byte_data(self.address, 0x80, 0x01)
            self.bus_obj.write_byte_data(self.address, 0xFF, 0x01)
            self.bus_obj.write_byte_data(self.address, 0x00, 0x00)
            logger.info(f"VL53 initialized at 0x{self.address:02X}")
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"VL53 init error: {e}")
            self.bus_obj = None

    async def _start_generic(self):
        pass

    async def stop(self):
        """Stop rangefinder"""
        self.running = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self.serial and self.serial.is_open:
            self.serial.close()
        if self.bus_obj:
            try:
                self.bus_obj.close()
            except:
                pass
        logger.info("Rangefinder stopped")

    async def _read_loop(self):
        """Read rangefinder data continuously"""
        interval = 1.0 / self.sample_rate
        while self.running:
            try:
                reading = await self._read_raw()
                if reading:
                    # Clamp to valid range
                    if reading.distance < self.min_range:
                        reading.distance = self.min_range
                        reading.valid = False
                    elif reading.distance > self.max_range:
                        reading.distance = self.max_range
                        reading.valid = False

                    self._latest = reading
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Rangefinder read error: {e}")
                await asyncio.sleep(0.01)

    async def _read_raw(self) -> Optional[RangeReading]:
        """Read raw rangefinder data"""
        if self.bus_obj:
            if self.rangefinder_type == RangefinderType.TF_LUNA:
                return self._read_tf_luna_i2c()
            elif self.rangefinder_type == RangefinderType.GARMIN_LIDAR_LITE:
                return self._read_garmin_i2c()
            elif self.rangefinder_type == RangefinderType.GARMIN_LIDAR_LITE_V3:
                return self._read_garmin_v3_i2c()
            elif self.rangefinder_type == RangefinderType.TERARANGER_EVO:
                return self._read_teraranger_i2c()
            elif self.rangefinder_type in (RangefinderType.VL53L0X, RangefinderType.VL53L1X):
                return self._read_vl53_i2c()
        elif self.serial:
            if self.rangefinder_type in (RangefinderType.TFMINI, RangefinderType.TFMINI_S,
                                          RangefinderType.TFMINI_PLUS):
                return self._read_tfmini_serial()
            elif self.rangefinder_type in (RangefinderType.LIGHTWARE_SF11,
                                            RangefinderType.LIGHTWARE_SF20,
                                            RangefinderType.LIGHTWARE_SF30):
                return self._read_lightware_serial()

        # Mock data for testing
        import random
        return RangeReading(
            timestamp=time.time(),
            distance=random.uniform(1.0, 50.0),
            signal_strength=random.randint(100, 255),
            valid=True
        )

    def _read_tf_luna_i2c(self) -> Optional[RangeReading]:
        """Read TF-Luna via I2C"""
        try:
            # TF-Luna: distance at 0x00 (2 bytes), strength at 0x02 (2 bytes), temp at 0x04 (2 bytes)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x00, 7)
            dist = (data[1] << 8) | data[0]  # cm
            strength = (data[3] << 8) | data[2]
            temp = (data[5] << 8) | data[4]
            if temp & 0x8000:
                temp -= 0x10000
            temp_c = temp / 8.0

            return RangeReading(
                timestamp=time.time(),
                distance=dist / 100.0,  # cm to m
                signal_strength=strength,
                temperature=temp_c,
                valid=dist > 0
            )
        except Exception as e:
            logger.debug(f"TF-Luna I2C read error: {e}")
            return None

    def _read_tfmini_serial(self) -> Optional[RangeReading]:
        """Read TFmini via serial (frame: 0x59 0x59 dist_l dist_h strength_l strength_h temp_l temp_h)"""
        try:
            while self.serial.in_waiting >= 9:
                header = self.serial.read(1)
                if header == b'Y':  # 0x59
                    header2 = self.serial.read(1)
                    if header2 == b'Y':
                        frame = self.serial.read(7)
                        if len(frame) == 7:
                            dist = frame[0] | (frame[1] << 8)
                            strength = frame[2] | (frame[3] << 8)
                            temp = frame[4] | (frame[5] << 8)
                            if temp & 0x8000:
                                temp -= 0x10000
                            temp_c = temp / 8.0
                            return RangeReading(
                                timestamp=time.time(),
                                distance=dist / 100.0,
                                signal_strength=strength,
                                temperature=temp_c,
                                valid=dist > 0
                            )
                else:
                    # Sync lost, keep reading
                    pass
        except Exception as e:
            logger.debug(f"TFmini serial read error: {e}")
        return None

    def _read_lightware_serial(self) -> Optional[RangeReading]:
        """Read LightWare SF series via serial"""
        try:
            # LightWare SF: binary or ASCII
            # Binary: 0xAA 0xAA len dist_l dist_h strength_l strength_h ...
            while self.serial.in_waiting >= 2:
                header = self.serial.read(2)
                if header == b'\xAA\xAA':
                    length = self.serial.read(1)[0]
                    if length >= 4:
                        data = self.serial.read(length)
                        if len(data) >= 4:
                            dist = (data[1] << 8) | data[0]
                            strength = (data[3] << 8) | data[2]
                            return RangeReading(
                                timestamp=time.time(),
                                distance=dist / 100.0,
                                signal_strength=strength,
                                valid=dist > 0
                            )
        except Exception as e:
            logger.debug(f"LightWare serial read error: {e}")
        return None

    def _read_garmin_i2c(self) -> Optional[RangeReading]:
        """Read Garmin Lidar Lite v1/v2 via I2C"""
        try:
            # Trigger measurement
            self.bus_obj.write_byte_data(self.address, 0x00, 0x04)
            # Wait for busy flag to clear
            for _ in range(100):
                status = self.bus_obj.read_byte_data(self.address, 0x01)
                if not (status & 0x01):
                    break
                time.sleep(0.001)
            else:
                return None

            # Read distance (2 bytes, big endian, cm)
            dist_data = self.bus_obj.read_i2c_block_data(self.address, 0x0F, 2)
            dist = (dist_data[0] << 8) | dist_data[1]

            return RangeReading(
                timestamp=time.time(),
                distance=dist / 100.0,
                signal_strength=0,
                valid=dist > 0
            )
        except Exception as e:
            logger.debug(f"Garmin Lidar Lite read error: {e}")
            return None

    def _read_garmin_v3_i2c(self) -> Optional[RangeReading]:
        """Read Garmin Lidar Lite v3 via I2C"""
        try:
            # v3: continuous mode, read from 0x14 (4 bytes)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x14, 4)
            dist = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
            # dist is in mm

            return RangeReading(
                timestamp=time.time(),
                distance=dist / 1000.0,
                signal_strength=0,
                valid=dist > 0
            )
        except Exception as e:
            logger.debug(f"Garmin Lidar Lite v3 read error: {e}")
            return None

    def _read_teraranger_i2c(self) -> Optional[RangeReading]:
        """Read Teraranger Evo via I2C"""
        try:
            # Teraranger Evo: read 2 bytes from 0x00 (distance in mm)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x00, 2)
            dist = (data[0] << 8) | data[1]

            return RangeReading(
                timestamp=time.time(),
                distance=dist / 1000.0,
                signal_strength=0,
                valid=dist > 0
            )
        except Exception as e:
            logger.debug(f"Teraranger read error: {e}")
            return None

    def _read_vl53_i2c(self) -> Optional[RangeReading]:
        """Read VL53L0X/VL53L1X via I2C"""
        try:
            # VL53L0X: start single shot, wait, read result
            self.bus_obj.write_byte_data(self.address, 0x80, 0x01)
            # Wait for interrupt
            for _ in range(50):
                status = self.bus_obj.read_byte_data(self.address, 0x13)
                if status & 0x07:
                    break
                time.sleep(0.001)
            else:
                return None

            # Read distance from 0x14 (2 bytes, mm)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x14, 2)
            dist = (data[0] << 8) | data[1]

            # Read signal rate from 0x16 (2 bytes, MCPS)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x16, 2)
            signal = (data[0] << 8) | data[1]

            return RangeReading(
                timestamp=time.time(),
                distance=dist / 1000.0,
                signal_strength=signal,
                valid=dist > 0
            )
        except Exception as e:
            logger.debug(f"VL53 read error: {e}")
            return None

    def get_latest(self) -> RangeReading:
        """Get latest rangefinder reading"""
        return self._latest

    def get_status(self) -> dict:
        """Get rangefinder status"""
        r = self._latest
        return {
            'type': self.rangefinder_type.value,
            'orientation': self.orientation,
            'distance': r.distance,
            'signal_strength': r.signal_strength,
            'temperature': r.temperature,
            'valid': r.valid,
            'min_range': self.min_range,
            'max_range': self.max_range,
            'sample_rate': self.sample_rate,
            'timestamp': r.timestamp,
            'age': time.time() - r.timestamp if r.timestamp > 0 else None
        }


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'rangefinder': {
            'type': 'tf_luna',
            'bus': 1,
            'address': 0x10,
            'orientation': 'down',
            'min_range': 0.2,
            'max_range': 8.0,
            'sample_rate': 100
        }
    }
    rng = Rangefinder(config)
    try:
        await rng.start()
        while True:
            r = rng.get_latest()
            print(f"RANGE: dist={r.distance:.2f}m strength={r.signal_strength} valid={r.valid}")
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        await rng.stop()


if __name__ == '__main__':
    asyncio.run(main())