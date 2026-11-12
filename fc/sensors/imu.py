#!/usr/bin/env python3
"""
IMU Driver for Flight Controller
Supports MPU6000/6050/9250, ICM20602/20689, BMI088, and other IMUs via SPI/I2C
"""

import asyncio
import logging
import struct
import time
from typing import Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


@dataclass
class IMUReading:
    """Single IMU reading"""
    timestamp: float
    accel: tuple  # (x, y, z) in m/s^2
    gyro: tuple   # (x, y, z) in rad/s
    mag: tuple = (0.0, 0.0, 0.0)  # (x, y, z) in uT
    temperature: float = 0.0


class IMUType(Enum):
    MPU6000 = "mpu6000"
    MPU6050 = "mpu6050"
    MPU9250 = "mpu9250"
    ICM20602 = "icm20602"
    ICM20689 = "icm20689"
    BMI088 = "bmi088"
    LSM6DSO = "lsm6dso"


class IMU:
    """IMU interface for flight controller"""

    def __init__(self, config: dict):
        self.config = config.get('imu', {})
        self.imu_type = IMUType(self.config.get('type', 'icm20689'))
        self.bus = self.config.get('bus', 1)  # I2C bus or SPI bus
        self.address = self.config.get('address', 0x68)  # I2C address
        self.sample_rate = self.config.get('sample_rate', 400)  # Hz
        self.accel_range = self.config.get('accel_range', 16)  # +/- g
        self.gyro_range = self.config.get('gyro_range', 2000)  # +/- dps

        self.serial = None
        self.running = False
        self._read_task = None
        self._latest = IMUReading(
            timestamp=0,
            accel=(0.0, 0.0, 0.0),
            gyro=(0.0, 0.0, 0.0)
        )

        # Calibration
        self.accel_bias = (0.0, 0.0, 0.0)
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.accel_scale = (1.0, 1.0, 1.0)
        self.gyro_scale = (1.0, 1.0, 1.0)

    async def start(self):
        """Start IMU"""
        try:
            if self.imu_type in (IMUType.MPU6000, IMUType.MPU6050, IMUType.MPU9250):
                await self._start_mpu()
            elif self.imu_type in (IMUType.ICM20602, IMUType.ICM20689):
                await self._start_icm()
            elif self.imu_type == IMUType.BMI088:
                await self._start_bmi088()
            else:
                await self._start_generic()

            self.running = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info(f"IMU started: {self.imu_type.value} @ {self.sample_rate}Hz")
        except Exception as e:
            logger.error(f"Failed to start IMU: {e}")
            raise

    async def _start_mpu(self):
        """Initialize MPU6000/6050/9250"""
        # For SPI: use spidev
        # For I2C: use smbus2
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # Wake up
            self.bus_obj.write_byte_data(self.address, 0x6B, 0x00)
            await asyncio.sleep(0.1)

            # Set sample rate divider
            # Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
            # Gyro Output Rate = 8kHz (DLPF disabled) or 1kHz (DLPF enabled)
            smplrt_div = max(0, int(1000 / self.sample_rate) - 1)
            self.bus_obj.write_byte_data(self.address, 0x19, smplrt_div)

            # Config: DLPF_CFG = 3 (42Hz BW, 1kHz rate)
            self.bus_obj.write_byte_data(self.address, 0x1A, 0x03)

            # Gyro config: +/- 2000 dps
            gyro_fs = {250: 0, 500: 1, 1000: 2, 2000: 3}.get(self.gyro_range, 3)
            self.bus_obj.write_byte_data(self.address, 0x1B, gyro_fs << 3)

            # Accel config: +/- 16g
            accel_fs = {2: 0, 4: 1, 8: 2, 16: 3}.get(self.accel_range, 3)
            self.bus_obj.write_byte_data(self.address, 0x1C, accel_fs << 3)

            # Accel config 2: A_DLPFCFG = 3 (44.8Hz BW, 1kHz rate)
            self.bus_obj.write_byte_data(self.address, 0x1D, 0x03)

            # Enable FIFO
            self.bus_obj.write_byte_data(self.address, 0x6A, 0x40)

            logger.info(f"MPU configured: sample_rate={self.sample_rate}Hz, "
                        f"accel=±{self.accel_range}g, gyro=±{self.gyro_range}dps")

        except ImportError:
            logger.warning("smbus2 not available, using mock IMU")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"MPU init error: {e}")
            self.bus_obj = None

    async def _start_icm(self):
        """Initialize ICM20602/20689"""
        # Similar to MPU but with different registers
        await self._start_mpu()

    async def _start_bmi088(self):
        """Initialize BMI088"""
        # BMI088 has separate accel and gyro I2C addresses
        try:
            import smbus2
            self.accel_bus = smbus2.SMBus(self.bus)
            self.gyro_bus = smbus2.SMBus(self.bus)
            self.accel_addr = 0x18  # BMI088 accel
            self.gyro_addr = 0x68   # BMI088 gyro

            # Soft reset
            self.accel_bus.write_byte_data(self.accel_addr, 0x7E, 0xB6)
            self.gyro_bus.write_byte_data(self.gyro_addr, 0x14, 0xB6)
            await asyncio.sleep(0.1)

            # Configure accel: +/- 24g, 400Hz ODR, normal mode
            self.accel_bus.write_byte_data(self.accel_addr, 0x40, 0xA0 | 0x05)
            await asyncio.sleep(0.01)

            # Configure gyro: +/- 2000 dps, 400Hz ODR
            self.gyro_bus.write_byte_data(self.gyro_addr, 0x0F, 0x00)
            self.gyro_bus.write_byte_data(self.gyro_addr, 0x10, 0x07)
            await asyncio.sleep(0.01)

            logger.info("BMI088 configured")
        except ImportError:
            logger.warning("smbus2 not available, using mock IMU")
            self.accel_bus = None
            self.gyro_bus = None
        except Exception as e:
            logger.error(f"BMI088 init error: {e}")
            self.accel_bus = None
            self.gyro_bus = None

    async def _start_generic(self):
        """Generic IMU initialization"""
        pass

    async def stop(self):
        """Stop IMU"""
        self.running = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass

        # Close buses
        for bus in [getattr(self, 'bus_obj', None),
                    getattr(self, 'accel_bus', None),
                    getattr(self, 'gyro_bus', None)]:
            if bus:
                try:
                    bus.close()
                except:
                    pass

        logger.info("IMU stopped")

    async def _read_loop(self):
        """Read IMU data continuously"""
        interval = 1.0 / self.sample_rate
        while self.running:
            try:
                reading = await self._read_raw()
                if reading:
                    # Apply calibration
                    reading = self._apply_calibration(reading)
                    self._latest = reading
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"IMU read error: {e}")
                await asyncio.sleep(0.01)

    async def _read_raw(self) -> Optional[IMUReading]:
        """Read raw IMU data"""
        if self.bus_obj:
            return self._read_mpu()
        elif hasattr(self, 'accel_bus') and self.accel_bus:
            return self._read_bmi088()
        else:
            # Mock data for testing
            return IMUReading(
                timestamp=time.time(),
                accel=(0.01, -0.02, 9.81),
                gyro=(0.001, 0.002, 0.0005),
                temperature=25.0
            )

    def _read_mpu(self) -> Optional[IMUReading]:
        """Read MPU6000/6050/9250 data"""
        try:
            # Read 14 bytes: accel(6) + temp(2) + gyro(6)
            data = self.bus_obj.read_i2c_block_data(self.address, 0x3B, 14)
            ax = struct.unpack('>h', bytes(data[0:2]))[0]
            ay = struct.unpack('>h', bytes(data[2:4]))[0]
            az = struct.unpack('>h', bytes(data[4:6]))[0]
            temp = struct.unpack('>h', bytes(data[6:8]))[0]
            gx = struct.unpack('>h', bytes(data[8:10]))[0]
            gy = struct.unpack('>h', bytes(data[10:12]))[0]
            gz = struct.unpack('>h', bytes(data[12:14]))[0]

            # Convert to physical units
            # Accel: 16g range = 2048 LSB/g
            # Gyro: 2000 dps range = 16.4 LSB/(deg/s)
            accel_scale = 9.81 / 2048.0
            gyro_scale = 0.0174533 / 16.4  # rad/s per LSB

            return IMUReading(
                timestamp=time.time(),
                accel=(ax * accel_scale, ay * accel_scale, az * accel_scale),
                gyro=(gx * gyro_scale, gy * gyro_scale, gz * gyro_scale),
                temperature=temp / 340.0 + 36.53
            )
        except Exception as e:
            logger.debug(f"MPU read error: {e}")
            return None

    def _read_bmi088(self) -> Optional[IMUReading]:
        """Read BMI088 data"""
        try:
            # Read accel (6 bytes from 0x12)
            accel_data = self.accel_bus.read_i2c_block_data(self.accel_addr, 0x12, 6)
            # Read gyro (6 bytes from 0x02)
            gyro_data = self.gyro_bus.read_i2c_block_data(self.gyro_addr, 0x02, 6)
            # Read temp (2 bytes from 0x22)
            temp_data = self.accel_bus.read_i2c_block_data(self.accel_addr, 0x22, 2)

            ax = struct.unpack('<h', bytes(accel_data[0:2]))[0]
            ay = struct.unpack('<h', bytes(accel_data[2:4]))[0]
            az = struct.unpack('<h', bytes(accel_data[4:6]))[0]
            gx = struct.unpack('<h', bytes(gyro_data[0:2]))[0]
            gy = struct.unpack('<h', bytes(gyro_data[2:4]))[0]
            gz = struct.unpack('<h', bytes(gyro_data[4:6]))[0]
            temp = struct.unpack('<h', bytes(temp_data))[0]

            # BMI088 scales: 24g = 3.33 LSB/g, 2000dps = 16.4 LSB/(deg/s)
            accel_scale = 9.81 / (32768 / 24.0)
            gyro_scale = 0.0174533 / 16.4

            return IMUReading(
                timestamp=time.time(),
                accel=(ax * accel_scale, ay * accel_scale, az * accel_scale),
                gyro=(gx * gyro_scale, gy * gyro_scale, gz * gyro_scale),
                temperature=temp * 0.125 + 23.0
            )
        except Exception as e:
            logger.debug(f"BMI088 read error: {e}")
            return None

    def _apply_calibration(self, reading: IMUReading) -> IMUReading:
        """Apply calibration offsets and scaling"""
        ax, ay, az = reading.accel
        gx, gy, gz = reading.gyro

        # Apply bias and scale
        ax = (ax - self.accel_bias[0]) * self.accel_scale[0]
        ay = (ay - self.accel_bias[1]) * self.accel_scale[1]
        az = (az - self.accel_bias[2]) * self.accel_scale[2]
        gx = (gx - self.gyro_bias[0]) * self.gyro_scale[0]
        gy = (gy - self.gyro_bias[1]) * self.gyro_scale[1]
        gz = (gz - self.gyro_bias[2]) * self.gyro_scale[2]

        return IMUReading(
            timestamp=reading.timestamp,
            accel=(ax, ay, az),
            gyro=(gx, gy, gz),
            mag=reading.mag,
            temperature=reading.temperature
        )

    def get_latest(self) -> IMUReading:
        """Get latest IMU reading"""
        return self._latest

    def get_status(self) -> dict:
        """Get IMU status"""
        r = self._latest
        return {
            'type': self.imu_type.value,
            'sample_rate': self.sample_rate,
            'accel': r.accel,
            'gyro': r.gyro,
            'mag': r.mag,
            'temperature': r.temperature,
            'timestamp': r.timestamp,
            'age': time.time() - r.timestamp if r.timestamp > 0 else None,
            'calibrated': any(b != 0 for b in self.accel_bias) or any(b != 0 for b in self.gyro_bias)
        }

    async def calibrate(self, samples: int = 2000):
        """Calibrate IMU (must be stationary!)"""
        logger.info(f"Starting IMU calibration with {samples} samples...")
        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        count = 0

        for _ in range(samples):
            reading = await self._read_raw()
            if reading:
                for i in range(3):
                    accel_sum[i] += reading.accel[i]
                    gyro_sum[i] += reading.gyro[i]
                count += 1
            await asyncio.sleep(1.0 / self.sample_rate)

        if count > 0:
            # Expected: accel = (0, 0, 9.81) when level
            self.accel_bias = (
                accel_sum[0] / count,
                accel_sum[1] / count,
                accel_sum[2] / count - 9.81
            )
            self.gyro_bias = (
                gyro_sum[0] / count,
                gyro_sum[1] / count,
                gyro_sum[2] / count
            )
            logger.info(f"Calibration complete: accel_bias={self.accel_bias}, gyro_bias={self.gyro_bias}")
            return True
        return False


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'imu': {
            'type': 'icm20689',
            'bus': 1,
            'address': 0x68,
            'sample_rate': 400,
            'accel_range': 16,
            'gyro_range': 2000
        }
    }
    imu = IMU(config)
    try:
        await imu.start()
        await imu.calibrate(1000)
        while True:
            r = imu.get_latest()
            print(f"IMU: accel={r.accel[0]:.2f},{r.accel[1]:.2f},{r.accel[2]:.2f} "
                  f"gyro={r.gyro[0]:.4f},{r.gyro[1]:.4f},{r.gyro[2]:.4f}")
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        await imu.stop()


if __name__ == '__main__':
    asyncio.run(main())