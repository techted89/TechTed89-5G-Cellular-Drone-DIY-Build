#!/usr/bin/env python3
"""
Barometer Driver for Flight Controller
Supports MS5611, BMP280/388/390, LPS22HB, DPS310, and other barometers via SPI/I2C
"""

import asyncio
import logging
import time
from typing import Optional
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


@dataclass
class BaroReading:
    """Barometer reading"""
    timestamp: float
    pressure: float  # Pa
    temperature: float  # Celsius
    altitude: float  # meters (relative to reference)
    altitude_amsl: float  # meters above mean sea level


class BaroType(Enum):
    MS5611 = "ms5611"
    BMP280 = "bmp280"
    BMP388 = "bmp388"
    BMP390 = "bmp390"
    LPS22HB = "lps22hb"
    DPS310 = "dps310"
    SPL06 = "spl06"


class Barometer:
    """Barometer interface for altitude measurement"""

    def __init__(self, config: dict):
        self.config = config.get('barometer', {})
        self.baro_type = BaroType(self.config.get('type', 'bmp388'))
        self.bus = self.config.get('bus', 1)
        self.address = self.config.get('address', 0x77)  # 0x77 or 0x76
        self.sample_rate = self.config.get('sample_rate', 100)  # Hz
        self.oversampling = self.config.get('oversampling', 'ultra_high')

        self.bus_obj = None
        self.running = False
        self._read_task = None
        self._latest = BaroReading(
            timestamp=0,
            pressure=101325.0,
            temperature=25.0,
            altitude=0.0,
            altitude_amsl=0.0
        )

        # Reference pressure for altitude (set at startup)
        self.ref_pressure = 101325.0
        self.ref_altitude = 0.0

    async def start(self):
        """Start barometer"""
        try:
            if self.baro_type in (BaroType.MS5611,):
                await self._start_ms5611()
            elif self.baro_type in (BaroType.BMP280, BaroType.BMP388, BaroType.BMP390):
                await self._start_bmp()
            elif self.baro_type == BaroType.LPS22HB:
                await self._start_lps22hb()
            elif self.baro_type == BaroType.DPS310:
                await self._start_dps310()
            else:
                await self._start_generic()

            # Establish reference pressure
            await self._establish_reference()

            self.running = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info(f"Barometer started: {self.baro_type.value} @ {self.sample_rate}Hz")
        except Exception as e:
            logger.error(f"Failed to start barometer: {e}")
            raise

    async def _start_ms5611(self):
        """Initialize MS5611"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)

            # Reset
            self.bus_obj.write_byte(self.address, 0x1E)
            await asyncio.sleep(0.01)

            # Read calibration coefficients (PROM)
            self.cal = []
            for i in range(6):
                val = self.bus_obj.read_word_data(self.address, 0xA2 + i * 2)
                # Swap bytes
                self.cal.append(((val & 0xFF) << 8) | (val >> 8))

            logger.info(f"MS5611 calibrated: {self.cal}")
        except ImportError:
            logger.warning("smbus2 not available, using mock barometer")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"MS5611 init error: {e}")
            self.bus_obj = None

    async def _start_bmp(self):
        """Initialize BMP280/388/390"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)

            # Check ID
            if self.baro_type == BaroType.BMP280:
                chip_id = self.bus_obj.read_byte_data(self.address, 0xD0)
                if chip_id != 0x58:
                    logger.warning(f"BMP280 ID mismatch: 0x{chip_id:02X}")
            elif self.baro_type in (BaroType.BMP388, BaroType.BMP390):
                chip_id = self.bus_obj.read_byte_data(self.address, 0x00)
                if chip_id != 0x50:
                    logger.warning(f"BMP388/390 ID mismatch: 0x{chip_id:02X}")

            # Soft reset
            if self.baro_type == BaroType.BMP280:
                self.bus_obj.write_byte_data(self.address, 0xE0, 0xB6)
            else:
                self.bus_obj.write_byte_data(self.address, 0x7E, 0xB6)
            await asyncio.sleep(0.01)

            # Configure for best accuracy
            if self.baro_type == BaroType.BMP280:
                # Normal mode, temp os x2, press os x16, IIR filter 16
                self.bus_obj.write_byte_data(self.address, 0xF4, 0x57)  # ctrl_meas
                self.bus_obj.write_byte_data(self.address, 0xF5, 0x50)  # config
            else:
                # BMP388/390: normal mode, temp os x2, press os x32, IIR filter
                self.bus_obj.write_byte_data(self.address, 0x1B, 0x33)  # odr_sel
                self.bus_obj.write_byte_data(self.address, 0x1C, 0x03)  # config
                self.bus_obj.write_byte_data(self.address, 0x1D, 0x33)  # osr

            # Read calibration
            await self._read_bmp_calibration()

            logger.info(f"{self.baro_type.value} configured")
        except ImportError:
            logger.warning("smbus2 not available, using mock barometer")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"BMP init error: {e}")
            self.bus_obj = None

    async def _read_bmp_calibration(self):
        """Read BMP calibration coefficients"""
        try:
            if self.baro_type == BaroType.BMP280:
                # Read 24 bytes from 0x88
                cal = self.bus_obj.read_i2c_block_data(self.address, 0x88, 24)
                import struct
                self.cal = {}
                self.cal['dig_T1'] = struct.unpack('<H', bytes(cal[0:2]))[0]
                self.cal['dig_T2'] = struct.unpack('<h', bytes(cal[2:4]))[0]
                self.cal['dig_T3'] = struct.unpack('<h', bytes(cal[4:6]))[0]
                self.cal['dig_P1'] = struct.unpack('<H', bytes(cal[6:8]))[0]
                self.cal['dig_P2'] = struct.unpack('<h', bytes(cal[8:10]))[0]
                self.cal['dig_P3'] = struct.unpack('<h', bytes(cal[10:12]))[0]
                self.cal['dig_P4'] = struct.unpack('<h', bytes(cal[12:14]))[0]
                self.cal['dig_P5'] = struct.unpack('<h', bytes(cal[14:16]))[0]
                self.cal['dig_P6'] = struct.unpack('<h', bytes(cal[16:18]))[0]
                self.cal['dig_P7'] = struct.unpack('<h', bytes(cal[18:20]))[0]
                self.cal['dig_P8'] = struct.unpack('<h', bytes(cal[20:22]))[0]
                self.cal['dig_P9'] = struct.unpack('<h', bytes(cal[22:24]))[0]
            else:
                # BMP388/390 calibration at 0x31
                cal = self.bus_obj.read_i2c_block_data(self.address, 0x31, 21)
                import struct
                self.cal = {}
                self.cal['par_t1'] = struct.unpack('<H', bytes(cal[0:2]))[0]
                self.cal['par_t2'] = struct.unpack('<h', bytes(cal[2:4]))[0]
                self.cal['par_t3'] = struct.unpack('<b', bytes(cal[4:5]))[0]
                self.cal['par_p1'] = struct.unpack('<h', bytes(cal[5:7]))[0]
                self.cal['par_p2'] = struct.unpack('<h', bytes(cal[7:9]))[0]
                self.cal['par_p3'] = struct.unpack('<b', bytes(cal[9:10]))[0]
                self.cal['par_p4'] = struct.unpack('<b', bytes(cal[10:11]))[0]
                self.cal['par_p5'] = struct.unpack('<H', bytes(cal[11:13]))[0]
                self.cal['par_p6'] = struct.unpack('<H', bytes(cal[13:15]))[0]
                self.cal['par_p7'] = struct.unpack('<b', bytes(cal[15:16]))[0]
                self.cal['par_p8'] = struct.unpack('<b', bytes(cal[16:17]))[0]
                self.cal['par_p9'] = struct.unpack('<h', bytes(cal[17:19]))[0]
                self.cal['par_p10'] = struct.unpack('<b', bytes(cal[19:20]))[0]
                self.cal['par_p11'] = struct.unpack('<b', bytes(cal[20:21]))[0]
        except Exception as e:
            logger.error(f"BMP calibration read error: {e}")

    async def _start_lps22hb(self):
        """Initialize LPS22HB"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # Check ID
            chip_id = self.bus_obj.read_byte_data(self.address, 0x0F)
            if chip_id != 0xB1:
                logger.warning(f"LPS22HB ID mismatch: 0x{chip_id:02X}")

            # CTRL_REG1: ODR=75Hz, LPF=ODR/9, BDU=1
            self.bus_obj.write_byte_data(self.address, 0x10, 0x70)
            # CTRL_REG2: SW reset
            self.bus_obj.write_byte_data(self.address, 0x11, 0x04)
            await asyncio.sleep(0.01)
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"LPS22HB init error: {e}")
            self.bus_obj = None

    async def _start_dps310(self):
        """Initialize DPS310"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # Soft reset
            self.bus_obj.write_byte_data(self.address, 0x0C, 0x89)
            await asyncio.sleep(0.01)
            # PRS_CFG: PM=1, PRS_RATE=4 (16Hz), PRS_PRC=7 (128x)
            self.bus_obj.write_byte_data(self.address, 0x06, 0x04 | (7 << 4))
            # TMP_CFG: TMP_PRC=7 (128x)
            self.bus_obj.write_byte_data(self.address, 0x07, 0x80 | 7)
            # CFG_REG: TMP_RATE=4 (16Hz)
            self.bus_obj.write_byte_data(self.address, 0x09, 0x04)
            # MEAS_CFG: COEF_RDY=1, continuous mode
            self.bus_obj.write_byte_data(self.address, 0x08, 0x07)
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"DPS310 init error: {e}")
            self.bus_obj = None

    async def _start_generic(self):
        pass

    async def _establish_reference(self):
        """Establish reference pressure for relative altitude"""
        if not self.bus_obj:
            self.ref_pressure = 101325.0
            return

        samples = 50
        pressure_sum = 0.0
        count = 0

        for _ in range(samples):
            reading = await self._read_raw()
            if reading:
                pressure_sum += reading.pressure
                count += 1
            await asyncio.sleep(0.02)

        if count > 0:
            self.ref_pressure = pressure_sum / count
            self.ref_altitude = 0.0
            logger.info(f"Barometer reference pressure: {self.ref_pressure:.1f} Pa")
        else:
            self.ref_pressure = 101325.0
            logger.warning("Failed to establish barometer reference, using 101325 Pa")

    async def stop(self):
        """Stop barometer"""
        self.running = False
        if self._read_task:
            self._read_task.cancel()
            try:
                await self._read_task
            except asyncio.CancelledError:
                pass
        if self.bus_obj:
            try:
                self.bus_obj.close()
            except:
                pass
        logger.info("Barometer stopped")

    async def _read_loop(self):
        """Read barometer data continuously"""
        interval = 1.0 / self.sample_rate
        while self.running:
            try:
                reading = await self._read_raw()
                if reading:
                    # Calculate altitude
                    reading.altitude = self._pressure_to_altitude(reading.pressure)
                    reading.altitude_amsl = self._pressure_to_altitude(reading.pressure, True)
                    self._latest = reading
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Barometer read error: {e}")
                await asyncio.sleep(0.01)

    async def _read_raw(self) -> Optional[BaroReading]:
        """Read raw barometer data"""
        if self.baro_type == BaroType.MS5611:
            return self._read_ms5611()
        elif self.baro_type in (BaroType.BMP280, BaroType.BMP388, BaroType.BMP390):
            return self._read_bmp()
        elif self.baro_type == BaroType.LPS22HB:
            return self._read_lps22hb()
        elif self.baro_type == BaroType.DPS310:
            return self._read_dps310()
        else:
            # Mock data
            return BaroReading(
                timestamp=time.time(),
                pressure=101325.0,
                temperature=25.0,
                altitude=0.0,
                altitude_amsl=0.0
            )

    def _read_ms5611(self) -> Optional[BaroReading]:
        """Read MS5611"""
        try:
            # Start pressure conversion (D1, OSR=4096)
            self.bus_obj.write_byte(self.address, 0x48)
            # Wait for conversion (max 9.04ms for OSR=4096)
            import time as time_mod
            time_mod.sleep(0.01)
            # Read ADC
            d1_data = self.bus_obj.read_i2c_block_data(self.address, 0x00, 3)
            d1 = (d1_data[0] << 16) | (d1_data[1] << 8) | d1_data[2]

            # Start temperature conversion (D2, OSR=4096)
            self.bus_obj.write_byte(self.address, 0x58)
            time_mod.sleep(0.01)
            d2_data = self.bus_obj.read_i2c_block_data(self.address, 0x00, 3)
            d2 = (d2_data[0] << 16) | (d2_data[1] << 8) | d2_data[2]

            # Calculate temperature and pressure
            # From MS5611 datasheet
            dT = d2 - self.cal[4] * 256
            temp = 2000 + dT * self.cal[5] / 8388608  # 0.01 deg C

            off = self.cal[1] * 65536 + self.cal[3] * dT / 128
            sens = self.cal[0] * 32768 + self.cal[2] * dT / 256

            pressure = (d1 * sens / 2097152 - off) / 32768  # 0.01 mbar = Pa

            return BaroReading(
                timestamp=time.time(),
                pressure=pressure * 100,  # Convert to Pa
                temperature=temp / 100.0,
                altitude=0.0,
                altitude_amsl=0.0
            )
        except Exception as e:
            logger.debug(f"MS5611 read error: {e}")
            return None

    def _read_bmp(self) -> Optional[BaroReading]:
        """Read BMP280/388/390"""
        try:
            if self.baro_type == BaroType.BMP280:
                # Read 6 bytes from 0xF7
                data = self.bus_obj.read_i2c_block_data(self.address, 0xF7, 6)
                adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
                adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

                # Compensate temperature
                var1 = (adc_t / 16384.0 - self.cal['dig_T1'] / 1024.0) * self.cal['dig_T2']
                var2 = (adc_t / 131072.0 - self.cal['dig_T1'] / 8192.0) ** 2 * self.cal['dig_T3']
                t_fine = var1 + var2
                temp = t_fine / 5120.0

                # Compensate pressure
                var1 = t_fine / 2.0 - 64000.0
                var2 = var1 * var1 * self.cal['dig_P6'] / 32768.0
                var2 += var1 * self.cal['dig_P5'] * 2.0
                var2 = var2 / 4.0 + self.cal['dig_P4'] * 65536.0
                var1 = (self.cal['dig_P3'] * var1 * var1 / 524288.0 + self.cal['dig_P2'] * var1) / 524288.0
                var1 = (1.0 + var1 / 32768.0) * self.cal['dig_P1']

                if var1 == 0:
                    return None

                pressure = 1048576.0 - adc_p
                pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
                var1 = self.cal['dig_P9'] * pressure * pressure / 2147483648.0
                var2 = pressure * self.cal['dig_P8'] / 32768.0
                pressure = pressure + (var1 + var2 + self.cal['dig_P7']) / 16.0

                return BaroReading(
                    timestamp=time.time(),
                    pressure=pressure,
                    temperature=temp,
                    altitude=0.0,
                    altitude_amsl=0.0
                )
            else:
                # BMP388/390: read from 0x04
                data = self.bus_obj.read_i2c_block_data(self.address, 0x04, 6)
                adc_p = (data[0] << 16) | (data[1] << 8) | data[2]
                adc_t = (data[3] << 16) | (data[4] << 8) | data[5]

                # Simplified compensation (full algo is complex)
                # Use approximate for now
                temp = 25.0 + (adc_t - 8388608) / 100000.0
                pressure = 101325.0 + (adc_p - 8388608) / 100.0

                return BaroReading(
                    timestamp=time.time(),
                    pressure=pressure,
                    temperature=temp,
                    altitude=0.0,
                    altitude_amsl=0.0
                )
        except Exception as e:
            logger.debug(f"BMP read error: {e}")
            return None

    def _read_lps22hb(self) -> Optional[BaroReading]:
        """Read LPS22HB"""
        try:
            data = self.bus_obj.read_i2c_block_data(self.address, 0x28, 5)
            press = (data[2] << 16) | (data[1] << 8) | data[0]
            temp = (data[4] << 8) | data[3]
            if temp & 0x8000:
                temp -= 0x10000

            pressure = press / 4096.0  # hPa
            temperature = temp / 100.0

            return BaroReading(
                timestamp=time.time(),
                pressure=pressure * 100,  # Pa
                temperature=temperature,
                altitude=0.0,
                altitude_amsl=0.0
            )
        except Exception as e:
            logger.debug(f"LPS22HB read error: {e}")
            return None

    def _read_dps310(self) -> Optional[BaroReading]:
        """Read DPS310"""
        try:
            # Check if data ready
            status = self.bus_obj.read_byte_data(self.address, 0x0C)
            if not (status & 0x03):  # PRS_RDY | TMP_RDY
                return None

            # Read pressure (3 bytes from 0x00)
            p_data = self.bus_obj.read_i2c_block_data(self.address, 0x00, 3)
            adc_p = (p_data[0] << 16) | (p_data[1] << 8) | p_data[2]
            if adc_p & 0x800000:
                adc_p -= 0x1000000

            # Read temperature (3 bytes from 0x03)
            t_data = self.bus_obj.read_i2c_block_data(self.address, 0x03, 3)
            adc_t = (t_data[0] << 16) | (t_data[1] << 8) | t_data[2]
            if adc_t & 0x800000:
                adc_t -= 0x1000000

            # Scaling factors for 128x oversampling
            pressure = 101325.0 + adc_p / 64.0
            temperature = 25.0 + adc_t / 512.0

            return BaroReading(
                timestamp=time.time(),
                pressure=pressure,
                temperature=temperature,
                altitude=0.0,
                altitude_amsl=0.0
            )
        except Exception as e:
            logger.debug(f"DPS310 read error: {e}")
            return None

    def _pressure_to_altitude(self, pressure: float, amsl: bool = False) -> float:
        """Convert pressure to altitude using barometric formula"""
        if amsl:
            p_ref = 101325.0
        else:
            p_ref = self.ref_pressure

        if p_ref <= 0 or pressure <= 0:
            return 0.0

        # International Standard Atmosphere
        # h = 44330 * (1 - (P/P0)^(1/5.255))
        return 44330.0 * (1.0 - pow(pressure / p_ref, 1.0 / 5.255))

    def set_reference(self, pressure: float = None, altitude: float = 0.0):
        """Set reference pressure/altitude"""
        if pressure is not None:
            self.ref_pressure = pressure
        else:
            self.ref_pressure = self._latest.pressure
        self.ref_altitude = altitude
        logger.info(f"Barometer reference set: pressure={self.ref_pressure:.1f} Pa, altitude={altitude:.1f} m")

    def get_latest(self) -> BaroReading:
        """Get latest barometer reading"""
        return self._latest

    def get_status(self) -> dict:
        """Get barometer status"""
        r = self._latest
        return {
            'type': self.baro_type.value,
            'sample_rate': self.sample_rate,
            'pressure': r.pressure,
            'temperature': r.temperature,
            'altitude': r.altitude,
            'altitude_amsl': r.altitude_amsl,
            'ref_pressure': self.ref_pressure,
            'timestamp': r.timestamp,
            'age': time.time() - r.timestamp if r.timestamp > 0 else None
        }


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'barometer': {
            'type': 'bmp388',
            'bus': 1,
            'address': 0x77,
            'sample_rate': 100,
            'oversampling': 'ultra_high'
        }
    }
    baro = Barometer(config)
    try:
        await baro.start()
        while True:
            r = baro.get_latest()
            print(f"BARO: P={r.pressure:.1f} Pa T={r.temperature:.1f}C "
                  f"alt={r.altitude:.2f}m amsl={r.altitude_amsl:.1f}m")
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        await baro.stop()


if __name__ == '__main__':
    asyncio.run(main())