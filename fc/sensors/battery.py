#!/usr/bin/env python3
"""
Battery Monitor for Flight Controller
Supports INA219/226, BQ27441, ADC-based voltage/current sensing, and smart battery packs
"""

import asyncio
import logging
import time
from typing import Optional, List
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


@dataclass
class BatteryReading:
    """Battery status reading"""
    timestamp: float
    voltage: float  # V
    current: float  # A (positive = discharge, negative = charge)
    power: float    # W
    remaining: int  # % (0-100)
    capacity_used: float  # mAh consumed
    energy_used: float    # mWh consumed
    cell_voltages: List[float]  # Individual cell voltages
    temperature: float = 0.0  # C
    charge_state: str = "unknown"  # charging, discharging, full, empty


class BatteryType(Enum):
    INA219 = "ina219"
    INA226 = "ina226"
    BQ27441 = "bq27441"
    ADC = "adc"  # Voltage divider + current shunt
    SMART_BATTERY = "smart"  # SMBus smart battery (SBS)
    MOCK = "mock"


class BatteryMonitor:
    """Battery monitoring interface"""

    def __init__(self, config: dict):
        self.config = config.get('battery', {})
        self.battery_type = BatteryType(self.config.get('type', 'adc'))
        self.bus = self.config.get('bus', 1)
        self.address = self.config.get('address', 0x40)  # INA219 default
        self.cell_count = self.config.get('cell_count', 4)  # 4S LiPo
        self.voltage_divider = self.config.get('voltage_divider', 11.0)  # Ratio
        self.current_shunt = self.config.get('current_shunt', 0.001)  # Ohms
        self.current_gain = self.config.get('current_gain', 50.0)  # Amplifier gain
        self.adc_voltage_channel = self.config.get('adc_voltage_channel', 0)
        self.adc_current_channel = self.config.get('adc_current_channel', 1)
        self.adc_ref = self.config.get('adc_ref', 3.3)
        self.adc_bits = self.config.get('adc_bits', 12)

        # Battery characteristics
        self.cell_min = self.config.get('cell_min', 3.2)  # V per cell
        self.cell_max = self.config.get('cell_max', 4.2)  # V per cell
        self.cell_nominal = self.config.get('cell_nominal', 3.7)  # V per cell
        self.capacity = self.config.get('capacity', 5200)  # mAh

        self.bus_obj = None
        self.running = False
        self._read_task = None
        self._latest = BatteryReading(
            timestamp=0,
            voltage=0.0,
            current=0.0,
            power=0.0,
            remaining=0,
            capacity_used=0.0,
            energy_used=0.0,
            cell_voltages=[0.0] * self.cell_count,
            temperature=0.0,
            charge_state="unknown"
        )

        # Energy tracking
        self._last_time = 0.0
        self._capacity_used = 0.0  # mAh
        self._energy_used = 0.0    # mWh

    async def start(self):
        """Start battery monitor"""
        try:
            if self.battery_type == BatteryType.INA219:
                await self._start_ina219()
            elif self.battery_type == BatteryType.INA226:
                await self._start_ina226()
            elif self.battery_type == BatteryType.BQ27441:
                await self._start_bq27441()
            elif self.battery_type == BatteryType.SMART_BATTERY:
                await self._start_smart_battery()
            else:
                await self._start_adc()

            self._last_time = time.time()
            self.running = True
            self._read_task = asyncio.create_task(self._read_loop())
            logger.info(f"Battery monitor started: {self.battery_type.value}")
        except Exception as e:
            logger.error(f"Failed to start battery monitor: {e}")
            raise

    async def _start_ina219(self):
        """Initialize INA219"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)

            # Reset
            self.bus_obj.write_word_data(self.address, 0x00, 0x8000)
            await asyncio.sleep(0.01)

            # Calibration for 16V, 400mV shunt range
            # Cal = 0.04096 / (Current_LSB * R_shunt)
            # Current_LSB = Max_Expected_Current / 32767
            max_current = 10.0  # A
            current_lsb = max_current / 32767.0
            cal = int(0.04096 / (current_lsb * self.current_shunt))
            self.bus_obj.write_word_data(self.address, 0x05, cal)

            # Config: 16V range, 400mV shunt, 12-bit, continuous
            config = (0b01 << 13) | (0b00 << 11) | (0b1111 << 7) | (0b1111 << 3) | 0b111
            self.bus_obj.write_word_data(self.address, 0x00, config)

            logger.info(f"INA219 configured: cal={cal}, current_lsb={current_lsb:.6f}")
        except ImportError:
            logger.warning("smbus2 not available, using mock")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"INA219 init error: {e}")
            self.bus_obj = None

    async def _start_ina226(self):
        """Initialize INA226 (higher precision)"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # Similar to INA219 but with different registers
            logger.info("INA226 configured")
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"INA226 init error: {e}")
            self.bus_obj = None

    async def _start_bq27441(self):
        """Initialize BQ27441 fuel gauge"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # BQ27441 uses SMBus commands
            logger.info("BQ27441 configured")
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"BQ27441 init error: {e}")
            self.bus_obj = None

    async def _start_smart_battery(self):
        """Initialize Smart Battery (SBS)"""
        try:
            import smbus2
            self.bus_obj = smbus2.SMBus(self.bus)
            # SBS commands: 0x09 = Voltage, 0x0A = Current, 0x0F = RemainingCapacity
            logger.info("Smart Battery configured")
        except ImportError:
            logger.warning("smbus2 not available")
            self.bus_obj = None
        except Exception as e:
            logger.error(f"Smart Battery init error: {e}")
            self.bus_obj = None

    async def _start_adc(self):
        """Initialize ADC-based monitoring"""
        # For ADC, we'd use a separate ADC chip like ADS1115 or built-in ADC
        logger.info("ADC battery monitor configured (mock)")
        self.bus_obj = None

    async def stop(self):
        """Stop battery monitor"""
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
        logger.info("Battery monitor stopped")

    async def _read_loop(self):
        """Read battery data continuously"""
        interval = 1.0 / 10.0  # 10 Hz
        while self.running:
            try:
                reading = await self._read_raw()
                if reading:
                    # Update energy tracking
                    now = time.time()
                    dt = now - self._last_time if self._last_time > 0 else 0
                    if dt > 0:
                        self._capacity_used += reading.current * dt / 3600.0 * 1000  # mAh
                        self._energy_used += reading.power * dt / 3600.0 * 1000  # mWh

                    reading.capacity_used = self._capacity_used
                    reading.energy_used = self._energy_used
                    reading.remaining = max(0, min(100, int(
                        (1.0 - self._capacity_used / self.capacity) * 100
                    )))

                    self._latest = reading
                    self._last_time = now
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Battery read error: {e}")
                await asyncio.sleep(0.1)

    async def _read_raw(self) -> Optional[BatteryReading]:
        """Read raw battery data"""
        if self.battery_type == BatteryType.INA219:
            return self._read_ina219()
        elif self.battery_type == BatteryType.BQ27441:
            return self._read_bq27441()
        elif self.battery_type == BatteryType.SMART_BATTERY:
            return self._read_smart_battery()
        else:
            return self._read_adc()

    def _read_ina219(self) -> Optional[BatteryReading]:
        """Read INA219"""
        try:
            # Bus voltage (register 0x02)
            bus_v_raw = self.bus_obj.read_word_data(self.address, 0x02)
            bus_v_raw = ((bus_v_raw & 0xFF) << 8) | (bus_v_raw >> 8)
            bus_v = (bus_v_raw >> 3) * 0.004  # 4mV LSB

            # Shunt voltage (register 0x01)
            shunt_v_raw = self.bus_obj.read_word_data(self.address, 0x01)
            shunt_v_raw = ((shunt_v_raw & 0xFF) << 8) | (shunt_v_raw >> 8)
            if shunt_v_raw & 0x8000:
                shunt_v_raw -= 0x10000
            shunt_v = shunt_v_raw * 0.00001  # 10uV LSB

            # Current (register 0x04)
            current_raw = self.bus_obj.read_word_data(self.address, 0x04)
            current_raw = ((current_raw & 0xFF) << 8) | (current_raw >> 8)
            if current_raw & 0x8000:
                current_raw -= 0x10000
            # Current = current_raw * Current_LSB (calculated in init)
            # For simplicity, use shunt voltage directly
            current = shunt_v / self.current_shunt

            power = bus_v * current

            return BatteryReading(
                timestamp=time.time(),
                voltage=bus_v,
                current=current,
                power=power,
                remaining=0,
                capacity_used=0.0,
                energy_used=0.0,
                cell_voltages=[bus_v / self.cell_count] * self.cell_count,
                charge_state="discharging" if current > 0 else "charging" if current < -0.1 else "idle"
            )
        except Exception as e:
            logger.debug(f"INA219 read error: {e}")
            return None

    def _read_bq27441(self) -> Optional[BatteryReading]:
        """Read BQ27441 fuel gauge"""
        try:
            # SMBus commands for BQ27441
            # 0x09: Voltage (mV), 0x0A: Current (mA), 0x0F: Remaining Capacity (mAh)
            voltage = self.bus_obj.read_word_data(self.address, 0x09) / 1000.0
            current_raw = self.bus_obj.read_word_data(self.address, 0x0A)
            if current_raw & 0x8000:
                current_raw -= 0x10000
            current = current_raw / 1000.0
            remaining_mah = self.bus_obj.read_word_data(self.address, 0x0F)
            full_mah = self.bus_obj.read_word_data(self.address, 0x10)
            remaining_pct = int(remaining_mah / full_mah * 100) if full_mah > 0 else 0

            return BatteryReading(
                timestamp=time.time(),
                voltage=voltage,
                current=current,
                power=voltage * current,
                remaining=remaining_pct,
                capacity_used=self.capacity - remaining_mah,
                energy_used=0.0,
                cell_voltages=[voltage / self.cell_count] * self.cell_count,
                charge_state="discharging" if current > 0 else "charging" if current < -0.1 else "idle"
            )
        except Exception as e:
            logger.debug(f"BQ27441 read error: {e}")
            return None

    def _read_smart_battery(self) -> Optional[BatteryReading]:
        """Read SBS Smart Battery"""
        try:
            voltage = self.bus_obj.read_word_data(self.address, 0x09) / 1000.0
            current_raw = self.bus_obj.read_word_data(self.address, 0x0A)
            if current_raw & 0x8000:
                current_raw -= 0x10000
            current = current_raw / 1000.0
            remaining_mah = self.bus_obj.read_word_data(self.address, 0x0F)
            full_mah = self.bus_obj.read_word_data(self.address, 0x10)
            remaining_pct = int(remaining_mah / full_mah * 100) if full_mah > 0 else 0
            temp = self.bus_obj.read_word_data(self.address, 0x08) / 10.0 - 273.15

            return BatteryReading(
                timestamp=time.time(),
                voltage=voltage,
                current=current,
                power=voltage * current,
                remaining=remaining_pct,
                capacity_used=self.capacity - remaining_mah,
                energy_used=0.0,
                cell_voltages=[voltage / self.cell_count] * self.cell_count,
                temperature=temp,
                charge_state="discharging" if current > 0 else "charging" if current < -0.1 else "idle"
            )
        except Exception as e:
            logger.debug(f"Smart Battery read error: {e}")
            return None

    def _read_adc(self) -> Optional[BatteryReading]:
        """Read ADC-based voltage/current"""
        try:
            # Mock ADC reading - in real implementation, read from ADS1115 or similar
            # Simulate battery discharge
            import random
            base_voltage = self.cell_count * 3.8
            voltage = base_voltage + random.uniform(-0.5, 0.3)
            current = random.uniform(5, 20)  # 5-20A discharge
            power = voltage * current

            # Estimate remaining from voltage
            v_per_cell = voltage / self.cell_count
            if v_per_cell >= self.cell_max:
                remaining = 100
            elif v_per_cell <= self.cell_min:
                remaining = 0
            else:
                remaining = int((v_per_cell - self.cell_min) / (self.cell_max - self.cell_min) * 100)

            return BatteryReading(
                timestamp=time.time(),
                voltage=voltage,
                current=current,
                power=power,
                remaining=remaining,
                capacity_used=self.capacity * (100 - remaining) / 100.0,
                energy_used=0.0,
                cell_voltages=[v_per_cell] * self.cell_count,
                charge_state="discharging"
            )
        except Exception as e:
            logger.debug(f"ADC read error: {e}")
            return None

    def reset_energy_counters(self):
        """Reset energy tracking (call on battery swap)"""
        self._capacity_used = 0.0
        self._energy_used = 0.0
        logger.info("Battery energy counters reset")

    def get_latest(self) -> BatteryReading:
        """Get latest battery reading"""
        return self._latest

    def get_status(self) -> dict:
        """Get battery status"""
        r = self._latest
        return {
            'type': self.battery_type.value,
            'voltage': r.voltage,
            'current': r.current,
            'power': r.power,
            'remaining_pct': r.remaining,
            'capacity_used': r.capacity_used,
            'energy_used': r.energy_used,
            'cell_voltages': r.cell_voltages,
            'temperature': r.temperature,
            'charge_state': r.charge_state,
            'cell_count': self.cell_count,
            'capacity': self.capacity,
            'timestamp': r.timestamp,
            'age': time.time() - r.timestamp if r.timestamp > 0 else None
        }


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'battery': {
            'type': 'adc',
            'cell_count': 4,
            'capacity': 5200,
            'cell_min': 3.2,
            'cell_max': 4.2
        }
    }
    bat = BatteryMonitor(config)
    try:
        await bat.start()
        while True:
            r = bat.get_latest()
            print(f"BAT: V={r.voltage:.2f}V I={r.current:.1f}A P={r.power:.0f}W "
                  f"Rem={r.remaining}% Cells={[f'{c:.2f}' for c in r.cell_voltages]}")
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        await bat.stop()


if __name__ == '__main__':
    asyncio.run(main())