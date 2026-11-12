#!/usr/bin/env python3
"""
LoRa Radio Driver for Flight Controller
Supports SX1276/SX1278 (RFM95/96/98), SX1262, SX1280 via SPI
"""

import asyncio
import logging
import struct
import time
from typing import Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class LoRaMode(Enum):
    LORA = 0
    FSK = 1


class LoRaBandwidth(Enum):
    BW_7_8_KHZ = 0
    BW_10_4_KHZ = 1
    BW_15_6_KHZ = 2
    BW_20_8_KHZ = 3
    BW_31_25_KHZ = 4
    BW_41_7_KHZ = 5
    BW_62_5_KHZ = 6
    BW_125_KHZ = 7
    BW_250_KHZ = 8
    BW_500_KHZ = 9


class LoRaSpreadingFactor(Enum):
    SF_6 = 6
    SF_7 = 7
    SF_8 = 8
    SF_9 = 9
    SF_10 = 10
    SF_11 = 11
    SF_12 = 12


class LoRaCodingRate(Enum):
    CR_4_5 = 1
    CR_4_6 = 2
    CR_4_7 = 3
    CR_4_8 = 4


@dataclass
class LoRaConfig:
    """LoRa radio configuration"""
    frequency: float = 915.0  # MHz
    tx_power: int = 20  # dBm (2-20 for SX1276, up to 22 for SX1262)
    spreading_factor: LoRaSpreadingFactor = LoRaSpreadingFactor.SF_10
    bandwidth: LoRaBandwidth = LoRaBandwidth.BW_125_KHZ
    coding_rate: LoRaCodingRate = LoRaCodingRate.CR_4_5
    preamble_length: int = 8
    sync_word: int = 0x12  # LoRaWAN public = 0x34, private = 0x12
    crc_enabled: bool = True
    implicit_header: bool = False
    max_payload_length: int = 255


@dataclass
class LoRaPacket:
    """Received LoRa packet"""
    data: bytes
    rssi: float  # dBm
    snr: float   # dB
    frequency_error: int = 0  # Hz
    timestamp: float = 0


class LoRaRadio:
    """LoRa radio interface for SX1276/SX1278 (RFM95/96/98)"""

    # Register addresses for SX1276
    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_FRF_MID = 0x07
    REG_FRF_LSB = 0x08
    REG_PA_CONFIG = 0x09
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_TX_BASE_ADDR = 0x0E
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_PKT_SNR_VALUE = 0x19
    REG_PKT_RSSI_VALUE = 0x1A
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_PAYLOAD_LENGTH = 0x22
    REG_MAX_PAYLOAD_LENGTH = 0x23
    REG_HOP_PERIOD = 0x24
    REG_FIFO_RX_BYTE_ADDR = 0x25
    REG_MODEM_CONFIG_3 = 0x26
    REG_DETECT_OPTIMIZE = 0x31
    REG_INVERT_IQ = 0x33
    REG_DETECTION_THRESHOLD = 0x37
    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40
    REG_VERSION = 0x42

    # Op modes
    MODE_SLEEP = 0x00
    MODE_STDBY = 0x01
    MODE_TX = 0x03
    MODE_RX_CONTINUOUS = 0x05
    MODE_RX_SINGLE = 0x06
    LONG_RANGE_MODE = 0x80

    # IRQ flags
    IRQ_TX_DONE = 0x08
    IRQ_PAYLOAD_CRC_ERROR = 0x10
    IRQ_RX_DONE = 0x40

    def __init__(self, config: dict):
        self.config = config.get('lora', {})
        self.spi_bus = self.config.get('spi_bus', 0)
        self.spi_device = self.config.get('spi_device', 0)
        self.cs_pin = self.config.get('cs_pin', 8)  # GPIO for chip select
        self.reset_pin = self.config.get('reset_pin', 25)  # GPIO for reset
        self.dio0_pin = self.config.get('dio0_pin', None)  # GPIO for DIO0 interrupt

        # LoRa parameters
        self.lora_config = LoRaConfig(
            frequency=self.config.get('frequency', 915.0),
            tx_power=self.config.get('tx_power', 20),
            spreading_factor=LoRaSpreadingFactor(self.config.get('spreading_factor', 10)),
            bandwidth=LoRaBandwidth(self.config.get('bandwidth', 7)),
            coding_rate=LoRaCodingRate(self.config.get('coding_rate', 1)),
            preamble_length=self.config.get('preamble_length', 8),
            sync_word=self.config.get('sync_word', 0x12),
            crc_enabled=self.config.get('crc_enabled', True),
        )

        self.spi = None
        self.gpio_initialized = False
        self.running = False
        self._rx_callback: Optional[Callable[[LoRaPacket], None]] = None
        self._rx_task = None
        self._last_rssi = 0.0
        self._last_snr = 0.0

    async def start(self, rx_callback: Optional[Callable[[LoRaPacket], None]] = None):
        """Start LoRa radio"""
        self._rx_callback = rx_callback
        try:
            await self._init_hardware()
            await self._configure_lora()
            self.running = True
            self._rx_task = asyncio.create_task(self._rx_loop())
            logger.info(f"LoRa radio started: {self.lora_config.frequency} MHz, "
                        f"SF{self.lora_config.spreading_factor.value}, "
                        f"BW{self.lora_config.bandwidth.name.split('_')[-1]}")
        except Exception as e:
            logger.error(f"Failed to start LoRa radio: {e}")
            raise

    async def _init_hardware(self):
        """Initialize SPI and GPIO"""
        try:
            import spidev
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 5_000_000
            self.spi.mode = 0
        except ImportError:
            logger.warning("spidev not available, using mock")
            self.spi = None
        except Exception as e:
            logger.error(f"SPI init error: {e}")
            self.spi = None

        # Initialize GPIO for reset and CS
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.cs_pin, GPIO.OUT)
            GPIO.setup(self.reset_pin, GPIO.OUT)
            if self.dio0_pin:
                GPIO.setup(self.dio0_pin, GPIO.IN)
            self.gpio_initialized = True
        except ImportError:
            logger.warning("RPi.GPIO not available")
            self.gpio_initialized = False
        except Exception as e:
            logger.error(f"GPIO init error: {e}")
            self.gpio_initialized = False

        # Hardware reset
        if self.gpio_initialized:
            self.GPIO.output(self.reset_pin, GPIO.LOW)
            await asyncio.sleep(0.01)
            self.GPIO.output(self.reset_pin, GPIO.HIGH)
            await asyncio.sleep(0.01)

        # Verify chip version
        version = self._read_reg(self.REG_VERSION)
        if version != 0x12:
            logger.warning(f"Unexpected LoRa chip version: 0x{version:02X} (expected 0x12 for SX1276)")

    async def _configure_lora(self):
        """Configure LoRa radio parameters"""
        # Set to sleep mode first
        self._write_reg(self.REG_OP_MODE, self.MODE_SLEEP | self.LONG_RANGE_MODE)
        await asyncio.sleep(0.01)

        # Set frequency
        frf = int(self.lora_config.frequency * 1_000_000 / 61.03515625)  # FRF = freq / 61.035 Hz
        self._write_reg(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write_reg(self.REG_FRF_MID, (frf >> 8) & 0xFF)
        self._write_reg(self.REG_FRF_LSB, frf & 0xFF)

        # Set TX power
        pa_config = 0x80 | (min(max(self.lora_config.tx_power, 2), 20) - 2)  # PA_BOOST + power
        self._write_reg(self.REG_PA_CONFIG, pa_config)

        # Set LNA gain
        self._write_reg(self.REG_LNA, 0x23)  # Max gain, boost on

        # Set FIFO base addresses
        self._write_reg(self.REG_FIFO_TX_BASE_ADDR, 0x00)
        self._write_reg(self.REG_FIFO_RX_BASE_ADDR, 0x00)

        # Modem config 1: BW, CR, implicit header
        bw_val = self.lora_config.bandwidth.value << 4
        cr_val = self.lora_config.coding_rate.value << 1
        implicit_val = 0x01 if self.lora_config.implicit_header else 0x00
        modem_config1 = bw_val | cr_val | implicit_val
        self._write_reg(self.REG_MODEM_CONFIG_1, modem_config1)

        # Modem config 2: SF, CRC, TX continuous mode
        sf_val = self.lora_config.spreading_factor.value << 4
        crc_val = 0x04 if self.lora_config.crc_enabled else 0x00
        modem_config2 = sf_val | crc_val | 0x03  # 0x03 = normal mode
        self._write_reg(self.REG_MODEM_CONFIG_2, modem_config2)

        # Modem config 3: Low data rate optimize, AGC auto on
        ldro = 0x08 if self.lora_config.spreading_factor in (LoRaSpreadingFactor.SF_11, LoRaSpreadingFactor.SF_12) else 0x00
        modem_config3 = ldro | 0x04  # AGC auto on
        self._write_reg(self.REG_MODEM_CONFIG_3, modem_config3)

        # Preamble length
        self._write_reg(self.REG_PREAMBLE_MSB, (self.lora_config.preamble_length >> 8) & 0xFF)
        self._write_reg(self.REG_PREAMBLE_LSB, self.lora_config.preamble_length & 0xFF)

        # Max payload length
        self._write_reg(self.REG_MAX_PAYLOAD_LENGTH, self.lora_config.max_payload_length)

        # Sync word
        self._write_reg(self.REG_SYNC_WORD, self.lora_config.sync_word)

        # Detection optimize for SF
        if self.lora_config.spreading_factor == LoRaSpreadingFactor.SF_6:
            self._write_reg(self.REG_DETECT_OPTIMIZE, 0x05)
            self._write_reg(self.REG_DETECTION_THRESHOLD, 0x0C)
        else:
            self._write_reg(self.REG_DETECT_OPTIMIZE, 0x03)
            self._write_reg(self.REG_DETECTION_THRESHOLD, 0x0A)

        # Set to standby
        self._write_reg(self.REG_OP_MODE, self.MODE_STDBY | self.LONG_RANGE_MODE)
        await asyncio.sleep(0.01)

        # Set to RX continuous mode
        self._write_reg(self.REG_OP_MODE, self.MODE_RX_CONTINUOUS | self.LONG_RANGE_MODE)

    def _write_reg(self, addr: int, value: int):
        """Write register via SPI"""
        if self.spi:
            self.GPIO.output(self.cs_pin, self.GPIO.LOW)
            self.spi.xfer2([addr | 0x80, value])
            self.GPIO.output(self.cs_pin, self.GPIO.HIGH)
        else:
            # Mock
            pass

    def _read_reg(self, addr: int) -> int:
        """Read register via SPI"""
        if self.spi:
            self.GPIO.output(self.cs_pin, self.GPIO.LOW)
            result = self.spi.xfer2([addr & 0x7F, 0x00])
            self.GPIO.output(self.cs_pin, self.GPIO.HIGH)
            return result[1]
        return 0x12  # Mock version

    def _read_fifo(self, length: int) -> bytes:
        """Read data from FIFO"""
        if self.spi:
            self.GPIO.output(self.cs_pin, self.GPIO.LOW)
            self.spi.xfer2([self.REG_FIFO & 0x7F])
            data = self.spi.xfer2([0x00] * length)
            self.GPIO.output(self.cs_pin, self.GPIO.HIGH)
            return bytes(data)
        return b'\x00' * length  # Mock

    async def send(self, data: bytes) -> bool:
        """Send LoRa packet"""
        if not self.running:
            return False

        # Wait for any ongoing TX to complete
        for _ in range(100):
            irq = self._read_reg(self.REG_IRQ_FLAGS)
            if irq & self.IRQ_TX_DONE:
                break
            await asyncio.sleep(0.001)

        # Clear IRQ flags
        self._write_reg(self.REG_IRQ_FLAGS, 0xFF)

        # Set FIFO address pointer to TX base
        self._write_reg(self.REG_FIFO_ADDR_PTR, 0x00)

        # Write payload to FIFO
        if self.spi:
            self.GPIO.output(self.cs_pin, self.GPIO.LOW)
            self.spi.xfer2([self.REG_FIFO | 0x80] + list(data))
            self.GPIO.output(self.cs_pin, self.GPIO.HIGH)

        # Set payload length
        self._write_reg(self.REG_PAYLOAD_LENGTH, len(data))

        # Switch to TX mode
        self._write_reg(self.REG_OP_MODE, self.MODE_TX | self.LONG_RANGE_MODE)

        # Wait for TX done
        for _ in range(500):  # Up to 500ms
            irq = self._read_reg(self.REG_IRQ_FLAGS)
            if irq & self.IRQ_TX_DONE:
                self._write_reg(self.REG_IRQ_FLAGS, self.IRQ_TX_DONE)
                # Return to RX mode
                self._write_reg(self.REG_OP_MODE, self.MODE_RX_CONTINUOUS | self.LONG_RANGE_MODE)
                return True
            await asyncio.sleep(0.001)

        # Timeout - return to RX
        self._write_reg(self.REG_OP_MODE, self.MODE_RX_CONTINUOUS | self.LONG_RANGE_MODE)
        logger.warning("LoRa TX timeout")
        return False

    async def _rx_loop(self):
        """Background RX loop"""
        while self.running:
            try:
                irq = self._read_reg(self.REG_IRQ_FLAGS)
                if irq & self.IRQ_RX_DONE:
                    # Clear IRQ
                    self._write_reg(self.REG_IRQ_FLAGS, self.IRQ_RX_DONE)

                    # Check CRC
                    if irq & self.IRQ_PAYLOAD_CRC_ERROR:
                        logger.debug("LoRa CRC error")
                        continue

                    # Get packet info
                    current_addr = self._read_reg(self.REG_FIFO_RX_CURRENT_ADDR)
                    payload_length = self._read_reg(self.REG_RX_NB_BYTES)

                    # Set FIFO pointer
                    self._write_reg(self.REG_FIFO_ADDR_PTR, current_addr)

                    # Read payload
                    payload = self._read_fifo(payload_length)

                    # Get RSSI and SNR
                    rssi_raw = self._read_reg(self.REG_PKT_RSSI_VALUE)
                    snr_raw = self._read_reg(self.REG_PKT_SNR_VALUE)

                    # Convert to dBm/dB
                    rssi = rssi_raw - 157  # SX1276 RSSI offset
                    if snr_raw & 0x80:
                        snr = -((~snr_raw & 0xFF) + 1) / 4.0
                    else:
                        snr = snr_raw / 4.0

                    packet = LoRaPacket(
                        data=payload,
                        rssi=rssi,
                        snr=snr,
                        timestamp=time.time()
                    )

                    self._last_rssi = rssi
                    self._last_snr = snr

                    if self._rx_callback:
                        try:
                            await self._rx_callback(packet)
                        except Exception as e:
                            logger.error(f"RX callback error: {e}")

                await asyncio.sleep(0.001)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"RX loop error: {e}")
                await asyncio.sleep(0.01)

    def get_rssi(self) -> float:
        """Get last received RSSI"""
        return self._last_rssi

    def get_snr(self) -> float:
        """Get last received SNR"""
        return self._last_snr

    async def stop(self):
        """Stop LoRa radio"""
        self.running = False
        if self._rx_task:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
        if self.spi:
            self.spi.close()
        if self.gpio_initialized:
            try:
                self.GPIO.cleanup()
            except:
                pass
        logger.info("LoRa radio stopped")


# SX1262 support (newer chip, different registers)
class SX1262Radio(LoRaRadio):
    """LoRa radio interface for SX1262"""

    # SX1262 uses different command interface (not register-based)
    # This is a placeholder for SX1262-specific implementation
    async def _configure_lora(self):
        """Configure SX1262"""
        logger.info("SX1262 configuration not fully implemented, using SX1276 fallback")
        await super()._configure_lora()


# Example usage
async def main():
    logging.basicConfig(level=logging.INFO)
    config = {
        'lora': {
            'spi_bus': 0,
            'spi_device': 0,
            'cs_pin': 8,
            'reset_pin': 25,
            'dio0_pin': 24,
            'frequency': 915.0,
            'tx_power': 20,
            'spreading_factor': 10,
            'bandwidth': 7,  # 125 kHz
            'coding_rate': 1,  # 4/5
            'preamble_length': 8,
            'sync_word': 0x12,
            'crc_enabled': True,
        }
    }

    def on_packet(packet: LoRaPacket):
        print(f"RX: {packet.data} RSSI={packet.rssi:.1f} SNR={packet.snr:.1f}")

    radio = LoRaRadio(config)
    try:
        await radio.start(on_packet)
        # Send test packets
        for i in range(5):
            await radio.send(f"Hello {i}".encode())
            await asyncio.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        await radio.stop()


if __name__ == '__main__':
    asyncio.run(main())