#!/usr/bin/env python3
"""
LoRa Security Module
Provides authentication, encryption, and replay protection for LoRa control
"""

import os
import struct
import time
import hmac
import hashlib
import logging
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding

logger = logging.getLogger(__name__)


class LoRaSecurityLevel(Enum):
    """Security level for LoRa communication"""
    NONE = 0          # No security (legacy)
    AUTH_ONLY = 1     # Authentication only (HMAC)
    ENCRYPTED = 2     # Authentication + encryption (AES-128-CTR)
    FULL = 3          # Full security with device binding


@dataclass
class LoRaSecurityConfig:
    """LoRa security configuration"""
    level: LoRaSecurityLevel = LoRaSecurityLevel.AUTH_ONLY
    pre_shared_key: bytes = b''           # 32 bytes for HMAC-SHA256
    encryption_key: bytes = b''            # 16 bytes for AES-128
    device_id: int = 0                     # Unique device identifier (0 = any)
    allowed_device_ids: Tuple[int, ...] = ()  # Whitelist of allowed device IDs
    sequence_window: int = 64              # Replay protection window size
    timestamp_tolerance: int = 30          # Seconds of timestamp tolerance
    require_timestamp: bool = True         # Require valid timestamp


class LoRaAuthError(Exception):
    """Authentication failure"""
    pass


class LoRaReplayError(Exception):
    """Replay attack detected"""
    pass


class LoRaTimestampError(Exception):
    """Timestamp validation failure"""
    pass


class LoRaEncryptionError(Exception):
    """Encryption/decryption failure"""
    pass


@dataclass
class SecurityContext:
    """Security state for a LoRa session"""
    config: LoRaSecurityConfig
    rx_sequences: Dict[int, int] = field(default_factory=dict)  # device_id -> last sequence
    tx_sequence: int = 0
    last_rx_time: Dict[int, float] = field(default_factory=dict)
    failed_attempts: int = 0
    last_failed_time: float = 0

    def check_sequence(self, device_id: int, sequence: int) -> bool:
        """Check if sequence number is valid (not replayed)"""
        last_seq = self.rx_sequences.get(device_id, 0)
        if sequence <= last_seq:
            return False  # Replay or old message
        if sequence - last_seq > self.config.sequence_window:
            return False  # Too far ahead (possible attack)
        return True

    def update_sequence(self, device_id: int, sequence: int):
        """Update last seen sequence for device"""
        self.rx_sequences[device_id] = sequence
        self.last_rx_time[device_id] = time.time()

    def check_timestamp(self, timestamp: int) -> bool:
        """Check if timestamp is within tolerance"""
        if not self.config.require_timestamp:
            return True
        now = int(time.time())
        return abs(now - timestamp) <= self.config.timestamp_tolerance

    def check_device_id(self, device_id: int) -> bool:
        """Check if device ID is allowed"""
        if self.config.device_id == 0:
            return True  # No device binding
        if self.config.allowed_device_ids:
            return device_id in self.config.allowed_device_ids
        return device_id == self.config.device_id


class LoRaSecurity:
    """LoRa security handler"""

    HMAC_SIZE = 32  # SHA256
    NONCE_SIZE = 16  # For AES-CTR

    def __init__(self, config: LoRaSecurityConfig):
        self.config = config
        self.context = SecurityContext(config)

    def authenticate_frame(self, frame_data: bytes, device_id: int = 0) -> bytes:
        """Add HMAC authentication to frame"""
        if self.config.level == LoRaSecurityLevel.NONE:
            return frame_data

        if not self.config.pre_shared_key:
            logger.warning("No pre-shared key configured, skipping authentication")
            return frame_data

        # Include device_id in authenticated data
        auth_data = struct.pack('>I', device_id) + frame_data
        hmac_tag = hmac.new(
            self.config.pre_shared_key,
            auth_data,
            hashlib.sha256
        ).digest()

        return frame_data + hmac_tag

    def verify_frame(self, data: bytes, device_id: int = 0) -> Tuple[bytes, bool]:
        """Verify HMAC and return payload if valid"""
        if self.config.level == LoRaSecurityLevel.NONE:
            return data, True

        if not self.config.pre_shared_key:
            logger.warning("No pre-shared key configured, skipping verification")
            return data, True

        if len(data) < self.HMAC_SIZE:
            raise LoRaAuthError("Frame too short for HMAC")

        payload = data[:-self.HMAC_SIZE]
        received_hmac = data[-self.HMAC_SIZE:]

        # Verify HMAC
        auth_data = struct.pack('>I', device_id) + payload
        expected_hmac = hmac.new(
            self.config.pre_shared_key,
            auth_data,
            hashlib.sha256
        ).digest()

        if not hmac.compare_digest(received_hmac, expected_hmac):
            self.context.failed_attempts += 1
            self.context.last_failed_time = time.time()
            raise LoRaAuthError("HMAC verification failed")

        return payload, True

    def encrypt_payload(self, payload: bytes, sequence: int) -> bytes:
        """Encrypt payload with AES-128-CTR"""
        if self.config.level.value < LoRaSecurityLevel.ENCRYPTED.value:
            return payload

        if not self.config.encryption_key:
            logger.warning("No encryption key configured, skipping encryption")
            return payload

        # Generate nonce from sequence (8 bytes) + random (8 bytes)
        nonce = struct.pack('>Q', sequence) + os.urandom(8)

        cipher = Cipher(
            algorithms.AES(self.config.encryption_key),
            modes.CTR(nonce)
        )
        encryptor = cipher.encryptor()
        ciphertext = encryptor.update(payload) + encryptor.finalize()

        return nonce + ciphertext

    def decrypt_payload(self, data: bytes) -> Tuple[bytes, int]:
        """Decrypt payload and return (plaintext, sequence)"""
        if self.config.level.value < LoRaSecurityLevel.ENCRYPTED.value:
            return data, 0

        if not self.config.encryption_key:
            logger.warning("No encryption key configured, skipping decryption")
            return data, 0

        if len(data) < self.NONCE_SIZE:
            raise LoRaEncryptionError("Data too short for nonce")

        nonce = data[:self.NONCE_SIZE]
        ciphertext = data[self.NONCE_SIZE:]

        # Extract sequence from nonce
        sequence = struct.unpack('>Q', nonce[:8])[0]

        cipher = Cipher(
            algorithms.AES(self.config.encryption_key),
            modes.CTR(nonce)
        )
        decryptor = cipher.decryptor()
        plaintext = decryptor.update(ciphertext) + decryptor.finalize()

        return plaintext, sequence

    def secure_encode(self, frame_data: bytes, device_id: int = 0, sequence: int = 0) -> bytes:
        """Apply full security: encrypt then authenticate"""
        if self.config.level == LoRaSecurityLevel.NONE:
            return frame_data

        # Encrypt if enabled
        if self.config.level.value >= LoRaSecurityLevel.ENCRYPTED.value:
            frame_data = self.encrypt_payload(frame_data, sequence)

        # Authenticate
        secured = self.authenticate_frame(frame_data, device_id)
        return secured

    def secure_decode(self, data: bytes, device_id: int = 0) -> Tuple[bytes, int, bool]:
        """Apply full security: verify then decrypt"""
        if self.config.level == LoRaSecurityLevel.NONE:
            return data, 0, True

        # Verify authentication
        payload, _ = self.verify_frame(data, device_id)

        # Decrypt if enabled
        sequence = 0
        if self.config.level.value >= LoRaSecurityLevel.ENCRYPTED.value:
            payload, sequence = self.decrypt_payload(payload)

        return payload, sequence, True

    def validate_received(self, device_id: int, sequence: int, timestamp: int) -> bool:
        """Validate received frame against security policies"""
        # Check device ID
        if not self.context.check_device_id(device_id):
            raise LoRaAuthError(f"Device ID {device_id} not authorized")

        # Check sequence (replay protection)
        if not self.context.check_sequence(device_id, sequence):
            raise LoRaReplayError(f"Invalid sequence {sequence} from device {device_id}")

        # Check timestamp
        if not self.context.check_timestamp(timestamp):
            raise LoRaTimestampError(f"Timestamp {timestamp} out of tolerance")

        # All checks passed
        self.context.update_sequence(device_id, sequence)
        return True


def generate_keys() -> Dict[str, bytes]:
    """Generate secure random keys for LoRa security"""
    return {
        'pre_shared_key': os.urandom(32),  # HMAC-SHA256
        'encryption_key': os.urandom(16),  # AES-128
    }


def keys_to_hex(keys: Dict[str, bytes]) -> Dict[str, str]:
    """Convert binary keys to hex strings for config storage"""
    return {k: v.hex() for k, v in keys.items()}


def keys_from_hex(keys: Dict[str, str]) -> Dict[str, bytes]:
    """Convert hex strings back to binary keys"""
    return {k: bytes.fromhex(v) for k, v in keys.items()}


# Security level helpers
def get_security_level_name(level: LoRaSecurityLevel) -> str:
    """Get human-readable security level name"""
    names = {
        LoRaSecurityLevel.NONE: "None",
        LoRaSecurityLevel.AUTH_ONLY: "Authentication Only",
        LoRaSecurityLevel.ENCRYPTED: "Encrypted",
        LoRaSecurityLevel.FULL: "Full Security"
    }
    return names.get(level, "Unknown")


if __name__ == "__main__":
    def demo_key_generation():
        """Demo function to generate keys"""
        keys = generate_keys()
        print("Generated keys (hex format):")
        hex_keys = keys_to_hex(keys)
        for k, v in hex_keys.items():
            print(f"  {k}: {v}")
        return keys

    # Example usage
    def main():
        """Main function for demo"""
        keys = demo_key_generation()
        print("\nDemo of secure encoding and decoding:")

        # Configure security
        config = LoRaSecurityConfig(
            level=LoRaSecurityLevel.ENCRYPTED,
            pre_shared_key=keys['pre_shared_key'],
            encryption_key=keys['encryption_key'],
            device_id=12345,
            timestamp_tolerance=15
        )

        security = LoRaSecurity(config)
        protocol = LoRaProtocol(security, device_id=12345)

        # Encode a simple message
        message_data = b"test payload"
        encoded = protocol.encode_command(
            LoRaMessageType.CMD_TAKEOFF,
            message_data,
            device_id=12345
        )

        print(f"Encoded message (len={len(encoded)}): {encoded.hex()}")

        # Decode the message
        decoded_frame = protocol.decode_frame(encoded, device_id=12345)
        if decoded_frame:
            print(f"Decoded: device_id={decoded_frame.device_id}, type={decoded_frame.msg_type}")
        else:
            print("Failed to decode - may be security issue")

        # Show key management functions available
        print("\nKey management functions available:")
        print("  - generate_keys() -> dict with pre_shared_key and encryption_key")
        print("  - keys_to_hex() -> hex strings for storage")
        print("  - keys_from_hex() -> binary keys from hex")

    main()