#!/usr/bin/env python3
"""
SMS Emergency Beacon System
Sends emergency alerts via Twilio, GSM modem, or email-to-SMS gateway
"""

import asyncio
import logging
import os
import json
from typing import Optional, List, Dict
from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)


class Severity(Enum):
    INFO = 1
    WARNING = 2
    CRITICAL = 3


@dataclass
class SMSProvider(ABC):
    """Abstract SMS provider"""
    
    @abstractmethod
    async def send(self, to: str, message: str) -> bool:
        pass
    
    @abstractmethod
    async def test(self) -> bool:
        pass


class TwilioProvider(SMSProvider):
    """Twilio SMS provider"""
    
    def __init__(self, account_sid: str, auth_token: str, from_number: str):
        self.account_sid = account_sid
        self.auth_token = auth_token
        self.from_number = from_number
        self.client = None
        
        try:
            from twilio.rest import Client
            self.client = Client(account_sid, auth_token)
        except ImportError:
            logger.warning("Twilio library not installed. Run: pip install twilio")
    
    async def send(self, to: str, message: str) -> bool:
        if not self.client:
            logger.error("Twilio client not initialized")
            return False
        
        try:
            message_obj = self.client.messages.create(
                body=message,
                from_=self.from_number,
                to=to
            )
            logger.info(f"SMS sent via Twilio to {to}: SID={message_obj.sid}")
            return True
        except Exception as e:
            logger.error(f"Twilio send failed: {e}")
            return False
    
    async def test(self) -> bool:
        if not self.client:
            return False
        try:
            # Check account status
            self.client.api.accounts(self.account_sid).fetch()
            return True
        except:
            return False


class ModemProvider(SMSProvider):
    """GSM/USB modem SMS provider using AT commands"""
    
    def __init__(self, device: str, baudrate: int, apn: str):
        self.device = device
        self.baudrate = baudrate
        self.apn = apn
        self.serial = None
    
    async def connect(self):
        import serial
        try:
            self.serial = serial.Serial(self.device, self.baudrate, timeout=5)
            # Initialize modem
            await self._send_at("AT")
            await self._send_at("AT+CMGF=1")  # Text mode
            await self._send_at(f'AT+CGDCONT=1,"IP","{self.apn}"')
            logger.info(f"Modem connected on {self.device}")
        except Exception as e:
            logger.error(f"Modem connection failed: {e}")
            raise
    
    async def _send_at(self, cmd: str, expected: str = "OK", timeout: float = 5.0) -> str:
        if not self.serial:
            raise RuntimeError("Modem not connected")
        
        self.serial.write(f"{cmd}\r\n".encode())
        await asyncio.sleep(0.5)
        
        response = ""
        start = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start < timeout:
            if self.serial.in_waiting:
                response += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                if expected in response:
                    return response
            await asyncio.sleep(0.1)
        
        if expected not in response:
            raise TimeoutError(f"AT command timeout: {cmd}")
        return response
    
    async def send(self, to: str, message: str) -> bool:
        if not self.serial:
            await self.connect()
        
        try:
            # Set recipient
            await self._send_at(f'AT+CMGS="{to}"', ">")
            # Send message + Ctrl+Z
            self.serial.write(f"{message}\x1a".encode())
            await asyncio.sleep(3)
            
            response = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
            if "OK" in response or "+CMGS:" in response:
                logger.info(f"SMS sent via modem to {to}")
                return True
            else:
                logger.error(f"Modem SMS failed: {response}")
                return False
        except Exception as e:
            logger.error(f"Modem send error: {e}")
            return False
    
    async def test(self) -> bool:
        try:
            await self.connect()
            await self._send_at("AT")
            return True
        except:
            return False


class Email2SMSProvider(SMSProvider):
    """Email-to-SMS gateway provider (carrier-specific)"""
    
    CARRIER_GATEWAYS = {
        'verizon': '@vtext.com',
        'att': '@txt.att.net',
        'tmobile': '@tmomail.net',
        'sprint': '@messaging.sprintpcs.com',
        'google_fi': '@msg.fi.google.com',
        'us_cellular': '@email.uscc.net',
    }
    
    def __init__(self, smtp_host: str, smtp_port: int, username: str, password: str, from_email: str):
        self.smtp_host = smtp_host
        self.smtp_port = smtp_port
        self.username = username
        self.password = password
        self.from_email = from_email
    
    def _get_sms_email(self, phone: str, carrier: str = None) -> str:
        """Convert phone number to email-to-SMS address"""
        # Strip non-digits
        digits = ''.join(filter(str.isdigit, phone))
        if len(digits) == 11 and digits[0] == '1':
            digits = digits[1:]
        elif len(digits) != 10:
            raise ValueError(f"Invalid phone number: {phone}")
        
        if carrier and carrier.lower() in self.CARRIER_GATEWAYS:
            return f"{digits}{self.CARRIER_GATEWAYS[carrier.lower()]}"
        
        # Try all gateways (will send multiple emails)
        return [f"{digits}{gw}" for gw in self.CARRIER_GATEWAYS.values()]
    
    async def send(self, to: str, message: str) -> bool:
        import aiosmtplib
        from email.message import EmailMessage
        
        try:
            # Try to determine carrier or use all
            recipients = self._get_sms_email(to)
            if isinstance(recipients, str):
                recipients = [recipients]
            
            for recipient in recipients:
                msg = EmailMessage()
                msg['From'] = self.from_email
                msg['To'] = recipient
                msg['Subject'] = ""
                msg.set_content(message)
                
                await aiosmtplib.send(
                    msg,
                    hostname=self.smtp_host,
                    port=self.smtp_port,
                    username=self.username,
                    password=self.password,
                    start_tls=True
                )
            
            logger.info(f"SMS sent via email gateway to {to}")
            return True
        except Exception as e:
            logger.error(f"Email-to-SMS failed: {e}")
            return False
    
    async def test(self) -> bool:
        try:
            import aiosmtplib
            await aiosmtplib.connect(hostname=self.smtp_host, port=self.smtp_port, start_tls=True)
            return True
        except:
            return False


class SMSBeacon:
    def __init__(self, config: dict, mavlink_bridge=None):
        self.config = config
        self.mavlink_bridge = mavlink_bridge
        self.sms_config = config.get('failsafe', {}).get('sms', {})
        
        self.enabled = self.sms_config.get('enabled', True)
        self.provider_name = self.sms_config.get('provider', 'twilio')
        self.recipients = self.sms_config.get('recipients', [])
        self.templates = self.sms_config.get('templates', {})
        
        self.provider: Optional[SMSProvider] = None
        self.drone_id = config.get('drone', {}).get('id', 'drone-001')
        self.running = False
        
        self._init_provider()
    
    def _init_provider(self):
        """Initialize SMS provider based on config"""
        if self.provider_name == 'twilio':
            account_sid = os.environ.get(self.sms_config.get('twilio', {}).get('account_sid_env', 'TWILIO_ACCOUNT_SID'))
            auth_token = os.environ.get(self.sms_config.get('twilio', {}).get('auth_token_env', 'TWILIO_AUTH_TOKEN'))
            from_number = os.environ.get(self.sms_config.get('twilio', {}).get('from_number_env', 'TWILIO_FROM_NUMBER'))
            
            if all([account_sid, auth_token, from_number]):
                self.provider = TwilioProvider(account_sid, auth_token, from_number)
            else:
                logger.warning("Twilio credentials not fully configured")
        
        elif self.provider_name == 'modem':
            modem_config = self.sms_config.get('modem', {})
            device = modem_config.get('device', '/dev/ttyUSB0')
            baudrate = modem_config.get('baudrate', 115200)
            apn = modem_config.get('apn', 'internet')
            self.provider = ModemProvider(device, baudrate, apn)
        
        elif self.provider_name == 'email2sms':
            # Would need SMTP config
            logger.warning("Email2SMS provider not fully implemented")
    
    async def start(self):
        """Start SMS beacon system"""
        if not self.enabled:
            logger.info("SMS beacon disabled")
            return
        
        if not self.provider:
            logger.warning("No SMS provider configured")
            return
        
        # Test provider
        if await self.provider.test():
            logger.info(f"SMS provider ({self.provider_name}) connected and tested")
        else:
            logger.warning(f"SMS provider ({self.provider_name}) test failed")
        
        self.running = True
        logger.info("SMS emergency beacon active")
    
    async def stop(self):
        """Stop SMS beacon"""
        self.running = False
        logger.info("SMS emergency beacon stopped")
    
    async def send_emergency_beacon(self, reason: str, message: str, severity: int = 3):
        """Send emergency beacon to all recipients"""
        if not self.enabled or not self.provider or not self.recipients:
            return
        
        # Get current position from MAVLink bridge
        position = {'lat': 0, 'lon': 0, 'alt': 0}
        if self.mavlink_bridge:
            status = self.mavlink_bridge.get_connection_status()
            # Would need to get actual position from telemetry
        
        # Select template based on severity
        if severity >= Severity.CRITICAL.value:
            template = self.templates.get('critical', '')
        elif severity >= Severity.WARNING.value:
            template = self.templates.get('warning', '')
        else:
            template = self.templates.get('info', '')
        
        # Format message
        formatted_message = template.format(
            drone_id=self.drone_id,
            lat=position['lat'],
            lon=position['lon'],
            alt=position['alt'],
            reason=reason,
            message=message
        )
        
        # Send to all recipients
        for recipient in self.recipients:
            try:
                await self.provider.send(recipient, formatted_message)
            except Exception as e:
                logger.error(f"Failed to send SMS to {recipient}: {e}")
    
    async def send_custom_sms(self, to: str, message: str) -> bool:
        """Send custom SMS to specific number"""
        if not self.enabled or not self.provider:
            return False
        
        return await self.provider.send(to, message)
    
    def get_status(self) -> dict:
        """Get SMS beacon status"""
        return {
            'enabled': self.enabled,
            'provider': self.provider_name,
            'provider_connected': self.provider is not None,
            'recipients': len(self.recipients),
            'running': self.running
        }