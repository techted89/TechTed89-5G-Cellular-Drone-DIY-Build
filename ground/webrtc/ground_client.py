#!/usr/bin/env python3
"""
Ground Station WebRTC Client
Receives ultra-low latency video and telemetry from drone via WebRTC
Sends control commands via data channel
"""

import asyncio
import logging
import json
import uuid
from typing import Dict, Optional, Callable, Any, Set
from dataclasses import dataclass, field
from datetime import datetime

from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaRecorder, MediaPlayer
import fractions

logger = logging.getLogger(__name__)


@dataclass
class TelemetryData:
    """Parsed telemetry data from drone"""
    timestamp: float = 0
    position: Dict = field(default_factory=dict)
    attitude: Dict = field(default_factory=dict)
    battery: Dict = field(default_factory=dict)
    heartbeat: Dict = field(default_factory=dict)
    rc_channels: Dict = field(default_factory=dict)
    vfr: Dict = field(default_factory=dict)
    armed: bool = False
    flight_mode: int = 0
    failsafe_active: bool = False
    failsafe_reason: str = ""
    signal: Dict = field(default_factory=dict)


class GroundStationClient:
    """
    Ground Station WebRTC Client
    Connects to drone SBC via WebRTC for video, telemetry, and control
    """
    
    def __init__(self, config: dict, signaling_url: str = None):
        self.config = config
        self.network_config = config.get('network', {}).get('ground_station', {})
        self.webrtc_config = config.get('network', {}).get('webrtc', {})
        self.video_config = config.get('video', {}).get('webrtc_video', {})
        
        # Signaling
        self.signaling_url = signaling_url or f"http://localhost:{self.network_config.get('webrtc_port', 8080)}"
        
        # ICE servers
        self.ice_servers = []
        for stun in self.webrtc_config.get('stun_servers', []):
            self.ice_servers.append(RTCIceServer(urls=[stun]))
        
        # Peer connection
        self.pc: Optional[RTCPeerConnection] = None
        self.data_channel = None
        self.video_track = None
        
        # State
        self.connected = False
        self.client_id = str(uuid.uuid4())
        self.telemetry = TelemetryData()
        
        # Callbacks
        self.on_telemetry: Optional[Callable] = None
        self.on_video_frame: Optional[Callable] = None
        self.on_connection_state_change: Optional[Callable] = None
        self.on_message: Optional[Callable] = None
        
        # Recording
        self.recorder: Optional[MediaRecorder] = None
        self.recording = False
        
        # Telemetry history
        self.telemetry_history: list = []
        self.max_history = 1000
    
    async def connect(self, drone_id: str = None) -> bool:
        """Connect to drone via WebRTC"""
        logger.info(f"Connecting to drone via {self.signaling_url}...")
        
        # Create peer connection
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=self.ice_servers))
        
        # Set up event handlers
        self._setup_handlers()
        
        # Create data channel for control/telemetry
        self.data_channel = self.pc.createDataChannel(
            "control",
            ordered=True,
            maxRetransmits=0
        )
        self._setup_data_channel()
        
        # Create offer
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        
        # Send offer via HTTP signaling
        success = await self._send_offer(offer)
        if not success:
            await self.disconnect()
            return False
        
        return True
    
    def _setup_handlers(self):
        """Set up peer connection event handlers"""
        
        @self.pc.on("connectionstatechange")
        async def on_connection_state_change():
            state = self.pc.connectionState
            logger.info(f"Connection state: {state}")
            self.connected = (state == "connected")
            
            if self.on_connection_state_change:
                await self.on_connection_state_change(state)
            
            if state in ("failed", "closed", "disconnected"):
                await self._cleanup()
        
        @self.pc.on("track")
        def on_track(track):
            logger.info(f"Received track: {track.kind}")
            if track.kind == "video":
                self.video_track = track
                asyncio.create_task(self._handle_video_track(track))
        
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            logger.info(f"Received data channel: {channel.label}")
            self.data_channel = channel
            self._setup_data_channel()
    
    def _setup_data_channel(self):
        """Set up data channel handlers"""
        if not self.data_channel:
            return
        
        @self.data_channel.on("message")
        def on_message(message):
            asyncio.create_task(self._handle_data_channel_message(message))
        
        @self.data_channel.on("open")
        def on_open():
            logger.info("Data channel opened")
            # Request telemetry stream
            self.send_message({'type': 'telemetry_request'})
        
        @self.data_channel.on("close")
        def on_close():
            logger.info("Data channel closed")
    
    async def _handle_data_channel_message(self, message):
        """Handle incoming data channel message"""
        try:
            if isinstance(message, str):
                data = json.loads(message)
            else:
                data = message
            
            msg_type = data.get('type')
            
            if msg_type == 'telemetry':
                await self._handle_telemetry(data.get('data', {}))
            elif msg_type == 'pong':
                # Latency measurement
                pass
            elif msg_type == 'command_ack':
                logger.debug(f"Command ack: {data}")
            
            if self.on_message:
                await self.on_message(data)
                
        except Exception as e:
            logger.error(f"Data channel message error: {e}")
    
    async def _handle_telemetry(self, telemetry_data: dict):
        """Process incoming telemetry"""
        self.telemetry.timestamp = telemetry_data.get('timestamp_us', 0) / 1_000_000
        
        # Update telemetry fields
        for key in ['position', 'attitude', 'battery', 'heartbeat', 'rc_channels', 'vfr']:
            if key in telemetry_data:
                setattr(self.telemetry, key, telemetry_data[key])
        
        self.telemetry.armed = telemetry_data.get('armed', False)
        self.telemetry.flight_mode = telemetry_data.get('flight_mode', 0)
        self.telemetry.failsafe_active = telemetry_data.get('failsafe_active', False)
        self.telemetry.failsafe_reason = telemetry_data.get('failsafe_reason', '')
        self.telemetry.signal = telemetry_data.get('signal', {})
        
        # Add to history
        self.telemetry_history.append({
            'timestamp': self.telemetry.timestamp,
            'data': telemetry_data
        })
        
        # Trim history
        if len(self.telemetry_history) > self.max_history:
            self.telemetry_history = self.telemetry_history[-self.max_history:]
        
        # Callback
        if self.on_telemetry:
            await self.on_telemetry(self.telemetry)
    
    async def _handle_video_track(self, track):
        """Handle incoming video track"""
        logger.info("Video track started")
        
        while True:
            try:
                frame = await track.recv()
                if frame is None:
                    break
                
                # Convert to numpy array for processing
                # frame is an aiortc VideoFrame
                if self.on_video_frame:
                    await self.on_video_frame(frame)
                    
            except Exception as e:
                logger.error(f"Video frame error: {e}")
                break
        
        logger.info("Video track ended")
    
    async def _send_offer(self, offer: RTCSessionDescription) -> bool:
        """Send offer to drone via HTTP signaling"""
        import aiohttp
        
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.signaling_url}/offer",
                    json={
                        'sdp': offer.sdp,
                        'type': offer.type,
                        'client_id': self.client_id
                    },
                    timeout=aiohttp.ClientTimeout(total=10)
                ) as resp:
                    if resp.status == 200:
                        answer = await resp.json()
                        await self.pc.setRemoteDescription(
                            RTCSessionDescription(sdp=answer['sdp'], type=answer['type'])
                        )
                        logger.info("WebRTC connection established")
                        return True
                    else:
                        error = await resp.text()
                        logger.error(f"Offer failed: {resp.status} - {error}")
                        return False
        except Exception as e:
            logger.error(f"Signaling error: {e}")
            return False
    
    def send_message(self, message: dict):
        """Send message via data channel"""
        if self.data_channel and self.data_channel.readyState == "open":
            try:
                self.data_channel.send(json.dumps(message))
            except Exception as e:
                logger.error(f"Send message error: {e}")
    
    def send_control(self, command: str, params: dict = None):
        """Send control command to drone"""
        self.send_message({
            'type': 'control',
            'command': command,
            'params': params or {}
        })
    
    # Convenience control methods
    def arm(self, arm: bool = True):
        self.send_control('arm', {'arm': arm})
    
    def disarm(self):
        self.send_control('disarm')
    
    def rtl(self):
        self.send_control('rtl')
    
    def land(self):
        self.send_control('land')
    
    def takeoff(self, altitude: float = 10.0):
        self.send_control('takeoff', {'altitude': altitude})
    
    def set_mode(self, mode: int, custom_mode: int = 0):
        self.send_control('set_mode', {'mode': mode, 'custom_mode': custom_mode})
    
    def manual_control(self, x: int, y: int, z: int, r: int, buttons: int = 0):
        self.send_control('manual_control', {'x': x, 'y': y, 'z': z, 'r': r, 'buttons': buttons})
    
    def rc_override(self, channels: list):
        self.send_control('rc_override', {'channels': channels})
    
    async def start_recording(self, filename: str):
        """Start recording video"""
        if self.video_track and not self.recording:
            self.recorder = MediaRecorder(filename)
            self.recorder.addTrack(self.video_track)
            await self.recorder.start()
            self.recording = True
            logger.info(f"Recording started: {filename}")
    
    async def stop_recording(self):
        """Stop recording video"""
        if self.recorder and self.recording:
            await self.recorder.stop()
            self.recording = False
            logger.info("Recording stopped")
    
    async def disconnect(self):
        """Disconnect from drone"""
        await self._cleanup()
    
    async def _cleanup(self):
        """Clean up connection"""
        if self.recorder and self.recording:
            await self.recorder.stop()
        
        if self.pc:
            await self.pc.close()
            self.pc = None
        
        self.connected = False
        self.data_channel = None
        self.video_track = None
    
    def get_telemetry(self) -> TelemetryData:
        """Get latest telemetry"""
        return self.telemetry
    
    def get_telemetry_history(self, duration: float = 60) -> list:
        """Get telemetry history for last N seconds"""
        cutoff = time.time() - duration
        return [t for t in self.telemetry_history if t['timestamp'] > cutoff]
    
    def get_status(self) -> dict:
        """Get client status"""
        return {
            'connected': self.connected,
            'client_id': self.client_id,
            'signaling_url': self.signaling_url,
            'recording': self.recording,
            'telemetry_age': time.time() - self.telemetry.timestamp if self.telemetry.timestamp > 0 else None,
            'history_size': len(self.telemetry_history)
        }


# Import time for telemetry history
import time