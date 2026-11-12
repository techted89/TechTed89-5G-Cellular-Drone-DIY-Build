#!/usr/bin/env python3
"""
WebRTC Server for Ultra-Low Latency Video Streaming
Uses aiortc for WebRTC implementation with GStreamer for hardware-accelerated encoding
"""

import asyncio
import logging
import json
import uuid
from typing import Dict, Optional, Set, Callable
from dataclasses import dataclass, field

from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.mediastreams import MediaStreamTrack
import fractions

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)

@dataclass
class WebRTCClient:
    peer_connection: RTCPeerConnection
    client_id: str
    data_channel = None
    video_track = None
    connected: bool = False
    last_activity: float = 0
    metadata: dict = field(default_factory=dict)


class VideoStreamTrack(MediaStreamTrack):
    """Custom video track that reads from GStreamer pipeline"""
    kind = "video"
    
    def __init__(self, config: dict, mavlink_bridge=None):
        super().__init__()
        self.config = config
        self.mavlink_bridge = mavlink_bridge
        self.video_config = config.get('video', {})
        self.camera_config = self.video_config.get('camera', {})
        self.webrtc_config = self.video_config.get('webrtc_video', {})
        
        self.player = None
        self.relay = None
        self._running = False
        self._frame_callback = None
    
    async def start(self):
        """Start the video pipeline"""
        self._running = True
        await self._create_pipeline()
    
    async def _create_pipeline(self):
        """Create GStreamer pipeline for hardware-accelerated video"""
        device = self.camera_config.get('device', '/dev/video0')
        width = self.camera_config.get('width', 1280)
        height = self.camera_config.get('height', 720)
        fps = self.camera_config.get('fps', 30)
        format_ = self.camera_config.get('format', 'YUYV')
        bitrate = self.camera_config.get('bitrate', 4000000)
        codec = self.camera_config.get('codec', 'H264')
        encoder = self.camera_config.get('encoder', 'v4l2h264enc')
        keyframe_interval = self.camera_config.get('keyframe_interval', 30)
        
        # Build GStreamer pipeline
        if codec == 'H264':
            if 'jetson' in encoder or 'nv' in encoder:
                # Jetson hardware encoder
                pipeline = (
                    f"v4l2src device={device} ! "
                    f"video/x-raw,width={width},height={height},framerate={fps}/1,format={format_} ! "
                    f"nvvidconv ! "
                    f"video/x-raw(memory:NVMM),format=I420 ! "
                    f"{encoder} bitrate={bitrate} iframeinterval={keyframe_interval} ! "
                    f"h264parse ! "
                    f"rtph264pay config-interval=1 pt=96 ! "
                    f"appsink name=sink emit-signals=true max-buffers=1 drop=true"
                )
            else:
                # V4L2 H264 encoder (Pi, etc.)
                pipeline = (
                    f"v4l2src device={device} ! "
                    f"video/x-raw,width={width},height={height},framerate={fps}/1,format={format_} ! "
                    f"v4l2h264enc extra-controls=\"encode,bitrate={bitrate},iframeperiod={keyframe_interval}\" ! "
                    f"h264parse ! "
                    f"rtph264pay config-interval=1 pt=96 ! "
                    f"appsink name=sink emit-signals=true max-buffers=1 drop=true"
                )
        elif codec == 'VP8':
            pipeline = (
                f"v4l2src device={device} ! "
                f"video/x-raw,width={width},height={height},framerate={fps}/1,format={format_} ! "
                f"vp8enc target-bitrate={bitrate} keyframe-max-dist={keyframe_interval} ! "
                f"rtpvp8pay pt=96 ! "
                f"appsink name=sink emit-signals=true max-buffers=1 drop=true"
            )
        else:
            # Raw fallback
            pipeline = (
                f"v4l2src device={device} ! "
                f"video/x-raw,width={width},height={height},framerate={fps}/1,format={format_} ! "
                f"appsink name=sink emit-signals=true max-buffers=1 drop=true"
            )
        
        try:
            self.player = MediaPlayer(pipeline, format="gstreamer")
            self.relay = MediaRelay()
            logger.info(f"GStreamer pipeline created: {pipeline[:100]}...")
        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            # Fallback to synthetic video for testing
            self.player = MediaPlayer("smpte", format="lavfi", options={"size": f"{width}x{height}", "rate": str(fps)})
            self.relay = MediaRelay()
    
    async def recv(self):
        """Receive next frame"""
        if not self.player or not self.player.video:
            await asyncio.sleep(1/30)
            return None
        
        try:
            frame = await self.player.video.recv()
            if self._frame_callback:
                self._frame_callback(frame)
            return frame
        except Exception as e:
            logger.debug(f"Frame receive error: {e}")
            await asyncio.sleep(1/30)
            return None
    
    def set_frame_callback(self, callback: Callable):
        self._frame_callback = callback
    
    async def stop(self):
        self._running = False
        if self.player:
            self.player.close()


class WebRTCServer:
    def __init__(self, config: dict, mavlink_bridge=None):
        self.config = config
        self.mavlink_bridge = mavlink_bridge
        self.webrtc_config = config.get('network', {}).get('webrtc', {})
        self.network_config = config.get('network', {}).get('ground_station', {})
        
        # ICE servers
        self.ice_servers = []
        for stun in self.webrtc_config.get('stun_servers', []):
            self.ice_servers.append(RTCIceServer(urls=[stun]))
        
        # Clients
        self.clients: Dict[str, WebRTCClient] = {}
        self.video_track: Optional[VideoStreamTrack] = None
        
        # Telemetry broadcast
        self.telemetry_subscribers: Set[str] = set()
        
        # Server
        self.server = None
        self.port = self.network_config.get('webrtc_port', 8080)
    
    async def start(self):
        """Start WebRTC server"""
        logger.info(f"Starting WebRTC server on port {self.port}")
        
        # Initialize video track
        self.video_track = VideoStreamTrack(self.config, self.mavlink_bridge)
        await self.video_track.start()
        
        # Start HTTP/WebSocket server for signaling
        from aiohttp import web
        
        app = web.Application()
        app.router.add_post('/offer', self._handle_offer)
        app.router.add_get('/health', self._handle_health)
        app.router.add_get('/clients', self._handle_clients)
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.port)
        await site.start()
        
        self.server = runner
        logger.info(f"WebRTC signaling server started on port {self.port}")
    
    async def stop(self):
        """Stop WebRTC server"""
        logger.info("Stopping WebRTC server...")
        
        # Close all peer connections
        for client in self.clients.values():
            await client.peer_connection.close()
        self.clients.clear()
        
        # Stop video track
        if self.video_track:
            await self.video_track.stop()
        
        # Stop server
        if self.server:
            await self.server.cleanup()
        
        logger.info("WebRTC server stopped")
    
    async def _handle_offer(self, request):
        """Handle WebRTC offer from client"""
        from aiohttp import web
        
        try:
            data = await request.json()
            sdp = data.get('sdp')
            client_id = data.get('client_id', str(uuid.uuid4()))
            
            # Create peer connection
            pc = RTCPeerConnection(RTCConfiguration(iceServers=self.ice_servers))
            client = WebRTCClient(peer_connection=pc, client_id=client_id)
            self.clients[client_id] = client
            
            # Set up event handlers
            @pc.on("connectionstatechange")
            async def on_connection_state_change():
                logger.info(f"Client {client_id} connection state: {pc.connectionState}")
                client.connected = pc.connectionState == "connected"
                if pc.connectionState == "connected":
                    self.telemetry_subscribers.add(client_id)
                else:
                    self.telemetry_subscribers.discard(client_id)
                    
                    if pc.connectionState in ("failed", "closed", "disconnected"):
                        await self._cleanup_client(client_id)
            
            @pc.on("datachannel")
            def on_datachannel(channel):
                logger.info(f"Data channel opened: {channel.label}")
                client.data_channel = channel
                
                if self.mavlink_bridge:
                    self.mavlink_bridge.register_ground_channel(client_id, channel)
                
                @channel.on("message")
                def on_message(message):
                    asyncio.create_task(self._handle_data_channel_message(client_id, message))
                
                @channel.on("close")
                def on_close():
                    logger.info(f"Data channel closed: {channel.label}")
                    if self.mavlink_bridge:
                        self.mavlink_bridge.unregister_ground_channel(client_id)
            
            @pc.on("track")
            def on_track(track):
                logger.info(f"Track received: {track.kind}")
                if track.kind == "video":
                    # Client wants to receive video
                    if self.video_track:
                        pc.addTrack(self.video_track)
            
            # Set remote description
            offer = RTCSessionDescription(sdp=sdp, type="offer")
            await pc.setRemoteDescription(offer)
            
            # Create answer
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            
            # Register with MAVLink bridge for telemetry forwarding
            if self.mavlink_bridge and client.data_channel:
                self.mavlink_bridge.register_ground_channel(client_id, client.data_channel)
            
            return web.json_response({
                'sdp': pc.localDescription.sdp,
                'type': pc.localDescription.type,
                'client_id': client_id
            })
            
        except Exception as e:
            logger.error(f"Offer handling error: {e}")
            return web.json_response({'error': str(e)}, status=500)
    
    async def _handle_data_channel_message(self, client_id: str, message):
        """Handle message from ground station data channel"""
        try:
            if isinstance(message, str):
                data = json.loads(message)
            else:
                data = message
            
            msg_type = data.get('type')
            
            if msg_type == 'control':
                await self._handle_control_command(client_id, data)
            elif msg_type == 'ping':
                if client_id in self.clients and self.clients[client_id].data_channel:
                    self.clients[client_id].data_channel.send(json.dumps({'type': 'pong'}))
            elif msg_type == 'telemetry_request':
                self.telemetry_subscribers.add(client_id)
            
        except Exception as e:
            logger.error(f"Data channel message error: {e}")
    
    async def _handle_control_command(self, client_id: str, data: dict):
        """Handle control command from ground station"""
        if not self.mavlink_bridge:
            return
        
        command = data.get('command')
        params = data.get('params', {})
        
        try:
            if command == 'arm':
                await self.mavlink_bridge.arm(params.get('arm', True))
            elif command == 'disarm':
                await self.mavlink_bridge.arm(False)
            elif command == 'rtl':
                await self.mavlink_bridge.trigger_rtl()
            elif command == 'land':
                await self.mavlink_bridge.land()
            elif command == 'takeoff':
                await self.mavlink_bridge.takeoff(params.get('altitude', 10.0))
            elif command == 'set_mode':
                await self.mavlink_bridge.set_mode(params.get('mode', 0), params.get('custom_mode', 0))
            elif command == 'manual_control':
                await self.mavlink_bridge.send_manual_control(
                    params.get('x', 0), params.get('y', 0),
                    params.get('z', 0), params.get('r', 0),
                    params.get('buttons', 0)
                )
            elif command == 'rc_override':
                await self.mavlink_bridge.send_rc_channels_override(params.get('channels', []))
            elif command == 'rth_config':
                # Update RTH config (would need implementation)
                pass
            
            # Send ack
            if client_id in self.clients and self.clients[client_id].data_channel:
                self.clients[client_id].data_channel.send(json.dumps({
                    'type': 'command_ack',
                    'command': command,
                    'success': True
                }))
                
        except Exception as e:
            logger.error(f"Control command error: {e}")
            if client_id in self.clients and self.clients[client_id].data_channel:
                self.clients[client_id].data_channel.send(json.dumps({
                    'type': 'command_ack',
                    'command': command,
                    'success': False,
                    'error': str(e)
                }))
    
    async def _cleanup_client(self, client_id: str):
        """Clean up disconnected client"""
        if client_id in self.clients:
            client = self.clients[client_id]
            if client.data_channel and self.mavlink_bridge:
                self.mavlink_bridge.unregister_ground_channel(client_id)
            try:
                await client.peer_connection.close()
            except:
                pass
            del self.clients[client_id]
            self.telemetry_subscribers.discard(client_id)
            logger.info(f"Client cleaned up: {client_id}")
    
    async def broadcast_telemetry(self, telemetry: dict):
        """Broadcast telemetry to all connected ground stations"""
        if not self.telemetry_subscribers:
            return
        
        message = json.dumps({
            'type': 'telemetry',
            'data': telemetry,
            'timestamp': asyncio.get_event_loop().time()
        })
        
        dead_clients = []
        for client_id in self.telemetry_subscribers:
            if client_id in self.clients:
                client = self.clients[client_id]
                if client.data_channel:
                    try:
                        client.data_channel.send(message)
                    except:
                        dead_clients.append(client_id)
        
        for cid in dead_clients:
            await self._cleanup_client(cid)
    
    async def _handle_health(self, request):
        from aiohttp import web
        return web.json_response({
            'status': 'healthy',
            'clients': len(self.clients),
            'connected_clients': sum(1 for c in self.clients.values() if c.connected)
        })
    
    async def _handle_clients(self, request):
        from aiohttp import web
        return web.json_response({
            'clients': [
                {
                    'id': c.client_id,
                    'connected': c.connected,
                    'has_video': c.video_track is not None,
                    'has_data_channel': c.data_channel is not None
                }
                for c in self.clients.values()
            ]
        })
    
    def get_status(self) -> dict:
        """Get server status"""
        return {
            'port': self.port,
            'clients': len(self.clients),
            'connected_clients': sum(1 for c in self.clients.values() if c.connected),
            'video_track_active': self.video_track is not None and self.video_track._running,
            'telemetry_subscribers': len(self.telemetry_subscribers)
        }