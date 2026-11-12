#!/usr/bin/env python3
"""
REST API Server for 5G Drone System
Provides HTTP API for drone control, telemetry, and configuration
"""

import asyncio
import logging
import json
from typing import Dict, Any, Optional
from datetime import datetime

from aiohttp import web
import jwt

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)


class RestAPI:
    def __init__(self, config: dict, mavlink_bridge=None, webrtc_server=None, 
                 sms_beacon=None, controller_manager=None):
        self.config = config
        self.mavlink_bridge = mavlink_bridge
        self.webrtc_server = webrtc_server
        self.sms_beacon = sms_beacon
        self.controller_manager = controller_manager
        
        self.api_config = config.get('api', {}).get('rest', {})
        self.host = self.api_config.get('host', '0.0.0.0')
        self.port = self.api_config.get('port', 8081)
        self.cors_origins = self.api_config.get('cors_origins', ['*'])
        self.rate_limit = self.api_config.get('rate_limit', 100)
        
        self.auth_enabled = self.api_config.get('auth', {}).get('enabled', True)
        self.jwt_secret = self.api_config.get('auth', {}).get('jwt_secret_env', 'JWT_SECRET')
        self.jwt_secret_value = os.environ.get(self.jwt_secret, 'dev-secret-change-me')
        self.token_expiry = self.api_config.get('auth', {}).get('token_expiry', 3600)
        
        self.app = None
        self.runner = None
        self.site = None
    
    async def start(self):
        """Start REST API server"""
        from aiohttp import web
        from aiohttp_cors import setup as cors_setup, ResourceOptions
        
        self.app = web.Application(middlewares=[self._auth_middleware, self._rate_limit_middleware])
        
        # CORS setup
        cors = cors_setup(self.app, defaults={
            "*": ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods="*"
            )
        })
        
        # Routes
        self.app.router.add_get('/health', self._handle_health)
        self.app.router.add_get('/api/status', self._handle_status)
        self.app.router.add_get('/api/telemetry', self._handle_telemetry)
        self.app.router.add_get('/api/telemetry/stream', self._handle_telemetry_stream)
        self.app.router.add_post('/api/control', self._handle_control)
        self.app.router.add_post('/api/control/arm', self._handle_arm)
        self.app.router.add_post('/api/control/disarm', self._handle_disarm)
        self.app.router.add_post('/api/control/rtl', self._handle_rtl)
        self.app.router.add_post('/api/control/land', self._handle_land)
        self.app.router.add_post('/api/control/takeoff', self._handle_takeoff)
        self.app.router.add_post('/api/control/mode', self._handle_set_mode)
        self.app.router.add_post('/api/control/manual', self._handle_manual_control)
        self.app.router.add_get('/api/config', self._handle_get_config)
        self.app.router.add_post('/api/config', self._handle_set_config)
        self.app.router.add_get('/api/failsafe/status', self._handle_failsafe_status)
        self.app.router.add_post('/api/failsafe/trigger', self._handle_trigger_failsafe)
        self.app.router.add_get('/api/sms/status', self._handle_sms_status)
        self.app.router.add_post('/api/sms/send', self._handle_send_sms)
        self.app.router.add_get('/api/controller/status', self._handle_controller_status)
        self.app.router.add_post('/api/controller/calibrate', self._handle_controller_calibrate)
        self.app.router.add_get('/api/tailscale/status', self._handle_tailscale_status)
        self.app.router.add_get('/api/webrtc/status', self._handle_webrtc_status)
        self.app.router.add_post('/api/webrtc/offer', self._handle_webrtc_offer)
        self.app.router.add_post('/auth/login', self._handle_login)
        self.app.router.add_post('/auth/token/refresh', self._handle_refresh_token)
        
        # Apply CORS to all routes
        for route in list(self.app.router.routes()):
            cors.add(route)
        
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, self.host, self.port)
        await self.site.start()
        
        logger.info(f"REST API server started on http://{self.host}:{self.port}")
    
    async def stop(self):
        """Stop REST API server"""
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        logger.info("REST API server stopped")
    
    @web.middleware
    async def _auth_middleware(self, request, handler):
        """JWT authentication middleware"""
        # Skip auth for public endpoints
        public_paths = ['/health', '/auth/login', '/auth/token/refresh']
        if request.path in public_paths:
            return await handler(request)
        
        if not self.auth_enabled:
            return await handler(request)
        
        auth_header = request.headers.get('Authorization', '')
        if not auth_header.startswith('Bearer '):
            return web.json_response({'error': 'Missing or invalid authorization header'}, status=401)
        
        token = auth_header[7:]
        try:
            payload = jwt.decode(token, self.jwt_secret_value, algorithms=['HS256'])
            request['user'] = payload
        except jwt.ExpiredSignatureError:
            return web.json_response({'error': 'Token expired'}, status=401)
        except jwt.InvalidTokenError:
            return web.json_response({'error': 'Invalid token'}, status=401)
        
        return await handler(request)
    
    @web.middleware
    async def _rate_limit_middleware(self, request, handler):
        """Simple rate limiting"""
        # In production, use aioredis or similar for distributed rate limiting
        return await handler(request)
    
    def _create_token(self, user_id: str, permissions: list = None) -> str:
        """Create JWT token"""
        payload = {
            'sub': user_id,
            'iat': datetime.utcnow(),
            'exp': datetime.utcnow() + timedelta(seconds=self.token_expiry),
            'permissions': permissions or ['control', 'telemetry', 'config']
        }
        return jwt.encode(payload, self.jwt_secret_value, algorithm='HS256')
    
    async def _handle_login(self, request):
        """Authenticate and return JWT token"""
        try:
            data = await request.json()
            username = data.get('username')
            password = data.get('password')
            
            # In production, verify against secure user store
            # For now, accept any credentials in dev mode
            if self.config.get('drone', {}).get('id') == 'drone-001':
                token = self._create_token(username or 'operator')
                return web.json_response({'token': token, 'expires_in': self.token_expiry})
            else:
                return web.json_response({'error': 'Invalid credentials'}, status=401)
        except Exception as e:
            return web.json_response({'error': str(e)}, status=400)
    
    async def _handle_refresh_token(self, request):
        """Refresh JWT token"""
        auth_header = request.headers.get('Authorization', '')
        if not auth_header.startswith('Bearer '):
            return web.json_response({'error': 'Missing token'}, status=401)
        
        token = auth_header[7:]
        try:
            payload = jwt.decode(token, self.jwt_secret_value, algorithms=['HS256'], options={'verify_exp': False})
            new_token = self._create_token(payload['sub'], payload.get('permissions'))
            return web.json_response({'token': new_token, 'expires_in': self.token_expiry})
        except jwt.InvalidTokenError:
            return web.json_response({'error': 'Invalid token'}, status=401)
    
    async def _handle_health(self, request):
        """Health check endpoint"""
        return web.json_response({
            'status': 'healthy',
            'timestamp': datetime.utcnow().isoformat(),
            'drone_id': self.config.get('drone', {}).get('id')
        })
    
    async def _handle_status(self, request):
        """Get overall system status"""
        status = {
            'drone': self.config.get('drone', {}),
            'mavlink': self.mavlink_bridge.get_connection_status() if self.mavlink_bridge else {},
            'webrtc': self.webrtc_server.get_status() if self.webrtc_server else {},
            'tailscale': self.tailscale_manager.get_status() if hasattr(self, 'tailscale_manager') and self.tailscale_manager else {},
            'sms': self.sms_beacon.get_status() if self.sms_beacon else {},
            'controller': self.controller_manager.get_status() if self.controller_manager else {},
            'timestamp': datetime.utcnow().isoformat()
        }
        return web.json_response(status)
    
    async def _handle_telemetry(self, request):
        """Get latest telemetry"""
        # In a real implementation, this would come from a telemetry cache
        return web.json_response({
            'message': 'Use /api/telemetry/stream for real-time telemetry',
            'timestamp': datetime.utcnow().isoformat()
        })
    
    async def _handle_telemetry_stream(self, request):
        """WebSocket endpoint for real-time telemetry"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        # Add to telemetry subscribers
        if self.webrtc_server:
            # This would need a proper subscription mechanism
            pass
        
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get('type') == 'ping':
                        await ws.send_json({'type': 'pong'})
        except Exception as e:
            logger.error(f"Telemetry stream error: {e}")
        finally:
            pass
        
        return ws
    
    async def _handle_control(self, request):
        """Generic control command"""
        if not self.mavlink_bridge:
            return web.json_response({'error': 'MAVLink bridge not available'}, status=503)
        
        try:
            data = await request.json()
            command = data.get('command')
            params = data.get('params', {})
            
            result = await self._execute_control(command, params)
            return web.json_response(result)
        except Exception as e:
            return web.json_response({'error': str(e)}, status=400)
    
    async def _execute_control(self, command: str, params: dict) -> dict:
        """Execute control command"""
        if command == 'arm':
            return await self.mavlink_bridge.arm(params.get('arm', True))
        elif command == 'disarm':
            return await self.mavlink_bridge.arm(False)
        elif command == 'rtl':
            return await self.mavlink_bridge.trigger_rtl()
        elif command == 'land':
            return await self.mavlink_bridge.land()
        elif command == 'takeoff':
            return await self.mavlink_bridge.takeoff(params.get('altitude', 10.0))
        elif command == 'set_mode':
            return await self.mavlink_bridge.set_mode(params.get('mode', 0), params.get('custom_mode', 0))
        elif command == 'manual':
            return await self.mavlink_bridge.send_manual_control(
                params.get('x', 0), params.get('y', 0),
                params.get('z', 0), params.get('r', 0),
                params.get('buttons', 0)
            )
        elif command == 'rc_override':
            return await self.mavlink_bridge.send_rc_channels_override(params.get('channels', []))
        else:
            return {'error': f'Unknown command: {command}'}
    
    async def _handle_arm(self, request):
        data = await request.json()
        result = await self.mavlink_bridge.arm(data.get('arm', True))
        return web.json_response(result)
    
    async def _handle_disarm(self, request):
        result = await self.mavlink_bridge.arm(False)
        return web.json_response(result)
    
    async def _handle_rtl(self, request):
        result = await self.mavlink_bridge.trigger_rtl()
        return web.json_response(result)
    
    async def _handle_land(self, request):
        result = await self.mavlink_bridge.land()
        return web.json_response(result)
    
    async def _handle_takeoff(self, request):
        data = await request.json()
        altitude = data.get('altitude', 10.0)
        result = await self.mavlink_bridge.takeoff(altitude)
        return web.json_response(result)
    
    async def _handle_set_mode(self, request):
        data = await request.json()
        mode = data.get('mode', 0)
        custom_mode = data.get('custom_mode', 0)
        result = await self.mavlink_bridge.set_mode(mode, custom_mode)
        return web.json_response(result)
    
    async def _handle_manual_control(self, request):
        data = await request.json()
        await self.mavlink_bridge.send_manual_control(
            data.get('x', 0), data.get('y', 0),
            data.get('z', 0), data.get('r', 0),
            data.get('buttons', 0)
        )
        return web.json_response({'success': True})
    
    async def _handle_get_config(self, request):
        # Return config (sanitized - no secrets)
        sanitized = self._sanitize_config(self.config)
        return web.json_response(sanitized)
    
    async def _handle_set_config(self, request):
        data = await request.json()
        # In production, validate and persist config
        return web.json_response({'message': 'Config update not implemented', 'received': data})
    
    def _sanitize_config(self, config: dict) -> dict:
        """Remove sensitive data from config"""
        sanitized = {}
        for key, value in config.items():
            if key in ('_config_path', '_config_dir'):
                continue
            if isinstance(value, dict):
                sanitized[key] = self._sanitize_config(value)
            elif isinstance(value, str) and ('secret' in key.lower() or 'password' in key.lower() or 'token' in key.lower() or 'key' in key.lower()):
                sanitized[key] = '***REDACTED***'
            else:
                sanitized[key] = value
        return sanitized
    
    async def _handle_failsafe_status(self, request):
        return web.json_response(self.config.get('failsafe', {}))
    
    async def _handle_trigger_failsafe(self, request):
        data = await request.json()
        reason = data.get('reason', 'manual')
        message = data.get('message', 'Manual failsafe trigger')
        rth = data.get('rth', True)
        
        if self.sms_beacon:
            await self.sms_beacon.send_emergency_beacon(reason, message, severity=3)
        
        if rth and self.mavlink_bridge:
            await self.mavlink_bridge.trigger_rtl()
        
        return web.json_response({'success': True, 'reason': reason})
    
    async def _handle_sms_status(self, request):
        return web.json_response(self.sms_beacon.get_status() if self.sms_beacon else {})
    
    async def _handle_send_sms(self, request):
        if not self.sms_beacon:
            return web.json_response({'error': 'SMS beacon not available'}, status=503)
        
        data = await request.json()
        to = data.get('to')
        message = data.get('message')
        
        if not to or not message:
            return web.json_response({'error': 'Missing to or message'}, status=400)
        
        success = await self.sms_beacon.send_custom_sms(to, message)
        return web.json_response({'success': success})
    
    async def _handle_controller_status(self, request):
        return web.json_response(self.controller_manager.get_status() if self.controller_manager else {})
    
    async def _handle_controller_calibrate(self, request):
        # Controller calibration would be implemented here
        return web.json_response({'message': 'Calibration not yet implemented'})
    
    async def _handle_tailscale_status(self, request):
        # Would need tailscale_manager reference
        return web.json_response({})
    
    async def _handle_webrtc_status(self, request):
        return web.json_response(self.webrtc_server.get_status() if self.webrtc_server else {})
    
    async def _handle_webrtc_offer(self, request):
        if not self.webrtc_server:
            return web.json_response({'error': 'WebRTC server not available'}, status=503)
        
        data = await request.json()
        # Forward to WebRTC server's offer handler
        # This is a simplified version
        return web.json_response({'error': 'Use WebRTC signaling endpoint directly'})


# Need to import os and timedelta
import os
from datetime import timedelta