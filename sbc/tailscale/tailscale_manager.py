#!/usr/bin/env python3
"""
Tailscale Manager for CGNAT Bypass
Manages Tailscale daemon and mesh networking for drone communication
"""

import asyncio
import logging
import os
import subprocess
import json
from typing import Optional, Dict, List

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)


class TailscaleManager:
    def __init__(self, config: dict):
        self.config = config
        self.tailscale_config = config.get('network', {}).get('tailscale', {})
        self.running = False
        self.process = None
        self.tailscale_ip = None
        self.peers: List[Dict] = []
    
    async def start(self):
        """Start Tailscale daemon and join mesh"""
        auth_key = os.environ.get(self.tailscale_config.get('auth_key_env', 'TAILSCALE_AUTH_KEY'))
        if not auth_key:
            logger.warning("No Tailscale auth key found in environment. Set TAILSCALE_AUTH_KEY")
            return
        
        hostname = self.tailscale_config.get('hostname', 'drone-sbc')
        advertise_routes = self.tailscale_config.get('advertise_routes', [])
        accept_routes = self.tailscale_config.get('accept_routes', True)
        accept_dns = self.tailscale_config.get('accept_dns', False)
        ssh = self.tailscale_config.get('ssh', True)
        state_dir = self.tailscale_config.get('state_dir', '/var/lib/tailscale')
        socket = self.tailscale_config.get('socket', '/var/run/tailscale/tailscaled.sock')
        
        # Ensure state directory exists
        os.makedirs(state_dir, exist_ok=True)
        os.makedirs(os.path.dirname(socket), exist_ok=True)
        
        # Start tailscaled daemon
        await self._start_tailscaled(state_dir, socket)
        
        # Wait for daemon to be ready
        await asyncio.sleep(2)
        
        # Join mesh
        await self._join_mesh(auth_key, hostname, advertise_routes, accept_routes, accept_dns, ssh)
        
        # Get Tailscale IP
        self.tailscale_ip = await self._get_tailscale_ip()
        
        # Start peer monitor
        self.running = True
        asyncio.create_task(self._monitor_peers())
        
        logger.info(f"Tailscale mesh joined. IP: {self.tailscale_ip}")
    
    async def _start_tailscaled(self, state_dir: str, socket: str):
        """Start tailscaled daemon"""
        # Check if already running
        try:
            result = subprocess.run(['tailscale', 'status'], capture_output=True, timeout=5)
            if result.returncode == 0:
                logger.info("Tailscale daemon already running")
                return
        except:
            pass
        
        # Start daemon
        cmd = [
            'tailscaled',
            f'--state={state_dir}/tailscaled.state',
            f'--socket={socket}',
            '--port=41641',
            '--no-logs-no-support',
        ]
        
        self.process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        
        logger.info("Tailscale daemon started")
    
    async def _join_mesh(self, auth_key: str, hostname: str, advertise_routes: List[str],
                         accept_routes: bool, accept_dns: bool, ssh: bool):
        """Join Tailscale mesh network"""
        cmd = ['tailscale', 'up', f'--authkey={auth_key}', f'--hostname={hostname}']
        
        if advertise_routes:
            cmd.append(f'--advertise-routes={",".join(advertise_routes)}')
        if accept_routes:
            cmd.append('--accept-routes')
        if accept_dns:
            cmd.append('--accept-dns')
        else:
            cmd.append('--accept-dns=false')
        if ssh:
            cmd.append('--ssh')
        
        process = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await process.communicate()
        
        if process.returncode != 0:
            logger.error(f"Tailscale up failed: {stderr.decode()}")
            raise RuntimeError(f"Failed to join Tailscale mesh: {stderr.decode()}")
        
        logger.info("Joined Tailscale mesh successfully")
    
    async def _get_tailscale_ip(self) -> Optional[str]:
        """Get Tailscale IP address"""
        try:
            result = subprocess.run(['tailscale', 'ip', '-4'], capture_output=True, timeout=5)
            if result.returncode == 0:
                return result.stdout.decode().strip()
        except Exception as e:
            logger.error(f"Failed to get Tailscale IP: {e}")
        return None
    
    async def _monitor_peers(self):
        """Monitor Tailscale peers"""
        while self.running:
            try:
                result = subprocess.run(['tailscale', 'status', '--json'], capture_output=True, timeout=10)
                if result.returncode == 0:
                    status = json.loads(result.stdout.decode())
                    self.peers = status.get('Peers', [])
                    self.tailscale_ip = status.get('Self', {}).get('TailscaleIPs', [None])[0]
            except Exception as e:
                logger.debug(f"Peer monitor error: {e}")
            
            await asyncio.sleep(30)
    
    async def stop(self):
        """Stop Tailscale"""
        self.running = False
        
        try:
            subprocess.run(['tailscale', 'down'], timeout=10)
        except:
            pass
        
        if self.process:
            self.process.terminate()
            try:
                await asyncio.wait_for(self.process.wait(), timeout=5)
            except asyncio.TimeoutError:
                self.process.kill()
        
        logger.info("Tailscale stopped")
    
    def get_status(self) -> dict:
        """Get Tailscale status"""
        return {
            'running': self.running,
            'tailscale_ip': self.tailscale_ip,
            'peers': [
                {
                    'hostname': p.get('HostName'),
                    'ip': p.get('TailscaleIPs', [None])[0],
                    'online': p.get('Online', False),
                    'active': p.get('Active', False)
                }
                for p in self.peers
            ],
            'peer_count': len(self.peers),
            'online_peers': sum(1 for p in self.peers if p.get('Online', False))
        }