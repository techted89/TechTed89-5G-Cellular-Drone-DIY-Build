#!/usr/bin/env python3
"""
Dynamic Return-to-Home (RTH) Failsafe for Flight Controller
Implements intelligent RTH with obstacle avoidance, precision landing, and adaptive altitude
"""

import asyncio
import logging
import math
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)


class RTHPhase(Enum):
    """RTH execution phases"""
    IDLE = 0
    CLIMBING = 1          # Climbing to RTH altitude
    RETURNING = 2         # Flying to home position
    LOITERING = 3         # Loitering above home
    DESCENDING = 4        # Descending for landing
    PRECISION_LANDING = 5 # Precision landing approach
    LANDED = 6            # Landed
    ABORTED = 7           # Aborted due to new command


class RTHTrigger(Enum):
    """RTH trigger reasons"""
    SIGNAL_LOSS = "signal_loss"
    LOW_BATTERY = "low_battery"
    GEOFENCE = "geofence"
    MANUAL = "manual"
    CRITICAL_FAILURE = "critical_failure"
    PILOT_REQUEST = "pilot_request"


@dataclass
class RTHConfig:
    """RTH configuration"""
    enabled: bool = True
    min_altitude: float = 30.0        # Minimum RTH altitude (m)
    max_altitude: float = 120.0       # Maximum RTH altitude (m)
    default_altitude: float = 50.0    # Default RTH altitude (m)
    descent_rate: float = 2.0         # Descent rate (m/s)
    approach_speed: float = 8.0       # Approach speed (m/s)
    loiter_radius: float = 10.0       # Loiter radius (m)
    landing_approach_angle: float = 15.0  # Approach angle (degrees)
    
    # Dynamic altitude
    dynamic_altitude: bool = False
    obstacle_clearance: float = 5.0   # Obstacle clearance (m)
    
    # Precision landing
    precision_landing: bool = False
    marker_id: int = 0
    search_radius: float = 5.0
    
    # Obstacle avoidance
    obstacle_avoidance: bool = False
    sensor_type: str = "lidar"  # lidar, stereo, tof


@dataclass
class Waypoint:
    """3D waypoint"""
    lat: float
    lon: float
    alt: float
    speed: float = 0
    hold_time: float = 0


@dataclass
class RTHState:
    """RTH execution state"""
    phase: RTHPhase = RTHPhase.IDLE
    trigger: Optional[RTHTrigger] = None
    start_time: float = 0
    target_altitude: float = 0
    home_position: Tuple[float, float, float] = (0, 0, 0)
    current_waypoint: int = 0
    waypoints: List[Waypoint] = field(default_factory=list)
    distance_to_home: float = 0
    estimated_time: float = 0
    battery_at_start: float = 0
    aborted: bool = False


class DynamicRTH:
    """
    Dynamic Return-to-Home System
    Calculates optimal RTH path based on current conditions
    """
    
    def __init__(self, config: dict, fc_interface=None):
        self.config = config
        self.fc = fc_interface
        self.rth_config = RTHConfig(**config.get('failsafe', {}).get('rth', {}))
        
        self.state = RTHState()
        self.running = False
        self._task = None
        
        # Terrain/obstacle data (would come from sensors)
        self.terrain_data: Dict[Tuple[float, float], float] = {}
        self.obstacles: List[Tuple[float, float, float, float]] = []  # lat, lon, alt, radius
    
    async def start(self):
        """Start RTH monitor"""
        self.running = True
        self._task = asyncio.create_task(self._monitor_loop())
        logger.info("Dynamic RTH system started")
    
    async def stop(self):
        """Stop RTH monitor"""
        self.running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        logger.info("Dynamic RTH system stopped")
    
    async def trigger_rth(self, trigger: RTHTrigger, reason: str = "") -> bool:
        """Trigger Return to Home"""
        if not self.rth_config.enabled:
            logger.warning("RTH disabled in config")
            return False
        
        if self.state.phase != RTHPhase.IDLE:
            logger.warning(f"RTH already in progress: {self.state.phase.name}")
            return False
        
        # Get current position from FC
        if self.fc:
            fc_status = self.fc.get_status()
            pos = fc_status['position']
            home = fc_status['home']
            battery = fc_status['battery']
        else:
            pos = {'lat': 0, 'lon': 0, 'alt': 0, 'relative_alt': 0}
            home = {'lat': 0, 'lon': 0, 'alt': 0}
            battery = {'voltage': 0, 'remaining': 100}
        
        # Validate home position
        if home['lat'] == 0 and home['lon'] == 0:
            logger.error("Home position not set!")
            return False
        
        # Calculate RTH altitude
        target_alt = await self._calculate_rth_altitude(pos, home)
        
        # Initialize state
        self.state = RTHState(
            phase=RTHPhase.CLIMBING,
            trigger=trigger,
            start_time=time.time(),
            target_altitude=target_alt,
            home_position=(home['lat'], home['lon'], home['alt']),
            battery_at_start=battery.get('voltage', 0)
        )
        
        # Generate waypoints
        self.state.waypoints = await self._generate_rth_waypoints(pos, home, target_alt)
        
        logger.warning(f"RTH TRIGGERED: {trigger.value} - {reason}")
        logger.info(f"RTH Plan: {len(self.state.waypoints)} waypoints, target alt: {target_alt:.1f}m")
        
        # Execute RTH (would send to FC navigation)
        await self._execute_rth()
        
        return True
    
    async def _calculate_rth_altitude(self, pos: dict, home: dict) -> float:
        """Calculate optimal RTH altitude"""
        current_alt = pos.get('relative_alt', pos.get('alt', 0))
        
        if not self.rth_config.dynamic_altitude:
            # Use default or current altitude, whichever is higher (but within bounds)
            target = max(self.rth_config.default_altitude, current_alt)
            return min(max(target, self.rth_config.min_altitude), self.rth_config.max_altitude)
        
        # Dynamic altitude: consider terrain and obstacles along path
        # This would query terrain database and obstacle sensors
        min_safe_alt = self.rth_config.min_altitude
        
        # Check terrain along direct path to home
        path_alt = await self._get_max_terrain_along_path(pos, home)
        if path_alt > 0:
            min_safe_alt = max(min_safe_alt, path_alt + self.rth_config.obstacle_clearance)
        
        # Check obstacles
        for obs_lat, obs_lon, obs_alt, obs_radius in self.obstacles:
            # Check if obstacle is near path
            dist = self._distance(pos['lat'], pos['lon'], obs_lat, obs_lon)
            if dist < obs_radius + 50:  # 50m buffer
                min_safe_alt = max(min_safe_alt, obs_alt + self.rth_config.obstacle_clearance)
        
        # Use current altitude if higher than minimum safe
        target = max(current_alt, min_safe_alt)
        
        return min(max(target, self.rth_config.min_altitude), self.rth_config.max_altitude)
    
    async def _get_max_terrain_along_path(self, pos: dict, home: dict) -> float:
        """Get maximum terrain elevation along path (simplified)"""
        # In real implementation, would query terrain database (SRTM, etc.)
        # For now, return 0 (no terrain data)
        return 0.0
    
    async def _generate_rth_waypoints(self, pos: dict, home: dict, target_alt: float) -> List[Waypoint]:
        """Generate RTH waypoint sequence"""
        waypoints = []
        current_alt = pos.get('relative_alt', pos.get('alt', 0))
        
        # 1. Climb to RTH altitude (if needed)
        if current_alt < target_alt - 1.0:
            waypoints.append(Waypoint(
                lat=pos['lat'],
                lon=pos['lon'],
                alt=target_alt,
                speed=5.0  # Climb speed
            ))
        
        # 2. Fly to home position at RTH altitude
        waypoints.append(Waypoint(
            lat=home['lat'],
            lon=home['lon'],
            alt=target_alt,
            speed=self.rth_config.approach_speed
        ))
        
        # 3. Loiter above home
        waypoints.append(Waypoint(
            lat=home['lat'],
            lon=home['lon'],
            alt=target_alt,
            speed=self.rth_config.approach_speed,
            hold_time=5.0  # Loiter for 5 seconds
        ))
        
        # 4. Descend for landing
        if self.rth_config.precision_landing:
            # Precision landing approach
            waypoints.append(Waypoint(
                lat=home['lat'],
                lon=home['lon'],
                alt=10.0,  # Approach altitude
                speed=3.0
            ))
            waypoints.append(Waypoint(
                lat=home['lat'],
                lon=home['lon'],
                alt=0.0,  # Land
                speed=self.rth_config.descent_rate
            ))
        else:
            # Standard descent
            waypoints.append(Waypoint(
                lat=home['lat'],
                lon=home['lon'],
                alt=0.0,
                speed=self.rth_config.descent_rate
            ))
        
        return waypoints
    
    async def _execute_rth(self):
        """Execute RTH waypoint sequence"""
        # This would send waypoints to FC navigation system
        # For now, simulate execution
        logger.info("Executing RTH...")
        
        for i, wp in enumerate(self.state.waypoints):
            if self.state.aborted:
                break
            
            self.state.current_waypoint = i
            
            # Update phase based on waypoint
            if i == 0 and wp.alt > self.state.target_altitude - 1:
                self.state.phase = RTHPhase.CLIMBING
            elif i == 1:
                self.state.phase = RTHPhase.RETURNING
            elif i == 2:
                self.state.phase = RTHPhase.LOITERING
            elif i >= 3:
                self.state.phase = RTHPhase.DESCENDING
            
            # Send waypoint to FC
            if self.fc:
                await self._send_waypoint(wp)
            
            # Wait for waypoint reached (simplified)
            await self._wait_for_waypoint(wp)
        
        if not self.state.aborted:
            self.state.phase = RTHPhase.LANDED
            logger.info("RTH completed successfully")
        else:
            self.state.phase = RTHPhase.ABORTED
            logger.info("RTH aborted")
    
    async def _send_waypoint(self, wp: Waypoint):
        """Send waypoint to FC"""
        # Would use MAVLink MISSION_ITEM or SET_POSITION_TARGET_GLOBAL_INT
        logger.debug(f"Sending waypoint: {wp.lat:.7f}, {wp.lon:.7f}, {wp.alt:.1f}m @ {wp.speed}m/s")
    
    async def _wait_for_waypoint(self, wp: Waypoint):
        """Wait for waypoint reached"""
        # In real implementation, would monitor FC position
        # For simulation, estimate time based on distance and speed
        if self.fc:
            fc_status = self.fc.get_status()
            pos = fc_status['position']
            
            dist = self._distance(pos['lat'], pos['lon'], wp.lat, wp.lon)
            alt_diff = abs(pos['alt'] - wp.alt)
            total_dist = math.sqrt(dist**2 + alt_diff**2)
            
            if wp.speed > 0:
                est_time = total_dist / wp.speed
            else:
                est_time = 10.0
            
            # Wait with timeout
            await asyncio.sleep(min(est_time + 5, 30))
    
    def abort_rth(self):
        """Abort current RTH"""
        if self.state.phase != RTHPhase.IDLE:
            logger.warning("RTH aborted by pilot")
            self.state.aborted = True
            self.state.phase = RTHPhase.ABORTED
    
    async def _monitor_loop(self):
        """Monitor RTH execution"""
        while self.running:
            try:
                if self.state.phase != RTHPhase.IDLE:
                    await self._update_rth_status()
                await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"RTH monitor error: {e}")
                await asyncio.sleep(1)
    
    async def _update_rth_status(self):
        """Update RTH status and check for issues"""
        if not self.fc:
            return
        
        fc_status = self.fc.get_status()
        pos = fc_status['position']
        battery = fc_status['battery']
        
        # Update distance to home
        home_lat, home_lon, home_alt = self.state.home_position
        self.state.distance_to_home = self._distance(pos['lat'], pos['lon'], home_lat, home_lon)
        
        # Check battery during RTH
        if battery.get('voltage', 0) > 0:
            voltage_drop = self.state.battery_at_start - battery['voltage']
            if voltage_drop > 1.0:  # Significant voltage drop
                logger.warning(f"Battery dropping fast during RTH: {voltage_drop:.1f}V drop")
                # Could accelerate descent
        
        # Check if landed
        if self.state.phase in (RTHPhase.DESCENDING, RTHPhase.PRECISION_LANDING):
            if pos.get('relative_alt', 0) < 0.5 and abs(pos.get('vz', 0)) < 0.1:
                self.state.phase = RTHPhase.LANDED
                await self.fc.disarm()
    
    def _distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points (meters)"""
        # Haversine formula
        R = 6371000  # Earth radius in meters
        
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def get_status(self) -> dict:
        """Get RTH status"""
        return {
            'active': self.state.phase != RTHPhase.IDLE,
            'phase': self.state.phase.name,
            'trigger': self.state.trigger.value if self.state.trigger else None,
            'target_altitude': self.state.target_altitude,
            'home_position': {
                'lat': self.state.home_position[0],
                'lon': self.state.home_position[1],
                'alt': self.state.home_position[2]
            },
            'current_waypoint': self.state.current_waypoint,
            'total_waypoints': len(self.state.waypoints),
            'distance_to_home': self.state.distance_to_home,
            'elapsed_time': time.time() - self.state.start_time if self.state.start_time > 0 else 0,
            'aborted': self.state.aborted,
            'battery_at_start': self.state.battery_at_start
        }


# Emergency landing site selection
class EmergencyLanding:
    """Select emergency landing sites"""
    
    def __init__(self, config: dict):
        self.config = config
        self.known_sites: List[dict] = []  # Pre-surveyed safe landing zones
    
    def add_site(self, lat: float, lon: float, radius: float, description: str = ""):
        """Add known landing site"""
        self.known_sites.append({
            'lat': lat,
            'lon': lon,
            'radius': radius,
            'description': description
        })
    
    async def find_best_site(self, current_lat: float, current_lon: float, 
                            current_alt: float, max_range: float = 1000) -> Optional[dict]:
        """Find best emergency landing site within range"""
        best_site = None
        best_score = -1
        
        for site in self.known_sites:
            dist = self._distance(current_lat, current_lon, site['lat'], site['lon'])
            
            if dist <= max_range:
                # Score based on distance, size, and elevation
                score = (1000 - dist) + site['radius'] * 10
                if score > best_score:
                    best_score = score
                    best_site = site
                    best_site['distance'] = dist
        
        return best_site
    
    def _distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R = 6371000
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c