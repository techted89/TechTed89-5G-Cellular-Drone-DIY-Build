#!/usr/bin/env python3
"""
Game Controller Input Handler for Drone Control
Supports Xbox, PlayStation, and generic HID game controllers via USB/Bluetooth
Maps controller inputs to MAVLink manual control and RC channel overrides
"""

import asyncio
import logging
import os
import struct
from typing import Dict, Optional, Callable, List, Any
from dataclasses import dataclass, field
from enum import Enum
from abc import ABC, abstractmethod

try:
    import evdev
    from evdev import InputDevice, categorize, ecodes, ff
    EVDEV_AVAILABLE = True
except ImportError:
    EVDEV_AVAILABLE = False
    logging.warning("evdev not installed. Run: pip install evdev")

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    logging.warning("pygame not installed. Run: pip install pygame")

from shared.config.config_loader import load_config

logger = logging.getLogger(__name__)


class ControllerType(Enum):
    XBOX = "xbox"
    PLAYSTATION = "playstation"
    GENERIC = "generic"
    CUSTOM = "custom"
    MOBILE = "mobile"        # Android/iPhone via web/websocket
    LORA = "lora"            # LoRa radio board controller


@dataclass
class ControllerMapping:
    """Maps controller axes/buttons to drone controls"""
    # Axes (typically -32768 to 32767 or -1.0 to 1.0)
    axis_left_x: int = 0      # Roll (left stick horizontal)
    axis_left_y: int = 1      # Pitch (left stick vertical)
    axis_right_x: int = 2     # Yaw (right stick horizontal)
    axis_right_y: int = 3     # Throttle (right stick vertical)
    axis_lt: int = 4          # Left trigger
    axis_rt: int = 5          # Right trigger
    
    # Standard buttons
    btn_a: int = 0            # Arm/Disarm
    btn_b: int = 1            # RTL
    btn_x: int = 2            # Land
    btn_y: int = 3            # Takeoff
    btn_lb: int = 4           # Mode switch
    btn_rb: int = 5           # Mode switch
    btn_back: int = 6         # Emergency stop
    btn_start: int = 7        # Mission start
    btn_lstick: int = 8       # Precision mode
    btn_rstick: int = 9       # Camera control
    btn_dpad_up: int = 10     # Altitude up
    btn_dpad_down: int = 11   # Altitude down
    btn_dpad_left: int = 12   # Yaw left
    btn_dpad_right: int = 13  # Yaw right
    
    # Custom programmable buttons (M1, M2, N1, N2)
    btn_m1: int = 14          # Custom button M1
    btn_m2: int = 15          # Custom button M2
    btn_n1: int = 16          # Custom button N1
    btn_n2: int = 17          # Custom button N2
    
    # Axis inversion
    invert_left_y: bool = True
    invert_right_y: bool = True
    invert_right_x: bool = False
    
    # Deadzone
    deadzone: float = 0.1
    
    # Scaling
    max_roll: int = 1000
    max_pitch: int = 1000
    max_yaw: int = 1000
    max_throttle: int = 1000
    
    # Custom button actions (configurable)
    custom_actions: Dict[str, str] = field(default_factory=lambda: {
        'm1': 'none',
        'm2': 'none',
        'n1': 'none',
        'n2': 'none'
    })


# Default mappings for common controllers
CONTROLLER_MAPPINGS = {
    ControllerType.XBOX: ControllerMapping(
        axis_left_x=0, axis_left_y=1,
        axis_right_x=3, axis_right_y=4,
        axis_lt=2, axis_rt=5,
        btn_a=0, btn_b=1, btn_x=2, btn_y=3,
        btn_lb=4, btn_rb=5, btn_back=6, btn_start=7,
        btn_lstick=8, btn_rstick=9,
        btn_dpad_up=12, btn_dpad_down=13,
        btn_dpad_left=14, btn_dpad_right=15,
        invert_left_y=True, invert_right_y=True
    ),
    ControllerType.PLAYSTATION: ControllerMapping(
        axis_left_x=0, axis_left_y=1,
        axis_right_x=2, axis_right_y=3,
        axis_lt=4, axis_rt=5,
        btn_a=1, btn_b=2, btn_x=0, btn_y=3,  # Cross, Circle, Square, Triangle
        btn_lb=4, btn_rb=5, btn_back=8, btn_start=9,
        btn_lstick=10, btn_rstick=11,
        btn_dpad_up=12, btn_dpad_down=13,
        btn_dpad_left=14, btn_dpad_right=15,
        invert_left_y=True, invert_right_y=True
    ),
    ControllerType.GENERIC: ControllerMapping(),
}


@dataclass
class ControllerState:
    """Current controller state"""
    connected: bool = False
    axes: Dict[int, float] = field(default_factory=dict)
    buttons: Dict[int, bool] = field(default_factory=dict)
    timestamp: float = 0
    battery_level: Optional[int] = None


class ControllerBackend(ABC):
    """Abstract controller backend"""
    
    @abstractmethod
    async def start(self) -> bool:
        pass
    
    @abstractmethod
    async def stop(self):
        pass
    
    @abstractmethod
    async def poll(self) -> Optional[ControllerState]:
        pass
    
    @abstractmethod
    def get_device_info(self) -> Dict[str, Any]:
        pass


class EvdevBackend(ControllerBackend):
    """Linux evdev backend for USB/Bluetooth controllers"""
    
    def __init__(self, device_path: str, mapping: ControllerMapping):
        self.device_path = device_path
        self.mapping = mapping
        self.device: Optional[InputDevice] = None
        self.running = False
    
    async def start(self) -> bool:
        if not EVDEV_AVAILABLE:
            logger.error("evdev not available")
            return False
        
        try:
            self.device = InputDevice(self.device_path)
            logger.info(f"Opened controller: {self.device.name} at {self.device_path}")
            logger.info(f"Capabilities: {self.device.capabilities(verbose=True)}")
            
            # Grab exclusive access (optional)
            # self.device.grab()
            
            self.running = True
            return True
        except Exception as e:
            logger.error(f"Failed to open controller {self.device_path}: {e}")
            return False
    
    async def stop(self):
        self.running = False
        if self.device:
            try:
                # self.device.ungrab()
                self.device.close()
            except:
                pass
            self.device = None
    
    async def poll(self) -> Optional[ControllerState]:
        if not self.device or not self.running:
            return None
        
        state = ControllerState(connected=True)
        
        try:
            # Non-blocking read
            events = self.device.read()
            for event in events:
                if event.type == ecodes.EV_ABS:
                    # Axis event
                    normalized = self._normalize_axis(event.code, event.value)
                    state.axes[event.code] = normalized
                elif event.type == ecodes.EV_KEY:
                    # Button event
                    state.buttons[event.code] = bool(event.value)
                elif event.type == ecodes.EV_BATTERY:
                    # Battery level (if supported)
                    state.battery_level = event.value
            
            state.timestamp = asyncio.get_event_loop().time()
            return state
            
        except BlockingIOError:
            # No events available
            return state
        except Exception as e:
            logger.error(f"Controller poll error: {e}")
            state.connected = False
            return state
    
    def _normalize_axis(self, code: int, value: int) -> float:
        """Normalize axis value to -1.0 to 1.0"""
        # Get axis info
        absinfo = self.device.absinfo(code) if self.device else None
        if absinfo:
            min_val = absinfo.min
            max_val = absinfo.max
            # Normalize to -1.0 to 1.0
            normalized = 2.0 * (value - min_val) / (max_val - min_val) - 1.0
        else:
            # Assume 16-bit signed
            normalized = value / 32767.0
        
        return max(-1.0, min(1.0, normalized))
    
    def get_device_info(self) -> Dict[str, Any]:
        if not self.device:
            return {}
        return {
            'name': self.device.name,
            'path': self.device.path,
            'phys': self.device.phys,
            'uniq': self.device.uniq,
            'id': {
                'bustype': self.device.info.bustype,
                'vendor': self.device.info.vendor,
                'product': self.device.info.product,
                'version': self.device.info.version
            }
        }


class PygameBackend(ControllerBackend):
    """Pygame backend (cross-platform fallback)"""
    
    def __init__(self, joystick_id: int, mapping: ControllerMapping):
        self.joystick_id = joystick_id
        self.mapping = mapping
        self.joystick = None
        self.running = False
    
    async def start(self) -> bool:
        if not PYGAME_AVAILABLE:
            logger.error("pygame not available")
            return False
        
        try:
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() <= self.joystick_id:
                logger.error(f"No joystick at index {self.joystick_id}")
                return False
            
            self.joystick = pygame.joystick.Joystick(self.joystick_id)
            self.joystick.init()
            
            logger.info(f"Opened controller: {self.joystick.get_name()}")
            logger.info(f"Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()}")
            
            self.running = True
            return True
        except Exception as e:
            logger.error(f"Failed to initialize pygame joystick: {e}")
            return False
    
    async def stop(self):
        self.running = False
        if self.joystick:
            self.joystick.quit()
        pygame.joystick.quit()
    
    async def poll(self) -> Optional[ControllerState]:
        if not self.joystick or not self.running:
            return None
        
        # Pump events
        pygame.event.pump()
        
        state = ControllerState(connected=True)
        
        # Read axes
        for i in range(self.joystick.get_numaxes()):
            state.axes[i] = self.joystick.get_axis(i)
        
        # Read buttons
        for i in range(self.joystick.get_numbuttons()):
            state.buttons[i] = bool(self.joystick.get_button(i))
        
        state.timestamp = asyncio.get_event_loop().time()
        return state
    
    def get_device_info(self) -> Dict[str, Any]:
        if not self.joystick:
            return {}
        return {
            'name': self.joystick.get_name(),
            'id': self.joystick_id,
            'axes': self.joystick.get_numaxes(),
            'buttons': self.joystick.get_numbuttons(),
            'hats': self.joystick.get_numhats()
        }


class NetworkControllerBackend(ControllerBackend):
    """Network backend for mobile phone controllers (Android/iPhone) via WebSocket"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8765, mapping: ControllerMapping = None):
        self.host = host
        self.port = port
        self.mapping = mapping or ControllerMapping()
        self.server = None
        self.running = False
        self.connected_clients: Dict[str, Dict[str, Any]] = {}
        self.last_state = ControllerState()
        
    async def start(self) -> bool:
        try:
            import websockets
            self.server = await websockets.serve(
                self._handle_client,
                self.host,
                self.port
            )
            self.running = True
            logger.info(f"Mobile controller server started on {self.host}:{self.port}")
            return True
        except ImportError:
            logger.error("websockets not installed. Run: pip install websockets")
            return False
        except Exception as e:
            logger.error(f"Failed to start mobile controller server: {e}")
            return False
    
    async def stop(self):
        self.running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            self.server = None
        self.connected_clients.clear()
    
    async def poll(self) -> Optional[ControllerState]:
        # Return the most recent state from any connected client
        if self.connected_clients:
            # Return first connected client's state
            for client_info in self.connected_clients.values():
                if client_info.get('state'):
                    return client_info['state']
        return self.last_state if self.last_state.connected else None
    
    async def _handle_client(self, websocket, path):
        """Handle incoming WebSocket connection from mobile controller"""
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"Mobile controller connected: {client_id}")
        
        try:
            async for message in websocket:
                try:
                    import json
                    data = json.loads(message)
                    await self._process_mobile_message(client_id, data, websocket)
                except json.JSONDecodeError:
                    logger.warning(f"Invalid JSON from {client_id}: {message[:100]}")
                except Exception as e:
                    logger.error(f"Error processing message from {client_id}: {e}")
        except Exception as e:
            logger.error(f"Client {client_id} error: {e}")
        finally:
            self.connected_clients.pop(client_id, None)
            logger.info(f"Mobile controller disconnected: {client_id}")
    
    async def _process_mobile_message(self, client_id: str, data: dict, websocket):
        """Process incoming message from mobile controller"""
        msg_type = data.get('type', 'unknown')
        
        if msg_type == 'register':
            # Client registration
            self.connected_clients[client_id] = {
                'websocket': websocket,
                'device_info': data.get('device_info', {}),
                'state': ControllerState(connected=True)
            }
            await websocket.send(json.dumps({
                'type': 'registered',
                'client_id': client_id,
                'config': self._get_controller_config()
            }))
            
        elif msg_type == 'state':
            # Controller state update
            if client_id in self.connected_clients:
                state = self.connected_clients[client_id]['state']
                state.connected = True
                state.axes = data.get('axes', {})
                state.buttons = data.get('buttons', {})
                state.timestamp = asyncio.get_event_loop().time()
                state.battery_level = data.get('battery_level')
                self.last_state = state
                
        elif msg_type == 'heartbeat':
            # Client heartbeat
            await websocket.send(json.dumps({'type': 'heartbeat_ack'}))
    
    def _get_controller_config(self) -> dict:
        """Get controller configuration to send to mobile client"""
        return {
            'mapping': {
                'axis_left_x': self.mapping.axis_left_x,
                'axis_left_y': self.mapping.axis_left_y,
                'axis_right_x': self.mapping.axis_right_x,
                'axis_right_y': self.mapping.axis_right_y,
                'btn_a': self.mapping.btn_a,
                'btn_b': self.mapping.btn_b,
                'btn_x': self.mapping.btn_x,
                'btn_y': self.mapping.btn_y,
                'btn_m1': self.mapping.btn_m1,
                'btn_m2': self.mapping.btn_m2,
                'btn_n1': self.mapping.btn_n1,
                'btn_n2': self.mapping.btn_n2,
            },
            'custom_actions': self.mapping.custom_actions,
            'deadzone': self.mapping.deadzone,
        }
    
    def get_device_info(self) -> Dict[str, Any]:
        return {
            'type': 'network_mobile',
            'host': self.host,
            'port': self.port,
            'connected_clients': len(self.connected_clients),
            'clients': list(self.connected_clients.keys())
        }


class MobileControllerBackend(NetworkControllerBackend):
    """Mobile controller backend - alias for NetworkControllerBackend with mobile-specific defaults"""
    
    def __init__(self, host: str = '0.0.0.0', port: int = 8765, mapping: ControllerMapping = None):
        # Use mobile-optimized defaults
        if mapping is None:
            mapping = ControllerMapping(
                axis_left_x=0, axis_left_y=1,
                axis_right_x=2, axis_right_y=3,
                btn_a=0, btn_b=1, btn_x=2, btn_y=3,
                btn_m1=4, btn_m2=5, btn_n1=6, btn_n2=7,
            )
        super().__init__(host, port, mapping)


class LoRaControllerBackend(ControllerBackend):
    """LoRa radio board controller backend - receives controller input over LoRa"""
    
    def __init__(self, lora_bridge=None, mapping: ControllerMapping = None):
        self.lora_bridge = lora_bridge
        self.mapping = mapping or ControllerMapping()
        self.running = False
        self.last_state = ControllerState()
        self._rx_callback = None
        
    async def start(self) -> bool:
        if not self.lora_bridge:
            logger.error("LoRa bridge not provided")
            return False
            
        self._rx_callback = self.lora_bridge.set_received_callback(self._handle_lora_frame)
        self.running = True
        logger.info("LoRa controller backend started")
        return True
    
    async def stop(self):
        self.running = False
        if self._rx_callback and self.lora_bridge:
            self.lora_bridge.set_received_callback(None)
        self._rx_callback = None
    
    def _handle_lora_frame(self, frame):
        """Handle received LoRa frame with controller input"""
        try:
            import struct
            # Frame type for controller input
            from fc.mavlink.lora_protocol import LoRaMessageType
            
            if frame.msg_type == LoRaMessageType.CMD_SET_POSITION:
                # Repurpose position command for controller state
                payload = frame.payload
                if len(payload) >= 32:
                    # Parse controller state from payload
                    axes_data = struct.unpack('<ffff', payload[:16])  # roll, pitch, yaw, throttle
                    buttons_data = struct.unpack('<II', payload[16:24])  # buttons low, buttons high
                    buttons_extra = struct.unpack('<H', payload[24:26])[0]  # M1,M2,N1,N2
                    
                    state = ControllerState(connected=True)
                    state.axes = {
                        self.mapping.axis_left_x: axes_data[0],
                        self.mapping.axis_left_y: axes_data[1],
                        self.mapping.axis_right_x: axes_data[2],
                        self.mapping.axis_right_y: axes_data[3],
                    }
                    
                    # Decode buttons
                    buttons = {}
                    for i in range(32):
                        if i < 16:
                            buttons[i] = bool(buttons_data[0] & (1 << i))
                        else:
                            buttons[i] = bool(buttons_data[1] & (1 << (i - 16)))
                    
                    # Custom buttons from extra field
                    buttons[self.mapping.btn_m1] = bool(buttons_extra & 0x01)
                    buttons[self.mapping.btn_m2] = bool(buttons_extra & 0x02)
                    buttons[self.mapping.btn_n1] = bool(buttons_extra & 0x04)
                    buttons[self.mapping.btn_n2] = bool(buttons_extra & 0x08)
                    
                    state.buttons = buttons
                    state.timestamp = frame.timestamp
                    self.last_state = state
                    
        except Exception as e:
            logger.error(f"Error processing LoRa controller frame: {e}")
    
    async def poll(self) -> Optional[ControllerState]:
        if self.last_state.connected:
            return self.last_state
        return None
    
    def get_device_info(self) -> Dict[str, Any]:
        return {
            'type': 'lora_radio',
            'connected': self.last_state.connected,
            'last_update': self.last_state.timestamp,
            'device_id': getattr(self.lora_bridge, 'device_id', 0) if self.lora_bridge else 0
        }


class GameControllerManager:
    """
    Manages game controller input for drone control
    Maps controller inputs to MAVLink commands via the bridge
    """
    
    def __init__(self, config: dict, mavlink_bridge=None):
        self.config = config
        self.mavlink_bridge = mavlink_bridge
        self.controller_config = config.get('controller', {})
        
        self.enabled = self.controller_config.get('enabled', True)
        self.backend_type = self.controller_config.get('backend', 'evdev')  # evdev, pygame, auto
        self.device_path = self.controller_config.get('device_path', '/dev/input/js0')
        self.controller_type = ControllerType(self.controller_config.get('type', 'generic'))
        self.poll_rate = self.controller_config.get('poll_rate', 50)  # Hz
        
        # Mapping
        self.mapping = CONTROLLER_MAPPINGS.get(self.controller_type, ControllerMapping())
        # Override with custom mapping from config
        custom_mapping = self.controller_config.get('mapping', {})
        for key, value in custom_mapping.items():
            if hasattr(self.mapping, key):
                setattr(self.mapping, key, value)
        
        # Backend
        self.backend: Optional[ControllerBackend] = None
        
        # State
        self.running = False
        self.poll_task = None
        self.last_state = ControllerState()
        
        # Callbacks
        self.control_callback: Optional[Callable] = None
        
        # Arming state
        self.armed = False
        self.last_arm_button = False
        self.arm_hold_time = 0
        self.arm_hold_required = 1.0  # seconds
        
        # Mode switching
        self.current_mode = 0
        self.modes = self.controller_config.get('modes', [0, 1, 2, 3, 4, 5, 6])  # ArduPilot modes
        self.mode_index = 0
        
        # RC channel mapping for override
        self.rc_channel_map = self.controller_config.get('rc_channels', {
            'roll': 1, 'pitch': 2, 'throttle': 3, 'yaw': 4,
            'mode': 5, 'aux1': 6, 'aux2': 7, 'aux3': 8
        })
    
    async def start(self):
        """Start controller manager"""
        if not self.enabled:
            logger.info("Controller disabled in config")
            return
        
        logger.info("Starting game controller manager...")
        
        # Initialize backend
        if self.backend_type in ('evdev', 'auto'):
            if EVDEV_AVAILABLE:
                self.backend = EvdevBackend(self.device_path, self.mapping)
                if not await self.backend.start():
                    self.backend = None
        
        if not self.backend and self.backend_type in ('pygame', 'auto'):
            if PYGAME_AVAILABLE:
                # Find joystick index
                joystick_id = self.controller_config.get('joystick_id', 0)
                self.backend = PygameBackend(joystick_id, self.mapping)
                if not await self.backend.start():
                    self.backend = None
        
        if not self.backend:
            logger.error("No controller backend available")
            return
        
        info = self.backend.get_device_info()
        logger.info(f"Controller connected: {info.get('name', 'Unknown')}")
        
        # Start polling loop
        self.running = True
        self.poll_task = asyncio.create_task(self._poll_loop())
        
        logger.info("Game controller manager started")
    
    async def stop(self):
        """Stop controller manager"""
        self.running = False
        
        if self.poll_task:
            self.poll_task.cancel()
            try:
                await self.poll_task
            except asyncio.CancelledError:
                pass
        
        if self.backend:
            await self.backend.stop()
            self.backend = None
        
        logger.info("Game controller manager stopped")
    
    def set_control_callback(self, callback: Callable):
        """Set callback for control commands"""
        self.control_callback = callback
    
    async def _poll_loop(self):
        """Main polling loop"""
        interval = 1.0 / self.poll_rate
        
        while self.running:
            try:
                state = await self.backend.poll()
                if state:
                    await self._process_state(state)
                    self.last_state = state
                
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Poll loop error: {e}")
                await asyncio.sleep(0.1)
    
    async def _process_state(self, state: ControllerState):
        """Process controller state and send commands"""
        if not self.mavlink_bridge:
            return
        
        # Check connection
        if not state.connected:
            logger.warning("Controller disconnected")
            return
        
        # Apply deadzone
        axes = self._apply_deadzone(state.axes)
        
        # Get mapped values
        roll = self._get_axis(axes, self.mapping.axis_left_x, self.mapping.max_roll)
        pitch = self._get_axis(axes, self.mapping.axis_left_y, self.mapping.max_pitch)
        if self.mapping.invert_left_y:
            pitch = -pitch
        
        yaw = self._get_axis(axes, self.mapping.axis_right_x, self.mapping.max_yaw)
        if self.mapping.invert_right_x:
            yaw = -yaw
        
        throttle = self._get_axis(axes, self.mapping.axis_right_y, self.mapping.max_throttle)
        if self.mapping.invert_right_y:
            throttle = -throttle
        
        # Triggers for auxiliary control
        lt = axes.get(self.mapping.axis_lt, 0)
        rt = axes.get(self.mapping.axis_rt, 0)
        
        # Send manual control (for ArduPilot/px4 in guided/manual mode)
        await self.mavlink_bridge.send_manual_control(
            x=int(roll),
            y=int(pitch),
            z=int(throttle * 500 + 500),  # Throttle 0-1000
            r=int(yaw),
            buttons=0
        )
        
        # Also send RC override for more direct control
        rc_channels = [0] * 18
        rc_channels[self.rc_channel_map.get('roll', 1) - 1] = self._axis_to_pwm(roll / self.mapping.max_roll)
        rc_channels[self.rc_channel_map.get('pitch', 2) - 1] = self._axis_to_pwm(pitch / self.mapping.max_pitch)
        rc_channels[self.rc_channel_map.get('throttle', 3) - 1] = self._axis_to_pwm(throttle / self.mapping.max_throttle)
        rc_channels[self.rc_channel_map.get('yaw', 4) - 1] = self._axis_to_pwm(yaw / self.mapping.max_yaw)
        
        # Mode switch from buttons
        mode_btn_lb = state.buttons.get(self.mapping.btn_lb, False)
        mode_btn_rb = state.buttons.get(self.mapping.btn_rb, False)
        
        # Handle buttons
        await self._handle_buttons(state)
        
        # Send RC override if any channel changed
        if any(c != 0 for c in rc_channels):
            await self.mavlink_bridge.send_rc_channels_override(rc_channels)
        
        # Call custom callback
        if self.control_callback:
            await self.control_callback({
                'roll': roll, 'pitch': pitch, 'yaw': yaw, 'throttle': throttle,
                'lt': lt, 'rt': rt,
                'buttons': state.buttons,
                'timestamp': state.timestamp
            })
    
    def _apply_deadzone(self, axes: Dict[int, float]) -> Dict[int, float]:
        """Apply deadzone to axis values"""
        result = {}
        for code, value in axes.items():
            if abs(value) < self.mapping.deadzone:
                result[code] = 0.0
            else:
                # Rescale to remove deadzone
                sign = 1 if value > 0 else -1
                result[code] = sign * (abs(value) - self.mapping.deadzone) / (1.0 - self.mapping.deadzone)
        return result
    
    def _get_axis(self, axes: Dict[int, float], axis_code: int, max_val: int) -> int:
        """Get axis value scaled to max_val"""
        value = axes.get(axis_code, 0.0)
        return int(value * max_val)
    
    def _axis_to_pwm(self, axis: float) -> int:
        """Convert -1.0..1.0 axis to 1000..2000 PWM"""
        return int(axis * 500 + 1500)
    
    async def _handle_buttons(self, state: ControllerState):
        """Handle button presses"""
        buttons = state.buttons
        
        # Arm/Disarm (A button - hold for 1 second)
        btn_arm = buttons.get(self.mapping.btn_a, False)
        if btn_arm and not self.last_arm_button:
            self.arm_hold_time = state.timestamp
        elif btn_arm and self.last_arm_button:
            if state.timestamp - self.arm_hold_time >= self.arm_hold_required:
                if not self.armed:
                    await self.mavlink_bridge.arm(True)
                    self.armed = True
                    logger.info("Armed via controller")
                else:
                    await self.mavlink_bridge.arm(False)
                    self.armed = False
                    logger.info("Disarmed via controller")
                self.arm_hold_time = 0  # Prevent retrigger
        else:
            self.arm_hold_time = 0
        self.last_arm_button = btn_arm
        
        # RTL (B button)
        if buttons.get(self.mapping.btn_b, False) and not getattr(self, '_last_btn_b', False):
            await self.mavlink_bridge.trigger_rtl()
            logger.info("RTL triggered via controller")
        self._last_btn_b = buttons.get(self.mapping.btn_b, False)
        
        # Land (X button)
        if buttons.get(self.mapping.btn_x, False) and not getattr(self, '_last_btn_x', False):
            await self.mavlink_bridge.land()
            logger.info("Land triggered via controller")
        self._last_btn_x = buttons.get(self.mapping.btn_x, False)
        
        # Takeoff (Y button)
        if buttons.get(self.mapping.btn_y, False) and not getattr(self, '_last_btn_y', False):
            altitude = self.controller_config.get('takeoff_altitude', 10.0)
            await self.mavlink_bridge.takeoff(altitude)
            logger.info(f"Takeoff to {altitude}m triggered via controller")
        self._last_btn_y = buttons.get(self.mapping.btn_y, False)
        
        # Mode switch (LB/RB)
        if buttons.get(self.mapping.btn_lb, False) and not getattr(self, '_last_btn_lb', False):
            self.mode_index = (self.mode_index - 1) % len(self.modes)
            await self.mavlink_bridge.set_mode(self.modes[self.mode_index])
            logger.info(f"Mode switched to {self.modes[self.mode_index]}")
        self._last_btn_lb = buttons.get(self.mapping.btn_lb, False)
        
        if buttons.get(self.mapping.btn_rb, False) and not getattr(self, '_last_btn_rb', False):
            self.mode_index = (self.mode_index + 1) % len(self.modes)
            await self.mavlink_bridge.set_mode(self.modes[self.mode_index])
            logger.info(f"Mode switched to {self.modes[self.mode_index]}")
        self._last_btn_rb = buttons.get(self.mapping.btn_rb, False)
        
        # Emergency stop (Back button)
        if buttons.get(self.mapping.btn_back, False) and not getattr(self, '_last_btn_back', False):
            await self.mavlink_bridge.arm(False)
            await self.mavlink_bridge.trigger_rtl()
            logger.warning("EMERGENCY STOP via controller")
        self._last_btn_back = buttons.get(self.mapping.btn_back, False)
        
        # D-pad for fine control
        if buttons.get(self.mapping.btn_dpad_up, False):
            # Could adjust altitude target
            pass
        if buttons.get(self.mapping.btn_dpad_down, False):
            pass
        if buttons.get(self.mapping.btn_dpad_left, False):
            pass
        if buttons.get(self.mapping.btn_dpad_right, False):
            pass
    
    def get_status(self) -> dict:
        """Get controller status"""
        return {
            'enabled': self.enabled,
            'connected': self.last_state.connected,
            'backend': type(self.backend).__name__ if self.backend else None,
            'controller_type': self.controller_type.value,
            'device_info': self.backend.get_device_info() if self.backend else {},
            'last_update': self.last_state.timestamp,
            'battery_level': self.last_state.battery_level,
            'armed': self.armed,
            'current_mode': self.modes[self.mode_index] if self.modes else 0,
            'axes': self.last_state.axes,
            'buttons': self.last_state.buttons
        }


async def discover_controllers() -> List[Dict[str, Any]]:
    """Discover available game controllers"""
    controllers = []
    
    if EVDEV_AVAILABLE:
        # Scan /dev/input for joysticks
        import glob
        for device_path in glob.glob('/dev/input/event*'):
            try:
                device = InputDevice(device_path)
                caps = device.capabilities()
                # Check if it has joystick axes/buttons
                if ecodes.EV_ABS in caps or ecodes.EV_KEY in caps:
                    abs_caps = caps.get(ecodes.EV_ABS, [])
                    key_caps = caps.get(ecodes.EV_KEY, [])
                    
                    # Heuristic: has at least 2 axes and some buttons
                    if len(abs_caps) >= 2 and len(key_caps) >= 4:
                        controllers.append({
                            'path': device_path,
                            'name': device.name,
                            'phys': device.phys,
                            'vendor': device.info.vendor,
                            'product': device.info.product,
                            'backend': 'evdev',
                            'axes': len(abs_caps),
                            'buttons': len(key_caps)
                        })
                device.close()
            except:
                pass
    
    if PYGAME_AVAILABLE:
        pygame.init()
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            controllers.append({
                'id': i,
                'name': js.get_name(),
                'backend': 'pygame',
                'axes': js.get_numaxes(),
                'buttons': js.get_numbuttons(),
                'hats': js.get_numhats()
            })
            js.quit()
        pygame.joystick.quit()
    
    return controllers