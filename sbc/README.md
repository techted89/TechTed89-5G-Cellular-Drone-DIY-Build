# SBC (Single Board Computer) - Companion Computer Software
# Runs on Jetson, Raspberry Pi, or similar Linux SBC

sbc/
├── main.py                 # Main entry point
├── mavlink/
│   ├── __init__.py
│   └── mavlink_bridge.py   # MAVLink serial <-> WebRTC bridge
├── webrtc/
│   ├── __init__.py
│   └── webrtc_server.py    # WebRTC video + data channels
├── tailscale/
│   ├── __init__.py
│   └── tailscale_manager.py # Tailscale mesh management
├── sms/
│   ├── __init__.py
│   └── sms_beacon.py       # Emergency SMS alerts
├── controller/
│   ├── __init__.py
│   └── game_controller.py  # USB/Bluetooth gamepad support
├── api/
│   ├── __init__.py
│   └── rest_api.py         # REST API server
└── tests/
    ├── __init__.py
    └── test_mavlink.py