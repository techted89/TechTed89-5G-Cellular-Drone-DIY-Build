# Flight Controller (FC) Software
# Runs on Pixhawk, Cube, or similar autopilot hardware

fc/
├── mavlink/
│   ├── __init__.py
│   ├── flight_controller.py    # MAVLink interface to SBC
│   └── sensor_hub.py           # Sensor data aggregation
├── failsafe/
│   ├── __init__.py
│   └── dynamic_rth.py          # Dynamic Return-to-Home
├── sensors/
│   ├── __init__.py
│   ├── gps.py                  # GPS driver
│   ├── imu.py                  # IMU driver
│   ├── barometer.py            # Barometer driver
│   ├── battery.py              # Battery monitor
│   └── rangefinder.py          # Lidar/sonar for obstacle avoidance
├── config/
│   ├── __init__.py
│   └── fc_config.yaml          # FC-specific configuration
└── tests/
    ├── __init__.py
    └── test_failsafe.py