# SITL diagnostic tools

Run these commands from the project root while the simulation stack is active.

```bash
# Betaflight version, build, MSP API, features, sensors, GPS, and serial ports
python3 scripts/tools/inspect_sitl_msp.py

# Parsed virtual GPS fix and coordinates
python3 scripts/tools/check_gps_msp.py

# Raw Betaflight motor UDP packets
python3 scripts/tools/receive_motors.py

# Interactive motor-velocity publisher
python3 scripts/tools/motor_velocity_gui.py
```

The MSP tools default to `127.0.0.1:5761`; use `--host` and `--port` to override it.
