# Bridge observability

The bridge logs lifecycle, first-packet, timeout, and periodic status messages.

Startup logs include:

```text
Using config [...]
Subscribed to IMU [...]
Publishing actuators on [...]
Listening for Betaflight motors on UDP 9002
Sending FDM to 127.0.0.1:9003
```

First packet logs:

```text
First FDM packet sent: altitude=... pressure=...
First motor packet received: [...]
```

Periodic status:

```text
status imu=true altimeter=true fdm_packets=... motor_packets=... malformed_motor_packets=...
```

## Useful checks

List Gazebo topics:

```bash
gz topic -l | sort
```

Watch actuator output:

```bash
gz topic -e -t /X3/gazebo/command/motor_speed
```

Check SITL UDP sockets:

```bash
ss -lunp | grep -E '9002|9003|9004'
```

Check bridge logs for:

- `imu=true`
- `altimeter=true`
- increasing `fdm_packets`
- increasing `motor_packets`
- `malformed_motor_packets=0`

If motor packets stop, the bridge logs a timeout and publishes zero motor velocity.
