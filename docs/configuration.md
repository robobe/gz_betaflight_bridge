# Bridge configuration

Default config:

```text
config/bridge.yaml
```

The executable also looks for `bridge.yaml` in the executable directory when `--config` is not passed. CMake copies `config/bridge.yaml` next to the built executable.

Run with an explicit config:

```bash
./build/debug/betaflight_gazebo_bridge --config config/bridge.yaml
```

Run through the helper script:

```bash
scripts/run_bridge.sh
```

## YAML

```yaml
sitl:
  address: 127.0.0.1
  motor_port: 9002
  fdm_port: 9003
  rc_port: 9004

gazebo:
  imu_topic: /imu
  altimeter_topic: /altimeter
  actuator_topic: /X3/gazebo/command/motor_speed

fdm:
  rate_hz: 500
  frame_mode: gazebo_bridge
  pressure_mode: from_altitude
  sea_level_pressure_pa: 101325.0

motors:
  input: normalized_9002
  map: [1, 2, 3, 0]
  min_rotor_velocity_rad_s: 0.0
  max_rotor_velocity_rad_s: 800.0
  timeout_seconds: 0.10
  publish_zero_on_timeout: true

logging:
  level: info
  status_period_seconds: 1.0
  log_first_packets: true
```

## Motor map

The map is interpreted as:

```text
Gazebo actuator N receives Betaflight motor map[N]
```

Example:

```yaml
map: [1, 2, 3, 0]
```

means:

```text
Gazebo actuator 0 receives Betaflight motor 1
Gazebo actuator 1 receives Betaflight motor 2
Gazebo actuator 2 receives Betaflight motor 3
Gazebo actuator 3 receives Betaflight motor 0
```

This default maps Betaflight Quad X motor order to the Gazebo X3 actuator order:

```text
Gazebo X3:   front-right, rear-left, front-left, rear-right
Betaflight:  rear-right, front-right, rear-left, front-left
```

The config loader rejects maps that do not contain exactly four unique indices in `0..3`.

## FDM frame mode

`fdm.frame_mode` controls how Gazebo IMU data is written into the Betaflight `fdm_packet`.

Use the default for Betaflight's `SITL_GAZEBO` target:

```yaml
fdm:
  frame_mode: gazebo_bridge
```

This converts raw Gazebo FLU IMU data into the packet convention expected by Betaflight's Gazebo bridge path:

```text
angular_velocity:  [x, y, z] -> [x, -y, -z]
linear_accel:      [x, y, z] -> [x, -y, -z]
quaternion wxyz:   [w, x, y, z] -> [w, x, -y, -z]
```

This matters for ANGLE mode. If the frame signs are wrong, Betaflight corrects tilt in the wrong direction and the vehicle can oscillate or flip over.

For debugging only, raw pass-through is available:

```yaml
fdm:
  frame_mode: passthrough
```
