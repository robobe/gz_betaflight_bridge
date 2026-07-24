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
  pressure_mode: from_altitude
  sea_level_pressure_pa: 101325.0

motors:
  input: normalized_9002
  map: [0, 1, 2, 3]
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
map: [1, 0, 3, 2]
```

means:

```text
Gazebo actuator 0 receives Betaflight motor 1
Gazebo actuator 1 receives Betaflight motor 0
Gazebo actuator 2 receives Betaflight motor 3
Gazebo actuator 3 receives Betaflight motor 2
```

The config loader rejects maps that do not contain exactly four unique indices in `0..3`.

