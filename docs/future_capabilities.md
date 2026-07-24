# Future capabilities

This document collects improvement ideas that would make the simulation more realistic, easier to test, and easier to tune.

## Battery-Aware Motor Scaling

Current behavior:

```text
normalized_motor_command -> fixed max_rotor_velocity_rad_s
```

Suggested behavior:

```text
normalized_motor_command
  -> battery voltage model
  -> motor KV model
  -> propeller load factor
  -> dynamic max rotor velocity
```

Example configuration:

```yaml
battery:
  cells: 4
  capacity_mah: 1500
  nominal_cell_voltage: 3.7
  full_cell_voltage: 4.2
  internal_resistance_ohm: 0.018
  initial_state_of_charge: 1.0

motor_model:
  kv: 2300
  load_factor: 0.25
  min_voltage: 12.0
```

Estimated no-load speed:

```text
rpm_no_load = kv * battery_voltage
rad_s_no_load = rpm_no_load * 2*pi / 60
max_rotor_velocity = rad_s_no_load * load_factor
```

Benefits:

- Fresh battery produces more available rotor speed.
- Voltage sag reduces thrust during high throttle.
- Long tests can model declining battery state.
- Battery choice becomes part of simulation tuning.

## ESC and Motor Response Model

Current motor scaling is linear. Real systems have delay, saturation, idle behavior, and nonlinear thrust response.

Suggested parameters:

```yaml
esc:
  update_rate_hz: 480
  command_delay_ms: 4
  spin_min: 0.05
  deadband: 0.02
  response_time_up_s: 0.025
  response_time_down_s: 0.040
  curve: quadratic
```

Benefits:

- More realistic throttle response.
- Better PID tuning behavior.
- Easier comparison with real Betaflight logs.

## Full Gazebo State Feedback

Current FDM feedback uses IMU and altimeter data. A richer version should subscribe to model pose, linear velocity, and angular velocity.

Suggested additions:

- Model pose from `/world/quadcopter/pose/info`.
- Dynamic pose from `/world/quadcopter/dynamic_pose/info`.
- Velocity in world frame.
- Frame conversion from Gazebo ENU to Betaflight expectations.

Benefits:

- Better altitude and vertical velocity behavior.
- Future GPS simulation.
- Better debugging of position drift and estimator behavior.

## Sensor Noise, Bias, and Delay

Real sensors are not perfect. Add configurable models:

```yaml
sensors:
  imu:
    gyro_noise_stddev: 0.002
    accel_noise_stddev: 0.03
    gyro_bias_walk: 0.0001
    delay_ms: 2
  altimeter:
    noise_stddev_m: 0.05
    delay_ms: 10
```

Benefits:

- More realistic estimator behavior.
- Ability to test robustness.
- Easier transition from SITL to hardware expectations.

## Wind and Disturbance Model

Add an optional Gazebo wind system or bridge-side disturbance test mode.

Useful test cases:

- Constant wind.
- Gusts.
- Vertical drafts.
- Random turbulence.

Benefits:

- Better controller tuning.
- More realistic takeoff and hover tests.
- Repeatable disturbance scenarios.

## Motor Failure Injection

Add runtime controls to degrade or fail motors:

```yaml
failures:
  motor_2:
    scale: 0.6
    start_time_s: 15
```

Possible failure modes:

- Motor stuck at zero.
- Motor limited to a percentage.
- Delayed motor response.
- Intermittent motor command dropout.

Benefits:

- Failsafe testing.
- Control robustness experiments.
- Better documentation of expected failure behavior.

## RC and Mission Profiles

The current RC helper is a smoke-test sender. It could become a small scenario runner:

```yaml
scenario:
  - duration: 3
    arm: false
    angle: true
    throttle: 1000
  - duration: 5
    arm: true
    angle: true
    throttle: 1000
  - duration: 10
    arm: true
    angle: true
    throttle_ramp: [1000, 1600]
  - duration: 20
    arm: true
    angle: true
    throttle: 1500
```

Benefits:

- Repeatable test flights.
- Easier regression tests.
- No need to hard-code RC behavior in Python arguments.

## Motor Order Verification

Add a tool that commands one motor at a time and asks the user to confirm the expected rotor.

Benefits:

- Catches wrong `motors.map` values.
- Prevents unstable takeoff from incorrect motor order.
- Useful when switching models.

## Multi-Vehicle Support

Support multiple Betaflight SITL instances:

```yaml
vehicles:
  x3_1:
    namespace: /X3_1
    motor_port: 9002
    fdm_port: 9003
    rc_port: 9004
  x3_2:
    namespace: /X3_2
    motor_port: 9012
    fdm_port: 9013
    rc_port: 9014
```

Benefits:

- Swarm experiments.
- Collision avoidance tests.
- Multiple controller comparisons in one world.

## Better Observability

Add optional metrics output:

- Prometheus text endpoint.
- CSV logs.
- Flight summary JSON.
- Packet rate and latency statistics.
- Min, max, and average motor commands.
- Last arming blocker observed from SITL logs if available.

Benefits:

- Easier debugging.
- Better automated test reports.
- Faster tuning feedback.

