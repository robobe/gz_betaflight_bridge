# MSP yaw excursion mission

## Mission

The simulation POC synthesizes RC commands through MSP; no joystick is
required. It uses the proven `msp_hover` altitude policy and inserts two yaw
legs before landing:

```text
PREARM -> ARMING -> LIFTOFF -> TAKEOFF (continuous 1 m/s ramp)
       -> SETTLE -> CCW 180 degrees at 15 deg/s
       -> CW to the captured home heading at 15 deg/s
       -> DESCEND at 1 m/s -> confirm landing -> DISARM
```

The default target altitude is 5 m. All mission and controller values live in
`scripts/msp_yaw_mission/msp_yaw_mission.yaml`.

## Run

Start Gazebo, Betaflight SITL, and the bridge. Betaflight must expose MSP TCP
on `127.0.0.1:5761` and map AUX1 to ARM and AUX2 to ANGLE.

From the repository root:

```bash
python3 scripts/msp_yaw_mission/run_msp_yaw_mission.py
```

Useful one-run overrides are:

```bash
python3 scripts/msp_yaw_mission/run_msp_yaw_mission.py \
  --target-altitude 5 \
  --yaw-angle 180 \
  --yaw-rate 15
```

Use `--reverse-yaw` only when the simulated model maps RC yaw in the opposite
direction. Use `--help` for all supported overrides; edit the colocated YAML
for the complete tuning surface.

## Control ownership

The altitude PID has exclusive ownership of throttle during takeoff, both yaw
legs, and descent. Yaw control never adds throttle compensation. While yawing,
the controller continues to evaluate altitude at the normal loop rate. It
centers yaw whenever altitude error exceeds 0.30 m or vertical speed exceeds
0.50 m/s, then resumes the yaw ramp after altitude recovers. An altitude error
above 0.75 m aborts the maneuver and starts a controlled landing.

Heading is unwrapped across the -180/180 boundary. A closed-loop yaw-rate
controller targets 15 deg/s, reduces authority over the last 60 degrees, and
requires the heading to remain within 5 degrees for 0.5 s. The first leg is
CCW; the second leg is CW back to the heading captured after takeoff settles.

## Modules

| Module | Responsibility |
|---|---|
| `msp_core` | MSP framing, TCP transport, typed telemetry, RC frames, timing |
| `flight_control.altitude` | Reusable altitude PID, velocity estimator, throttle slew limiter |
| `msp_hover.controller.HoverConfig` | Shared takeoff, hover, descent, and safety parameters |
| `msp_yaw_mission/controller.py` | Mission phases and altitude/yaw coordination |
| `msp_yaw_mission/yaw_control.py` | Heading unwrapping, rate estimation, yaw-rate command |
| `msp_yaw_mission/flight_log.py` | Yaw mission CSV and summary metrics |
| `msp_yaw_mission/cli.py` | YAML loading and command-line overrides |

This keeps MSP communication reusable and leaves `msp_yaw_mission` as a small
flight-policy module.

## Logs and tuning

Each run writes full-rate data to `logs/msp-yaw/yaw-*.csv` plus a JSON summary.
The CSV includes raw Betaflight vario, the filtered velocity used by the
altitude PID, heading target/error inputs, measured yaw rate, desired/sent
throttle, desired/sent yaw, mode state, and mission phase. The summary reports
yaw-rate RMSE, peak yaw rate, maximum altitude error during yaw, landing
confirmation, and failure reason.

Tune altitude hold with `msp_hover` first. For yaw tuning, change one YAML
parameter per run, compare the JSON metrics and CSV trace, and keep altitude
gating enabled. Tune `rate_feedforward_pwm` to overcome the yaw deadband,
`rate_kp_pwm_per_dps` for rate tracking, and `max_offset_pwm` as the hard
authority limit.

## Safety

Throttle is raised only after ARM and ANGLE confirmation. Unexpected disarm,
loss of ANGLE, stale telemetry, phase timeout, excessive yaw altitude error,
or MSP failure ends normal execution. In-flight policy failures use controlled
descent where possible. Every exit attempts a low-throttle DISARM burst.

This is simulation-only POC code. Betaflight and bridge failsafes remain
necessary because a disconnected MSP link cannot receive the final disarm
frame.

## Tests

```bash
PYTHONPATH=scripts python3 -m unittest test.test_msp_yaw_mission -v
```

The mission-level test drives the public controller seam through takeoff,
CCW, CW, landing confirmation, and final disarm.
