# MSP hover tuning

This module owns the hover flight policy. Reusable MSP communication lives in
`scripts/msp_core`, altitude-control primitives live in
`scripts/flight_control`, and generic CSV storage lives in
`scripts/flight_log`.

## Safety boundary

Use this mission only with Gazebo and Betaflight SITL. Keep the Gazebo vehicle
visible during a trial. Stop after any failed takeoff, uncontrolled oscillation,
unexpected disarm, or failed landing; do not continue changing gains on a dirty
simulator state.

The controller requires ARM and ANGLE confirmation, uses a ramped takeoff,
scores hover separately, performs a controlled descent, confirms landing, and
always sends a low-throttle disarm burst. These checks reduce risk but cannot
make unstable gains safe.

`takeoff.climb_rate_mps` controls the continuous altitude ramp.
`takeoff.ready_speed_mps` and `takeoff.ready_dwell_s` apply only after the final
target is reached; they do not interrupt the climb.
`takeoff.climb_feedforward_pwm` adds thrust only while the takeoff target is
ramping; the normal PID and throttle slew limiter still bound the command.
`takeoff.max_lag_m` triggers the controlled abort path when the aircraft cannot
keep up with the ramp, rather than allowing the target to run away indefinitely.
The `liftoff_*` fields define the acquisition gate before that ramp: the target
stays at launch altitude until height or upward-speed evidence appears, and a
timeout aborts safely if the motors never produce liftoff.

## 1. Start from a clean simulation

Start the configured stack:

```bash
tmuxp load -d config/run_sim.yaml
```

Verify MSP, the bridge, and Gazebo before flight:

```bash
ss -ltnp '( sport = :5761 )'
ps -ef | rg 'betaflight_SITL|betaflight_gazebo_bridge|gz sim server'
tmux capture-pane -pt run_sim:1.4 -S -20
```

There must be one Gazebo server for this workspace. Multiple orphaned servers
can publish duplicate `/clock`, IMU, or altimeter data and invalidate every
measurement. Stop a stack gracefully by sending `Ctrl-C` to each pane before
killing its tmux session. Start the entire stack together between failed
trials; restarting SITL alone while sensors are active can prevent gyro
calibration and arming.

Wait until the bridge reports both `imu=true` and `altimeter=true`. During
prearm, the mission log must show `angle=1`. Takeoff starts only after it also
observes `armed=1`.

## 2. Run a baseline

The default configuration is colocated with the mission entrypoint:

```text
scripts/msp_hover/hover_msp_controller.py
scripts/msp_hover/msp_hover.yaml
```

Run at least three baseline trials from identical simulator initial conditions:

```bash
python3 scripts/msp_hover/hover_msp_controller.py
```

Each run creates a full-rate CSV and summary JSON in `logs/msp-hover`. A valid
tuning sample must reach `scored_hover`, complete its configured duration,
confirm landing, and have `safety.passed: true`. Arming failures, takeoff
timeouts, abort landings, and incomplete summaries are diagnostics—not tuning
samples.

## 3. Change one variable

Use CLI overrides for candidates so the checked-in YAML remains the baseline:

```bash
python3 scripts/msp_hover/hover_msp_controller.py --hover-throttle 1670
python3 scripts/msp_hover/hover_msp_controller.py --kp 18
python3 scripts/msp_hover/hover_msp_controller.py --kd 35
python3 scripts/msp_hover/hover_msp_controller.py --ki 8
```

Tune in this order:

1. Calibrate `hover_throttle` near steady flight.
2. Adjust `kd` in small increments for damping.
3. Adjust `kp` for altitude response.
4. Add only enough `ki` to remove persistent steady bias.

Do not change multiple gains in one comparison. Reset the complete stack after
a failed flight so the vehicle pose, Betaflight arming state, and sensors all
begin cleanly.

## 4. Compare repeated trials

Collect at least three valid baseline and three valid candidate summaries:

```bash
python3 scripts/tools/compare_hover_trials.py \
  --baseline logs/msp-hover/baseline-{1,2,3}-summary.json \
  --candidate logs/msp-hover/candidate-{1,2,3}-summary.json
```

The tool compares medians and prints `KEEP` or `REJECT`. Its default rule
requires at least 5% lower hover RMSE with no greater than 10% regression in
maximum error, vertical-speed RMS, throttle saturation, or oscillation rate.
It rejects unsafe and incomplete trials before comparing them.

Only copy a candidate into `scripts/msp_hover/msp_hover.yaml` after it passes
this repeated-trial comparison.

## 5. Inspect a failure

The CSV columns distinguish control intent from actual output:

- `desired_throttle_pwm` is the PID result;
- `sent_throttle_pwm` includes throttle slew limiting;
- `control_velocity_mps` is the filtered altitude derivative used by control;
- `raw_vario_mps` preserves the MSP-reported vario for diagnosis;
- `integral_gate` shows whether integration was allowed;
- `armed` and `angle_mode` show confirmed Betaflight state.

Plot altitude, target, vertical speed, and both throttle columns against
`elapsed_s`. Check the phase column first: takeoff transients must not be judged
as scored-hover performance.

## Live tuning note: 2026-08-07

No new gains were promoted during this session.

- The original `1660 / P20 / I10 / D30` run reached takeoff step 5 of 6, then
  developed a growing vertical oscillation and landed through the abort path.
- Duplicate orphaned Gazebo servers were discovered, so that run is not a valid
  baseline.
- After removing duplicate servers, `1660 / P10 / I2 / D60` was below reliable
  liftoff thrust.
- Raising only the base to `1680` produced a divergent vertical oscillation.
  Its abort descent also timed out, so `1680 / P10 / I2 / D60` is rejected.

The next engineering step is to improve the controller/abort robustness and
verify telemetry latency before resuming gain search. Do not treat the rejected
values above as recommended defaults.
