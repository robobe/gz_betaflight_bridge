# MSP hover Python usage

The controller connects to Betaflight SITL over MSP, performs a ramped takeoff,
a measured hover trial, and a confirmed landing. It does not start Gazebo, SITL,
or the bridge.

## One-time setup

Generate the SITL EEPROM profile that maps MSP RC AUX1 to ARM and AUX2 to ANGLE:

```bash
scripts/run/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

## Start the simulation

In VS Code run `Tasks: Run Task` -> `Stack: run all`. Wait for live bridge IMU
and altimeter messages. Confirm the MSP listener if connection fails:

```bash
ss -ltnp | grep 5761
```

## Run a trial

Review `scripts/msp_hover/msp_hover.yaml`, then from the repository root run:

```bash
python3 scripts/msp_hover/hover_msp_controller.py
```

Common temporary overrides are available without editing YAML:

```bash
python3 scripts/msp_hover/hover_msp_controller.py \
  --target-altitude 3 --duration 10 --hover-throttle 1660 \
  --kp 20 --ki 10 --kd 30
```

`--duration` is only the scored-hover interval. Prearm, ramped takeoff, settle,
descent, and landing confirmation are additional bounded phases.

| Phase | Behavior |
|---|---|
| `prearm` | Low throttle, ARM low, ANGLE requested |
| `arming` | ARM requested; waits for ARM and ANGLE telemetry confirmation |
| `liftoff` | Holds launch target, applies climb feed-forward, and waits for movement evidence |
| `takeoff` | Launch-relative altitude target ramps continuously at the configured climb rate |
| `settle` | Holds final target until altitude and speed dwell gates pass |
| `scored_hover` | Holds altitude and records the comparison metrics |
| `descend` | Ramps the altitude target down at `landing.descent_rate_mps` |
| `abort_descend` | Controlled landing following an in-flight failure |
| disarm | Requires confirmed low height and vertical speed, then sends low throttle |

Safety limits, takeoff ramp, landing confirmation, PID gains, and throttle
limits live in the YAML beside the entrypoint. The controller also fails on
unexpected disarm, ANGLE loss, stale MSP responses, or excessive altitude error.

## Reading the output

Every trial writes:

```text
logs/msp-hover/hover-<UTC timestamp>.csv
logs/msp-hover/hover-<UTC timestamp>-summary.json
```

The CSV includes phase, target, measured altitude, error, raw MSP vario,
filtered control velocity,
desired/sent throttle, integrator, and ARM/ANGLE state. The summary separates
takeoff, scored hover, landing, and safety results.

Run three repeatable baseline trials and three trials with one parameter changed,
then compare median performance:

```bash
python3 scripts/tools/compare_hover_trials.py \
  --baseline baseline-1-summary.json baseline-2-summary.json baseline-3-summary.json \
  --candidate candidate-1-summary.json candidate-2-summary.json candidate-3-summary.json
```

Tune one variable per cycle. Establish hover throttle first, use `kd` for
damping, adjust `kp` for response, and use `ki` only to remove persistent bias.
Never keep a gain change that improves RMSE by violating a safety result or by
materially worsening maximum error, vertical-speed RMS, saturation, or
oscillation.

See `scripts/msp_hover/README.md` for the clean-stack procedure, repeated-trial
acceptance rules, failure analysis, and recorded live tuning results.
