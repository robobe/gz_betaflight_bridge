# MSP yaw excursion mission

## Purpose

This proof-of-concept Python mission controls Betaflight SITL through MSP. It
does not require a physical joystick. It synthesizes a complete RC frame with
`MSP_SET_RAW_RC`, observes altitude and attitude, and performs this sequence:

```text
PREARM
  -> ARMING
  -> TAKEOFF steps at +0.5, +1.0, +1.5, +2.0, +2.5, +3.0 m
  -> SETTLE
  -> YAW clockwise 180 degrees
  -> YAW counterclockwise to the initial heading
  -> DESCEND at a 1 m/s target rate
  -> confirm landing
  -> DISARM
```

The supported starting point is Betaflight `2025.12.x` SITL.

## Run it

Start Gazebo, Betaflight SITL, and the bridge using the existing project
workflow. Betaflight must expose MSP TCP on `127.0.0.1:5761`, and its EEPROM
must map AUX1 to ARM and AUX2 to ANGLE as described in
`docs/betaflight_sitl_eeprom.md`.

From the repository root, run:

```bash
python3 scripts/missions/run_msp_yaw_mission.py
```

The equivalent module entry point is:

```bash
PYTHONPATH=scripts python3 -m msp_yaw_mission
```

Useful tuning options include:

```bash
python3 scripts/missions/run_msp_yaw_mission.py \
  --target-height 3 \
  --takeoff-step 0.5 \
  --step-dwell 0.75 \
  --step-speed 0.30 \
  --descent-rate 1 \
  --hover-throttle 1660 \
  --altitude-kp 20 \
  --altitude-ki 10 \
  --altitude-kd 30 \
  --min-throttle 1300 \
  --max-throttle 1850 \
  --throttle-slew-rate 1000 \
  --yaw-max-offset 60 \
  --yaw-min-offset 20 \
  --yaw-slew-rate 60
```

Use `--help` for the complete option list.

State-transition and mission-outcome lines use ANSI colors when standard
output is an interactive terminal. Set the standard `NO_COLOR` environment
variable to disable them:

```bash
NO_COLOR=1 python3 scripts/missions/run_msp_yaw_mission.py
```

## MSP messages

| Direction | MSP message | Use |
|---|---|---|
| Script to Betaflight | `MSP_SET_RAW_RC` (`200`) | Roll, pitch, throttle, yaw, ARM, ANGLE, and ALTHOLD switch positions |
| Betaflight to script | `MSP_ALTITUDE` (`109`) | Estimated altitude and vertical velocity |
| Betaflight to script | `MSP_ATTITUDE` (`108`) | Roll, pitch, and yaw heading |
| Betaflight to script | `MSP_BOXIDS` (`119`) | Maps mode bits to permanent ARM and ANGLE identifiers |
| Betaflight to script | `MSP_STATUS` (`101`) | Confirms armed state and active ANGLE mode |

The script polls telemetry in one deterministic loop. It sends centered roll
and pitch, keeps native ALTHOLD disabled on AUX3, adjusts throttle with a
Python altitude PID, and adjusts yaw with a direction-constrained controller.

## Control behavior

### Altitude

The script captures the pre-arm altitude instead of assuming ground is exactly
zero. The hover target is:

```text
hover target = launch altitude + 3 m
```

Takeoff does not apply that target in one jump. It builds launch-relative
0.5 m targets and advances only when altitude is within 0.15 m of the current
step and absolute vertical speed is at most 0.30 m/s for 0.75 seconds. Each
step has 20 seconds to settle. After the final step, the yaw maneuver starts
only after the final hover remains stable for 1 second. During yaw, leaving
the target by more than 0.75 m is a safety failure.

The altitude controller includes integral correction by default so a modest
difference between configured and actual hover throttle does not leave a
permanent altitude error. The integrator is bounded and stops accumulating
when its error would push an already saturated throttle farther into
saturation.

The X3 model's observed lift / hover baseline is approximately 1660 RC units,
so that is the default feed-forward throttle. With the 0.5 m first-step error
and `Kp=20`, initial PID demand is 1670: above lift threshold without returning
to the aggressive 1850-range launch command.

Integral correction is enabled only within 0.30 m of the active altitude
target and below 0.50 m/s vertical speed. The learned correction persists
between takeoff steps, but fast or distant transients cannot wind it up.

The derivative term uses Betaflight's vario value from `MSP_ALTITUDE` directly.
It does not differentiate successive altitude fields because those fields may
update in quantized steps at a lower rate than the MSP control loop, producing
false vertical-speed spikes and large throttle kicks.

The default proportional gain and flight throttle limits are deliberately
conservative for the X3 model's delayed thrust response. Restricting commanded
throttle to `1300..1850` while armed prevents the controller from alternating
between near-minimum and maximum throttle and exciting a growing vertical
oscillation. DISARM still sends throttle `1000`.

PID throttle demand is also passed through a 1000 RC-unit-per-second slew
limiter. At the default 25 Hz loop rate, the transmitted throttle changes by
at most 40 units per cycle. Emergency DISARM bypasses the limiter.

If a takeoff step or final settle times out, the mission does not immediately
disarm in the air. It records the failure, descends through 0.5 m targets,
confirms landing, sends DISARM, and exits with a failure status. A successful
yaw mission retains the continuous 1 m/s landing profile.

After the return yaw completes, the altitude target decreases at 1 m/s until
it reaches launch altitude. Landing requires altitude no more than 0.15 m
above launch and vertical speed no more than 0.15 m/s for 0.5 seconds.

### Yaw

`MSP_ATTITUDE` yaw wraps at the heading boundary, so the script unwraps it
into a continuous heading. This avoids ambiguity at exactly 180 degrees.

The first target is the captured heading plus 180 degrees and is constrained
to clockwise motion. The second target is the captured initial heading and is
constrained to counterclockwise motion. The command is neutral inside 5
degrees of the target and must remain there for 0.5 seconds.

The default synthesized yaw values are:

| Command | RC value |
|---|---:|
| Full configured clockwise authority | `1560` |
| Neutral | `1500` |
| Full configured counterclockwise authority | `1440` |

Yaw authority ramps at 60 RC units per second instead of stepping directly
from center to full command. Authority decreases during the final 60 degrees,
but remains at least 20 RC units away from center until the target tolerance
is reached.

Altitude has priority over yaw. If altitude error exceeds 0.15 m or absolute
vertical speed exceeds 0.30 m/s, yaw returns to center immediately. Rotation
resumes with a gradual ramp after the altitude loop recovers. The default yaw
timeout is 40 seconds to allow for these recovery pauses.

The altitude PID has exclusive ownership of throttle during yaw. Yaw control
never adds to or subtracts from the PID throttle command. If yaw disturbs the
vehicle beyond the altitude gate, yaw returns to center and the PID recovers
altitude before rotation resumes.

If the vehicle's RC yaw polarity is reversed, add `--reverse-yaw`. Direction
verification still uses attitude feedback and aborts if the selected polarity
does not produce the requested rotation.

## Safety behavior

The script raises throttle only after `MSP_STATUS` confirms ARM. It never
automatically re-arms. All normal exits, failures, and keyboard interrupts run
a low-throttle DISARM burst.

The mission fails and disarms when any of these occurs:

- Betaflight does not arm within the configured timeout.
- Betaflight unexpectedly disarms in flight.
- ANGLE mode is not active after arming.
- Roll or pitch exceeds 30 degrees for 0.2 seconds.
- Altitude error exceeds 0.75 m during yaw.
- An altitude/attitude request cycle exceeds 0.3 seconds.
- A phase exceeds its timeout.
- The yaw heading moves in the wrong direction or makes no progress.
- MSP disconnects or returns malformed telemetry.

This is a simulation POC. A communication failure can prevent the disarm
frame from reaching Betaflight, so the Betaflight and bridge failsafes remain
necessary independent protections.

Takeoff logs include `step`, `err`, `dwell`, `igate`, `pid`, and `throttle`.
`pid` is the bounded PID demand; `throttle` is the slew-limited command that
was actually sent. `igate=1` means integral correction is currently allowed.

## SOLID structure

The implementation keeps shared protocol mechanics separate from flight
policy:

| Module | Responsibility |
|---|---|
| `scripts/msp_core/protocol.py` | MSP v1 framing and command identifiers |
| `scripts/msp_core/client.py` | TCP connection and request/response transport |
| `scripts/msp_core/telemetry.py` | Typed altitude, attitude, and status decoding |
| `scripts/msp_core/rc.py` | Complete synthesized RC frames |
| `scripts/msp_core/timing.py` | Fixed-rate loop timing |
| `scripts/msp_yaw_mission/control.py` | Altitude PID, heading unwrapping, and yaw command calculation |
| `scripts/msp_yaw_mission/mission.py` | Mission phases, transitions, and safety policy |
| `scripts/msp_yaw_mission/cli.py` | User configuration boundary |

This applies SOLID pragmatically:

- Each module has one reason to change.
- Mission code depends on small MSP collaborators supplied to its constructor.
- Later scripts can reuse the MSP core without inheriting this mission.
- Existing `msp_hover` imports remain as compatibility adapters, avoiding a
  forced migration of every current script.

The architectural boundary is recorded in
`docs/adr/0001-separate-msp-core-from-flight-missions.md`.

## Tests

Run the new focused tests with:

```bash
PYTHONPATH=scripts python3 -m unittest \
  test.test_msp_core \
  test.test_msp_yaw_mission
```

They cover MSP field decoding, status-bit mapping, heading wraparound,
direction-constrained yaw commands, reduced authority near the target,
throttle limits, dwell gates, phase timeout, altitude escape, and wrong yaw
direction.

## First simulation check

Watch the terminal state transitions and keep Gazebo visible. If the first yaw
moves counterclockwise, stop the mission and rerun it with `--reverse-yaw`;
the mission should detect the wrong
direction and disarm rather than correcting polarity while airborne.

Tune altitude control before increasing yaw authority. A yaw maneuver should
not be used to compensate for a vehicle that cannot already hold 3 m with
centered roll, pitch, and yaw.
