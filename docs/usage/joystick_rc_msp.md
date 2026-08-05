# MSP joystick RC usage

This guide runs manual joystick control through Betaflight MSP. The script reads a Linux joystick from `/dev/input/js0`, maps it to RC channels, and sends `MSP_SET_RAW_RC` commands to Betaflight SITL on TCP `127.0.0.1:5761`.

Use this workflow when Betaflight is configured for `RX_MSP`.

## One-time setup

From the repository root:

```bash
cd /home/user/projects/gz_betaflight_bridge
```

Generate the MSP RC EEPROM profile:

```bash
scripts/run/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

This profile sets:

```text
RX_MSP enabled
RX_UDP disabled
AUX1 high = ARM
AUX2 high = ANGLE
```

Calibrate the joystick once:

```bash
scripts/tools/joystick_rc_msp.py calibrate --device /dev/input/js0
```

Calibration writes:

```text
config/joystick_rc.json
```

## Start the simulator stack

In VS Code:

```text
Command Palette -> Tasks: Run Task -> Stack: run all
```

Wait until Betaflight SITL is listening on MSP TCP:

```bash
ss -ltnp | grep 5761
```

## Run MSP joystick control

Run:

```bash
scripts/tools/joystick_rc_msp.py run --device /dev/input/js0
```

Useful options:

```bash
scripts/tools/joystick_rc_msp.py run --device /dev/input/js0 --host 127.0.0.1 --port 5761 --rate 50
scripts/tools/joystick_rc_msp.py run --device /dev/input/js0 --config config/my_joystick.json
scripts/tools/joystick_rc_msp.py run --device /dev/input/js0 --print-period 0.25
```

The script prints live RC values:

```text
rc: roll=1500 pitch=1500 throttle=1000 yaw=1500 arm=0 angle=0
```

Keep throttle low before enabling ARM. When the script exits with `Ctrl+C`, it sends a short disarm burst: throttle low and AUX1 low.

## RC channel mapping

The script sends these Betaflight RC channels:

| RC channel | Field | Meaning |
|---:|---|---|
| 1 | `roll` | Roll stick |
| 2 | `pitch` | Pitch stick |
| 3 | `throttle` | Throttle stick |
| 4 | `yaw` | Yaw stick |
| 5 | `arm` | AUX1 ARM |
| 6 | `angle` | AUX2 ANGLE |

Channels 7-16 are sent at `1500`.

## JSON config

Default config:

```text
config/joystick_rc.json
```

Example shape:

```json
{
  "roll": {
    "axis": 0,
    "center": 0,
    "invert": true
  },
  "pitch": {
    "axis": 1,
    "center": 0,
    "invert": true
  },
  "throttle": {
    "axis": 2,
    "invert": false
  },
  "yaw": {
    "axis": 3,
    "center": 0,
    "invert": false
  },
  "arm": {
    "button": 3,
    "toggle": false
  },
  "angle": {
    "button": 1,
    "toggle": false
  },
  "deadzone": 0.05,
  "axis_expo": 0.35,
  "throttle_expo": 0.0,
  "min_rc": 1000,
  "mid_rc": 1500,
  "max_rc": 2000
}
```

Axis fields:

| Field | Meaning |
|---|---|
| `axis` | Linux joystick axis number |
| `center` | Raw center value for centered axes |
| `invert` | Reverse the axis direction |

Button fields:

| Field | Meaning |
|---|---|
| `button` | Linux joystick button number |
| `toggle` | `true` toggles state on each press; `false` follows the physical button/switch |

Response fields:

| Field | Meaning |
|---|---|
| `deadzone` | Removes small stick noise near center |
| `axis_expo` | Curves roll, pitch, and yaw response |
| `throttle_expo` | Curves throttle response |
| `min_rc` | Low RC endpoint |
| `mid_rc` | Center RC value |
| `max_rc` | High RC endpoint |

If an axis moves in the wrong direction, flip `invert` for that axis and restart the script.

## Throttle tuning

If takeoff needs almost full stick, first make throttle response linear:

```json
"throttle_expo": 0.0
```

If hover still happens too high on the stick, increase the rotor speed limit in both files:

```yaml
# config/bridge.yaml
motors:
  max_rotor_velocity_rad_s: 1000.0
```

```xml
<!-- worlds/quadcopter.sdf -->
<maxRotVelocity>1000.0</maxRotVelocity>
```

Keep those two values the same. The bridge converts Betaflight's normalized motor output into rad/s, and Gazebo's motor model clamps accepted rotor speed with `<maxRotVelocity>`.

The current tuning target changes the estimated hover fraction from:

```text
660 / 800 = 0.825
```

to:

```text
660 / 1000 = 0.66
```

If the drone becomes too sensitive or climbs too aggressively, reduce both values to `900.0`.

## Troubleshooting

Check that Linux sees the joystick:

```bash
ls -l /dev/input/js*
```

If permission is denied:

```bash
sudo usermod -aG input "$USER"
```

Then log out and log in again.

If Betaflight does not react, confirm the MSP profile and TCP listener:

```bash
scripts/run/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
ss -ltnp | grep 5761
```

If the joystick mapping is wrong, recalibrate:

```bash
scripts/tools/joystick_rc_msp.py calibrate --device /dev/input/js0
```
