# Joystick RC over UDP 9004

This tool reads a Linux joystick device, usually `/dev/input/js0`, and sends Betaflight SITL RC packets to UDP port `9004`.

It uses the same UDP packet format as `scripts/send_rc_test.py`:

```text
double timestamp_seconds
uint16 channels[16]
```

Channel mapping:

| RC channel | Betaflight meaning | Joystick source |
|---|---|---|
| 1 | Roll | Roll axis |
| 2 | Pitch | Pitch axis |
| 3 | Throttle | Throttle axis |
| 4 | Yaw | Yaw axis |
| 5 | AUX1 | ARM button or switch |
| 6 | AUX2 | ANGLE button or switch |

## Calibration plan

Run calibration once:

```bash
scripts/joystick_rc_udp.py calibrate --device /dev/input/js0
```

The script asks you to:

1. Center all sticks and press Enter.
2. Move roll full right.
3. Move pitch full forward.
4. Move throttle full high.
5. Move yaw full right.
6. Press the ARM button or switch.
7. Choose whether ARM is a toggle.
8. Press the ANGLE mode button or switch.
9. Choose whether ANGLE is a toggle.

For a spring-loaded push button, answer `y` for toggle mode. For a physical two-position switch, answer `n` so the RC channel follows the switch state directly.

It saves the detected mapping to:

```text
config/joystick_rc.json
```

## Run

The joystick sends legacy UDP RC, so Betaflight must use `RX_UDP`.

Stop the stack, then generate the UDP EEPROM profile once:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_udp_modes.cli
```

Start Gazebo, SITL, and the bridge:

```bash
scripts/run_takeoff_stack.sh
```

Then run:

```bash
scripts/joystick_rc_udp.py run --device /dev/input/js0
```

The default output is:

```text
udp://127.0.0.1:9004
```

The script prints live RC state:

```text
rc: roll=1500 pitch=1500 throttle=1000 yaw=1500 arm=0 angle=1
```

Keep throttle low before enabling ARM.

## Axis response

The saved config supports deadband and expo:

```json
{
  "deadzone": 0.05,
  "axis_expo": 0.35,
  "throttle_expo": 0.35
}
```

`deadzone` removes small stick noise around center.

`axis_expo` changes roll, pitch, and yaw response after the deadzone:

```text
output = (1 - expo) * input + expo * input^3
```

`throttle_expo` changes the throttle response on the positive `0..1` throttle range:

```text
throttle_output = (1 - expo) * throttle_input + expo * throttle_input^3
```

Use a value from `0.0` to `1.0`:

| Value | Behavior |
|---:|---|
| `0.0` | Linear |
| `0.35` | Softer near center, stronger near full stick |
| `0.7` | Very soft center, aggressive end |

The endpoint values stay fixed: low throttle remains `1000` and full throttle remains `2000`.

If one axis is reversed, edit its `invert` value in `config/joystick_rc.json` and restart the joystick script:

```json
"pitch": {
  "axis": 1,
  "center": 0,
  "invert": false
}
```

## Useful options

```bash
scripts/joystick_rc_udp.py run --device /dev/input/js0 --ip 127.0.0.1 --port 9004 --rate 50
scripts/joystick_rc_udp.py calibrate --device /dev/input/js0 --config config/my_joystick.json
scripts/joystick_rc_udp.py run --device /dev/input/js0 --config config/my_joystick.json
```

## Troubleshooting

Check that Linux sees the joystick:

```bash
ls -l /dev/input/js*
```

If permission is denied, add your user to the `input` group and log out/in:

```bash
sudo usermod -aG input "$USER"
```

If Betaflight does not react:

```bash
ss -lunp | grep 9004
```

Make sure SITL is running and listening on UDP `9004`.

If you later use the MSP hover or square controllers again, switch back to the MSP EEPROM profile:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```
