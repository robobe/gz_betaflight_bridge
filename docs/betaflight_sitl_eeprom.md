# Betaflight SITL EEPROM setup

Betaflight SITL stores persistent settings in:

```text
eeprom.bin
```

This file is generated runtime state and is ignored by git.

## CLI file

The project includes:

```text
config/betaflight/sitl_modes.cli
```

It configures:

```text
AUX1 high = ARM
AUX2 high = ANGLE
```

The CLI commands are:

```text
aux 0 0 0 1700 2100 0 0
aux 1 1 1 1700 2100 0 0
save
```

CLI field meaning:

```text
aux <slot> <mode_id> <aux_index> <start_us> <end_us> <logic> <linked_mode>
```

Mode IDs:

```text
0 = ARM
1 = ANGLE
```

AUX indices are zero-based:

```text
0 = AUX1 / RC channel 5
1 = AUX2 / RC channel 6
```

Range:

```text
1700..2100 us = switch high
```

Logic:

```text
0 = OR
```

## Generate EEPROM

Stop any running SITL process first:

```bash
pgrep -af betaflight_SITL
pkill -f betaflight_SITL
```

Then load the CLI file:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

SITL loads the commands, saves `eeprom.bin`, and exits.

Start SITL normally after that:

```bash
scripts/run_betaflight_sitl.sh
```

## Send RC

Arm and enable ANGLE mode:

```bash
scripts/send_rc_test.py --arm --angle --duration 10
```

Arm, enable ANGLE mode, and run the verified Gazebo takeoff sequence:

```bash
scripts/send_rc_test.py --takeoff-sequence
```

The takeoff sequence starts with AUX1 low so Betaflight sees a valid disarmed state first. It then arms at low throttle before ramping throttle. Starting the script with AUX1 already high can trigger Betaflight's `NOT_DISARMED` arming blocker.

If motors stay at zero, inspect Betaflight arming flags. The EEPROM config maps the modes, but Betaflight can still block arming for other reasons.
