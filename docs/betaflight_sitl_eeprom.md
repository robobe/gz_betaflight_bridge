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
RX_MSP enabled
RX_UDP disabled
```

The CLI commands are:

```text
feature -RX_UDP
feature RX_MSP
aux 0 0 0 1700 2100 0 0
aux 1 1 1 1700 2100 0 0
save
```

`RX_MSP` is required for scripts that send RC with `MSP_SET_RAW_RC`, such as:

```text
scripts/missions/hover_msp_controller.py
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
scripts/run/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

SITL loads the commands, saves `eeprom.bin`, and exits.

Start SITL normally after that:

```bash
scripts/run/run_betaflight_sitl.sh
```

## Send RC

The legacy UDP RC script only works when `RX_UDP` is enabled. The current profile enables `RX_MSP`, so use the MSP hover controller for RC control.

```bash
scripts/missions/hover_msp_controller.py --target-altitude 5
```

To return to UDP RC smoke tests, change the features back to:

```text
feature -RX_MSP
feature RX_UDP
```

The MSP hover sequence starts with AUX1 low so Betaflight sees a valid disarmed state first. It then arms at low throttle before controlling throttle. Starting with AUX1 already high can trigger Betaflight's `NOT_DISARMED` arming blocker.

If motors stay at zero, inspect Betaflight arming flags. The EEPROM config maps the modes, but Betaflight can still block arming for other reasons.
