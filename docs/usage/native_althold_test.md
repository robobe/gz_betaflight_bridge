# Native Betaflight ALT HOLD SITL test

This scenario verifies Betaflight's native `ALTHOLD` mode through AUX3. The Python controller is used only to reach a stable takeoff altitude and to land. Betaflight controls altitude throughout the measured hold, climb, and descent phases.

## Configure the SITL EEPROM

Generate the EEPROM once, or repeat this step after changing the mode profile:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

The profile uses AUX1 for ARM, AUX2 for ANGLE, and AUX3 for ALTHOLD. It sets `ap_hover_throttle = 1700`, matching the upper end of the X3 model's observed hover range and avoiding a throttle drop caused by RC shaping during mode entry. AUX3 remains low until after arming and takeoff because Betaflight prevents arming while ALTHOLD is selected.

## Run the scenario

Start the simulation stack from VS Code:

```text
Command Palette -> Tasks: Run Task -> Stack: run tmuxp
```

Leave that tmux session running. In a second terminal, run:

```bash
python3 scripts/run_althold_test.py
```

Alternatively, select the VS Code task `Test: Betaflight ALT HOLD` after the stack is ready and the bridge is publishing live IMU and altimeter data.

The default scenario:

1. Arms with AUX3 low and takes off to 3 m using the Python PID.
2. Raises AUX3 and requires MSP to report native ALTHOLD active.
3. Holds the captured altitude for 5 seconds.
4. Uses throttle to command a 1 m climb, then holds the new altitude.
5. Uses throttle to descend to the original altitude, then holds again.
6. Lowers AUX3, verifies ALTHOLD is inactive, lands with the Python PID, and disarms.

Every measured hold must stay within 0.5 m of its target. A successful run prints `ALT HOLD scenario: PASS` and exits with status 0. A timeout, mode-state mismatch, or tolerance violation prints `FAIL`, safely lowers AUX3 and disarms, and exits nonzero.

## Useful options

```bash
python3 scripts/run_althold_test.py \
  --takeoff-altitude 3 \
  --altitude-step 1 \
  --althold-throttle 1700 \
  --hold-duration 5 \
  --tolerance 0.5 \
  --phase-timeout 30
```

Use `python3 scripts/run_althold_test.py --help` for all timing, throttle, and MSP connection options.
