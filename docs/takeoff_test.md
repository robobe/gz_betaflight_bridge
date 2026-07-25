# RC and takeoff smoke test

The bridge does not auto-arm Betaflight and does not generate RC commands. RC testing is handled by:

```text
scripts/send_rc_test.py
```

This separation keeps production bridge behavior simple and avoids accidental arming.

## Basic traffic test

Terminal 1:

```bash
scripts/run_quadcopter_world.sh
```

Terminal 2:

```bash
scripts/run_betaflight_sitl.sh
```

Terminal 3:

```bash
scripts/run_bridge.sh
```

Expected bridge status:

```text
imu=true altimeter=true
fdm_packets increasing
motor_packets increasing
```

## Neutral RC

Send centered sticks with low throttle:

```bash
scripts/send_rc_test.py --duration 10
```

## Explicit arming test

Only run this when Gazebo, SITL, and the bridge are already connected:

```bash
scripts/send_rc_test.py --arm --angle --duration 10
```

This checks that AUX1 high arms and AUX2 high enables ANGLE mode. It is not a takeoff command because throttle remains low.

## Legacy UDP throttle ramp

The UDP RC script is still useful for old smoke tests, but the current EEPROM profile enables `RX_MSP` for MSP hover. To use UDP RC again, switch Betaflight back to `RX_UDP`.

Use the continuous UDP takeoff sequence after Gazebo, SITL, and the bridge are running:

```bash
scripts/send_rc_test.py --takeoff-sequence
```

The sequence sends:

```text
3 seconds: disarmed, ANGLE enabled, low throttle
5 seconds: armed, ANGLE enabled, low throttle
10 seconds: armed throttle ramp from 1000 to 2000
5 seconds: armed hold at 2000
```

The default `--ramp-end 2000` is intentionally strong for the X3 example model. For a gentler test, lower it:

```bash
scripts/send_rc_test.py --takeoff-sequence --ramp-end 1600
```

If Gazebo receives motor commands but the vehicle does not move, confirm the world loads the physics system:

```xml
<plugin
  filename="gz-sim-physics-system"
  name="gz::sim::systems::Physics">
</plugin>
```

Without this plugin, the motor model may receive commands but Gazebo will not integrate the forces into vehicle motion.

## Verify lift from Gazebo only

This command bypasses Betaflight and the bridge. It proves that the Gazebo X3 model and motor plugins can lift:

```bash
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
```

Stop the command with `Ctrl+C`, then publish zeros if needed:

```bash
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[0, 0, 0, 0]'
```

If Betaflight does not arm, inspect arming flags through Betaflight Configurator or MSP. RC packets alone are not sufficient unless the SITL EEPROM maps AUX1 to ARM and AUX2 to ANGLE.

## MSP hover

For altitude hold, prefer the MSP hover controller. It reads Betaflight altitude with `MSP_ALTITUDE` and sends RC with `MSP_SET_RAW_RC`.

Regenerate EEPROM once after enabling MSP RC:

```bash
scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli
```

Start the full MSP hover stack:

```bash
scripts/run_msp_hover_stack.sh --headless --target-altitude 5
```

For debugging, start only Gazebo, SITL, and the bridge:

```bash
scripts/run_takeoff_stack.sh
```

Then run hover in another terminal:

```bash
scripts/hover_msp_controller.py --target-altitude 5
```

See:

```text
docs/msp_hover_controller.md
docs/msp_hover_code_design.md
```
