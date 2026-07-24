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

## Throttle ramp

Use the continuous takeoff sequence after Gazebo, SITL, and the bridge are running:

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
