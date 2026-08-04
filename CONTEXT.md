# Betaflight Gazebo Flight Automation

This context defines the flight concepts shared by reusable MSP automation
scripts that command Betaflight SITL and observe vehicle state.

## Language

**Yaw excursion**:
A relative heading maneuver that rotates clockwise by 180 degrees and then counterclockwise back to the heading captured before the maneuver.
_Avoid_: Yaw spin, turn around

**Hover altitude**:
The commanded vertical position that the vehicle maintains during a maneuver; initially 3 metres for the yaw excursion.
_Avoid_: Flight height, throttle target

**Controlled descent**:
The post-maneuver phase that lowers the altitude target at 1 metre per second while retaining closed-loop vertical control.
_Avoid_: Throttle ramp, drop

**Landing detection**:
The evidence-based condition that the vehicle has reached the ground and may be disarmed safely.
_Avoid_: Descent timeout, zero throttle

**Synthesized RC command**:
A complete virtual stick-and-switch frame produced by automation and sent to Betaflight with `MSP_SET_RAW_RC`, without a physical joystick.
_Avoid_: Joystick input, motor command

**Safety shutdown**:
The terminal mission response to invalid state, stale telemetry, timeout, or unexpected disarming; it sends low throttle and DISARM repeatedly and never attempts automatic re-arming.
_Avoid_: Recovery, retry
