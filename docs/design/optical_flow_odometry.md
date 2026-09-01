# Gazebo optical-flow and relative-odometry design

## Goal

Publish simulated relative pose and velocity over the bridge's existing
MAVLink 2 UDP output, while documenting what would be required to make
Betaflight consume optical flow for position hold.

## Decision

Use Gazebo Sim's existing `OdometryPublisher` system and translate its
`gz.msgs.Odometry` output to one MAVLink `ODOMETRY` message. Do not derive
optical flow from pose in the first implementation.

```text
Gazebo OdometryPublisher (3D, simulation time)
        |
        | gz.msgs.Odometry
        v
bridge: ENU/FLU -> LOCAL_NED/BODY_FRD
        |
        | MAVLink 2 ODOMETRY, UDP
        v
external estimator / logger / ground application
```

This is the smallest message that carries pose, orientation, linear and
angular velocity, explicit pose and velocity frames, covariance, reset state,
estimator type, and quality together. The official
[`OdometryPublisher`](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
publishes 2D or 3D `gz.msgs.Odometry`, defaults to 50 Hz, supports a custom
topic, offsets, Gaussian noise, and an odometry-with-covariance topic. Gazebo's
[pose-component example](https://gazebosim.org/api/sim/8/posecomponent.html)
shows that this system reads world pose after physics update and packages pose
and twist; therefore a new pose plugin or numerical differentiation is not
needed.

## Gazebo configuration

Attach the installed system to the vehicle model:

```xml
<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
  <dimensions>3</dimensions>
  <odom_publish_frequency>50</odom_publish_frequency>
  <odom_topic>/model/X3/odometry</odom_topic>
  <odom_covariance_topic>/model/X3/odometry_with_covariance</odom_covariance_topic>
</plugin>
```

Prefer the covariance topic when its message is convenient to consume;
otherwise subscribe to `/model/X3/odometry` and mark MAVLink covariance as
unknown. Keep `gaussian_noise` in the SDF as the physical calibration knob;
zero is appropriate for initial ground-truth testing.

## Bridge configuration

Select the MAVLink representation in `config/bridge.yaml`:

```yaml
pose:
  enable: true
  gazebo_topic: /model/X3/odometry
  mavlink_message: odometry  # odometry or vision
```

`odometry` publishes one `ODOMETRY` message per Gazebo sample. `vision`
publishes one `VISION_POSITION_ESTIMATE` and one `VISION_SPEED_ESTIMATE` with
the same sample timestamp. An absent block or `enable: false` disables the
subscription and adds no traffic. Reject any other `mavlink_message` value at
startup; do not support configuring only half of the vision pair because the
stated interface is relative pose and velocity.

## MAVLink `ODOMETRY` contract

`ODOMETRY` is message 331 and requires MAVLink 2 because its ID exceeds 255.
Its official definition is in MAVLink
[`common.xml`](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml)
and rendered in the
[`ODOMETRY` reference](https://mavlink.io/en/messages/common.html#ODOMETRY).

| Field | Value | Unit / meaning |
|---|---|---|
| `time_usec` | Gazebo sample simulation time | microseconds since simulation start |
| `frame_id` | `MAV_FRAME_LOCAL_NED` | earth-fixed local frame: north, east, down |
| `child_frame_id` | `MAV_FRAME_BODY_FRD` | body-fixed velocity frame: forward, right, down |
| `x,y,z` | converted local position | metres |
| `q[4]` | converted attitude | Hamilton quaternion `w,x,y,z` |
| `vx,vy,vz` | converted body-frame linear velocity | m/s |
| `rollspeed,pitchspeed,yawspeed` | converted body angular velocity | rad/s |
| `pose_covariance[21]` | converted upper triangle, or first element `NaN` | covariance of `x,y,z,roll,pitch,yaw` |
| `velocity_covariance[21]` | converted upper triangle, or first element `NaN` | covariance of `vx,vy,vz,rollspeed,pitchspeed,yawspeed` |
| `reset_counter` | increment on simulation reset / time jump | wrapping `uint8_t` counter |
| `estimator_type` | `MAV_ESTIMATOR_TYPE_NAIVE` | truthful: Gazebo ground truth is not a vision estimator |
| `quality` | `0` initially | unknown; `-1` failed, `1..100` worst to best |

The MAVLink frame definitions specify `MAV_FRAME_LOCAL_NED` as an earth-fixed
local tangent frame whose axes are north, east, down, and
`MAV_FRAME_BODY_FRD` as body-fixed forward, right, down; see
[`MAV_FRAME`](https://mavlink.io/en/messages/common.html#MAV_FRAME). Convert the
Gazebo world/body convention used by this repository (ENU/FLU) explicitly:

```text
local position and earth-frame vectors: Gazebo [E, N, U] -> MAVLink [N, E, -U]
body-frame vectors:                    Gazebo [F, L, U] -> MAVLink [F, -L, -U]
```

Apply the corresponding basis change to the quaternion and covariance, not
only to their named vector fields. Establish the local origin at the first
valid sample after bridge start or simulation reset. Use one monotonic time
domain throughout; `time_usec` officially accepts UNIX epoch or time since
system boot, so simulation-start microseconds are valid and preserve paused
and stepped simulation behavior.

## Why not the other MAVLink messages?

All field definitions and units below come from the official
[`common.xml` message reference](https://mavlink.io/en/messages/common.html).

### `OPTICAL_FLOW_RAD` (106)

This is the correct message for a simulated angular-rate optical-flow sensor,
not for already-known Cartesian pose or velocity. It contains:

- `time_usec` (us), `sensor_id`, and `integration_time_us` (us);
- `integrated_x`, `integrated_y` (rad) over that interval;
- integrated right-handed gyro angles `integrated_xgyro`,
  `integrated_ygyro`, `integrated_zgyro` (rad);
- `temperature` (centi-degrees Celsius);
- `quality`, where 0 is invalid and 255 is maximum confidence;
- `time_delta_distance_us` and `distance` (m), with negative distance meaning
  unknown.

Generating it from perfect Gazebo pose would require choosing a virtual focal
model, field of view, integration window, gyro compensation, range sample,
noise and quality model. That synthetic sensor model is extra behavior and
less truthful than publishing odometry. Add it only when testing an autopilot
that actually fuses `OPTICAL_FLOW_RAD`, ideally from a Gazebo optical-flow
sensor model rather than pose differencing.

### `VISION_POSITION_ESTIMATE` (102) and `VISION_SPEED_ESTIMATE` (103)

`VISION_POSITION_ESTIMATE` carries local `x,y,z` in metres and
`roll,pitch,yaw` in radians, plus a 21-element upper-triangle pose covariance
and reset counter. `VISION_SPEED_ESTIMATE` separately carries global `x,y,z`
speed in m/s, a full 3x3 velocity covariance, and reset counter. Both use
microsecond epoch-or-boot timestamps. They require two messages, do not carry
explicit frame IDs, and lose angular velocity, so `ODOMETRY` is clearer and
smaller as a design surface. The configured `vision` compatibility mode sends
both from the same Gazebo sample and uses the same local-NED convention as the
canonical `ODOMETRY` output.

### `LOCAL_POSITION_NED` (32)

This message carries only filtered NED `x,y,z` (m) and `vx,vy,vz` (m/s) with
`time_boot_ms`; it has no attitude, covariance, reset counter, quality, or
frame selector. It is suitable as an optional compatibility telemetry output
for consumers that cannot decode message 331, but should not be the canonical
bridge output. Its contract is documented under
[`LOCAL_POSITION_NED`](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED).

## Betaflight compatibility boundary

MAVLink support in Betaflight does not mean MAVLink estimator input support.
The vendored Betaflight receiver queues messages other than RC override and
radio status, but its telemetry dispatch handles only `HEARTBEAT`, `PING`,
`TIMESYNC`, `COMMAND_LONG`, and optional mission messages. The upstream source
is
[`src/main/telemetry/mavlink.c`](https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/mavlink.c)
and RC-specific handling is in
[`src/main/rx/mavlink.c`](https://github.com/betaflight/betaflight/blob/master/src/main/rx/mavlink.c).
Consequently Betaflight currently ignores `ODOMETRY`, `OPTICAL_FLOW_RAD`,
`VISION_POSITION_ESTIMATE`, `VISION_SPEED_ESTIMATE`, and
`LOCAL_POSITION_NED`; publishing any of them will not affect its estimator or
position hold.

Betaflight's optical-flow subsystem instead has native serial sensor drivers
for supported combined range/flow devices. The upstream
[`opticalflow.c`](https://github.com/betaflight/betaflight/blob/master/src/main/sensors/opticalflow.c)
selects those hardware drivers, and
[`position_estimator.c`](https://github.com/betaflight/betaflight/blob/master/src/main/flight/position_estimator.c)
uses their angular flow rate, quality, and range to estimate horizontal
velocity.

Therefore:

- for external MAVLink publication, implement only `ODOMETRY`;
- for Betaflight position hold without changing Betaflight, emulate one of its
  supported optical-flow serial devices and its range data on a SITL UART;
- alternatively, add a Betaflight MAVLink input handler that maps
  `OPTICAL_FLOW_RAD` into its optical-flow device contract. That is a
  Betaflight feature change and is outside this bridge-only design.

## Failure and validation contract

- Reject samples with non-finite pose, orientation, velocity, or timestamps.
- Stop publishing stale data; keep the existing MAVLink heartbeat alive.
- Increment `reset_counter` when simulation time moves backward or the local
  origin is re-established.
- Publish at the Gazebo source rate (50 Hz initially); do not interpolate.
- Verify one stationary sample, +X/+Y/+Z translation, yaw rotation, and reset.
  Decode the generated packet and assert frame IDs, signs, units, timestamp,
  covariance sentinel, reset counter, and MAVLink 2 framing.
- Run that check once with `mavlink_message: odometry` and once with
  `mavlink_message: vision`; the vision check must receive both message IDs
  with identical timestamps.

## Deferred work

`OPTICAL_FLOW_RAD`, `LOCAL_POSITION_NED`, configurable origins, covariance
synthesis, and a Betaflight MAVLink estimator-input patch are deliberately
deferred. Add one only when a named consumer requires it.
