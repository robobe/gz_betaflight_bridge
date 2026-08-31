# Gazebo rangefinder output design

## Goal

The bridge subscribes to one-beam Gazebo lidar topics. Each mapping selects
either an emulated Benewake TFmini on a SITL UART TCP socket or an external
MAVLink 2 UDP message. The first scenario connects the downward sensor to
Betaflight UART2 and publishes the forward sensor as MAVLink:

```text
/X3/down_range/scan
        | gz.msgs.LaserScan
        v
GazeboRangefinderSubscriber
        | metres
        v
TFmini encoder (100 Hz)
        | 9-byte UART frames
        v
TCP 127.0.0.1:5762 (SITL UART2)
        |
        v
Betaflight TFmini driver and rangefinder subsystem

/X3/front_range/scan
        | gz.msgs.LaserScan
        v
MAVLink DISTANCE_SENSOR or OBSTACLE_DISTANCE
        | UDP
        v
127.0.0.1:14550 (external consumer)
```

UART1 remains the MSP connection on TCP `5761`. SITL maps UART number `n` to
TCP port `5760 + n`.

## Scope

- Support multiple independent Gazebo rangefinder mappings in the bridge.
- Select TFmini or MAVLink output per mapping.
- Emulate the TFmini UART protocol at 115200 baud.
- Use `/X3/down_range/scan` and TCP `5762` for the first scenario.
- Publish `/X3/front_range/scan` to UDP `127.0.0.1:14550` as selectable
  `DISTANCE_SENSOR` or `OBSTACLE_DISTANCE` MAVLink 2 messages.
- Keep the barometer as Betaflight's primary altitude source.
- Verify configuration, detection, and readings through existing MSP commands.

## Out of scope

- Rangefinder scale or offset calibration.
- Synthetic noise; noise belongs in the Gazebo sensor model.
- Using the forward rangefinder as altitude input.
- Feeding MAVLink rangefinder messages back into Betaflight. Betaflight does
  not consume `DISTANCE_SENSOR` or `OBSTACLE_DISTANCE` as sensor inputs.
- Changing Betaflight to consume multiple rangefinders. Betaflight currently
  has one global rangefinder instance even though the bridge can expose
  multiple UART mappings.

## Configuration

Add bridge-wide MAVLink settings and a rangefinder list to
`config/bridge.yaml`:

```yaml
mavlink:
  address: 127.0.0.1
  port: 14550
  system_id: 1
  component_id: 158

rangefinders:
  - name: down
    gazebo_topic: /X3/down_range/scan
    output: tfmini
    sitl_address: 127.0.0.1
    sitl_port: 5762

  - name: front
    gazebo_topic: /X3/front_range/scan
    output: mavlink
    mavlink_message: distance_sensor
    orientation: forward
    sensor_id: 0
```

`output` must be `tfmini` or `mavlink`. A MAVLink entry additionally selects
`mavlink_message` as `distance_sensor` or `obstacle_distance`. Change the
front entry to `mavlink_message: obstacle_distance` to publish the array form.

Names and topics must be unique. TFmini TCP endpoints and MAVLink
`DISTANCE_SENSOR` IDs must also be unique. `sitl_address` and `sitl_port` are
required only for TFmini; `orientation` is required for MAVLink; `sensor_id`
is required only for `distance_sensor`. The first implementation accepts one
`obstacle_distance` mapping because that message has no sensor instance ID.
An absent or empty list disables all rangefinder forwarding.

The `mavlink` block is bridge-wide because all MAVLink rangefinders share one
UDP publisher. `system_id` identifies the simulated vehicle and should match
that vehicle's MAVLink system ID. `component_id` identifies the bridge within
the vehicle and must not collide with another component. Values must be in
the MAVLink source-ID range `1..255`. The defaults use system `1` and
`MAV_COMP_ID_PERIPHERAL` (`158`). `sensor_id` remains per rangefinder because
it distinguishes multiple `DISTANCE_SENSOR` instances from the same MAVLink
component.

## Gazebo input contract

Use the existing sensor world:

```bash
scripts/worlds/run_quadcopter_sensor_world.sh -r
```

The model already provides one-beam sensors at 20 Hz. The forward sensor
points along body `+X`; the downward sensor points along body `-Z`. The bridge
accepts a scan only when:

- `ranges_size() == 1`;
- the value is finite and non-negative;
- it is within TFmini's supported range after conversion to centimetres;
- the sample is no more than 250 ms old.

Empty, multi-beam, invalid, or stale scans do not close the mapping. They
produce the output-specific invalid behavior described below and a throttled
warning.

## TFmini emulation

The adapter repeats the newest Gazebo sample at 100 Hz. Metres are converted
directly to integer centimetres. Calibration is deliberately deferred.

Each frame is nine bytes:

| Byte | Value |
|---:|---|
| 0–1 | Sync: `0x59 0x59` |
| 2–3 | Distance in centimetres, little-endian |
| 4–5 | Fixed valid signal strength, little-endian |
| 6 | Integral time; fixed value other than `7` |
| 7 | Reserved: `0` |
| 8 | Unsigned sum of bytes 0–7 modulo 256 |

Invalid input is encoded as `1201 cm`. Betaflight rejects values above the
TFmini driver's `1200 cm` maximum as out of range. The bridge must continue
sending these frames: silence can leave Betaflight holding the last valid
distance.

Betaflight sends an eight-byte TFmini rate command after opening the UART. The
adapter may recognize and discard it because output is already fixed at the
requested 100 Hz.

## TCP lifecycle and isolation

Each mapping owns its TCP connection and retries once per second. Connection
failure affects only that mapping; it must never stop FDM, GPS, motor traffic,
or another rangefinder. Connection transitions are logged immediately and
repeated failures are throttled.

The existing periodic bridge status should include the selected output:

```text
rangefinder[down] output=tfmini topic=ready tcp=connected frames=12345 invalid=0
rangefinder[front] output=mavlink topic=ready messages=2469 invalid=0
```

## MAVLink output

MAVLink output is an external telemetry feed, not a Betaflight UART. All
MAVLink rangefinders share one MAVLink 2 UDP publisher using the configured
address, port, system ID, and component ID. The publisher sends a 1 Hz
`HEARTBEAT` with `MAV_TYPE_ONBOARD_CONTROLLER` and
`MAV_AUTOPILOT_INVALID`. Range messages are sent at the Gazebo sensor's 20 Hz
update rate.

Use the generated MAVLink common headers already present under
`external/betaflight/lib/main/MAVLink`; do not add another MAVLink library.

### `DISTANCE_SENSOR`

- Convert metres to integer centimetres.
- Populate the scan minimum and maximum, laser sensor type, configured
  `sensor_id`, and configured orientation. `forward` maps to
  `MAV_SENSOR_ROTATION_NONE`.
- Use milliseconds since bridge startup for `time_boot_ms`.
- Set covariance and signal quality to unknown.
- Do not publish a range message for an invalid, no-return, or stale sample;
  the heartbeat continues so consumers can distinguish missing sensor data
  from a dead MAVLink component.

### `OBSTACLE_DISTANCE`

- Use `MAV_FRAME_BODY_FRD`, laser sensor type, and microseconds since bridge
  startup.
- Put the one-beam reading in the bin selected by `orientation`; for the
  current forward sensor this is index zero with `angle_offset = 0`.
- Mark all unused bins `UINT16_MAX`. Encode a valid no-return reading as
  `max_distance + 1`; encode invalid or stale data as `UINT16_MAX`.
- Set both angular increment fields to zero because the Gazebo source is a
  single ray rather than an angular scan.

The message fields and sentinel values follow the official
[MAVLink common message definitions](https://mavlink.io/en/messages/common.html).
The heartbeat follows the official
[heartbeat protocol](https://mavlink.io/en/services/heartbeat.html).

## Betaflight configuration

### Configurator

1. In **Ports**, assign UART2 sensor input to **LIDAR TF**.
2. In **Configuration**, enable **RANGEFINDER** and select **TFMINI**.
3. Save and reboot Betaflight.
4. Leave the altitude source unchanged so the barometer remains primary.

### CLI equivalent

```text
feature RANGEFINDER
serial UART2 32768 115200 115200 0 115200
set rangefinder_hardware=TFMINI
save
```

`32768` is Betaflight's `FUNCTION_LIDAR_TF` serial-function mask. The TFmini
driver itself opens the assigned UART at 115200 baud.

## Reading verification

No new Betaflight MSP command is required for the TFmini path. Add
`scripts/tools/check_rangefinder_msp.py` using existing commands:

- `MSP_SENSOR_CONFIG` (`96`) for configured hardware;
- `MSP_STATUS` (`101`) for detected rangefinder presence;
- `MSP_SONAR_ALTITUDE` (`58`) for tilt-corrected altitude in signed
  little-endian centimetres.

The tool prints configured hardware, detection state, and metres/centimetres,
or a clear out-of-range/hardware-failure result. Because command 58 returns
tilt-corrected altitude, end-to-end comparisons must keep the vehicle level.
MAVLink readings are checked directly on UDP `14550`; they are not visible
through Betaflight MSP.

## Test and acceptance plan

1. Validate the MAVLink address, port, source IDs, supported output and
   message names, required fields, unique TFmini endpoints, unique
   `DISTANCE_SENSOR` IDs, and the single `OBSTACLE_DISTANCE` limit.
2. Unit-check that `1.23 m` encodes as `123 cm` with the correct TFmini
   checksum.
3. Unit-check empty, multi-beam, non-finite, negative, stale, and over-range
   inputs produce the `1201 cm` out-of-range frame.
4. Decode generated MAVLink packets and verify framing, units, source IDs,
   orientation, obstacle bins, and invalid/no-return behavior.
5. Capture UDP `14550` and confirm the 1 Hz heartbeat plus the selected 20 Hz
   rangefinder message.
6. Start the sensor world, SITL, and bridge; confirm bridge status reports the
   down TFmini mapping connected and the front MAVLink mapping ready.
7. Confirm the SITL inspector reports the downward rangefinder as detected.
8. At level clearances of `0.5 m`, `1.0 m`, and `2.0 m`, confirm
   `check_rangefinder_msp.py` is within `±2 cm` of the Gazebo reading.
9. Stop lidar updates and confirm the check tool reports out of range within
   250 ms while FDM, GPS, and motor traffic continue.
10. Confirm Betaflight's barometer altitude configuration remains unchanged.

The implementation is accepted when all ten checks pass.
