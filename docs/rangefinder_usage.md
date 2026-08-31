# Rangefinder usage

The bridge can expose Gazebo one-beam lidars as either a TFmini connected to
Betaflight SITL UART2 or external MAVLink 2 range messages. Betaflight uses the
downward TFmini as its single rangefinder; MAVLink output is for external
consumers and is not fed back into Betaflight.

## Bridge configuration

Configure mappings in `config/bridge.yaml`:

```yaml
mavlink:
  address: 127.0.0.1
  port: 14550
  system_id: 1
  component_id: 158

rangefinders:
  - name: down
    enable: true
    gazebo_topic: /X3/down_range/scan
    output: tfmini
    sitl_address: 127.0.0.1
    sitl_port: 5762

  - name: front
    enable: true
    gazebo_topic: /X3/front_range/scan
    output: mavlink
    mavlink_message: distance_sensor
    orientation: forward
    sensor_id: 0
```

Set `enable: false` to prevent the bridge from subscribing to a mapping's
Gazebo topic. UART1 remains the MSP connection on TCP port `5761`; SITL maps
UART2 to TCP port `5762`.

## Configure Betaflight

Start Betaflight SITL, ensure no Configurator or other MSP client is using
port `5761`, then run:

```bash
scripts/tools/configure_rangefinder_cli.py
```

The script applies and saves these Betaflight CLI commands:

```text
feature RANGEFINDER
serial UART2 32768 115200 115200 0 115200
set rangefinder_hardware=TFMINI
save
```

`save` reboots Betaflight. Betaflight 2026.6 cloud-style SITL builds omit the
rangefinder unless it is explicitly requested. Build this project's binary
with TFmini support when the script reports that `RANGEFINDER` is unavailable:

```bash
scripts/builders/build_betaflight_sitl.sh
```

Restart SITL to load the new binary, rerun the configuration script, then
start or restart the bridge after Betaflight begins listening on UART2.

The Configurator equivalent is:

1. In **Ports**, assign UART2 sensor input to **LIDAR TF**.
2. In **Configuration**, enable **RANGEFINDER** and select **TFMINI**.
3. Save and reboot, leaving the barometer as the primary altitude source.

## Run and verify

Start the sensor stack:

```bash
tmuxp load config/run_sim.yaml
```

Verify the Betaflight path while the vehicle is level and at least 40 cm above
the surface:

```bash
scripts/tools/check_rangefinder_msp.py
```

Verify the external MAVLink path:

```bash
scripts/tools/check_mavlink_rangefinder.py
# Or, after selecting obstacle_distance in bridge.yaml:
scripts/tools/check_mavlink_rangefinder.py --message obstacle_distance
```

Expected bridge status includes `topic=ready`, `tcp=connected` for the TFmini
mapping, and increasing frame/message counters. `MSP_SONAR_ALTITUDE` is
tilt-corrected, so compare it with Gazebo only while the vehicle is level.

## Troubleshooting

- `hardware=NONE`: run the configuration script and allow Betaflight to reboot.
- `detected=False`: confirm UART2 is assigned function mask `32768`, the bridge
  uses TCP `5762`, and restart Betaflight before the bridge.
- Out of range near the ground: TFmini's supported minimum is 40 cm.
- No MAVLink data: ensure UDP port `14550` is free and the mapping is enabled.

## Implementation and diagnostic results

This implementation was completed and exercised with Betaflight SITL
`2026.6.1` (MSP API `1.48`). The bridge now supports multiple independent
rangefinder mappings, TFmini TCP output at 100 Hz, MAVLink 2
`DISTANCE_SENSOR` or `OBSTACLE_DISTANCE`, a 1 Hz MAVLink heartbeat, mapping
status logs, and the per-mapping `enable` switch.

### Betaflight 2026.6 build findings

The standard cloud-style SITL build reports `RANGEFINDER` as unavailable.
The build must add only:

```text
-DUSE_RANGEFINDER_TF
```

Do not also define `USE_RANGEFINDER` directly. In Betaflight 2026.6 that
causes `common_post.h` to enable all rangefinder drivers, including the
hardware-only HCSR04 driver, which fails to link for SITL. Defining the TF
driver lets Betaflight derive the shared rangefinder subsystem without pulling
in unsupported GPIO drivers. `build_betaflight_sitl.sh` applies the correct
flag automatically.

The 2026.6 CLI also requires the setting assignment without a space before
the equals sign:

```text
set rangefinder_hardware=TFMINI
```

The configuration script checks `feature list` before making changes and
reports a clear rebuild instruction if `RANGEFINDER` is unavailable.

### SITL startup crash

Enabling the TFmini driver initially made SITL terminate with `SIGSEGV` in
`pthread_mutex_lock`. GDB showed that the FDM UDP worker called
`virtualAccSet(virtualAccDev, ...)` while `virtualAccDev` was still null. The
worker starts before sensor initialization; TFmini initialization made the
existing race reliably visible.

`config/betaflight/sitl_sensor_startup.patch` fixes the root cause by ignoring
early FDM packets until both virtual accelerometer and gyro devices exist. The
builder applies this patch automatically. Before the fix SITL crashed in less
than one second; afterward the same GDB reproduction remained alive for its
full six-second timeout, and configuration plus reboot completed normally.

### MSP checker correction

Betaflight 2026.6 writes `MSP_SENSOR_CONFIG` as accelerometer, barometer,
magnetometer, rangefinder, and optical flow. The rangefinder is therefore byte
3, not byte 4. `check_rangefinder_msp.py` now reads byte 3 and prints the
hardware name.

The final live inspection reported:

```text
Features: RANGEFINDER
Sensors: RANGEFINDER
UART2: LIDAR_TF (0x8000)
hardware=TFMINI detected=True
```

The grounded vehicle reported `-1 cm`, which is the expected out-of-range
state because TFmini's minimum supported distance is 40 cm. A meaningful
distance check requires raising the level vehicle at least 40 cm above the
surface.

### Real-time factor investigation

Disabling or stopping the bridge did not materially improve Gazebo's real-time
factor: the measured interval changed from `0.711` to `0.720`. Running Gazebo
without its GUI improved it to `0.936`, and the final headless stack with the
bridge and both rangefinder mappings enabled measured `0.976`. The primary
cost was the Gazebo GUI/render path, not the rangefinder callbacks.

`config/run_sim.yaml` consequently starts the sensor world with `-s` for
headless operation. Remove `-s` when interactive visualization is required.

### Added tools

```text
scripts/tools/configure_rangefinder_cli.py  Configure and save Betaflight
scripts/tools/check_rangefinder_msp.py      Verify the Betaflight TFmini path
scripts/tools/check_mavlink_rangefinder.py  Verify heartbeat and MAVLink ranges
```

The CLI protocol, MSP payload decoding, MAVLink packet parsing, configuration
validation, and TFmini encoding have runnable checks in the test suite.
