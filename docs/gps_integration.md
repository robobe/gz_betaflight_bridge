# GPS integration

The bridge turns Gazebo NavSat data into Betaflight's virtual GPS input. No
serial GPS receiver or NMEA/UBLOX stream is required.

## Data flow

```text
Gazebo /navsat
    -> bridge SensorSnapshot
    -> FDM position_xyz and velocity_xyz
    -> UDP 9003
    -> Betaflight virtual GPS
    -> MSP_RAW_GPS on TCP 5761
```

The bridge sends longitude, latitude, altitude, and ENU velocity in the
existing 144-byte FDM packet. Betaflight derives ground speed and course.

## Build Betaflight SITL

Run the builder and select the required stable release:

```bash
scripts/builders/build_betaflight_sitl.sh
```

The builder enables virtual GPS where older releases need it and installs the
result as `bin/betaflight_SITL.elf`.

## Configuration

The default [bridge configuration](../config/bridge.yaml) already subscribes
to the NavSat topic:

```yaml
gazebo:
  navsat_topic: /navsat
```

The selected world and vehicle must provide:

- WGS84 spherical coordinates in the world.
- Gazebo's NavSat system plugin.
- A NavSat sensor publishing `/navsat`.

Confirm the topic before starting the bridge:

```bash
gz topic -l | rg navsat
gz topic -e -t /navsat
```

## Run

The normal tmux stack starts the processes in the safe order and waits for
SITL initialization before the bridge sends FDM packets:

```bash
tmuxp load config/run_sim.yaml
```

For a manual launch, use separate terminals and wait until SITL reports that
TCP port `5761` is listening before starting the bridge:

```bash
scripts/worlds/run_quadcopter_world.sh -r
scripts/run/run_betaflight_sitl.sh
scripts/run/run_bridge.sh config/bridge.yaml
```

## Verify

Inspect the firmware configuration:

```bash
python3 scripts/tools/inspect_sitl_msp.py
```

The important lines are:

```text
Features: ..., GPS, ...
Sensors: ..., GPS, ...
GPS: provider=VIRTUAL ...
```

Then check the parsed GPS fix:

```bash
python3 scripts/tools/check_gps_msp.py
```

A healthy result looks like:

```text
GPS OK: fix=2 satellites=12 latitude=32.0852999 longitude=34.7817999 ...
```

## Troubleshooting

`MSP command 106 returned an error response`
: The running SITL binary was built without GPS support. Rebuild it with the
  repository builder and restart SITL.

`Betaflight reports no GPS fix`
: Check that `/navsat` publishes, the bridge reports `navsat=ready`, and the
  inspector reports both the GPS feature and `provider=VIRTUAL`.

SITL exits with a segmentation fault during startup
: The bridge sent an FDM packet before SITL finished initialization. Use
  `config/run_sim.yaml`, or start the bridge only after TCP port `5761` opens.

For packet-field details and the Betaflight 2025.12 compatibility notes, see
[GPS/FDM design for Betaflight 2025.12](design/gps_fdm_2025_12.md).
