# Script index

Executable scripts are grouped by purpose. `msp_core/` contains the shared MSP
transport used by the diagnostic tools.

| Directory | Purpose |
|---|---|
| `builders/` | Build external dependencies such as Betaflight SITL |
| `run/` | Start SITL or the bridge |
| `worlds/` | Configure Gazebo and launch a selected world |
| `tools/` | Inspect SITL, GPS, and motor traffic |

Common commands:

```bash
scripts/worlds/run_quadcopter_world.sh -r
scripts/run/run_betaflight_sitl.sh
scripts/run/run_bridge.sh config/bridge.yaml
python3 scripts/tools/inspect_sitl_msp.py
python3 scripts/tools/configure_rangefinder_cli.py
```
