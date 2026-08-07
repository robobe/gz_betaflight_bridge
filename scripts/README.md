# Script index

Executable scripts are grouped by purpose. Reusable Python packages remain in
the `scripts/` root so every mission and tool shares the same import seam.

| Directory | Purpose |
|---|---|
| `builders/` | Build external dependencies such as Betaflight SITL |
| `run/` | Start SITL, the bridge, or a complete process stack |
| `worlds/` | Configure Gazebo and launch a selected world |
| `missions/` | Run autonomous missions that do not have a colocated package entrypoint |
| `tests/` | Run manual SITL integration and packet tests |
| `tools/` | Interactive joystick, motor, and inspection utilities |

Common commands:

```bash
scripts/worlds/run_quadcopter_world.sh -r
scripts/run/run_betaflight_sitl.sh
scripts/run/run_bridge.sh config/bridge.yaml
python3 scripts/msp_yaw_mission/run_msp_yaw_mission.py
```

`msp_hover/` and `msp_yaw_mission/` keep their entry script and YAML beside
their flight-policy code. `msp_core/` remains a reusable implementation module.
