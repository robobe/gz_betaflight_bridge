# Script index

Executable scripts are grouped by purpose. Reusable Python packages remain in
the `scripts/` root so every mission and tool shares the same import seam.

| Directory | Purpose |
|---|---|
| `builders/` | Build external dependencies such as Betaflight SITL |
| `run/` | Start SITL, the bridge, or a complete process stack |
| `worlds/` | Configure Gazebo and launch a selected world |
| `missions/` | Run autonomous MSP flight missions and controllers |
| `tests/` | Run manual SITL integration and packet tests |
| `tools/` | Interactive joystick, motor, and inspection utilities |

Common commands:

```bash
scripts/worlds/run_quadcopter_world.sh -r
scripts/run/run_betaflight_sitl.sh
scripts/run/run_bridge.sh config/bridge.yaml
python3 scripts/missions/run_msp_yaw_mission.py
```

Python module packages such as `msp_core/`, `msp_hover/`, and
`msp_yaw_mission/` are implementation modules rather than shell entrypoints.
