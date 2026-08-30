# Documentation index

Start here when navigating the project docs.

## Shortcuts

| Topic | Document | Use it for |
|---|---|---|
| Project overview | [project_summary_presentation.md](project_summary_presentation.md) | A presentation-style explanation of what the project does |
| Requirements, design, plan, usage | [requirements_design_usage.md](requirements_design_usage.md) | The main project guide |
| Architecture | [bridge_architecture.md](bridge_architecture.md) | Components and runtime data flow |
| Configuration | [configuration.md](configuration.md) | YAML fields, ports, topics, and motor map |
| Packet protocol | [packet_protocol.md](packet_protocol.md) | Betaflight UDP packet layout |
| Motor math | [motor_velocity_math.md](motor_velocity_math.md) | Normalized motor command to rotor velocity |
| Motor KV and battery | [motor_kv_battery_mapping.md](motor_kv_battery_mapping.md) | Estimating rotor limits from motor and battery data |
| Gazebo motor model | [gazebo_world_motor_model.md](gazebo_world_motor_model.md) | World structure and deep tuning of `MulticopterMotorModel` |
| Coordinate frames | [coordinate_frames.md](coordinate_frames.md) | IMU, quaternion, and frame conversion choices |
| GPS/FDM design | [design/gps_fdm_2025_12.md](design/gps_fdm_2025_12.md) | Adding Gazebo NavSat GPS for Betaflight 2025.12.x |
| SITL tools | [sitl_tools.md](sitl_tools.md) | Inspect Betaflight, GPS, and motor traffic |
| Observability | [observability.md](observability.md) | Logs, topics, UDP checks, and troubleshooting signals |
| Future capabilities | [future_capabilities.md](future_capabilities.md) | Ideas to improve simulation realism |
| Original implementation plan | [betaflight_gazebo_bridge_plan.md](betaflight_gazebo_bridge_plan.md) | Historical milestone plan and deeper design notes |

## Recommended Reading Order

1. [project_summary_presentation.md](project_summary_presentation.md)
2. [requirements_design_usage.md](requirements_design_usage.md)
3. [configuration.md](configuration.md)
4. [sitl_tools.md](sitl_tools.md)
5. [future_capabilities.md](future_capabilities.md)
