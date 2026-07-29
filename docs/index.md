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
| EEPROM setup | [betaflight_sitl_eeprom.md](betaflight_sitl_eeprom.md) | AUX1 ARM and AUX2 ANGLE setup for SITL |
| Takeoff test | [takeoff_test.md](takeoff_test.md) | Running and diagnosing the RC takeoff smoke test |
| Joystick RC | [joystick_rc.md](joystick_rc.md) | Calibrating a joystick and sending RC to UDP 9004 |
| MSP hover Python usage | [usage/msp_hover_python.md](usage/msp_hover_python.md) | Direct `hover_msp_controller.py` run flow, PID tuning, and VS Code stack task |
| MSP hover controller | [msp_hover_controller.md](msp_hover_controller.md) | Hover using MSP altitude and MSP RC |
| MSP square mission | [msp_square_mission.md](msp_square_mission.md) | Closed-loop takeoff, square flight, landing, and disarm |
| MSP hover code design | [msp_hover_code_design.md](msp_hover_code_design.md) | SOLID module design and diagrams |
| Observability | [observability.md](observability.md) | Logs, topics, UDP checks, and troubleshooting signals |
| Future capabilities | [future_capabilities.md](future_capabilities.md) | Ideas to improve simulation realism |
| Original implementation plan | [betaflight_gazebo_bridge_plan.md](betaflight_gazebo_bridge_plan.md) | Historical milestone plan and deeper design notes |

## Recommended Reading Order

1. [project_summary_presentation.md](project_summary_presentation.md)
2. [requirements_design_usage.md](requirements_design_usage.md)
3. [configuration.md](configuration.md)
4. [takeoff_test.md](takeoff_test.md)
5. [joystick_rc.md](joystick_rc.md)
6. [usage/msp_hover_python.md](usage/msp_hover_python.md)
7. [msp_hover_controller.md](msp_hover_controller.md)
8. [msp_square_mission.md](msp_square_mission.md)
9. [future_capabilities.md](future_capabilities.md)
