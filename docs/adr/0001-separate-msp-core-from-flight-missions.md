# Separate the MSP core from flight missions

Reusable MSP transport, framing, typed telemetry, status queries, synthesized RC commands, timing, and errors belong in `scripts/msp_core/`; flight-specific state machines and controllers belong in separate mission packages such as `scripts/msp_yaw_mission/`. This keeps the shared MSP connection usable by later scripts without coupling communication code to hover or yaw policy, while compatibility imports preserve existing `msp_hover` consumers during migration.
