# Separate the MSP core from flight missions

Reusable MSP transport, framing, typed telemetry, status queries, synthesized RC commands, timing, and errors belong in `scripts/msp_core/`; flight-specific state machines and controllers belong in separate mission packages such as `scripts/msp_hover/` and `scripts/msp_yaw_mission/`. All consumers import communication primitives directly from `msp_core`. This keeps the shared MSP connection usable without coupling communication code to hover or yaw policy.
