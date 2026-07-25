# MSP hover code design

The MSP hover controller is split into small modules so each concern is isolated.

## Component Diagram

```mermaid
flowchart TB
    WRAP[hover_msp_controller.py] --> MAIN[__main__.py]
    CLI[cli.py] --> MAIN
    MAIN --> CLIENT[msp_client.py]
    MAIN --> TEL[telemetry.py]
    MAIN --> RC[rc.py]
    MAIN --> CTRL[controller.py]
    MAIN --> TIME[timing.py]

    CLIENT --> PROTO[msp_protocol.py]
    TEL --> CLIENT
    RC --> CLIENT
    CTRL --> TEL
    CTRL --> RC
    CTRL --> TIME
```

## Module Responsibilities

| Module | Responsibility |
|---|---|
| `msp_protocol.py` | Build and parse MSP v1 byte frames |
| `msp_client.py` | TCP connection, reads, writes, request/response matching |
| `telemetry.py` | Decode `MSP_ALTITUDE` into meters and meters per second |
| `rc.py` | Build 16-channel RC payloads and send `MSP_SET_RAW_RC` |
| `controller.py` | Arm sequence, hover phases, throttle control, disarm |
| `timing.py` | Fixed-rate loop timing |
| `cli.py` | User options and runtime configuration |
| `__main__.py` | Wire dependencies and run the controller |

## SOLID Mapping

Single Responsibility:

- MSP framing does not know about sockets.
- TCP transport does not know about hover behavior.
- RC encoding does not know about altitude.
- The controller does not know about MSP bytes.

Open/Closed:

- A new telemetry source can replace `AltitudeTelemetry`.
- A new controller can replace `HoverController`.
- MSP v2 support can be added inside protocol/client code without changing RC channel encoding.

Liskov Substitution:

- Tests can substitute fake telemetry and fake RC senders that follow the same behavior expected by `HoverController`.

Interface Segregation:

- The controller only needs altitude samples and RC sending.
- It does not depend on low-level TCP operations.

Dependency Inversion:

- `__main__.py` builds concrete dependencies.
- `HoverController` receives dependencies instead of constructing them internally.

## MSP Frame Flow

```mermaid
flowchart LR
    BUILD[build_request] --> TCP[msp_client TCP write]
    TCP --> BF[Betaflight MSP]
    BF --> RX[TCP read buffer]
    RX --> PARSE[try_parse_response]
    PARSE --> PAYLOAD[command payload]
```

## Controller State Flow

```mermaid
stateDiagram-v2
    [*] --> Prearm
    Prearm --> ArmLow: after prearm_duration
    ArmLow --> Hover: after arm_low_duration
    Hover --> Disarm: Ctrl+C or duration reached
    Disarm --> [*]

    Prearm: AUX1 low, throttle 1000
    ArmLow: AUX1 high, throttle 1000
    Hover: AUX1 high, PD throttle
    Disarm: AUX1 low, throttle 1000
```

## Data Units

Betaflight returns `MSP_ALTITUDE` as:

```text
int32 altitude_cm
int16 vario_cm_s
```

The controller converts to:

```text
altitude_m = altitude_cm / 100
raw_vario_mps = vario_cm_s / 100
vertical_velocity_mps = delta_altitude_m / delta_time_s
```

The controller keeps the raw MSP vario as fallback for the first sample, then derives vertical velocity from altitude changes. This avoids depending on the raw vario sign convention for the damping term.

RC channels are sent as 16 little-endian `uint16` values.
