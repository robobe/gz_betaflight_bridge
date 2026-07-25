# Betaflight SITL packet protocol

Betaflight SITL uses UDP for simulator data.

| Port | Direction | Packet | Purpose |
|---:|---|---|---|
| 9002 | Betaflight to bridge | `servo_packet` | Normalized motor commands |
| 9003 | Bridge to Betaflight | `fdm_packet` | IMU, altitude, velocity, position, pressure |
| 9004 | RC script to Betaflight | `rc_packet` | RC channel input |

## FDM packet

```cpp
struct fdm_packet {
    double timestamp;
    double imu_angular_velocity_rpy[3];
    double imu_linear_acceleration_xyz[3];
    double imu_orientation_quat[4];
    double velocity_xyz[3];
    double position_xyz[3];
    double pressure;
};
```

Size: `144` bytes.

The bridge fills:

- `timestamp`: bridge runtime seconds.
- `imu_angular_velocity_rpy`: IMU angular velocity after `fdm.frame_mode` conversion.
- `imu_linear_acceleration_xyz`: IMU linear acceleration after `fdm.frame_mode` conversion.
- `imu_orientation_quat`: quaternion as `w, x, y, z` after `fdm.frame_mode` conversion.
- `velocity_xyz[2]`: altimeter vertical velocity.
- `position_xyz[2]`: altimeter vertical position.
- `pressure`: standard atmosphere pressure from altitude unless disabled.

For `fdm.frame_mode: gazebo_bridge`, the bridge sends the packet convention expected by Betaflight's `SITL_GAZEBO` target:

```text
angular_velocity:  Gazebo FLU [x, y, z] -> packet [x, -y, -z]
linear_accel:      Gazebo FLU [x, y, z] -> packet [x, -y, -z]
quaternion wxyz:   Gazebo FLU-to-ENU [w, x, y, z] -> packet [w, x, -y, -z]
```

## Motor packet

```cpp
struct servo_packet {
    float motor_speed[4];
};
```

Size: `16` bytes.

The values on UDP `9002` are normalized Betaflight motor commands, not raw PWM microseconds:

```text
normal mode: 0.0 to 1.0
3D mode:    -1.0 to 1.0
```

The current bridge clamps to `0.0..1.0`.

## RC packet

```cpp
struct rc_packet {
    double timestamp;
    uint16_t channels[16];
};
```

Size: `40` bytes.

The helper script sends centered channels by default and only arms or ramps throttle when explicit flags are passed.
