# Motor command to rotor velocity math

The bridge consumes Betaflight UDP `9002`, which contains four normalized `float` motor commands.

The configured conversion is linear:

```text
normalized = clamp(command, 0.0, 1.0)
velocity_rad_s = min_velocity + normalized * (max_velocity - min_velocity)
```

With the default config:

```yaml
min_rotor_velocity_rad_s: 0.0
max_rotor_velocity_rad_s: 800.0
```

the conversion is:

| Betaflight command | Gazebo rotor velocity |
|---:|---:|
| 0.00 | 0 rad/s |
| 0.25 | 200 rad/s |
| 0.50 | 400 rad/s |
| 0.75 | 600 rad/s |
| 1.00 | 800 rad/s |

## Why not raw PWM here?

Betaflight SITL has another UDP output on `9001` with raw PWM-like values, but the Gazebo-specific output on `9002` is already normalized. The bridge therefore uses `9002`.

If a future mode reads raw PWM, the usual normalization would be:

```text
normalized = clamp((pwm_us - min_pwm_us) / (max_pwm_us - min_pwm_us), 0.0, 1.0)
```

For example:

```text
min_pwm_us = 1000
max_pwm_us = 2000
pwm_us = 1500
normalized = 0.5
```

Then the same velocity formula applies.

## Safety behavior

- NaN and infinity are rejected.
- Values below `0.0` become `0.0`.
- Values above `1.0` become `1.0`.
- If no motor packet arrives before `timeout_seconds`, the bridge publishes zero velocity.

