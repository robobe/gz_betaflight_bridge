# Motor KV, battery voltage, and Gazebo parameters

Motor KV estimates no-load motor speed:

```text
rpm_no_load = motor_kv * battery_voltage
rad_s_no_load = rpm_no_load * 2*pi / 60
```

Example for a `2300 KV` motor on a nominal `4S` battery:

```text
nominal cell voltage = 3.7 V
4S voltage = 14.8 V
rpm_no_load = 2300 * 14.8 = 34040 rpm
rad_s_no_load = 34040 * 2*pi / 60 = 3565 rad/s
```

Real propeller load reduces the achievable speed. A first estimate for Gazebo is often a fraction of no-load speed:

```text
max_rotor_velocity = rad_s_no_load * load_factor
```

Typical initial `load_factor` range:

```text
0.20 to 0.40
```

For the example:

```text
3565 rad/s * 0.25 = 891 rad/s
```

That is close to the current world default:

```xml
<maxRotVelocity>800.0</maxRotVelocity>
```

## Gazebo plugin parameters

The bridge only publishes rotor angular velocity. The Gazebo motor plugin converts that velocity into force and torque using parameters such as:

- `maxRotVelocity`
- `motorConstant`
- `momentConstant`
- `rotorDragCoefficient`
- `rollingMomentCoefficient`
- `rotorVelocitySlowdownSim`

Keep these consistent:

```yaml
motors:
  max_rotor_velocity_rad_s: 800.0
```

```xml
<maxRotVelocity>800.0</maxRotVelocity>
```

If the bridge can command `800 rad/s`, but the Gazebo plugin is tuned for a different maximum, the simulated vehicle may not lift or may be too aggressive.

## Battery reference table

| Battery | Nominal voltage | Full voltage |
|---|---:|---:|
| 3S | 11.1 V | 12.6 V |
| 4S | 14.8 V | 16.8 V |
| 6S | 22.2 V | 25.2 V |

Use nominal voltage for conservative initial tuning. Use full voltage when modeling a freshly charged battery.

