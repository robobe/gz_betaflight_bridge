# Deferred MAVLink vision odometry design

This is a deferred compatibility option. The bridge currently publishes only
MAVLink `ODOMETRY` from Gazebo odometry.

`VISION_POSITION_ESTIMATE` (102) carries local `x,y,z` in metres and
`roll,pitch,yaw` in radians, plus a 21-element upper-triangle pose covariance
and reset counter. `VISION_SPEED_ESTIMATE` (103) separately carries global
`x,y,z` speed in m/s, a full 3x3 velocity covariance, and reset counter. Both
use microsecond epoch-or-boot timestamps.

If a named consumer requires this compatibility mode, send both messages from
the same Gazebo sample with identical timestamps and the same local-NED
convention as `ODOMETRY`. Do not support configuring only half of the pair.
They require two messages, have no explicit frame IDs, and omit angular
velocity, so they are not part of the current bridge implementation.
