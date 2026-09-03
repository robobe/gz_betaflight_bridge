# Contact hit event bridge

## Summary

Add contact-onset events for multiple Gazebo contact items. Each accepted drone
hit publishes MAVLink `NAMED_VALUE_INT`, optionally sends one TCP trigger byte,
and appears through Gazebo's built-in contact visualization.

## Implementation changes

- Instrument `front_sensor_target` in the sensor world with a 1000 Hz contact
  sensor on `/contacts/front_target`; load Gazebo's Contact system and
  `VisualizeContacts` GUI plugin.
- Add contact configuration containing:
  - unique item names and Gazebo topics;
  - drone collision prefix `X3::`;
  - configurable 100 ms clear time before re-arming;
  - an optional TCP listener, enabled on `127.0.0.1:5770`.
- Require item names to be 1-10 safe characters and reject duplicates or longer
  names at startup.
- Subscribe to `gz.msgs.Contacts`, ignore contacts not involving the drone
  prefix, and emit once per contact episode.
- Publish MAVLink `NAMED_VALUE_INT` with:
  - `name`: configured contact item name;
  - `value`: constant `1`;
  - `time_boot_ms`: Gazebo contact simulation time.
- Count contacts as MAVLink output so the existing shared heartbeat remains
  active.
- Accept one nonblocking TCP client and send byte `0x01` per accepted hit. Drop
  and count triggers when disconnected or backpressured; never replay them.
- Log per-item readiness and hit counts plus TCP connection, sent-trigger, and
  dropped-trigger counters.
- Document SDF setup, YAML configuration, MAVLink fields, TCP behavior, and
  native contact markers. Do not build a custom GUI plugin.

## Test plan

- Unit-check drone filtering, repeated-contact suppression, 100 ms clear
  re-arming, unrelated collisions, simulation timestamps, and multiple items.
- Decode MAVLink 2 `NAMED_VALUE_INT` and verify source IDs, checksum, item name,
  simulation time, and `value == 1`.
- Add an end-to-end checker that:
  - binds MAVLink UDP and connects to TCP;
  - moves `X3` into `front_sensor_target` using the `quadcopter_sensor` pose
    service;
  - requires heartbeat, the expected named-value message, and one `0x01`
    trigger;
  - restores the documented X3 starting pose.
- Run existing bridge tests and visually confirm built-in Gazebo contact
  markers.

## Assumptions

- Any matching physical contact counts; force and penetration thresholds are
  deferred.
- A hit means contact onset, not every 1000 Hz contact sample.
- The named value is an event pulse, not a persistent contact state or
  cumulative counter.
- TCP intentionally carries no item identity or timestamp.
