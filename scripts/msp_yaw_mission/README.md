# MSP yaw mission

This package extends the `msp_hover` flight policy with a heading excursion:
take off smoothly to 5 m, settle, rotate CCW 180 degrees at 15 deg/s, rotate
CW back to the starting heading at 15 deg/s, descend at 1 m/s, confirm landing,
and disarm.

Run it from the repository root after Gazebo, SITL, and the bridge are ready:

```bash
python3 scripts/msp_yaw_mission/run_msp_yaw_mission.py
```

Configuration is in `msp_yaw_mission.yaml` beside this file. For example:

```bash
python3 scripts/msp_yaw_mission/run_msp_yaw_mission.py \
  --target-altitude 5 --yaw-rate 15
```

The altitude PID remains active during yaw. Yaw pauses at center when altitude
or vertical speed leaves the configured gate; it resumes after the altitude
loop recovers. Logs and a summary are written under `logs/msp-yaw/`.

Phase transitions are bold and color-coded in an interactive terminal. Set
`NO_COLOR=1` to disable ANSI colors; redirected output is plain automatically.

See `docs/design/msp_yaw_mission.md` for architecture, safety, and tuning.
