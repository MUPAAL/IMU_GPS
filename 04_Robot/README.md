# 04_Robot — Output Field Reference

WebSocket output (`ws://localhost:8889`). Multiple message types are broadcast on the same connection.

---

## `type: "odom"` — Odometry (20 Hz)

Parsed from the Feather M4 serial line format: `O:{v},{w},{state},{soc}`

| Field | Unit | Description |
|-------|------|-------------|
| `type` | — | `"odom"` |
| `v` | m/s | Linear velocity (forward positive) |
| `w` | rad/s | Angular velocity (left positive) |
| `state` | int | Drive state integer from firmware |
| `soc` | % | Battery state of charge |
| `ts` | Unix s | Host timestamp when line was parsed |

---

## `type: "imu"` — IMU Frame (proxied from 01_IMU, ~50 Hz)

Forwarded directly from `imu_bridge` (`ws://localhost:8766`). Same fields as `01_IMU` output — see `01_IMU/README.md` for full field reference.

Key fields:

| Field | Description |
|-------|-------------|
| `rot` | Quaternion (qi, qj, qk, qr) — magnetic north referenced |
| `euler` | Roll / pitch / yaw in degrees |
| `heading.deg` | North-corrected heading (0=N, 90=E) |
| `heading.dir` | 16-point compass label |
| `accel` | Accelerometer x/y/z (m/s²) |
| `gyro` | Gyroscope x/y/z (rad/s) |
| `cal` | Calibration status 0–3 |
| `hz` | IMU frame rate |

---

## `type: "rtk"` — RTK Frame (proxied from 02_RTK, ~5 Hz)

Forwarded directly from `rtk_bridge` (`ws://localhost:8776`). Same fields as `02_RTK` output — see `02_RTK/README.md` for full field reference.

Key fields:

| Field | Description |
|-------|-------------|
| `lat` / `lon` | Position in decimal degrees (WGS-84) |
| `fix_quality` | 0=no fix, 1=GPS, 4=RTK fixed, 5=RTK float |
| `num_sats` | Satellites in use |
| `hdop` | Horizontal dilution of precision |
| `available` | `true` if fix_quality > 0 or source == "rtk" |

---

## `type: "state_status"` — Drive State Change (event-driven)

Sent once on browser connect, and again whenever the Feather M4 reports a state change.

| Field | Description |
|-------|-------------|
| `active` | `true` = AUTO MODE active, `false` = READY (manual) |

---

## Input (browser → robot)

| Message | Description |
|---------|-------------|
| `{"type": "joystick", "linear": 0.5, "angular": 0.1}` | Velocity command. Clamped to `±MAX_LINEAR` / `±MAX_ANGULAR` from config |
| `{"type": "toggle_state"}` | Toggle AUTO/READY on Feather M4 (sends `\r` over serial) |
| `{"type": "heartbeat"}` | Keep-alive. If missing for > `WATCHDOG_TIMEOUT` seconds, robot is emergency-stopped |

## Notes

- `imu` and `rtk` frames are pass-through proxies — `robot_bridge` does not modify the data beyond overwriting `type`.
- The watchdog timer resets on every `heartbeat` or `joystick` message. Default timeout is 2 seconds.
- Serial format from Feather M4: `O:{v},{w},{state},{soc}\n` for odometry, `S:ACTIVE` / `S:READY` for state changes.
