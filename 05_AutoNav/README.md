# 05_AutoNav — Autonomous Navigation

Pure Pursuit + PID path tracking module. Reads IMU heading and RTK position, follows waypoints defined in `path.csv`, and sends velocity commands to `04_Robot`.

## File Structure

```
05_AutoNav/
├── autonav_bridge.py   # I/O framework: sensor reading, state machine, WS/HTTP server
├── autonav_algo.py     # Control algorithm: reads params from config.py, edit to change steering
├── path.csv            # Default waypoint file (overridable via UI LOAD CSV button)
├── listen_autonav.py   # Debug tool: prints live navigation status
├── replay_imu_rtk.py   # Offline tool: replays recorded IMU/RTK data without hardware
└── web_static/         # Browser dashboard (opens automatically at http://localhost:8805)
```

## Quick Start

```bash
python autonav_bridge.py
```

A browser tab opens automatically at `http://localhost:8805`.

## Dashboard Controls

| Button | Effect |
|--------|--------|
| **START** | Begin navigation from waypoint 0 |
| **STOP** | Stop navigation, reset to idle |
| **PAUSE** | Hold in place, preserve current waypoint index |
| **RESUME** | Continue from paused position |
| **▲ W / ▼ S** | Manual straight-line drive (idle mode only); hold to move |
| **LOAD CSV** | Upload a new waypoint file at runtime; navigation resets to idle |
| **MARK POS** | Record current GPS position as forward calibration point |
| **CALIBRATE** | Compute heading offset from marked point and apply to `01_IMU` |

Keyboard shortcuts: hold `W` / `S` for manual drive (same as buttons).

## Data Flow

```
imu_bridge  :8766 ──→ ImuWsClient  ─┐
                                     ├─ AutoNavLoop → algo.compute() → joystick cmd
rtk_bridge  :8776 ──→ RtkWsClient  ─┘                                      │
                                                                             ↓
path.csv (or uploaded CSV) ─────────────────────────────────→  robot_bridge :8889
                                                                             │
                                          AutoNavWsServer :8806 ◄────────────┘
                                          (Dashboard status + control commands)
```

## Tuning Parameters — `config.py`

**All parameters live in the root `config.py`.** Change a value there and restart `autonav_bridge.py` — no need to edit any other file.

### Path / Waypoint

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTONAV_LOOKAHEAD_M` | `1.0` | Pure Pursuit lookahead distance (m). Larger = smoother path, cuts corners more. Smaller = tighter waypoint tracking. |
| `AUTONAV_REACH_TOL_M` | `0.5` | Arrival radius (m). Robot advances to next waypoint when raw GPS is within this distance. RTK cm accuracy supports values as low as 0.3 m. |
| `AUTONAV_ARRIVE_FRAMES` | `1` | Consecutive frames inside `REACH_TOL_M` required to confirm arrival. 1 = immediate (reliable with RTK). Raise to 2–3 if false arrivals occur from GPS jumps. |
| `AUTONAV_DECEL_RADIUS_M` | `1.5` | Distance from the **final** waypoint where the robot begins decelerating. Increase for heavier/faster robots that need more braking distance. |

### Speed

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTONAV_MAX_LINEAR_VEL` | `1.0` | Maximum forward speed (m/s). Hard ceiling applied before `speed_ratio` slider. |
| `AUTONAV_MIN_LINEAR_VEL` | `0.1` | Minimum speed during end-of-path deceleration (m/s). Prevents the robot from stopping mid-approach. |
| `AUTONAV_MAX_ANGULAR_VEL` | `1.0` | Maximum angular velocity (rad/s). Clamps PID output. |
| `AUTONAV_MANUAL_SPEED` | `0.4` | Speed used by the W/S manual drive buttons (m/s). |

### Steering Behaviour

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTONAV_TURN_IN_PLACE_DEG` | `10.0` | If heading error exceeds this threshold (°), the robot stops forward motion and rotates in place before driving. Set `0` to disable (always move forward). Larger values = more aggressive cornering. |
| `AUTONAV_TURN_SLOWDOWN` | `True` | Scale linear speed down proportionally with heading error (full speed at 0°, `MIN_LINEAR` at 60°+). Reduces overshoot on corrections. |
| `AUTONAV_DEAD_ZONE_DEG` | `3.0` | Heading errors smaller than this are treated as zero — no correction issued. Prevents continuous micro-corrections and motor chatter. Raise to 5–8° if the robot oscillates slightly while driving straight. |

### PID Gains

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTONAV_PID_KP` | `0.15` | Proportional gain. Saturation point = `MAX_ANGULAR / KP` (at 0.15: saturates at 6.7° error). Raise for faster response; lower if oscillation appears. |
| `AUTONAV_PID_KI` | `0.005` | Integral gain. Corrects persistent steady-state heading offset. Keep small — large values cause slow wind-up oscillation. |
| `AUTONAV_PID_KD` | `0.15` | Derivative gain. Damps overshoot. Raise (→ 0.3) if the robot overshoots after turns; lower if commands become jittery. |

### Filters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `AUTONAV_MA_WINDOW` | `5` | GPS sliding-average window (frames). At 5 Hz: 5 frames = 1 s of smoothing. Reduces bearing jumps from RTK multipath. Decrease if path response feels sluggish. |
| `AUTONAV_HEADING_ALPHA` | `0.3` | Heading exponential low-pass coefficient (0 < α ≤ 1). α = 1.0: raw IMU (no filter). α = 0.3: ~0.6 s time constant. α = 0.1: ~2 s (very slow). Lower if physical robot swing causes oscillation; raise if heading display lags. |

### Field Tuning Quick Reference

| Symptom | Fix |
|---------|-----|
| Robot snakes / oscillates | Lower `KP`, raise `DEAD_ZONE_DEG`, lower `HEADING_ALPHA` |
| Heading display lags behind actual | Raise `HEADING_ALPHA` |
| Robot doesn't reach waypoint, loops back | Raise `REACH_TOL_M`, lower `ARRIVE_FRAMES` |
| Robot cuts corners / skips waypoints | Lower `LOOKAHEAD_M`, lower `REACH_TOL_M` |
| Robot overshoots and spins | Raise `KD`, enable `TURN_IN_PLACE_DEG` |
| Robot turns wrong way | Set `ANGULAR_SIGN = -1` in `autonav_algo.py` |

## Waypoint File (`path.csv`)

Tab or comma separated. Only `lat` and `lon` are used — `tolerance_m` and `max_speed` are reserved for future per-waypoint overrides.

```
id,lat,lon,tolerance_m,max_speed
0,38.94130,-92.31896,0.5,1
1,38.94140,-92.31880,0.5,1
```

**Runtime upload**: click **LOAD CSV** in the dashboard to load any CSV without restarting. Navigation resets to idle and the waypoint table updates immediately.

## Heading Calibration

IMU `heading.deg` depends on `north_offset_deg`. To calibrate in the field:

1. Park robot at origin. Click **MARK POS** to record current GPS position.
2. Drive robot straight forward several metres (use W button or joystick).
3. Return robot to origin.
4. Click **CALIBRATE** — the bridge computes `bearing(origin → mark)`, subtracts `heading.raw`, and sends `set_north_offset` to `01_IMU` bridge via WebSocket. `heading.deg` is updated live.

The calibration panel shows:
- **MARKED** — recorded forward GPS point
- **CURRENT** — live GPS position
- **BEARING** — current→mark bearing (the expected heading at origin)
- **OFFSET** — last applied `north_offset_deg` (green = applied)

## Debugging

Enable verbose terminal output — set in `autonav_algo.py`:

```python
ALGO_DEBUG = True
```

Offline testing without hardware:

```bash
# Terminal 1: replay recorded IMU + RTK data
python replay_imu_rtk.py

# Terminal 2: start navigation
python autonav_bridge.py
```

Monitor output stream:

```bash
python listen_autonav.py
```

## Ports

| Purpose | Port |
|---------|------|
| HTTP Dashboard | 8805 |
| AutoNav WebSocket (status + control) | 8806 |
| Reads IMU WS | 8766 |
| Reads RTK WS | 8776 |
| Writes Robot WS | 8889 |

## Safety

- **Sensor timeout**: navigation auto-pauses if GPS or IMU data age exceeds `AUTONAV_GPS_TIMEOUT_S` (default 5 s); auto-resumes when sensors recover.
- **GPS fix filter**: frames with `fix_quality == 0` do not update GPS age, preventing false "data is fresh" readings.
- **Watchdog heartbeat**: zero-velocity command sent to `robot_bridge` every second to prevent runaway on connection loss.
- **Manual drive interlock**: W/S buttons only work in `idle` state — cannot override an active navigation session.
