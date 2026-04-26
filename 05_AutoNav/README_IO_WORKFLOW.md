# 05_AutoNav — I/O Workflow

Autonomous navigation module: follows a GPS waypoint path using Pure Pursuit + PID control.

## Architecture

```
INPUT   ImuWsClient      ws://localhost:8766  →  heading_deg
        RtkWsClient      ws://localhost:8776  →  lat, lon
        PathLoader       path.json           →  list[Waypoint]

CORE    GeoMath          haversine, bearing, normalize_angle
        PIDController    angular velocity PID (anti-windup)
        MovingAverage    angular output smoothing
        PurePursuitPlanner  lookahead waypoint selection
        AutoNavController   state machine + velocity computation

OUTPUT  RobotWsClient    ws://localhost:8889  →  joystick JSON
        AutoNavWsServer  ws://localhost:8806  →  autonav_status broadcast
        HttpFileServer   http://localhost:8805
```

## Running

```bash
# 1. Start autonav_bridge
python 05_AutoNav/autonav_bridge.py

# 2. Observe status (in another terminal)
python 05_AutoNav/listen_autonav.py

# 3. Send start command (in listen_autonav terminal, type and press Enter)
start
```

## Offline testing (no hardware)

```bash
# 1. Replay recorded IMU + RTK data
python 05_AutoNav/replay_imu_rtk.py

# 2. Start autonav_bridge (in another terminal)
python 05_AutoNav/autonav_bridge.py

# 3. Observe
python 05_AutoNav/listen_autonav.py
```

## Input contracts

### IMU frame — ws://localhost:8766
```json
{
  "type": "imu_frame",
  "heading": {
    "deg": 135.4,
    "dir": "SE"
  }
}
```
Used field: `heading.deg` (0–360, true north, north_offset applied)

### RTK frame — ws://localhost:8776
```json
{
  "type": "rtk_frame",
  "lat": 38.9412,
  "lon": -92.3188,
  "fix_quality": 4,
  "num_sats": 12
}
```
Used fields: `lat`, `lon`

### path.csv
```
id    lat    lon    tolerance_m    max_speed
0     38.94130  -92.31880  0.5  1
...
```
Columns (tab or comma separated):
- `id`: integer index
- `lat`: latitude (decimal degrees, +N)
- `lon`: longitude (decimal degrees, +E)
- `tolerance_m`: waypoint arrival radius (metres)
- `max_speed`: maximum linear speed for this segment (m/s)

## Output contracts

### → robot_bridge — ws://localhost:8889
```json
{"type": "joystick", "linear": 0.30, "angular": -0.12, "force": 1.0}
```
- `linear`: m/s, range [0.0, MAX_LINEAR_VEL]
- `angular`: rad/s, range [-MAX_ANGULAR_VEL, +MAX_ANGULAR_VEL]
- `force`: always 1.0

### Status broadcast — ws://localhost:8806
```json
{
  "type": "autonav_status",
  "version": 1,
  "state": "running",
  "current_wp_idx": 2,
  "total_wp": 5,
  "dist_to_wp_m": 1.23,
  "target_bearing_deg": 45.0,
  "heading_deg": 42.1,
  "bearing_error_deg": 2.9,
  "linear": 0.30,
  "angular": -0.05,
  "gps_age_s": 0.12,
  "imu_age_s": 0.08
}
```

### Control commands → ws://localhost:8806
```json
{"type": "start"}    // idle → running
{"type": "stop"}     // any  → idle
{"type": "pause"}    // running → paused
{"type": "resume"}   // paused → running
```

## State machine

```
idle ──start()──► running ──GPS/IMU timeout──► paused ──resume()──► running
     ◄──stop()──           ──all WP reached──► arrived
```

- `paused` by timeout: auto-resumes when sensors recover
- `paused` by command: requires explicit `resume`
- Entering `paused` or `arrived`: immediately sends linear=0, angular=0

## Algorithm

**Pure Pursuit (simplified):**
1. Advance `current_idx` when distance < `REACH_TOLERANCE_M` for `ARRIVE_FRAMES` consecutive ticks
2. Select first waypoint at distance ≥ `LOOKAHEAD_M` as steering target
3. Fallback to last waypoint if all remaining are within lookahead

**Steering:**
```
target_bearing = bearing(current_pos, target_wp)
error          = normalize(target_bearing - heading_deg)  # (-180, 180]
angular        = PID(error) → MovingAverage
```

**Deceleration (near final waypoint):**
```
if dist_to_final < DECEL_RADIUS_M:
    linear = max(0.10, MAX_LINEAR * dist_to_final / DECEL_RADIUS_M)
```

## Key parameters (config.py)

| Parameter | Default | Description |
|---|---|---|
| `AUTONAV_MAX_LINEAR_VEL` | 1.0 m/s | Maximum linear speed |
| `AUTONAV_MAX_ANGULAR_VEL` | 1.0 rad/s | Maximum turn rate |
| `AUTONAV_LOOKAHEAD_M` | 2.0 m | Pure Pursuit lookahead distance |
| `AUTONAV_DECEL_RADIUS_M` | 3.0 m | Deceleration zone near goal |
| `AUTONAV_REACH_TOLERANCE_M` | 1.5 m | (bridge constant) Waypoint arrival radius |
| `AUTONAV_PID_KP/KI/KD` | 0.8/0.01/0.05 | PID gains |
| `AUTONAV_GPS_TIMEOUT_S` | 5.0 s | Sensor timeout → pause |
| `AUTONAV_MA_WINDOW` | 10 | Angular smoothing window |

## Field calibration

If the robot turns in the wrong direction, change `ANGULAR_SIGN = -1` in `autonav_bridge.py` (line ~65).

## Observation boundaries

- **Input (IMU):** `ImuWsClient.run()` — async for loop in `autonav_bridge.py`
- **Input (RTK):** `RtkWsClient.run()` — async for loop in `autonav_bridge.py`
- **Output (Robot):** `RobotWsClient.run()` — `ws.send(msg)` in `autonav_bridge.py`
- **Status log:** `data_log/autonav_raw_<timestamp>.jsonl` (written by `listen_autonav.py`)
