# 08_CollabDemo — Collaborator Quick-Start

This folder contains everything a collaborator needs to start receiving
robot data **without** the physical hardware or any bridge processes.

---

## Files

| File | Purpose |
|------|---------|
| `sim_robot_ws_server.py` | **Simulator** — runs a WebSocket server that streams fake IMU / RTK / odom / nav_status data, identical in format to the real `robot_bridge.py` |
| `demo_filter_by_type.py` | **Listener demo** — connects and prints messages; use `--types` to filter to only the types you need |

---

## Quick Start

```bash
pip install websockets

# Terminal 1 — start the simulator (binds to port 9889)
python sim_robot_ws_server.py

# Terminal 2 — listen to all types (connects to port 9889 by default)
python demo_filter_by_type.py

# Or filter to only specific types
python demo_filter_by_type.py --types rtk nav_status
```

---

## Port Reference

| Mode | Port | How to use |
|------|------|------------|
| **Simulator** | `9889` | default — no conflict with real bridges |
| **Real robot** (`robot_bridge.py`) | `8889` | add `--port 8889` to listener scripts |

---

## Connecting to the Real Robot

```bash
# Same machine
python demo_filter_by_type.py --port 8889

# Over LAN
python demo_filter_by_type.py --host 192.168.1.100 --port 8889
```

The simulator and the real bridge use the **identical WebSocket message format**,
so code written against the simulator works against the real robot unchanged.

---

## Message Format Reference

All messages are JSON objects sent over WebSocket (`ws://host:8889`).

### Message cadence

| Type | Rate | Notes |
|------|------|-------|
| `imu` | 10 Hz | Always arrives together with `rtk` + `nav_status` in one burst |
| `rtk` | 10 Hz | Same burst as `imu` |
| `nav_status` | 10 Hz | Same burst as `imu` |
| `odom` | 20 Hz | Independent loop, interleaves between nav bursts |

Typical receive order on one connection:
```
odom odom [imu rtk nav_status] odom odom [imu rtk nav_status] ...
```

---

### `imu` — 10 Hz

```json
{
  "type": "imu",
  "rot":  { "qi": 0.01, "qj": -0.02, "qk": 0.70, "qr": 0.71 },
  "euler": { "roll": 1.2, "pitch": -0.5, "yaw": 135.4, "north_offset_deg": 0.0 },
  "heading": { "raw": 135.4, "deg": 135.4, "dir": "SE" },
  "hz": 20.0
}
```

| Field | Unit | Notes |
|-------|------|-------|
| `rot.qi/qj/qk/qr` | — | BNO085 quaternion (i,j,k,r) = (x,y,z,w) in Three.js |
| `euler.roll/pitch/yaw` | degrees | ZYX intrinsic, yaw includes north offset |
| `heading.deg` | degrees | 0=N, 90=E, clockwise |
| `heading.dir` | string | N / NE / E / SE / S / SW / W / NW |

---

### `rtk` — 1 Hz

```json
{
  "type": "rtk",
  "lat": 38.94129286, "lon": -92.31884601, "alt": 220.5,
  "fix_quality": 4, "num_sats": 14, "hdop": 0.8,
  "speed_knots": 0.3, "track_deg": 45.0,
  "source": "rtk", "available": true
}
```

| `fix_quality` | Meaning |
|---------------|---------|
| 0 | No Fix |
| 1 | GPS |
| 2 | DGPS |
| 4 | RTK Fixed |
| 5 | RTK Float |

---

### `odom` — 20 Hz

```json
{
  "type": "odom",
  "v": 0.45, "w": -0.12,
  "state": 1, "soc": 87,
  "ts": 1712345678.9
}
```

| Field | Unit | Notes |
|-------|------|-------|
| `v` | m/s | Linear velocity (positive = forward) |
| `w` | rad/s | Angular velocity (positive = left turn) |
| `state` | int | 1 = ACTIVE, 0 = READY |
| `soc` | % | Battery state of charge |

---

### `nav_status` — 5 Hz

```json
{
  "type": "nav_status",
  "state": "navigating",
  "progress": [2, 5],
  "distance_m": 3.7,
  "heading_deg": 92.4, "heading_dir": "E",
  "target_bearing": 92.4,
  "nav_mode": "PURE_PURSUIT",
  "filter_mode": "KALMAN",
  "tolerance_m": 0.5
}
```

| `state` | Meaning |
|---------|---------|
| `idle` | Stopped, no active goal |
| `navigating` | Autonomously following waypoints |
| `finished` | All waypoints reached |
