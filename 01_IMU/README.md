# 01_IMU — BNO085 IMU Bridge

## Quick Start

```bash
pip install pyserial websockets
python imu_bridge.py --port /dev/ttyACM0
# Browser: http://localhost:8765
```

---

A serial-to-WebSocket bridge for the BNO085 IMU mounted on an ESP32-C3. Reads quaternion + sensor data over UART at 50 Hz, computes Euler angles and heading in the backend, and streams JSON to the browser for real-time 3D visualization.

## Architecture

```
┌─────────────┐   SPI 1 MHz    ┌──────────────┐   UART 921600   ┌──────────────┐
│  BNO085 IMU │ ─────────────→ │  ESP32-C3    │ ──────────────→ │  imu_bridge  │
│             │   50 Hz JSON   │  (firmware)  │   compact JSON  │  (Python)    │
└─────────────┘                └──────────────┘                 └──────┬───────┘
                                                                        │ WS :8766
                                                               ┌────────▼────────┐
                                                               │ heading_override │  HTTP :8767
                                                               │   (optional)     │◄─── browser
                                                               └────────┬────────┘  (set heading)
                                                                        │ WS :8768
                                              ┌─────────────────────────┤
                                              ▼                         ▼
                                    nav_bridge / autonav          HTTP :8765
                                    recorder                  (browser 3D viewer)
```

**Data flow inside `imu_bridge.py`:**

```
Serial Port → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → Browser
                                  ↑
       _parse → _enrich_euler → _enrich_heading → _enrich_hz → _serialize
```

**Data flow through `heading_override.py` (when active):**

```
imu_bridge WS :8766 → _upstream_loop() → _apply_override() → asyncio.Queue → WebSocketServer WS :8768
                                                ↑
                                    browser sends {"set_heading_override": <deg>}
```

## Directory Structure

```
01_IMU/
├── bno085_esp32c3/
│   └── bno085_esp32c3.ino   # ESP32-C3 Arduino firmware (SPI → UART, 50 Hz compact JSON)
├── imu_bridge.py            # Serial → WebSocket bridge (OOP + Pipeline)
├── heading_override.py      # ★ Middleware filter — overrides heading when acc < threshold
├── listen_imu_websocket.py  # Debug WS client (prints tabular sensor data)
├── requirements.txt
├── web_static/
    ├── index.html
    ├── imu_visualizer.js    # Three.js 3D viewer + compass HUD + sensor data cards
    └── style.css
```

## Configuration

All defaults live in `../config.py`. CLI args override at runtime:

```python
IMU_SERIAL_PORT  = "/dev/cu.usbmodem1201"   # Mac: cu.usbmodem* · Linux: /dev/ttyACM0
IMU_BAUD         = 921600                   # must match firmware
IMU_WS_PORT      = 8765                     # HTTP port; WebSocket = IMU_WS_PORT + 1 (8766)
IMU_NORTH_OFFSET = 0.0                      # heading calibration (degrees)
```

## Usage

```bash
# Defaults from config.py
python imu_bridge.py

# Mac default port (matches config.py)
python imu_bridge.py --port /dev/cu.usbmodem1201

# Linux port override
python imu_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765 --north-offset 12.5
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `config.IMU_SERIAL_PORT` | Serial device path |
| `--baud` | `921600` | Serial baud rate |
| `--ws-port` | `8765` | HTTP port; WebSocket binds to `ws-port + 1` |
| `--north-offset` | `0.0` | Initial north offset applied to yaw (degrees) |

## Module Details

### Pipeline Stages (`IMUPipeline`)

| Stage | Method | Input → Output | Description |
|-------|--------|----------------|-------------|
| 1 | `_parse` | raw JSON str → `IMUFrame` | Decode and expand compact Arduino format |
| 2 | `_enrich_euler` | `IMUFrame` → `IMUFrame` | ZYX Euler angles from quaternion |
| 3 | `_enrich_heading` | `IMUFrame` → `IMUFrame` | Web-aligned heading from `game_rot` + north offset |
| 4 | `_enrich_hz` | `IMUFrame` → `IMUFrame` | Rolling-window frame rate (50-frame window) |
| 5 | `_serialize` | `IMUFrame` → JSON str | `to_dict()` → JSON for WebSocket broadcast |

### Key Design Notes

- **Heading computed in backend**: `_enrich_heading` applies the same BNO085-Z-up → Three.js-Y-up frame correction that was previously done in the browser, keeping the frontend stateless.
- **`game_rot` preferred for heading**: Game Rotation Vector (no magnetometer) gives a stable heading indoors and on metal-heavy machines. Falls back to `rot` if `game_rot` is absent.
- **North offset**: Settable via `--north-offset` CLI arg or at runtime from the browser (`{"set_north_offset": <deg>}` WebSocket message).
- **Serial auto-reconnect**: `SerialReader` retries every 3 s on `SerialException`.

### Classes

| Class | Role |
|-------|------|
| `IMUFrame` | `@dataclass` — single sensor frame through the pipeline |
| `FrameRateTracker` | Rolling-window Hz calculator |
| `IMUPipeline` | Stateful pipeline — holds `north_offset_deg`, runs stages |
| `SerialReader` | Daemon thread — reads serial, pushes to `asyncio.Queue` |
| `WebSocketServer` | Async — broadcasts queue items to all connected clients |
| `HttpFileServer` | Daemon thread — serves `web_static/` via Python's HTTP server |
| `IMUBridge` | Top-level app — wires all components, starts threads + event loop |

### WebSocket Message Format

Outbound JSON (one frame per message, ~50 Hz):

```json
{
  "rot":      { "qi": 0.0, "qj": 0.0, "qk": 0.0, "qr": 1.0, "acc": 3.0 },
  "euler":    { "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "north_offset_deg": 0.0 },
  "heading":  { "raw": 0.0, "deg": 0.0, "dir": "N" },
  "hz":       50.0,
  "game_rot": { "qi": 0.0, "qj": 0.0, "qk": 0.0, "qr": 1.0 },
  "accel":    { "x": 0.0, "y": 0.0, "z": 9.8 },
  "lin_accel":{ "x": 0.0, "y": 0.0, "z": 0.0 },
  "gravity":  { "x": 0.0, "y": 0.0, "z": 9.8 },
  "gyro":     { "x": 0.0, "y": 0.0, "z": 0.0 },
  "mag":      { "x": 0.0, "y": 0.0, "z": 0.0 },
  "steps":    0,
  "cal":      3,
  "ts":       12345
}
```

Inbound JSON (browser → bridge):

```json
{ "set_north_offset": 12.5 }
```

## Firmware — `bno085_esp32c3.ino`

### Hardware Wiring (ESP32-C3)

| Signal | GPIO |
|--------|------|
| MOSI | 1 |
| MISO | 6 |
| SCK | 7 |
| CS | 0 |
| INT | 5 |
| RST | 2 |
| BOOT | 9 (long-press 3 s to save calibration DCD) |

- **Library**: Adafruit BNO08x (Arduino Library Manager)
- **SPI frequency**: 1 MHz
- **Sensor poll rate**: 100 Hz (`REPORT_INTERVAL_US = 10000`)
- **JSON output rate**: 50 Hz (`OUTPUT_INTERVAL_MS = 20`)

### Enabled Sensor Reports

| Report | ID | Output fields |
|--------|----|---------------|
| Rotation Vector | `SH2_ROTATION_VECTOR` | `r` → qi, qj, qk, qr, acc |
| Game Rotation Vector | `SH2_GAME_ROTATION_VECTOR` | `g` → qi, qj, qk, qr |
| Accelerometer | `SH2_ACCELEROMETER` | `a` → x, y, z (m/s²) |
| Linear Acceleration | `SH2_LINEAR_ACCELERATION` | `l` → x, y, z (m/s²) |
| Gravity | `SH2_GRAVITY` | `v` → x, y, z (m/s²) |
| Gyroscope (calibrated) | `SH2_GYROSCOPE_CALIBRATED` | `w` → x, y, z (rad/s) |
| Magnetometer (calibrated) | `SH2_MAGNETIC_FIELD_CALIBRATED` | `m` → x, y, z (µT) |
| Step Counter | `SH2_STEP_COUNTER` | `s` → steps |

### Compact JSON Protocol

Frames are kept under 256 bytes to avoid USB CDC-ACM buffer truncation. `imu_bridge.py` expands them back to full names via `_expand_compact()`.

| Compact key | Full name | Format |
|-------------|-----------|--------|
| `t` | `ts` | scalar (uint32, millis) |
| `r` | `rot` | `[qi, qj, qk, qr, acc]` — %.3f quat, %.2f acc |
| `g` | `game_rot` | `[qi, qj, qk, qr]` — %.3f |
| `a` | `accel` | `[x, y, z]` — %.2f m/s² |
| `l` | `lin_accel` | `[x, y, z]` — %.2f m/s² |
| `v` | `gravity` | `[x, y, z]` — %.2f m/s² |
| `w` | `gyro` | `[x, y, z]` — %.3f rad/s |
| `m` | `mag` | `[x, y, z]` — %.1f µT |
| `s` | `steps` | scalar (uint32) |
| `c` | `cal` | scalar (uint8, 0–3) |

Example raw frame from firmware:
```
{"t":12345,"r":[0.001,-0.002,0.707,0.707,3.00],"g":[0.001,-0.002,0.707,0.707],"a":[0.01,-0.02,9.81],"l":[0.01,-0.02,0.00],"v":[0.00,0.00,9.81],"w":[0.001,-0.001,0.002],"m":[20.1,-5.3,42.7],"s":0,"c":3}
```

### Calibration Save

Long-press the BOOT button (GPIO9) for 3 seconds. The firmware calls `sh2_saveDcdNow()` to persist calibration to flash. Confirmation appears in serial output:

```json
{"event":"calibration_saved"}
```

## Heading Override — `heading_override.py`

An optional middleware filter that sits between `imu_bridge.py` and downstream consumers. When `rot.acc` drops below the accuracy threshold (default `2.0`), it replaces the heading fields with a user-supplied value from the browser and clamps `rot.acc` to the threshold so downstream systems receive a consistent frame.

### When to use

Run `heading_override.py` when the BNO085 magnetometer is poorly calibrated (acc < 2) and you have an external reference heading to inject — e.g. from a compass, GPS course-over-ground, or known landmark bearing.

### Usage

```bash
# Defaults: upstream ws://localhost:8766, HTTP :8767, WS :8768
python heading_override.py

# Custom upstream / threshold
python heading_override.py --upstream ws://localhost:8766 --ws-port 8767 --threshold 2.0
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--upstream` | `ws://localhost:8766` | WebSocket URL of `imu_bridge` |
| `--ws-port` | `8767` | HTTP port for control UI; WebSocket binds to `ws-port + 1` |
| `--threshold` | `2.0` | `rot.acc` value below which override activates (0–3) |

Open `http://localhost:8767` to set or clear the override heading from the browser.

> **Note:** `web_static_override/index.html` (the control UI) is not yet implemented. The Python bridge and WebSocket protocol are complete; the frontend is pending.

### What it modifies (when active)

| Field | Action |
|-------|--------|
| `heading.raw` | Replaced with override value |
| `heading.deg` | Replaced with override value |
| `heading.dir` | Recomputed from override value |
| `euler.yaw` | Replaced with override value |
| `rot.acc` | Clamped to threshold (e.g. 2.0) |
| `override_active` | Set to `true` (added to every frame) |

All other fields (quaternion, gyro, accel, etc.) pass through unchanged.

### Browser control messages (WS :8768)

```json
{ "set_heading_override": 182.5 }
{ "clear_heading_override": true }
```

### Downstream consumers

When `heading_override.py` is running, point downstream bridges at WS :8768 instead of :8766.

**`config.py` currently points these at `:8766` (unchanged):**

```python
NAV_IMU_WS     = "ws://localhost:8766"   # 03_Nav  — update to :8768 to use override
AUTONAV_IMU_WS = "ws://localhost:8766"   # 05_AutoNav — same
```

Update those two values in `config.py` and restart the affected bridges to route through `heading_override.py`.

### Implementation note

`heading_override.py` imports `WebSocketServer` and `HttpFileServer` directly from `imu_bridge.py` — no server infrastructure is re-implemented. The filter itself is a single function `_apply_override()` with module-level state.

## Debug Tools

### `listen_imu_websocket.py`

Connects to `imu_bridge.py` and prints a live tabular view of all sensor fields:

```bash
python listen_imu_websocket.py --url ws://localhost:8766
```

Output:
```
Roll (°) | Pitch (°) | Yaw (°)  | Heading (°) | Direction | Hz   | Quaternion
---------------------------------------------------------------------------------------------------
   0.12  |     -1.45 |   182.33 |      182.33 | S         | 50.0 | [ 0.001, -0.002,  0.707,  0.707]
```

## Browser UI Features

- Real-time 3D orientation (Three.js cube model)
- Compass HUD with heading and 16-point direction label
- North-offset calibration slider (writes `set_north_offset` back to bridge)
- Lock-yaw mode and top-north view toggle
- Sensor data cards: roll/pitch/yaw, heading, Hz, quaternion, accel, lin\_accel, gravity, gyro, mag, cal, steps
- **`rot.acc` accuracy display** in the Rotation Vector card, color-coded: green ≥ 3 · yellow = 2 · red < 2 (red = override will activate)

## Ports

| Script | HTTP | WebSocket | Notes |
|--------|------|-----------|-------|
| `imu_bridge.py` | 8765 | 8766 | source — always running |
| `heading_override.py` | 8767 | 8768 | optional middleware; re-broadcasts :8766 with heading injection |
