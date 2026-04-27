# IMU + RTK Navigation System

## Quick Start

```bash
pip install pyserial websockets depthai opencv-python numpy
./start_bridges.sh
```

Opens modules 01_IMU · 02_RTK · 03_Nav · 04_Robot · 06_Camera in one tmux session with separate windows.
Ctrl+C stops the process in the current window but leaves the tmux window open.
Detach: `Ctrl-B D` · Kill: `tmux kill-session -t bridges`

---

A real-time sensor fusion platform for farm robots, combining a BNO085 IMU with an RTK-GPS receiver. Seven independent modules — IMU visualization, RTK mapping, integrated Nav dashboard, robot control, autonomous navigation, camera streaming, and data recording — communicate via WebSocket bridges, all viewable in a browser.

## Architecture

```
┌─────────────┐     serial/SPI      ┌──────────────┐   WS :8766
│  BNO085 IMU │ ──────────────────→ │  imu_bridge  │ ──────────┐
│  (ESP32-C3) │                     │  (01_IMU)    │           │
└─────────────┘                     └──────────────┘           │
                                                               ▼
┌─────────────┐     serial/UART     ┌──────────────┐   WS :8776    ┌──────────────┐  WS :8786
│  RTK GPS    │ ──────────────────→ │  rtk_bridge  │ ──────────┬──→│  nav_bridge  │ ─────────→  Browser
│  receiver   │                     │  (02_RTK)    │           │   │  (03_Nav)    │           http :8785
└─────────────┘                     └──────────────┘           │   └──────────────┘
                                                               │          ▲
                                                               └──────────┘

┌─────────────┐     serial/USB-CDC  ┌───────────────┐  WS :8796
│  Farm-ng    │ ←──────────────────→│  robot_bridge │ ─────────→  Browser
│  Amiga CAN  │   (O:/S: + WASD/V) │  (04_Robot)   │           http :8795
│ (Feather M4)│                     └───────────────┘
└─────────────┘                            ▲
                                           │
                    ┌──────────────────┐    │  WS :8806
                    │  autonav_bridge  │────┤─────────→  Browser
                    │  (05_AutoNav)    │    │           http :8805
                    └──────────────────┘    │
                      ▲ IMU  ▲ RTK         │ velocity commands
                      │      │             │
                    ┌──────────────────┐    │  WS :8826
                    │   sim/replay hub │────┘─────────→  Browser
                    │  (07_SimReplay)  │
                    └──────────────────┘

┌─────────────┐                      ┌───────────────┐  WS :8816
│  OAK-D PoE  │ ──── TCP/IP ───────→ │ camera_bridge │ ─────────→  Browser
│  Camera(s)  │     depthai v3       │  (06_Camera)  │           http :8815
└─────────────┘                      └───────────────┘       MJPEG :8080/8081
```

Each bridge serves its own static web UI over HTTP:

| Module | HTTP | WebSocket | Description |
|--------|------|-----------|-------------|
| `01_IMU` | 8765 | 8766 | 3D IMU orientation + sensor data cards |
| `02_RTK` | 8775 | 8776 | Leaflet map + waypoint management |
| `03_Nav` | 8785 | 8786 | Integrated dashboard (3D + map + all panels) |
| `04_Robot` | 8795 | 8796 | Amiga robot controller (telemetry + WASD/velocity) |
| `05_AutoNav` | 8805 | 8806 | Autonomous navigation (GPS+IMU PID/PurePursuit) |
| `06_Camera` | 8815 | 8816 | OAK-D camera MJPEG streaming (video on 8080/8081) |
| `07_SimReplay` | - | - | Replay/simulation helpers for offline integration |

## Directory Structure

```
IMU_GPS/
├── config.py                    # ★ Unified hyperparameter config for all modules (01–06)
├── start_bridges.sh             # ★ One-command tmux launcher (01–04 + 06, separate windows)
│
├── 01_IMU/
│   ├── bno085_esp32c3/          # ESP32-C3 Arduino firmware (SPI, 50 Hz JSON)
│   │   └── bno085_esp32c3.ino
│   ├── imu_bridge.py            # Serial → WebSocket bridge
│   ├── listen_imu_websocket.py  # Debug WS client
│   ├── requirements.txt
│   └── web_static/
│
├── 02_RTK/
│   ├── rtk_bridge.py            # NMEA serial → WebSocket bridge
│   ├── requirements.txt
│   └── web_static/
│
├── 03_Nav/
│   ├── nav_bridge.py            # IMU + RTK aggregator → unified WS feed
│   ├── listen_nav_websocket.py  # Debug WS client
│   ├── requirements.txt
│   └── web_static/
│
├── 04_Robot/
│   ├── robot_bridge.py          # Amiga serial bridge (bidirectional)
│   ├── listen_robot_websocket.py
│   ├── send_robot_only_demo.py
│   ├── requirements.txt
│   └── web_static/
│
├── 05_AutoNav/
│   ├── autonav_bridge.py        # Autonomous nav (PID/PurePursuit + GPS filters)
│   ├── requirements.txt
│   └── web_static/
│
├── 06_Camera/
│   ├── camera_bridge.py         # OAK-D MJPEG bridge + two-layer plugin orchestrator
│   ├── plugins/
│   │   ├── __init__.py          # FrameProcessor ABC + registry + auto-discovery
│   │   ├── simple_color.py      # Pass-through RGB preview
│   │   ├── path_cam.py          # Yellow-tape path detection (HSV masking)
│   │   ├── depth_cam.py         # Depth colourmap + RGB blend  (requires --stereo)
│   │   ├── obstacle_cam.py      # Near-obstacle warning via disparity (requires --stereo --disparity)
│   │   └── disparity_demo.py    # Minimal raw disparity demo   (requires --stereo --disparity)
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── camera_visualizer.js
│       ├── style.css
│       └── snapshots/           # Auto-created; holds per-session HTML snapshot files
│
├── 07_SimReplay/
│   ├── start_nav_with_replay.sh # One-click: 01 replay + 02 replay + 03 nav + listener (tmux)
│   ├── sim_robot_ws_server.py   # Local fake robot WS simulator
│   ├── demo_filter_by_type.py   # Filtered listener demo for simulator/robot WS
│   └── README.md
│
├── CIRCUITPY/                   # CircuitPython firmware for Adafruit Feather M4 CAN
│   └── code.py
│
└── CLAUDE.md                    # AI coding conventions
```

## Global Configuration — `config.py`

All default hyperparameters for modules 01–06 live in a single file. Edit it once and restart the relevant bridge — no need to touch individual bridge scripts:

```python
# 01_IMU
IMU_SERIAL_PORT  = "/dev/cu.usbmodem101"
IMU_BAUD         = 921600
IMU_WS_PORT      = 8765
IMU_NORTH_OFFSET = 0.0              # heading calibration (degrees)

# 06_Camera
CAM_FPS              = 25
CAM_WIDTH            = 640
CAM_HEIGHT           = 400
CAM_ENABLE_STEREO    = True         # enable depth stream
CAM_ENABLE_DISPARITY = False        # enable raw disparity (extra load; off by default)
```

CLI arguments always override `config.py` values at runtime.

## Quick Start

### Prerequisites

- Python 3.10+
- Arduino IDE (for firmware upload)
- `tmux` (for `start_bridges.sh`)

### 1. Install dependencies

```bash
pip install pyserial websockets
```

### 2. Launch all bridges at once (recommended)

```bash
./start_bridges.sh
```

Opens modules 01–04 and 06 in a single tmux session with separate windows:

```
Window 1: 01_IMU
Window 2: 02_RTK
Window 3: 03_Nav
Window 4: 04_Robot
Window 5: 06_Camera
```

- Detach: `Ctrl-B D`
- Kill session: `tmux kill-session -t bridges`
- Re-attach: `./start_bridges.sh` (attaches if session already exists)

### 3. Run modules individually

```bash
cd 01_IMU && python imu_bridge.py          # http://localhost:8765
cd 02_RTK && python rtk_bridge.py          # http://localhost:8775
cd 03_Nav && python nav_bridge.py          # http://localhost:8785
cd 04_Robot && python robot_bridge.py      # http://localhost:8795
cd 05_AutoNav && python autonav_bridge.py  # http://localhost:8805
cd 06_Camera && python camera_bridge.py   # http://localhost:8815
cd 07_SimReplay && ./start_nav_with_replay.sh # 01/02 replay feed 03_Nav
```

### 4. Camera — depth and disparity flags

```bash
# RGB only (most stable, lowest load)
python camera_bridge.py

# RGB + depth (recommended for 3D perception)
python camera_bridge.py --stereo

# RGB + depth + raw disparity (enables obstacle_cam and disparity_demo plugins)
python camera_bridge.py --stereo --disparity
```

Or set permanently in `config.py`:

```python
CAM_ENABLE_STEREO    = True
CAM_ENABLE_DISPARITY = False   # turn on only when debugging disparity plugins
```

## Module Details

### 01_IMU — IMU Bridge

- **Data flow**: `Serial → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → Browser`
- **Pipeline**: `_parse → _enrich_euler (heading) → _enrich_hz → _serialize`
- **Features**: real-time 3D orientation (Three.js), compass HUD, north-offset calibration, lock-yaw mode, top-north view, 11 sensor data cards; **heading computed in backend**
- **Debug**: `listen_imu_websocket.py`

### 02_RTK — RTK Bridge

- **Data flow**: `Serial → SerialReader → NMEAPipeline → BroadcastLoop → WebSocketServer → Browser`
- **Pipeline**: `_verify_checksum → _dispatch → _parse_gga / _parse_rmc`
- **Features**: Leaflet map (satellite/OSM/offline tiles), waypoint CRUD, CSV import/export, path simulation, track recording, EN/ZH i18n

### 03_Nav — Navigation Dashboard

- **Data flow**: `imu_bridge(WS) + rtk_bridge(WS) → NavLoop(10 Hz) → NavController → WebSocketServer → Browser`
- **Layout**: 3D view (top 40%) + Leaflet map (bottom 60%) | right data panel (320 px)
- **Features**: all IMU + RTK features unified in one page, waypoint arrival detection, heading/north-offset forwarded from IMU backend
- **Debug**: `listen_nav_websocket.py`

### 04_Robot — Amiga Robot Controller

- **Data flow**: `Serial ↔ SerialReader → RobotPipeline → asyncio.Queue → WebSocketServer → Browser`
- **Pipeline**: `_parse → _enrich_state → _enrich_hz → _enrich_odometry → _serialize`
- **Serial protocol**: `O:{speed},{ang_rate},{state},{soc}` telemetry (~20 Hz); accepts WASD and `V{speed},{ang_rate}\n`
- **Features**: WASD/button control, velocity sliders, E-Stop, SOC bar, odometry, Three.js top-down view
- **Debug**: `listen_robot_websocket.py`, `send_robot_only_demo.py`

### 05_AutoNav — Autonomous Navigation Engine

- **Data flow**: `imu_bridge + rtk_bridge + robot_bridge → AutoNavPipeline → velocity commands → robot_bridge`
- **Components**: GeoUtils, MovingAverageFilter, KalmanFilter (4D), PIDController, P2PController, PurePursuitController, WaypointManager, CoveragePlanner (Boustrophedon)
- **Nav modes**: P2P (bearing control) / Pure Pursuit (lookahead path tracking)
- **Filter modes**: Moving Average / Kalman (IMU acceleration + odometry velocity)
- **State machine**: `IDLE → NAVIGATING → FINISHED`

### 06_Camera — OAK-D Camera Bridge

#### Two-layer architecture

```
Layer 1 — CameraDevice  (opened once at startup; never restarted on plugin switch)
  ├─ RGB node         → rgb queue        (always)
  ├─ StereoDepth node → depth queue      (when --stereo)
  └─ StereoDepth node → disparity queue  (when --stereo --disparity)

Layer 2 — FrameProcessor  (swapped atomically; zero stream downtime)
  plugin.required_streams() → ['rgb'] | ['rgb','depth'] | ...
  plugin.process(frames)    → output ndarray → MJPEG encode → Browser
```

#### Plugin system

Drop a `.py` file into `plugins/` with a `@register_processor`-decorated `FrameProcessor` subclass. It is auto-discovered at startup and appears in the browser dropdown with a `config_schema()` UI. No changes to existing code needed.

```python
from . import FrameProcessor, register_processor

@register_processor
class MyPlugin(FrameProcessor):
    PROCESSOR_NAME = "my_plugin"
    PROCESSOR_LABEL = "My Plugin"
    PROCESSOR_DESCRIPTION = "..."

    @classmethod
    def required_streams(cls):
        return ["rgb"]           # declare which streams you need

    def process(self, frames):
        img = frames["rgb"]     # ndarray (BGR, uint8)
        # ... your processing ...
        return img
```

Built-in plugins:

| Plugin | Required streams | Description |
|--------|-----------------|-------------|
| `simple_color` | rgb | Pass-through RGB preview |
| `path_cam` | rgb | Yellow-tape path detection (HSV masking, contour scoring) |
| `depth_cam` | rgb, depth | Depth colourmap + RGB blend |
| `obstacle_cam` | rgb, disparity | Near-obstacle warning (DANGER / CAUTION / CLEAR zones) |
| `disparity_demo` | disparity | Minimal raw disparity colourmap — starter template |

#### Snapshot (pixel inspector)

Click **Capture Frame** in the browser control panel. A self-contained HTML file is saved to `web_static/snapshots/` and opened automatically in a new tab:

- Canvas rendering of the captured output frame
- Hover anywhere to inspect pixel values:
  - All plugins: canvas R/G/B at cursor
  - `rgb` stream: raw R/G/B from sensor
  - `depth` stream: distance in **mm**
  - `disparity` stream: disparity value in **px**

#### Stream flags

| CLI flag | `config.py` key | Default | Effect |
|----------|----------------|---------|--------|
| `--stereo` / `--no-stereo` | `CAM_ENABLE_STEREO` | `True` | Enable Left/Right/StereoDepth nodes → depth stream |
| `--disparity` / `--no-disparity` | `CAM_ENABLE_DISPARITY` | `False` | Enable raw disparity queue (additional CPU/bandwidth) |

### 07_SimReplay — Replay/Simulation Hub

- **Goal**: run downstream modules without hardware by replaying recorded JSONL data.
- **One-click stack**: `./07_SimReplay/start_nav_with_replay.sh` starts IMU replay + RTK replay + Nav bridge + Nav listener.
- **Use case**: when `03_Nav` needs `01_IMU` and `02_RTK` inputs but devices are unavailable.

## Hardware

### BNO085 IMU (ESP32-C3)

| Signal | GPIO |
|--------|------|
| MOSI | 1 |
| MISO | 6 |
| SCK | 7 |
| CS | 0 |
| INT | 5 |
| RST | 2 |
| BOOT | 9 (long-press 3 s to save calibration) |

- **Library**: Adafruit BNO08x (Arduino Library Manager)
- **Interface**: SPI at 1 MHz
- **Output**: JSON over UART at 921600 baud, ~50 Hz

### RTK GPS Receiver

- **Interface**: UART (NMEA 0183)
- **Default baud**: 9600
- **Sentences parsed**: GGA (position/fix/sats), RMC (speed/course)

### OAK-D PoE Camera

- **Connection**: TCP/IP via depthai v3 (`dai.DeviceInfo(ip)`)
- **Default IPs**: cam1 = `10.95.76.11`, cam2 = `10.95.76.10`
- **Sockets used**: CAM_A (RGB), CAM_B (Left), CAM_C (Right)
- **Stereo output size**: 640×400 px per eye; RGB configurable (default 640×400)

## Code Conventions

All code follows the conventions defined in `CLAUDE.md`:

- OOP + Pipeline pattern
- `@dataclass` for data models
- `INPUT / CORE / OUTPUT` banner annotations at I/O boundaries
- English-only code comments, logs, and CLI output
- `logging` module with `.log` file output

## License

Internal project — not published.
