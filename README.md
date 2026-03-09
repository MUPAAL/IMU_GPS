# IMU + RTK Navigation System

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
                    │ recorder_bridge  │────┘─────────→  Browser
                    │  (07_Recorder)   │              http :8825
                    └──────────────────┘

┌─────────────┐                     ┌───────────────┐  WS :8816
│  OAK-D      │ ──────────────────→ │ camera_bridge │ ─────────→  Browser
│  Camera     │     depthai / USB   │  (06_Camera)  │           http :8815
└─────────────┘                     └───────────────┘        MJPEG :8080/8081
```

Each bridge also serves its own static web UI over HTTP:

| Module | HTTP | WebSocket | Description |
|--------|------|-----------|-------------|
| `01_IMU` | 8765 | 8766 | 3D IMU orientation + sensor data cards |
| `02_RTK` | 8775 | 8776 | Leaflet map + waypoint management |
| `03_Nav` | 8785 | 8786 | Integrated dashboard (3D + map + all panels) |
| `04_Robot` | 8795 | 8796 | Amiga robot controller (telemetry + WASD/velocity) |
| `05_AutoNav` | 8805 | 8806 | Autonomous navigation (GPS+IMU PID/PurePursuit) |
| `06_Camera` | 8815 | 8816 | OAK-D camera MJPEG streaming (video on 8080/8081) |
| `07_Recorder` | 8825 | 8826 | Multi-source CSV data recorder |

## Directory Structure

```
IMU_GPS/
├── 01_IMU/
│   ├── bno085_esp32c3/          # ESP32-C3 Arduino firmware (SPI, 50 Hz JSON output)
│   │   └── bno085_esp32c3.ino
│   ├── imu_bridge.py            # Serial → WebSocket bridge (OOP + Pipeline)
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── imu_visualizer.js    # Three.js r160 + OrbitControls
│       └── style.css            # Dark theme
│
├── 02_RTK/
│   ├── rtk_bridge.py            # NMEA serial → WebSocket bridge (OOP + Pipeline)
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── rtk_visualizer.js    # Leaflet map + waypoints + simulation
│       ├── style.css            # Light theme
│       └── assets/tiles/        # Offline map tiles (optional)
│
├── 03_Nav/
│   ├── nav_bridge.py            # Aggregator: merges IMU + RTK into single WS feed
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── nav_visualizer.js    # Three.js + Leaflet combined
│       ├── style.css            # Light theme
│       └── assets/tiles → ../../02_RTK/web_static/assets/tiles
│
├── 04_Robot/
│   ├── robot_bridge.py          # Amiga robot serial bridge (bidirectional, OOP + Pipeline)
│   ├── requirements.txt
│   └── web_static/
│       ├── index.html
│       ├── robot_visualizer.js  # Three.js top-down view + control panel
│       └── style.css            # Dark theme + control widgets
│
├── 05_AutoNav/
│   ├── autonav_bridge.py        # Autonomous nav engine (PID/PurePursuit + GPS filters)
│   ├── requirements.txt         # websockets, numpy
│   └── web_static/
│       ├── index.html
│       ├── autonav_visualizer.js # Leaflet map + waypoints + coverage planner
│       └── style.css            # Light theme
│
├── 06_Camera/
│   ├── camera_bridge.py         # OAK-D MJPEG streaming + WS control
│   ├── requirements.txt         # websockets, depthai, opencv-python, numpy
│   └── web_static/
│       ├── index.html
│       ├── camera_visualizer.js  # MJPEG display + camera switch
│       └── style.css            # Dark theme
│
├── 07_Recorder/
│   ├── recorder_bridge.py       # Multi-source CSV recorder (IMU+RTK+Robot)
│   ├── requirements.txt         # websockets
│   └── web_static/
│       ├── index.html
│       ├── recorder_visualizer.js # Recording control + file management
│       └── style.css            # Light theme
│
├── CIRCUITPY/                   # CircuitPython firmware for Adafruit Feather M4 CAN
│   └── code.py                  # Farm-ng Amiga CAN bridge (O:/S: output, WASD/V input)
│
└── CLAUDE.md                    # AI coding conventions
```

## Hardware

### BNO085 IMU (ESP32-C3)

| Signal | GPIO |
|--------|------|
| MOSI   | 1    |
| MISO   | 6    |
| SCK    | 7    |
| CS     | 0    |
| INT    | 5    |
| RST    | 2    |
| BOOT   | 9 (long-press 3 s to save calibration) |

- **Library**: Adafruit BNO08x (Arduino Library Manager)
- **Interface**: SPI at 1 MHz
- **Output**: JSON over UART at 921600 baud, ~50 Hz

### RTK GPS Receiver

- **Interface**: UART (NMEA 0183)
- **Default baud**: 9600
- **Sentences parsed**: GGA (position/fix/sats), RMC (speed/course)

## Quick Start

### Prerequisites

- Python 3.10+
- Arduino IDE (for firmware upload)

### 1. Install dependencies

```bash
pip install pyserial websockets
```

### 2. Flash ESP32-C3 firmware

Open `01_IMU/bno085_esp32c3/bno085_esp32c3.ino` in Arduino IDE, select ESP32-C3 board, and upload.

### 3. Run individual modules

```bash
# Terminal 1 — IMU
cd 01_IMU
python imu_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
# Browser: http://localhost:8765

# Terminal 2 — RTK
cd 02_RTK
python rtk_bridge.py --port /dev/ttyACM1 --baud 9600 --ws-port 8775
# Browser: http://localhost:8775
```

### 4. Run integrated Nav dashboard

```bash
# Terminal 3 — Nav (requires IMU + RTK bridges running)
cd 03_Nav
python nav_bridge.py --nav-port 8785 --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776
# Browser: http://localhost:8785
```

### 5. Run Amiga robot controller

```bash
# Terminal 4 — Robot (requires CIRCUITPY Feather M4 CAN connected)
cd 04_Robot
python robot_bridge.py --port /dev/ttyACM1 --baud 115200 --ws-port 8795
# Browser: http://localhost:8795
```

### 6. Run autonomous navigation

```bash
# Requires IMU + RTK + Robot bridges running
pip install numpy
cd 05_AutoNav
python autonav_bridge.py \
  --imu-ws ws://localhost:8766 --rtk-ws ws://localhost:8776 \
  --robot-ws ws://localhost:8796
# Browser: http://localhost:8805 → Upload waypoint CSV → Start Nav
```

### 7. Run camera streaming

```bash
# Requires OAK-D camera connected
pip install depthai opencv-python numpy
cd 06_Camera
python camera_bridge.py --cam1-ip 10.95.76.11
# Browser: http://localhost:8815 (control panel)
# MJPEG:   http://localhost:8080 (direct video stream)
```

### 8. Run data recorder

```bash
# Requires IMU + RTK + Robot bridges running
cd 07_Recorder
python recorder_bridge.py
# Browser: http://localhost:8825 → Start Recording → Download CSV
```

## Module Details

### 01_IMU — IMU Bridge

- **Data flow**: `Serial → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → Browser`
- **Pipeline stages**: `_parse → _enrich_euler → _enrich_hz → _serialize`
- **Features**: real-time 3D orientation (Three.js), compass HUD, north offset calibration, lock-yaw mode, top-north view (top-down with north-up), 11 sensor data cards

### 02_RTK — RTK Bridge

- **Data flow**: `Serial → SerialReader → NMEAPipeline → BroadcastLoop → WebSocketServer → Browser`
- **Pipeline stages**: `_verify_checksum → _dispatch → _parse_gga / _parse_rmc`
- **Features**: Leaflet map (satellite/OSM/offline tiles), waypoint management, CSV import/export, route editing, path simulation, track recording, i18n (EN/ZH)

### 03_Nav — Navigation Dashboard

- **Data flow**: `imu_bridge(WS) + rtk_bridge(WS) → NavLoop(10 Hz) → NavController → NavWebSocketServer → Browser`
- **Layout**: upper 3D view (40%) + lower map (60%) | right data panel (320 px)
- **Features**: all IMU + RTK features in a single page, unified WebSocket, heading computation, waypoint reach detection, north offset forwarding, top-north view

### 04_Robot — Amiga Robot Controller

- **Data flow**: `Serial ↔ SerialReader → RobotPipeline → asyncio.Queue → WebSocketServer → Browser`
- **Pipeline stages**: `_parse → _enrich_state → _enrich_hz → _enrich_odometry → _serialize`
- **Serial protocol**: `O:{speed},{ang_rate},{state},{soc}` telemetry (~20 Hz), `S:READY`/`S:ACTIVE` status; accepts WASD single-char and `V{speed},{ang_rate}\n` commands
- **Features**: WASD keyboard/button control, velocity sliders, E-Stop, state toggle, battery SOC bar, speed/angular rate visualizers, odometry (heading + distance), Three.js top-down robot view

### 05_AutoNav — Autonomous Navigation Engine

- **Data flow**: `imu_bridge(WS) + rtk_bridge(WS) + robot_bridge(WS) → AutoNavPipeline → velocity commands → robot_bridge(WS)`
- **Components**: GeoUtils, MovingAverageFilter, KalmanFilter (4D position+velocity), PIDController, P2PController, PurePursuitController, WaypointManager, CoveragePlanner (Boustrophedon)
- **State machine**: `IDLE → NAVIGATING → FINISHED`
- **Nav modes**: P2P (point-to-point bearing control) / Pure Pursuit (lookahead path tracking)
- **Filter modes**: Moving Average (sliding window GPS) / Kalman (4D with IMU acceleration + odometry velocity)
- **Features**: CSV waypoint upload, adaptive arrival tolerance (RTK quality-based), GPS timeout detection, coverage path generation (lawnmower pattern), Leaflet map UI

### 06_Camera — OAK-D Camera MJPEG Streaming

- **Data flow**: `OAK-D Camera → FrameSource → MJPEGServer (HTTP multipart) → Browser <img>`
- **Components**: FrameSource (ABC), SimpleColorSource (depthai v3), MJPEGServer, CameraPipeline
- **Features**: dual camera support (cam1/cam2 on separate MJPEG ports), camera switching, start/stop control, FPS tracking, WS status broadcast (1 Hz)
- **Note**: Video via HTTP MJPEG, WebSocket only for control/status

### 07_Recorder — Multi-Source Data Recorder

- **Data flow**: `imu_bridge(WS) + rtk_bridge(WS) + robot_bridge(WS) → RecordLoop(5 Hz) → DataRecorder → CSV`
- **Components**: DataRecorder (thread-safe CSV writer), RecorderPipeline, three WS clients (read-only)
- **CSV columns**: timestamp, quaternion (i/j/k/r), euler (yaw/pitch/roll), GPS (lat/lon/alt/fix/sats/hdop/speed/track), robot (speed/ang_rate/state/soc/distance/heading)
- **Features**: start/stop recording, file list with download/delete, data source connection indicators, dual HTTP paths (static UI + CSV file serving)

## Code Conventions

All code follows the conventions defined in `CLAUDE.md`:

- OOP + Pipeline pattern
- `@dataclass` for data models
- `INPUT / CORE / OUTPUT` banner annotations at I/O boundaries
- English-only code comments, logs, and CLI output
- `logging` module with `.log` file output

## License

Internal project — not published.
