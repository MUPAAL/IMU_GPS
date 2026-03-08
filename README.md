# IMU + RTK Navigation System

A real-time sensor fusion platform for farm robots, combining a BNO085 IMU with an RTK-GPS receiver. Three independent modules — IMU visualization, RTK mapping, and an integrated Nav dashboard — communicate via WebSocket bridges, all viewable in a browser.

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
```

Each bridge also serves its own static web UI over HTTP:

| Module | HTTP | WebSocket | Description |
|--------|------|-----------|-------------|
| `01_IMU` | 8765 | 8766 | 3D IMU orientation + sensor data cards |
| `02_RTK` | 8775 | 8776 | Leaflet map + waypoint management |
| `03_Nav` | 8785 | 8786 | Integrated dashboard (3D + map + all panels) |

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

## Module Details

### 01_IMU — IMU Bridge

- **Data flow**: `Serial → SerialReader → IMUPipeline → asyncio.Queue → WebSocketServer → Browser`
- **Pipeline stages**: `_parse → _enrich_euler → _enrich_hz → _serialize`
- **Features**: real-time 3D orientation (Three.js), compass HUD, north offset calibration, lock-yaw mode, 11 sensor data cards

### 02_RTK — RTK Bridge

- **Data flow**: `Serial → SerialReader → NMEAPipeline → BroadcastLoop → WebSocketServer → Browser`
- **Pipeline stages**: `_verify_checksum → _dispatch → _parse_gga / _parse_rmc`
- **Features**: Leaflet map (satellite/OSM/offline tiles), waypoint management, CSV import/export, route editing, path simulation, track recording, i18n (EN/ZH)

### 03_Nav — Navigation Dashboard

- **Data flow**: `imu_bridge(WS) + rtk_bridge(WS) → NavLoop(10 Hz) → NavController → NavWebSocketServer → Browser`
- **Layout**: upper 3D view (40%) + lower map (60%) | right data panel (320 px)
- **Features**: all IMU + RTK features in a single page, unified WebSocket, heading computation, waypoint reach detection, north offset forwarding

## Code Conventions

All code follows the conventions defined in `CLAUDE.md`:

- OOP + Pipeline pattern
- `@dataclass` for data models
- `INPUT / CORE / OUTPUT` banner annotations at I/O boundaries
- English-only code comments, logs, and CLI output
- `logging` module with `.log` file output

## License

Internal project — not published.
