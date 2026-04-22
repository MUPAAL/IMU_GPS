# PathFollower — Multi-Mode Autonomous Path Follower

A versatile autonomous path follower supporting point-to-point (P2P) heading tracking and pure pursuit GPS-based waypoint navigation. Real-time sensor fusion from IMU and RTK GPS with a web-based control interface.

## What It Does

PathFollower provides **three control modes** for the Amiga platform:

1. **Joystick Mode** — Direct manual control via virtual joystick (linear and angular velocity)
2. **Point-to-Point (P2P) Mode** — Autonomous heading tracking via PID controller (maintains target bearing using IMU feedback alone)
3. **Pure Pursuit Mode** — GPS-based waypoint following with lookahead steering (smooth multi-waypoint trajectories using IMU + RTK fusion)

The module runs sensor readers in background daemon threads and broadcasts live telemetry to a web browser via WebSocket. Supports graceful fallback (Pure Pursuit → P2P) when GPS unavailable.

## Data Flow

```
Sensors → Serial Readers (daemon threads) → Snapshots (thread-safe)
                                              ↓
                                          Control Loop (20 Hz)
                                              ↓
                                          Velocity Commands → Feather M4
                                              ↓
                                          Web UI (WebSocket broadcast)
```

**Input Sensors**:
- **IMU (BNO085)** — Heading, roll, pitch, yaw via ESP32-C3 serial (921600 baud)
- **RTK GPS (Emlid RS+)** — Latitude, longitude, altitude, fix quality via NMEA (9600 baud)

**Output**:
- **Velocity Commands** — Linear and angular velocity to Feather M4 motor controller via serial

## How to Start

### 1. Install Dependencies
```bash
cd 09_PathFollower
pip install -r requirements.txt
```

### 2. Verify Serial Connections
Ensure the following devices are connected and ports match your `config.py`:
- **IMU**: ESP32-C3 (default: `/dev/cu.usbmodem101` on macOS, `/dev/ttyACM0` on Linux)
- **RTK GPS**: Emlid RS+ (default: `/dev/cu.usbmodem11203`)
- **Motor Controller**: Feather M4 (default: `/dev/cu.usbmodem11301`)

Check available ports:
```bash
# macOS
ls /dev/cu.usbmodem*

# Linux
ls /dev/ttyACM*
```

### 3. Update config.py (if needed)
Edit the root `config.py` and verify the `PATHFOLLOWER_*` section:
```python
# Web and serial ports
PATHFOLLOWER_WS_PORT = 8890              # HTTP/WebSocket port
PATHFOLLOWER_FEATHER_PORT = "/dev/cu.usbmodem11301"
PATHFOLLOWER_IMU_PORT = "/dev/cu.usbmodem101"
PATHFOLLOWER_RTK_PORT = "/dev/cu.usbmodem11203"
PATHFOLLOWER_MAX_LINEAR_VEL = 0.8        # m/s
PATHFOLLOWER_MAX_ANGULAR_VEL = 1.5       # rad/s

# Point-to-Point (heading-based) control gains
PATHFOLLOWER_PID_KP = 0.5                # Proportional gain
PATHFOLLOWER_PID_KI = 0.05               # Integral gain
PATHFOLLOWER_PID_KD = 0.1                # Derivative gain

# Pure Pursuit uses shared NAV_* parameters (from section 03_Nav):
# NAV_PID_KP = 0.8, NAV_PID_KI = 0.01, NAV_PID_KD = 0.05
# NAV_LOOKAHEAD_M = 2.0, NAV_DECEL_RADIUS_M = 3.0
```

**Note**: Broadcast frequencies (control loop 20Hz, IMU 20Hz, RTK 1Hz, status 2Hz) are hardcoded in `web_controller.py` and do not require config parameters.

### 4. Run the Controller
```bash
python main.py
```

The application will:
- Start sensor reader threads (auto-reconnect on failure)
- Open the Feather M4 serial connection
- Launch HTTP server at `http://localhost:8890`
- Open WebSocket at `ws://localhost:8891`
- Begin broadcasting sensor data at configured rates

### 5. Open Web Interface
Navigate to:
```
http://localhost:8890
```

**Layout** (fullscreen, matching 00_robot_side farm robot UI):
- **Fullscreen Joystick Zone** — Touch or click to control; center-positioned virtual joystick
- **Top Bar** — Title "PATHFOLLOWER" with green/red connection status dot
- **Left HUD Panel** (top-left):
  - **Compass** — Rotating needle showing current heading, cardinal direction (N/S/E/W/etc.)
  - **Heading Control Box** — Input field for target heading (0–359°) + "Set" button + "Lock Yaw" toggle
  - **Orientation** — Roll, pitch, yaw angles (degrees)
  - **Status** — Current mode (JOYSTICK/HEADING/P2P), linear velocity, angular velocity
- **Right HUD Panel** (top-right) — RTK GPS data:
  - Latitude, longitude, altitude
  - Satellite count
  - Fix quality badge (NO FIX / GPS / RTK FIX / RTK FLOAT)
- **Warning Banner** — Red alert "⚠ WATCHDOG TIMEOUT — MOTORS STOPPED" (appears only when timeout triggers)

## Operating the Controller

### Joystick Mode (Default)
1. **Touch or click** in the center of the screen to activate the virtual joystick
2. **Drag to control**:
   - Upward (Y+) = forward linear velocity
   - Downward (Y−) = backward linear velocity  
   - Right (X+) = left turn (negative angular velocity)
   - Left (X−) = right turn (positive angular velocity)
3. **Release** to stop immediately (zeroes velocity)

**Velocity Output**: Sent at ~10 Hz (100 ms throttle) to Feather M4 during active joystick interaction. Joystick hint text ("TOUCH TO CONTROL") disappears while active, reappears on release.

### Point-to-Point (P2P) Heading Follow Mode
1. In the **Heading Control Box** (left panel, below compass), enter target heading (0–359°, where 0 = North)
2. Click **"Set"** button (cyan)
3. The robot will autonomously steer to and maintain that bearing using IMU feedback
4. Compass needle continuously shows current heading; heading error updates in real-time

**How It Works**: 
- Uses PID control on heading error (target − current heading) to compute angular velocity
- Linear velocity maintained from last joystick input (or zero if joystick never used)
- Sends velocity commands at 20 Hz until you set a different heading or move the joystick

**When to Use**: Simple bearing-based navigation when GPS is unavailable, or for precise heading control without waypoints.

### Pure Pursuit Waypoint Mode
**Status**: Backend implementation complete; **web UI waypoint upload not yet implemented**.

The backend supports Pure Pursuit mode (smooth multi-waypoint path following), but currently waypoints can only be loaded programmatically via web_controller.py. Future enhancement: add waypoint CSV upload button to left HUD panel.

**How It Works** (when waypoints are loaded):
- Projects robot position onto current path segment [previous → current waypoint]
- Computes lookahead point 2.0 m ahead along the path (configurable via `NAV_LOOKAHEAD_M`)
- Steers toward lookahead point for smooth, natural trajectories (avoids oscillation)
- Decelerates as it approaches each waypoint (smooth arrival)
- Automatically advances to next waypoint when within tolerance

**Fallback**: If Pure Pursuit encounters GPS loss or path projection error, automatically degrades to **P2P Mode** using last known bearing.

**When to Use**: GPS-based autonomous navigation with smooth, efficient paths through multiple waypoints (once UI waypoint upload is implemented).

### Switching Modes
Modes are **implicit** — determined by your last input:
- **Joystick Mode** (default) — Touch joystick → switch to joystick control
- **Heading Follow Mode** — Click "Set Heading" button → enter autonomous heading tracking (P2P)
- **Pure Pursuit Mode** (future) — Upload waypoints → switch to GPS-based path following

**Current Status Display**: Left HUD panel shows current **MODE** (JOYSTICK / HEADING / P2P)

**Fallback**: If in Heading Follow or Pure Pursuit modes and you use the joystick, mode immediately switches back to Joystick Mode.

## Output and Logging

### Console Logs
Standard Python logging to console (INFO level):
```
INFO: PathFollower starting...
INFO: Feather M4 connected on /dev/cu.usbmodem11301
INFO: HTTP server listening on :8890
INFO: WebSocket server listening on :8891
INFO: Control loop started at 20.0 Hz
```

### File Logs
Detailed logs written to:
```
09_PathFollower/log/pathfollower.log
```

Contains:
- Startup/shutdown events
- Serial connection state changes
- Watchdog timeouts
- Control mode switches
- Errors and exceptions

### Real-time Telemetry (WebSocket)
The web interface receives continuous broadcasts:

**IMU Broadcast** (20 Hz):
```json
{
  "type": "imu",
  "heading": 45.2,
  "roll": 2.1,
  "pitch": -1.3,
  "yaw": 44.8,
  "hz": 20.0
}
```

**RTK Broadcast** (1 Hz):
```json
{
  "type": "rtk",
  "lat": 38.94129,
  "lon": -92.31885,
  "alt": 250.5,
  "num_sats": 12,
  "fix_quality": 4
}
```

**Status Broadcast** (2 Hz):
```json
{
  "type": "status",
  "watchdog": false,
  "mode": "joystick",
  "linear_vel": 0.5,
  "angular_vel": 0.2,
  "heading_error": 3.5
}
```

### Motor Commands (Serial to Feather M4)
Velocity commands sent at 20 Hz (control loop rate):
```
linear_velocity (m/s) → clamped to [-0.8, 0.8]
angular_velocity (rad/s) → clamped to [-1.5, 1.5]
```

## Safety Features

### Watchdog Timeout
If no joystick/heartbeat message arrives within **2.0 seconds**:
- All velocity commands are zeroed (e-stop)
- Watchdog alert indicator appears in web UI (red, pulsing)
- Resume control by sending a joystick or heartbeat command

The web interface automatically sends heartbeat every 1 second.

### Velocity Limits
All outputs are clamped to safe ranges:
- Linear: 0–0.8 m/s
- Angular: 0–1.5 rad/s

## Troubleshooting

### "Cannot open serial port" Error
- Verify USB cables are connected
- Check port names match your system:
  ```bash
  # macOS
  ls /dev/cu.usbmodem*
  # Linux
  ls /dev/ttyACM*
  ```
- Update port names in root `config.py`
- Restart the application

### Sensor Data Not Appearing
- Check logs:
  ```bash
  tail -f log/pathfollower.log
  ```
- Verify serial devices are transmitting (baud rates: 921600 for IMU, 9600 for RTK)
- Confirm IMU and RTK are powered and connected

### P2P Heading Control Not Responding
- Ensure IMU is connected and sensor data arrives (check compass needle rotating in left HUD)
- Verify P2P PID gains in `config.py` (Kp=0.5 is conservative; increase for faster response)
- Check **Heading Control Box** (left HUD, below compass) — input heading and click "Set"
- Verify compass heading is updating in real-time; if not, IMU is not connected
- If watchdog warning appears (red banner), ensure browser tab is active; heartbeat requires focus

### Pure Pursuit Waypoints Not Loading
- **Note**: Waypoint upload UI not yet implemented. Backend supports Pure Pursuit, but waypoints must be loaded programmatically.
- Verify RTK GPS fix quality in right HUD (look for "RTK FIX" or "RTK FLT" badge, green color)
- Ensure first waypoint is reasonably close to robot's starting position
- Check logs: `tail -f log/pathfollower.log` for path projection or waypoint format errors

### Pure Pursuit Robot Not Following Path
- Verify GPS is working (RTK Position should update every second)
- Increase Pure Pursuit PID gain (Kp=0.8) if steering is sluggish
- Check lookahead distance (2.0 m default) — increase for faster paths, decrease for tighter control
- Ensure deceleration radius (3.0 m) is larger than tolerance to smooth arrivals
- Check robot actually reaches within tolerance of waypoint before advancing

### Joystick Input Laggy
- Check network latency (local WebSocket should be <50ms)
- Verify browser tab is in focus (heartbeat requires focus to maintain connection)
- Restart browser if UI becomes unresponsive
- Control loop runs at fixed 20 Hz; joystick sends at ~10 Hz (100 ms throttle)

## Control Mode Comparison

| Feature | Joystick | P2P (Heading) | Pure Pursuit (Waypoints) |
|---------|----------|---------------|--------------------------|
| **Input Required** | Real-time joystick input | Single target heading | Multiple GPS waypoints |
| **Requires GPS** | No | No | Yes (RTK recommended) |
| **Smooth Trajectories** | Manual control only | Single bearing, no path | Yes, lookahead steering |
| **Speed Control** | Direct from joystick | From last joystick or fixed | Automatic (speed factor) |
| **Best for** | Teleoperation, obstacle avoidance | Simple bearing runs | Autonomous multi-point missions |
| **Sensor Data Used** | None (direct velocity) | IMU heading only | IMU + RTK GPS fusion |
| **Control Loop Frequency** | Variable (on user input) | 20 Hz | 20 Hz |

## Tuning Guide

**P2P PID Gains** (for heading-based control, `PATHFOLLOWER_PID_*`):
- **Kp = 0.5** (conservative) — Start here, increase if unresponsive
- **Ki = 0.05** (slow integral) — Eliminates drift; too high causes oscillation
- **Kd = 0.1** (light damping) — Reduces overshoot; too high adds noise sensitivity

**Pure Pursuit Parameters** (for waypoint following, shared `NAV_*` constants):
- **NAV_LOOKAHEAD_M = 2.0 m** — Increase for faster, wider turns; decrease for tighter tracking
- **NAV_DECEL_RADIUS_M = 3.0 m** — Larger values create smoother approaches; should be > waypoint tolerance
- **NAV_PID_KP = 0.8** (aggressive) — Faster steering than P2P; adjust if overshooting path
- **NAV_PID_KI = 0.01, NAV_PID_KD = 0.05** — Conservative integral/derivative to avoid oscillation on path

**Field Tuning Tips**:
- Start with defaults; increase `NAV_PID_KP` if robot lags path, decrease if it oscillates
- If GPS is noisy, increase `NAV_LOOKAHEAD_M` for smoother paths
- Broadcast frequencies (20Hz control, 1Hz RTK) are hardcoded and stable; no tuning needed

## Module Dependencies

**Python Packages**:
- `pyserial>=3.5` — Serial port I/O
- `websockets>=12.0` — WebSocket server
- `numpy` — Numeric operations (used by navigation filters)

**Local Modules** (code reuse):
- `navigation/controller.py` — Copied from `00_robot_side/navigation/controller.py`
  - `PIDController`, `P2PController`, `PurePursuitController`
- `navigation/geo_utils.py` — Geodetic utilities (bearing, distance, path projection)
- `navigation/waypoint.py` — Waypoint management
- `navigation/gps_filter.py` — GPS filtering (moving average, Kalman)
- `navigation/nav_engine.py` — Full navigation state machine (optional, not currently used)
- `01_IMU/imu_bridge.py` — `IMUPipeline`, `FrameRateTracker`
- `02_RTK/rtk_bridge.py` — `NMEAPipeline`

**Note**: The `navigation/` folder is a complete copy from `00_robot_side/navigation/` with import paths fixed to work in the PATHFOLLOWER context. It contains no modifications to logic.

All are imported dynamically to avoid duplicate code.

## File Structure

```
09_PathFollower/
├── main.py                    # Entry point
├── web_controller.py          # HTTP + WebSocket server, control loop
├── heading_controller.py      # Multi-mode controller (P2P + Pure Pursuit)
├── sensor_threads.py          # IMU and RTK reader threads
├── requirements.txt           # Dependencies
├── README.md                  # This file
├── CLAUDE.md                  # Architecture and design documentation
├── log/                       # Runtime logs (created on first run)
├── navigation/                # Navigation modules (copied from 00_robot_side)
│   ├── controller.py          # PIDController, P2PController, PurePursuitController
│   ├── geo_utils.py           # Bearing, distance, path projection utilities
│   ├── waypoint.py            # Waypoint dataclass and manager
│   ├── gps_filter.py          # Moving average and Kalman filters
│   ├── nav_engine.py          # Full navigation state machine (optional)
│   └── __init__.py
└── web_static/
    ├── index.html            # Web UI layout
    ├── style.css             # Dark theme styling
    └── visualizer.js         # WebSocket client and joystick input
```

### Navigation Folder

The `navigation/` subdirectory is a complete copy of `00_robot_side/navigation/` with all imports fixed to work in this context. It provides:
- **Heading Control** via `P2PController` and `PurePursuitController`
- **GPS Filtering** via `MovingAverageFilter` and `KalmanFilter`
- **Geodetic Utilities** for bearing and distance calculations
- **Waypoint Management** for multi-point navigation

All navigation parameters are read from the root `config.py` using `NAV_*` keys.

## Performance Notes

- **Control loop**: 20 Hz (50ms cycle)
- **IMU broadcast**: 20 Hz (from sensor)
- **RTK broadcast**: 1 Hz (from sensor)
- **Status broadcast**: 2 Hz (computed)
- **Joystick input**: Variable (on user interaction)
- **Heartbeat**: 1 Hz (automatic from web UI)

For best performance, keep network latency <50ms and ensure serial devices are communicating at specified baud rates.
