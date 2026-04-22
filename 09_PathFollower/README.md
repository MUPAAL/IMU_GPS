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

# Point-to-Point (heading-based) PID gains
PATHFOLLOWER_PID_KP = 0.5                # Proportional gain
PATHFOLLOWER_PID_KI = 0.05               # Integral gain
PATHFOLLOWER_PID_KD = 0.1                # Derivative gain

# Pure Pursuit (waypoint-based) PID gains
PATHFOLLOWER_NAV_PID_KP = 0.8            # More aggressive steering
PATHFOLLOWER_NAV_PID_KI = 0.01
PATHFOLLOWER_NAV_PID_KD = 0.05
PATHFOLLOWER_NAV_LOOKAHEAD_M = 2.0       # Lookahead distance (m)
PATHFOLLOWER_NAV_DECEL_RADIUS_M = 3.0    # Deceleration radius (m)
```

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

You'll see:
- **System Status** — Current mode, watchdog state, velocity outputs
- **IMU Orientation** — Heading (with compass direction), roll, pitch, yaw, frame rate
- **RTK Position** — Latitude, longitude, altitude, satellite count, fix quality
- **Control Panel** — Virtual joystick, heading target input, and waypoint management (Pure Pursuit)

## Operating the Controller

### Joystick Mode (Default)
1. Move your mouse/finger on the joystick canvas
2. Distance from center controls linear velocity (forward/backward)
3. Angle controls angular velocity (left/right turn)
4. Release to stop

**Velocity Output**: Sent 20 times per second to Feather M4 whenever you interact with the joystick.

### Point-to-Point (P2P) Heading Follow Mode
1. Enter a target heading in degrees (0–359, where 0 = North)
2. Click "Set Heading"
3. The robot will autonomously maintain that bearing using IMU feedback
4. Current and target heading displayed in real-time

**How It Works**: Uses PID control on heading error (target - current heading) to compute angular velocity. Linear velocity can be set from last joystick input or fixed value.

**When to Use**: Simple heading-based navigation when GPS is unavailable or for precise bearing control.

### Pure Pursuit Waypoint Mode
1. Prepare a waypoint list (CSV format: `lat,lon,tolerance_m,max_speed`)
2. Upload waypoints via the web UI
3. Robot automatically loads waypoints and displays current target
4. Each waypoint reached triggers automatic advancement to next

**How It Works**: 
- Projects robot position onto current path segment [previous → current waypoint]
- Computes lookahead point 2.0 m ahead along the path
- Steers toward lookahead point for smooth, natural trajectories
- Decelerates as it approaches each waypoint
- Seamlessly handles multi-waypoint paths without the robot oscillating

**When to Use**: GPS-based autonomous navigation with smooth, efficient paths through multiple waypoints.

### Switching Modes
- **Joystick** ↔ **P2P**: Move joystick or set heading target (modes switch automatically)
- **Any mode** → **Pure Pursuit**: Upload waypoints (this mode is explicit)
- **Pure Pursuit** → **P2P**: Stop Pure Pursuit mode or disconnect waypoints (fallback on GPS loss)

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
- Ensure IMU is connected and sensor data arrives (check IMU panel in web UI)
- Verify P2P PID gains in `config.py` (Kp=0.5 is conservative; increase for faster response)
- Check heading error is updating in status display
- If watchdog is timing out, increase WATCHDOG_TIMEOUT or ensure heartbeat messages arrive

### Pure Pursuit Waypoints Not Loading
- Verify RTK GPS fix quality ≥ 1 (RTK Float or better) in web UI
- Check waypoint CSV format: `lat,lon,tolerance_m,max_speed` (one per line, no header)
- Ensure first waypoint is reasonably close to robot's starting position
- Check logs for path projection errors

### Pure Pursuit Robot Not Following Path
- Verify GPS is working (RTK Position should update every second)
- Increase Pure Pursuit PID gain (Kp=0.8) if steering is sluggish
- Check lookahead distance (2.0 m default) — increase for faster paths, decrease for tighter control
- Ensure deceleration radius (3.0 m) is larger than tolerance to smooth arrivals
- Check robot actually reaches within tolerance of waypoint before advancing

### Joystick Input Laggy
- Check network latency (local WebSocket should be <50ms)
- Reduce control loop rate if CPU is maxed (set CONTROL_LOOP_HZ in config.py)
- Restart browser if UI becomes unresponsive

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

**P2P PID Gains** (heading error → angular velocity):
- **Kp = 0.5** (conservative) — Start here, increase if unresponsive
- **Ki = 0.05** (slow integral) — Eliminates drift; too high causes oscillation
- **Kd = 0.1** (light damping) — Reduces overshoot; too high adds noise sensitivity

**Pure Pursuit Parameters** (waypoint following):
- **Lookahead Distance = 2.0 m** — Increase for faster, wider turns; decrease for tighter tracking
- **Deceleration Radius = 3.0 m** — Larger values create smoother approaches; should be > waypoint tolerance
- **Kp = 0.8** (aggressive) — Faster steering than P2P; adjust if overshooting path
- **Ki = 0.01, Kd = 0.05** — Conservative integral/derivative to avoid oscillation on path

For field tuning: Start with default values, increase Kp if robot lags path, decrease if it oscillates. If GPS is noisy, increase lookahead distance.

## Module Dependencies

**Python Packages**:
- `pyserial>=3.5` — Serial port I/O
- `websockets>=12.0` — WebSocket server

**Imported Modules** (code reuse):
- `00_robot_side/navigation/controller.py` — `PIDController`
- `00_robot_side/navigation/geo_utils.py` — `normalize_angle`, `bearing_to_target`, `haversine_distance`, `project_point_on_segment`
- `01_IMU/imu_bridge.py` — `IMUPipeline`, `FrameRateTracker`
- `02_RTK/rtk_bridge.py` — `NMEAPipeline`

All are imported dynamically to avoid duplicate code.

## File Structure

```
09_PathFollower/
├── main.py                    # Entry point
├── web_controller.py          # HTTP + WebSocket server, control loop
├── heading_controller.py      # PID heading tracker
├── sensor_threads.py          # IMU and RTK reader threads
├── requirements.txt           # Dependencies
├── README.md                  # This file
├── CLAUDE.md                  # Architecture and design documentation
├── log/                       # Runtime logs (created on first run)
└── web_static/
    ├── index.html            # Web UI layout
    ├── style.css             # Dark theme styling
    └── visualizer.js         # WebSocket client and joystick input
```

## Performance Notes

- **Control loop**: 20 Hz (50ms cycle)
- **IMU broadcast**: 20 Hz (from sensor)
- **RTK broadcast**: 1 Hz (from sensor)
- **Status broadcast**: 2 Hz (computed)
- **Joystick input**: Variable (on user interaction)
- **Heartbeat**: 1 Hz (automatic from web UI)

For best performance, keep network latency <50ms and ensure serial devices are communicating at specified baud rates.
