"""
config.py — Global hyperparameter configuration

All default parameters for modules 01~06 are defined here.
CLI arguments can still override these values at runtime.
Edit this file and restart the relevant module to apply changes.
"""

# ══════════════════════════════════════════════════════════════════════════════
# 01_IMU  — BNO085 serial → WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════
# ls /dev/cu.*
IMU_SERIAL_PORT   = "/dev/cu.usbmodem1201"   # Serial device (Mac: cu.usbmodem*, Linux: /dev/ttyACM0)
IMU_BAUD          = 921600                   # Serial baud rate
IMU_WS_PORT       = 8765                     # HTTP port; WebSocket = IMU_WS_PORT + 1
IMU_NORTH_OFFSET  = 0.0                      # North heading offset (degrees) for yaw calibration


# ══════════════════════════════════════════════════════════════════════════════
# 02_RTK  — RTK GPS serial → WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════

RTK_SERIAL_PORT   = "/dev/cu.usbmodem1103"  # RTK receiver serial port (Linux: /dev/ttyACM1)
RTK_BAUD          = 9600                     # Serial baud rate (NMEA standard)
RTK_WS_PORT       = 8775                     # HTTP port; WebSocket = RTK_WS_PORT + 1
RTK_HZ            = 5.0                      # Broadcast rate (Hz)
RTK_DEFAULT_LAT   = 38.9412928598587         # Fallback latitude when no GPS fix
RTK_DEFAULT_LON   = -92.31884600793728       # Fallback longitude when no GPS fix


# ══════════════════════════════════════════════════════════════════════════════
# 03_Nav  — IMU + RTK fused navigation
# ══════════════════════════════════════════════════════════════════════════════

NAV_WS_PORT           = 8785                      # HTTP port; WebSocket = NAV_WS_PORT + 1
NAV_IMU_WS            = "ws://localhost:8766"     # imu_bridge WebSocket URL
NAV_RTK_WS            = "ws://localhost:8776"     # rtk_bridge WebSocket URL
NAV_HZ                = 10.0                      # Navigation loop broadcast rate (Hz)
NAV_REACH_TOLERANCE_M = 0.5                       # Waypoint arrival tolerance (meters)


# ══════════════════════════════════════════════════════════════════════════════
# 04_Robot  — Robot serial control bridge
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_WS_PORT          = 8888                      # HTTP port; WebSocket = ROBOT_WS_PORT + 1
ROBOT_SERIAL_PORT      = "/dev/cu.usbmodem1301"   # Feather M4 serial port (Linux: /dev/ttyACM0)
ROBOT_SERIAL_BAUD      = 115200                    # Serial baud rate
ROBOT_SERIAL_TIMEOUT   = 1.0                       # Serial read timeout (seconds)
ROBOT_MAX_LINEAR       = 1.0                       # Max linear velocity (m/s)
ROBOT_MAX_ANGULAR      = 1.0                       # Max angular velocity (rad/s)
ROBOT_WATCHDOG_TIMEOUT = 2.0                       # Watchdog timeout (seconds); triggers e-stop
ROBOT_RECORD_INTERVAL  = 1.0                       # Recording sample interval (seconds); min 0.2


# ══════════════════════════════════════════════════════════════════════════════
# 05_AutoNav  — Autonomous navigation (PID + Pure Pursuit)
# ══════════════════════════════════════════════════════════════════════════════

AUTONAV_WS_PORT         = 8805                      # HTTP port; WebSocket = AUTONAV_WS_PORT + 1
AUTONAV_IMU_WS          = "ws://localhost:8766"     # imu_bridge WebSocket URL
AUTONAV_RTK_WS          = "ws://localhost:8776"     # rtk_bridge WebSocket URL
AUTONAV_ROBOT_WS        = "ws://localhost:8889"     # robot_bridge WebSocket URL
AUTONAV_GPS_TIMEOUT_S   = 5.0                       # GPS data timeout (seconds)
AUTONAV_CONTROL_HZ      = 5.0                       # Navigation control loop frequency (Hz)

# ── Path / waypoints ──────────────────────────────────────────────────────────
AUTONAV_LOOKAHEAD_M     = 1.0   # Pure Pursuit lookahead distance (m); larger = smoother path, smaller = tighter tracking
AUTONAV_REACH_TOL_M     = 0.5   # Waypoint arrival radius (m); RTK cm accuracy supports values as low as 0.3
AUTONAV_ARRIVE_FRAMES   = 1     # Consecutive frames inside arrival radius to confirm; 1 is reliable with RTK
AUTONAV_DECEL_RADIUS_M  = 1.5   # Deceleration distance before final waypoint (m); increase for heavier robots

# ── Speed ─────────────────────────────────────────────────────────────────────
AUTONAV_MAX_LINEAR_VEL  = 1.0   # Max forward speed (m/s)
AUTONAV_MIN_LINEAR_VEL  = 0.1   # Min speed during end-of-path deceleration (m/s)
AUTONAV_MAX_ANGULAR_VEL = 1.0   # Max angular velocity (rad/s)
AUTONAV_MANUAL_SPEED    = 0.4   # W/S manual straight-drive speed (m/s)

# ── Steering behaviour ────────────────────────────────────────────────────────
AUTONAV_TURN_IN_PLACE_DEG = 10.0  # Stop forward motion and rotate in place above this error (°); 0 = disabled
AUTONAV_TURN_SLOWDOWN     = True  # Scale linear speed down proportionally with heading error
AUTONAV_DEAD_ZONE_DEG     = 3.0   # Ignore heading errors smaller than this (°); prevents micro-corrections

# ── PID gains ─────────────────────────────────────────────────────────────────
AUTONAV_PID_KP          = 0.15   # Proportional gain; too high → oscillation, too low → slow response
AUTONAV_PID_KI          = 0.005  # Integral gain; corrects persistent offset, too high → wind-up oscillation
AUTONAV_PID_KD          = 0.15   # Derivative gain; damps overshoot, too high → jitter

# ── Filters ───────────────────────────────────────────────────────────────────
AUTONAV_MA_WINDOW       = 5      # GPS sliding-average window (frames); larger = smoother but more lag
AUTONAV_HEADING_ALPHA   = 0.3    # Heading low-pass coefficient (0–1); larger = faster response


# ══════════════════════════════════════════════════════════════════════════════
# 06_Camera  — OAK-D camera MJPEG streaming bridge
# ══════════════════════════════════════════════════════════════════════════════

CAM_WS_PORT        = 8815             # HTTP port; WebSocket = CAM_WS_PORT + 1
CAM1_IP            = "10.95.76.11"   # OAK-D camera 1 IP address
CAM2_IP            = "10.95.76.10"   # OAK-D camera 2 IP address
CAM1_STREAM_PORT   = 8080            # Camera 1 MJPEG stream port
CAM2_STREAM_PORT   = 8081            # Camera 2 MJPEG stream port
CAM_FPS            = 25              # Frame rate (matches cam_demo/Depth_Align.py)
CAM_WIDTH          = 640             # Frame width (pixels)
CAM_HEIGHT         = 400             # Frame height (pixels)
CAM_MJPEG_QUALITY  = 80              # MJPEG compression quality (1–100)
CAM_DEFAULT_PLUGIN = "simple_color"  # Default processing plugin
CAM_ENABLE_STEREO     = True         # Enable stereo depth (requires Left/Right/StereoDepth nodes)
CAM_ENABLE_DISPARITY  = False         # Enable raw disparity stream alongside depth (adds load; keep False unless debugging)
