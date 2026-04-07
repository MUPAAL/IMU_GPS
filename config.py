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
IMU_SERIAL_PORT   = "/dev/cu.usbmodem101"   # Serial device (Mac: cu.usbmodem*, Linux: /dev/ttyACM0)
IMU_BAUD          = 921600                   # Serial baud rate
IMU_WS_PORT       = 8765                     # HTTP port; WebSocket = IMU_WS_PORT + 1
IMU_NORTH_OFFSET  = 0.0                      # North heading offset (degrees) for yaw calibration


# ══════════════════════════════════════════════════════════════════════════════
# 02_RTK  — RTK GPS serial → WebSocket bridge
# ══════════════════════════════════════════════════════════════════════════════

RTK_SERIAL_PORT   = "/dev/cu.usbmodem11203"  # RTK receiver serial port (Linux: /dev/ttyACM1)
RTK_SERIAL_PORTS  = ["/dev/cu.usbmodem11203", "/dev/cu.usbmodem103"]  # Add a second port here when using two RTK receivers
RTK_BAUD          = 9600                     # Serial baud rate (NMEA standard)
RTK_WS_PORT       = 8775                     # HTTP port; WebSocket = RTK_WS_PORT + 1
RTK_HZ            = 5.0                      # Broadcast rate (Hz)
RTK_DEFAULT_LAT   = 38.9412928598587         # Fallback latitude when no GPS fix
RTK_DEFAULT_LON   = -92.31884600793728       # Fallback longitude when no GPS fix
RTK_HEADING_SOURCE_A = "rtk1"                # Baseline start source for dual-RTK heading
RTK_HEADING_SOURCE_B = "rtk2"                # Baseline end source for dual-RTK heading
RTK_HEADING_OFFSET_DEG = 0.0                 # Mounting yaw offset added to computed heading
RTK_HEADING_MIN_BASELINE_M = 0.3             # Reject heading if baseline is shorter than this


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
ROBOT_SERIAL_PORT      = "/dev/cu.usbmodem11301"   # Feather M4 serial port (Linux: /dev/ttyACM0)
ROBOT_SERIAL_BAUD      = 115200                    # Serial baud rate
ROBOT_SERIAL_TIMEOUT   = 1.0                       # Serial read timeout (seconds)
ROBOT_MAX_LINEAR       = 1.0                       # Max linear velocity (m/s)
ROBOT_MAX_ANGULAR      = 1.0                       # Max angular velocity (rad/s)
ROBOT_WATCHDOG_TIMEOUT = 2.0                       # Watchdog timeout (seconds); triggers e-stop
ROBOT_NAV_WS           = "ws://localhost:8786"     # nav_bridge WebSocket URL


# ══════════════════════════════════════════════════════════════════════════════
# 05_AutoNav  — Autonomous navigation (PID + Pure Pursuit)
# ══════════════════════════════════════════════════════════════════════════════

AUTONAV_WS_PORT         = 8805                      # HTTP port; WebSocket = AUTONAV_WS_PORT + 1
AUTONAV_IMU_WS          = "ws://localhost:8766"     # imu_bridge WebSocket URL
AUTONAV_RTK_WS          = "ws://localhost:8776"     # rtk_bridge WebSocket URL
AUTONAV_ROBOT_WS        = "ws://localhost:8796"     # robot_bridge WebSocket URL
AUTONAV_MAX_LINEAR_VEL  = 1.0                       # Max linear velocity (m/s)
AUTONAV_MAX_ANGULAR_VEL = 1.0                       # Max angular velocity (rad/s)
AUTONAV_PID_KP          = 0.8                       # PID proportional gain
AUTONAV_PID_KI          = 0.01                      # PID integral gain
AUTONAV_PID_KD          = 0.05                      # PID derivative gain
AUTONAV_LOOKAHEAD_M     = 2.0                       # Pure Pursuit lookahead distance (meters)
AUTONAV_DECEL_RADIUS_M  = 3.0                       # Deceleration radius before waypoint (meters)
AUTONAV_ARRIVE_FRAMES   = 5                         # Consecutive frames required to confirm arrival
AUTONAV_GPS_TIMEOUT_S   = 5.0                       # GPS data timeout (seconds)
AUTONAV_MA_WINDOW       = 10                        # Moving average filter window size


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
