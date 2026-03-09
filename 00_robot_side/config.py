"""
Robot-side configuration.
All parameters can be overridden via environment variables.

Environment variable quick reference:
  FEATHER_PORT, SERIAL_BAUD, SERIAL_TIMEOUT
  TCP_HOST, TCP_PORT
  WATCHDOG_TIMEOUT
  CAM1_IP, CAM2_IP, CAM1_STREAM_PORT, CAM2_STREAM_PORT, FRONT_CAM_IP
  CAM_FPS, CAM_WIDTH, CAM_HEIGHT, MJPEG_QUALITY, LOCAL_DISPLAY
  KEY_REPEAT_INTERVAL
  WEB_HTTP_PORT, WEB_WS_PORT
  MAX_LINEAR_VEL, MAX_ANGULAR_VEL
  RTK_PORT, RTK_BAUD, RTK_TIMEOUT, RTK_ENABLED
  DATA_LOG_DIR
  NAV_LOOKAHEAD_M, NAV_DECEL_RADIUS_M, NAV_ARRIVE_FRAMES, NAV_GPS_TIMEOUT_S
  NAV_PID_KP, NAV_PID_KI, NAV_PID_KD, NAV_MA_WINDOW
  COMPASS_MIN_ACCURACY, COMPASS_OFFSET_DEG
  IMU_SOURCE, ESP32_IMU_PORT, ESP32_IMU_BAUD
"""

import os
import platform

# ═══════════════════════════════════════════════════════
# Serial (Feather M4 CAN connection)
# ═══════════════════════════════════════════════════════
def _default_serial_port() -> str:
    """Return the default serial port path based on the current OS."""
    system = platform.system()
    if system == "Darwin":   # macOS
        return "/dev/cu.usbmodem2301"
    else:                    # Linux
        return "/dev/ttyACM0"

FEATHER_PORT: str   = os.environ.get("FEATHER_PORT", _default_serial_port())
SERIAL_BAUD: int    = int(os.environ.get("SERIAL_BAUD",    "115200"))
SERIAL_TIMEOUT: float = float(os.environ.get("SERIAL_TIMEOUT", "1.0"))

# Allowed command characters (serial whitelist for legacy WASD mode)
ALLOWED_COMMANDS: set = {"w", "s", "a", "d", " ", "\r"}
HEARTBEAT_CHAR: str   = "H"

# ═══════════════════════════════════════════════════════
# Network (TCP remote control server used by remote_sender.py)
# ═══════════════════════════════════════════════════════
TCP_HOST: str = os.environ.get("TCP_HOST", "0.0.0.0")   # listen on all interfaces
TCP_PORT: int = int(os.environ.get("TCP_PORT", "9000"))

# ═══════════════════════════════════════════════════════
# Watchdog
# ═══════════════════════════════════════════════════════
WATCHDOG_TIMEOUT: float = float(os.environ.get("WATCHDOG_TIMEOUT", "2.0"))

# ═══════════════════════════════════════════════════════
# Cameras (OAK-D PoE + MJPEG streams)
# ═══════════════════════════════════════════════════════
CAM1_IP: str = os.environ.get("CAM1_IP", "10.95.76.11")
CAM2_IP: str = os.environ.get("CAM2_IP", "10.95.76.10")

# Front camera IP, always assigned to CAM1_STREAM_PORT (8080).
# Used to distinguish front/rear when CAM_SELECTION=both; defaults to CAM1_IP.
FRONT_CAM_IP: str = os.environ.get("FRONT_CAM_IP", CAM1_IP)

CAM1_STREAM_PORT: int = int(os.environ.get("CAM1_STREAM_PORT", "8080"))
CAM2_STREAM_PORT: int = int(os.environ.get("CAM2_STREAM_PORT", "8081"))

CAM_FPS: int       = int(os.environ.get("CAM_FPS",       "30"))
CAM_WIDTH: int     = int(os.environ.get("CAM_WIDTH",     "1280"))
CAM_HEIGHT: int    = int(os.environ.get("CAM_HEIGHT",    "720"))
MJPEG_QUALITY: int = int(os.environ.get("MJPEG_QUALITY", "80"))   # 1-100
LOCAL_DISPLAY: bool = os.environ.get("LOCAL_DISPLAY", "0") == "1"

# ═══════════════════════════════════════════════════════
# Web joystick controller (HTTP + WebSocket)
# ═══════════════════════════════════════════════════════
WEB_HTTP_PORT: int = int(os.environ.get("WEB_HTTP_PORT", "8888"))
WEB_WS_PORT:   int = int(os.environ.get("WEB_WS_PORT",   "8889"))

MAX_LINEAR_VEL:  float = float(os.environ.get("MAX_LINEAR_VEL",  "1.0"))  # m/s
MAX_ANGULAR_VEL: float = float(os.environ.get("MAX_ANGULAR_VEL", "1.0"))  # rad/s

# ═══════════════════════════════════════════════════════
# Sensors: RTK GPS (Emlid RS+)
# ═══════════════════════════════════════════════════════
RTK_PORT:    str   = os.environ.get("RTK_PORT",    "/dev/cu.usbmodem2403")
RTK_BAUD:    int   = int(os.environ.get("RTK_BAUD",    "9600"))
RTK_TIMEOUT: float = float(os.environ.get("RTK_TIMEOUT", "1.0"))
RTK_ENABLED: bool  = os.environ.get("RTK_ENABLED", "1") == "1"

# ═══════════════════════════════════════════════════════
# Data recording
# ═══════════════════════════════════════════════════════
DATA_LOG_DIR: str = os.environ.get("DATA_LOG_DIR", "data_log")

# ═══════════════════════════════════════════════════════
# Navigation (path tracking / GPS filtering)
# ═══════════════════════════════════════════════════════
KEY_REPEAT_INTERVAL: float = float(os.environ.get("KEY_REPEAT_INTERVAL", "0.1"))  # Hz: 1/0.1 = 10 Hz

NAV_LOOKAHEAD_M:    float = float(os.environ.get("NAV_LOOKAHEAD_M",    "2.0"))   # Pure Pursuit lookahead
NAV_DECEL_RADIUS_M: float = float(os.environ.get("NAV_DECEL_RADIUS_M", "3.0"))   # Deceleration radius
NAV_ARRIVE_FRAMES:  int   = int(os.environ.get("NAV_ARRIVE_FRAMES",    "5"))     # Consecutive-frame arrival check
NAV_GPS_TIMEOUT_S:  float = float(os.environ.get("NAV_GPS_TIMEOUT_S",  "5.0"))   # Stop on GPS timeout
NAV_PID_KP:         float = float(os.environ.get("NAV_PID_KP",         "0.8"))
NAV_PID_KI:         float = float(os.environ.get("NAV_PID_KI",         "0.01"))
NAV_PID_KD:         float = float(os.environ.get("NAV_PID_KD",         "0.05"))
NAV_MA_WINDOW:      int   = int(os.environ.get("NAV_MA_WINDOW",        "10"))    # Moving-average window size

# ═══════════════════════════════════════════════════════
# Sensors: compass calibration control
# ═══════════════════════════════════════════════════════
# COMPASS_MIN_ACCURACY: BNO085 magnetometer accuracy threshold (0-3).
#   Default is 2 (calibrated). If figure-eight calibration is not feasible,
#   set to 0 to accept uncalibrated data.
# COMPASS_OFFSET_DEG: Static offset compensation (degrees).
#   Point the robot north, read current bearing, then set this to -bearing
#   for manual alignment.
COMPASS_MIN_ACCURACY: int   = int(os.environ.get("COMPASS_MIN_ACCURACY",   "2"))
COMPASS_OFFSET_DEG:   float = float(os.environ.get("COMPASS_OFFSET_DEG",   "0.0"))

# ═══════════════════════════════════════════════════════
# Sensors: IMU source selection
# ═══════════════════════════════════════════════════════
# IMU_SOURCE: "esp32" uses ESP32+BNO085 over serial UART (recommended for field use);
#             "oakd"  uses OAK-D depthai BNO085 (legacy, requires depthai installed).
# ESP32_IMU_PORT: serial port of the ESP32 IMU board.
# ESP32_IMU_BAUD: baud rate (default 115200, matches ESP32 firmware).
IMU_SOURCE:     str   = os.environ.get("IMU_SOURCE",     "esp32")
ESP32_IMU_PORT: str   = os.environ.get("ESP32_IMU_PORT", "/dev/ttyUSB0")
ESP32_IMU_BAUD: int   = int(os.environ.get("ESP32_IMU_BAUD", "115200"))
