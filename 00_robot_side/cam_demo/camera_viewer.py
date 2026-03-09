"""
Robot-side local camera viewer.
Displays 300×300 video from OAK-D CAM_A via cv2.imshow.

Usage:
    cd m2_system/00_robot_side
    python camera_viewer.py

Controls (in OpenCV window):
    q - quit
"""

import logging
import os
import signal
import sys
from pathlib import Path

import cv2
import depthai as dai

_DEVICE_IP = os.environ.get("DEVICE_IP", None)

# ── Logging configuration ──────────────────────────────────
_py_name = Path(__file__).stem
Path("log").mkdir(exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(f"log/{_py_name}.log", encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

_stop = False


def _signal_handler(signum, frame):
    global _stop
    logger.info(f"Signal {signum} received, stopping...")
    _stop = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def main():
    global _stop
    logger.info("Starting camera viewer (300×300, CAM_A)")
    try:
        if _DEVICE_IP:
            logger.info(f"Connecting to device IP: {_DEVICE_IP}")
            _device = dai.Device(dai.DeviceInfo(_DEVICE_IP))
            pipeline_cm = dai.Pipeline(_device)
        else:
            pipeline_cm = dai.Pipeline()
        with pipeline_cm as pipeline:
            cam = pipeline.create(dai.node.Camera).build()
            q = cam.requestOutput((300, 300)).createOutputQueue()
            pipeline.start()
            while pipeline.isRunning() and not _stop:
                frame = q.get()
                if frame is not None:
                    cv2.imshow("Camera 300x300 (press q to quit)", frame.getCvFrame())
                if cv2.waitKey(1) == ord("q"):
                    break
    except Exception as e:
        logger.error(f"Camera viewer error: {e}")
        raise
    finally:
        cv2.destroyAllWindows()
        logger.info("Camera viewer stopped")


if __name__ == "__main__":
    main()
