"""
main.py — Entry point for PathFollower module.

Launches the web controller which manages IMU + RTK sensors, heading control, and
joystick-based path following.

Usage:
    python main.py
    # Open browser at http://localhost:8890
"""

import asyncio
import logging
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import config
from web_controller import main as run_web_controller


def setup_logging():
    """Configure logging to file and console."""
    log_dir = config.LOG_DIR if hasattr(config, "LOG_DIR") else Path(__file__).parent / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "pathfollower.log"

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(log_file, encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )

    logger = logging.getLogger(__name__)
    logger.info("=" * 70)
    logger.info("PathFollower Module Starting")
    logger.info("=" * 70)
    logger.info(f"IMU Port: {config.IMU_SERIAL_PORT} @ {config.IMU_BAUD} baud")
    logger.info(f"RTK Port: {config.RTK_SERIAL_PORT} @ {config.RTK_BAUD} baud")
    logger.info(f"Feather Port: {config.ROBOT_SERIAL_PORT} @ {config.ROBOT_SERIAL_BAUD} baud")
    logger.info(f"HTTP: {config.PATHFOLLOWER_WS_PORT}")
    logger.info(f"WebSocket: {config.PATHFOLLOWER_WS_PORT + 1}")
    logger.info("=" * 70)

    return logger


if __name__ == "__main__":
    logger = setup_logging()
    try:
        asyncio.run(run_web_controller())
    except KeyboardInterrupt:
        logger.info("PathFollower stopped by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
