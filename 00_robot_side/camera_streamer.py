"""
MJPEG-over-HTTP camera streaming server for the robot side.

Exposes one MJPEG stream per configured camera via HTTP.
Any FrameSource implementation can be swapped in — the server itself
never changes.

Usage (standalone):
    export CAM1_IP=10.95.76.10        # optional, for OAK-D PoE
    export CAM_SELECTION=1            # "1" (default), "2", or "both"
    export LOCAL_DISPLAY=1            # show local preview window
    cd m2_system/00_robot_side
    python camera_streamer.py

Endpoints (default ports):
    http://localhost:8080/   ->  CAM1 MJPEG stream  (CAM_SELECTION=1 or both)
    http://localhost:8080/   ->  CAM2 MJPEG stream  (CAM_SELECTION=2, reuses port 8080)
    http://localhost:8081/   ->  CAM2 MJPEG stream  (CAM_SELECTION=both)

To swap the pipeline (no other code changes needed):
    from frame_source import YOLODetectionSource
    server = MJPEGServer(source=YOLODetectionSource(), port=8080)
    server.start()
"""

import logging
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import cv2

from config import (
    CAM1_IP,
    CAM1_STREAM_PORT,
    CAM2_IP,
    CAM2_STREAM_PORT,
    LOCAL_DISPLAY,
    MJPEG_QUALITY,
)
from frame_source import FrameSource, SimpleColorSource

# ── Logging ────────────────────────────────────────────────────────────────
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


# ── MJPEGServer ────────────────────────────────────────────────────────────

class MJPEGServer:
    """MJPEG-over-HTTP server that wraps a FrameSource.

    Always serves the *latest* available frame (no frame queue buildup).
    The HTTP server and the capture loop each run in their own daemon threads.

    Args:
        source:  Any FrameSource implementation.
        port:    TCP port to listen on.
        quality: JPEG encoding quality (1–100).
    """

    def __init__(
        self,
        source: FrameSource,
        port: int,
        quality: int = MJPEG_QUALITY,
    ) -> None:
        self._source = source
        self._port = port
        self._quality = quality

        self._latest_jpeg: Optional[bytes] = None
        self._frame_lock = threading.Lock()

        self._http_server: Optional[ThreadingHTTPServer] = None
        self._server_thread: Optional[threading.Thread] = None
        self._capture_thread: Optional[threading.Thread] = None
        self._running = False

    # ── Public API ──────────────────────────────────────────────────────────

    def start(self) -> None:
        """Open the FrameSource and start the capture + HTTP threads."""
        self._source.open()
        self._running = True

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"capture-{self._port}",
        )
        self._capture_thread.start()

        # Build handler class with a reference back to this MJPEGServer.
        server_ref = self

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                server_ref._handle_http(self)

            def log_message(self, fmt, *args):  # suppress default access log
                pass

        self._http_server = ThreadingHTTPServer(("0.0.0.0", self._port), _Handler)
        self._server_thread = threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
            name=f"mjpeg-{self._port}",
        )
        self._server_thread.start()
        logger.info(f"MJPEG server started → http://localhost:{self._port}/")

    def stop(self) -> None:
        """Stop the HTTP server and release the FrameSource."""
        self._running = False
        if self._http_server:
            try:
                self._http_server.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down HTTP server (port {self._port}): {e}")
        try:
            self._source.close()
        except Exception as e:
            logger.warning(f"Error closing frame source (port {self._port}): {e}")
        logger.info(f"MJPEG server stopped (port {self._port})")

    def get_latest_frame(self):
        """Return the latest BGR numpy frame (for local display). May be None."""
        # Re-decode from JPEG is wasteful; grab directly from capture loop instead.
        # Subclasses may override this if they cache the raw frame.
        return None  # local preview fetches directly from source

    # ── Internal ────────────────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        logger.info(f"Capture loop running (port {self._port})")
        while self._running:
            frame = self._source.get_frame()
            if frame is not None:
                ok, buf = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality]
                )
                if ok:
                    with self._frame_lock:
                        self._latest_jpeg = buf.tobytes()
            else:
                time.sleep(0.001)

    def _handle_http(self, handler: BaseHTTPRequestHandler) -> None:
        """Serve a single MJPEG streaming response."""
        if handler.path not in ("/", "/stream"):
            handler.send_error(404, "Not Found")
            return

        handler.send_response(200)
        handler.send_header(
            "Content-Type", "multipart/x-mixed-replace; boundary=frame"
        )
        handler.send_header("Cache-Control", "no-cache")
        handler.end_headers()

        try:
            while self._running:
                with self._frame_lock:
                    jpeg = self._latest_jpeg

                if jpeg is None:
                    time.sleep(0.01)
                    continue

                try:
                    handler.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
                    handler.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break  # client disconnected — normal, not an error

                time.sleep(0.001)  # yield CPU; frame rate is limited by capture loop

        except Exception as e:
            logger.warning(f"Streaming error (port {self._port}): {e}")


# ── Standalone entry point ─────────────────────────────────────────────────

def main() -> None:
    cam_sel = os.environ.get("CAM_SELECTION", "1")  # "1", "2", or "both"

    servers: list[tuple[MJPEGServer, SimpleColorSource]] = []
    src_for_display: Optional[SimpleColorSource] = None
    active_ports: list[int] = []

    if cam_sel in ("1", "both"):
        src1 = SimpleColorSource(device_ip=CAM1_IP)
        srv1 = MJPEGServer(source=src1, port=CAM1_STREAM_PORT)
        servers.append((srv1, src1))
        src_for_display = src1
        active_ports.append(CAM1_STREAM_PORT)

    if cam_sel in ("2", "both"):
        # "2" only → 复用 CAM1_STREAM_PORT 作为唯一端口
        port2 = CAM2_STREAM_PORT if cam_sel == "both" else CAM1_STREAM_PORT
        src2 = SimpleColorSource(device_ip=CAM2_IP)
        srv2 = MJPEGServer(source=src2, port=port2)
        servers.append((srv2, src2))
        if src_for_display is None:
            src_for_display = src2
        active_ports.append(port2)

    if not servers:
        logger.error(f"Invalid CAM_SELECTION value: '{cam_sel}', expected '1', '2', or 'both'")
        sys.exit(1)

    # Start all servers
    for srv, _ in servers:
        try:
            srv.start()
        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            for s, _ in servers:
                s.stop()
            sys.exit(1)

    def _shutdown(signum=None, frame=None) -> None:
        logger.info("Shutdown signal received, stopping servers...")
        for s, _ in servers:
            s.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    ports_str = ", ".join(str(p) for p in active_ports)
    logger.info(f"Streaming on port(s): {ports_str} — press Ctrl+C to stop")

    # Optional local preview (macOS: must be on main thread)
    if LOCAL_DISPLAY:
        logger.info("Local display enabled (LOCAL_DISPLAY=1)")
        while True:
            frame = src_for_display.get_frame()
            if frame is not None:
                cv2.imshow("Camera local preview", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                logger.info("Local display: 'q' pressed, stopping")
                _shutdown()
    else:
        # Just keep the main thread alive
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            _shutdown()


if __name__ == "__main__":
    main()
