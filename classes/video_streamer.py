###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the VideoStreamer and StreamReceiver classes, which handle the TCP
#         video streaming between the Jetson Orin NX and the Ground Station.
###############################################################################

import socket
import struct
import threading
import queue
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from io import BytesIO

import cv2
import numpy as np

from classes.config import VideoLinkConfig, MjpegServerConfig

# Constants for the TCP video streaming protocol
_PAYLOAD_SIZE = struct.calcsize(">L")  # 4-byte unsigned long

# Jetson-side TCP client: connects out to the ground station and sends frames
class VideoStreamer:
    # Initializes the VideoStreamer with the given video link configuration
    def __init__(self, video_link_config: VideoLinkConfig):
        self._config = video_link_config
        self._socket = None
        self._frame_queue = queue.Queue(maxsize=2)
        self._stop_event = threading.Event()
        self._thread = None

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    # Connects to the ground station using the configured host and port and starts the background thread
    def connect(self) -> None:
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(0.5)  # Add timeout to avoid blocking main loop
            self._socket.connect((self._config.host, self._config.port))
            self._socket.settimeout(None) # Reset timeout for normal operation
            print(f"Connected video stream to {self._config.host}:{self._config.port}")
        except Exception as e:
            print(f"Warning: Could not connect video stream: {e}")
            if self._socket:
                self._socket.close()
            self._socket = None

        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._stream_worker, daemon=True)
            self._thread.start()

    def disconnect(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._socket is not None:
            try:
                self._socket.close()
            except:
                pass
            self._socket = None

    # Queues a single video frame to be sent to the ground station in the background thread
    def send_frame(self, frame, jpeg_quality: int) -> None:
        if self._socket is None:
            return
            
        # Drop the oldest frame if the queue is full to prevent latency buildup
        if self._frame_queue.full():
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                pass
                
        self._frame_queue.put((frame, jpeg_quality))

    # Background worker that encodes and streams the frames over TCP
    def _stream_worker(self) -> None:
        while not self._stop_event.is_set():
            if self._socket is None:
                time.sleep(0.1)
                continue
                
            try:
                frame, jpeg_quality = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue
                
            try:
                ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                frame_data = buffer.tobytes()
                message = struct.pack(">L", len(frame_data)) + frame_data
                self._socket.sendall(message)
            except (socket.error, ConnectionError) as e:
                print(f"\nVideo stream connection lost: {e}")
                self._socket.close()
                self._socket = None


# Ground-station-side TCP server: listens for the Jetson to connect, then yields decoded frames one at a time via read_frame()
class StreamReceiver:
    # Initializes the StreamReceiver with the given video link configuration
    def __init__(self, video_link_config: VideoLinkConfig):
        self._config = video_link_config
        self._server_socket = None
        self._conn = None
        self._buffer = b""

    # Starts the TCP server, waits for the Jetson to connect, and accepts the connection
    def start(self) -> None:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.bind((self._config.host, self._config.port))
        self._server_socket.listen(1)
        print("Waiting for Jetson to connect...")

        self._conn, addr = self._server_socket.accept()
        print(f"Connected by {addr}")

    # Reads a single video frame from the TCP connection, returning None if the connection is closed
    def read_frame(self):
        if not self._fill_buffer(_PAYLOAD_SIZE):
            return None

        packed_msg_size = self._buffer[:_PAYLOAD_SIZE]
        self._buffer = self._buffer[_PAYLOAD_SIZE:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]

        if not self._fill_buffer(msg_size):
            return None

        frame_data = self._buffer[:msg_size]
        self._buffer = self._buffer[msg_size:]
        
        # cv2.putText(frame_data, "Press 'c' to Calibrate", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

    # Fills the internal buffer with data from the TCP connection until it has at least needed_bytes. Returns False if the connection is closed.
    def _fill_buffer(self, needed_bytes: int) -> bool:
        while len(self._buffer) < needed_bytes:
            chunk = self._conn.recv(4096)
            if not chunk:
                return False
            self._buffer += chunk
        return True


###############################################################################
# MjpegServer — Jetson-side HTTP server that streams MJPEG to any browser.
#
# Usage:
#   server = MjpegServer(config.mjpeg_server)
#   server.start()
#   ...main loop...
#   server.push_frame(frame, jpeg_quality)   # call once per processed frame
#
# Open on any device connected to Tailscale (or same LAN):
#   http://<jetson-tailscale-ip>:8080
###############################################################################

class MjpegServer:
    """HTTP MJPEG server that runs in a background daemon thread."""

    _BOUNDARY = b"--jpgboundary"

    def __init__(self, config: MjpegServerConfig) -> None:
        self._config = config
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None  # most-recent encoded frame
        self._server: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Start the HTTP server in a background daemon thread."""
        if not self._config.enabled:
            return

        server_instance = self  # capture for closure

        class _Handler(BaseHTTPRequestHandler):
            """Minimal HTTP handler — serves MJPEG on any path."""

            def do_GET(self):
                if self.path == "/" or self.path == "/stream":
                    self._serve_stream()
                else:
                    # Redirect bare root to /stream for convenience
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    html = (
                        b"<!DOCTYPE html><html><head>"
                        b"<title>Jetson Stream</title>"
                        b"<meta name='viewport' content='width=device-width,initial-scale=1'>"
                        b"<style>body{margin:0;background:#000;display:flex;"
                        b"justify-content:center;align-items:center;height:100vh;overflow:hidden}"
                        b"img{width:100vw;height:100vh;object-fit:cover;}</style>"
                        b"</head><body>"
                        b"<img src='/stream' />"
                        b"</body></html>"
                    )
                    self.wfile.write(html)

            def _serve_stream(self):
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    f"multipart/x-mixed-replace; boundary=jpgboundary",
                )
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "close")
                self.end_headers()

                try:
                    while True:
                        with server_instance._lock:
                            jpeg = server_instance._latest_jpeg

                        if jpeg is None:
                            time.sleep(0.033)  # ~30 fps cap while waiting
                            continue

                        header = (
                            MjpegServer._BOUNDARY + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n"
                            b"\r\n"
                        )
                        try:
                            self.wfile.write(header + jpeg + b"\r\n")
                            self.wfile.flush()
                        except (BrokenPipeError, ConnectionResetError):
                            break  # client disconnected — stop silently

                        time.sleep(0.033)  # ~30 fps
                except Exception:
                    pass  # any other socket-level error — client gone, exit cleanly

            def log_message(self, fmt, *args):  # suppress per-request console noise
                pass

        self._server = HTTPServer(("0.0.0.0", self._config.port), _Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name="mjpeg-http-server",
        )
        self._thread.start()
        print(f"MJPEG HTTP server started — open http://<jetson-ip>:{self._config.port} in a browser")

    def push_frame(self, frame, jpeg_quality: int = 70) -> None:
        """Encode *frame* as JPEG and make it available to connected HTTP clients.

        This is a non-blocking call — it simply overwrites the latest frame
        in memory. If no clients are connected the frame is silently discarded.
        """
        if not self._config.enabled:
            return

        ret, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if ret:
            with self._lock:
                self._latest_jpeg = buf.tobytes()

    def stop(self) -> None:
        """Gracefully shut down the HTTP server."""
        if self._server is not None:
            self._server.shutdown()
            self._server = None
