###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the VideoStreamer and StreamReceiver classes, which handle the TCP
#         video streaming between the Jetson Orin NX and the Ground Station.
###############################################################################

import socket
import struct

import cv2
import numpy as np

from classes.config import VideoLinkConfig

# Constants for the TCP video streaming protocol
_PAYLOAD_SIZE = struct.calcsize(">L")  # 4-byte unsigned long

# Jetson-side TCP client: connects out to the ground station and sends frames
class VideoStreamer:
    # Initializes the VideoStreamer with the given video link configuration
    def __init__(self, video_link_config: VideoLinkConfig):
        self._config = video_link_config
        self._socket = None

    # Connects to the ground station using the configured host and port
    def connect(self) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._config.host, self._config.port))

    # Sends a single video frame to the ground station, encoded as JPEG with the specified quality
    def send_frame(self, frame, jpeg_quality: int) -> None:
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        frame_data = buffer.tobytes()
        message = struct.pack(">L", len(frame_data)) + frame_data
        self._socket.sendall(message)


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

        return cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)

    # Fills the internal buffer with data from the TCP connection until it has at least needed_bytes. Returns False if the connection is closed.
    def _fill_buffer(self, needed_bytes: int) -> bool:
        while len(self._buffer) < needed_bytes:
            chunk = self._conn.recv(4096)
            if not chunk:
                return False
            self._buffer += chunk
        return True
