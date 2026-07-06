###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the CommandSender and CommandReceiver classes, which handle the UDP
#         command channel between the Ground Station and the Jetson Orin NX
###############################################################################

import socket
import struct
from config import CommandLinkConfig

_CLICK_FORMAT = ">ff"
_CLICK_SIZE = struct.calcsize(_CLICK_FORMAT)

# Defines the CommandSender class, which sends normalized click coordinates from the Ground Station to the Jetson Orin NX via UDP
class CommandSender:
    # Initializes the CommandSender with the given command link configuration
    def __init__(self, command_link_config: CommandLinkConfig):
        self._addr = (command_link_config.jetson_host, command_link_config.port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Sends a click with normalized coordinates (norm_x, norm_y) to the Jetson Orin NX
    def send_click(self, norm_x: float, norm_y: float) -> None:
        message = struct.pack(_CLICK_FORMAT, norm_x, norm_y)
        self._socket.sendto(message, self._addr)

# Defines the CommandReceiver class, which receives normalized click coordinates from the Ground Station on the Jetson Orin NX via UDP
class CommandReceiver:
    # Initializes the CommandReceiver with the given command link configuration
    def __init__(self, command_link_config: CommandLinkConfig):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", command_link_config.port))
        self._socket.setblocking(False)

    # Polls for a click from the Ground Station, returning (norm_x, norm_y) if a click arrived since the last call, else None
    def poll_click(self):
        """Returns (norm_x, norm_y) if a click arrived since the last call, else None."""
        latest = None
        while True:
            try:
                data, _addr = self._socket.recvfrom(1024)
            except BlockingIOError:
                break
            if len(data) == _CLICK_SIZE:
                latest = struct.unpack(_CLICK_FORMAT, data)
        return latest
