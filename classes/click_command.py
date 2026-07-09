###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the CommandSender and CommandReceiver classes, which handle the UDP
#         command channel between the Ground Station and the Jetson Orin NX.
#
#         Protocol: each UDP datagram starts with a 1-byte command type,
#         followed by type-specific payload:
#           CMD_CLICK (0x01)            + two floats (norm_x, norm_y)
#           CMD_CALIBRATE_START (0x02)  + one float  (distance_m)
#           CMD_CALIBRATE_STOP (0x03)   + (no payload)
###############################################################################

import socket
import struct
from classes.config import CommandLinkConfig

# Command type constants
CMD_CLICK = 0x01
CMD_CALIBRATE_START = 0x02
CMD_CALIBRATE_STOP = 0x03

# Struct formats (big-endian)
_FMT_CLICK = ">Bff"           # type + norm_x + norm_y
_FMT_CAL_START = ">Bf"        # type + distance_m
_FMT_CAL_STOP = ">B"          # type only

_SIZE_CLICK = struct.calcsize(_FMT_CLICK)
_SIZE_CAL_START = struct.calcsize(_FMT_CAL_START)
_SIZE_CAL_STOP = struct.calcsize(_FMT_CAL_STOP)


# Defines the CommandSender class, which sends commands from the Ground Station to the Jetson Orin NX via UDP
class CommandSender:
    # Initializes the CommandSender with the given command link configuration
    def __init__(self, command_link_config: CommandLinkConfig):
        self._addr = (command_link_config.jetson_host, command_link_config.port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Sends a click with normalized coordinates (norm_x, norm_y) to the Jetson Orin NX
    def send_click(self, norm_x: float, norm_y: float) -> None:
        message = struct.pack(_FMT_CLICK, CMD_CLICK, norm_x, norm_y)
        self._socket.sendto(message, self._addr)

    # Sends a calibration start command with the known distance in meters
    def send_calibrate_start(self, distance_m: float) -> None:
        message = struct.pack(_FMT_CAL_START, CMD_CALIBRATE_START, distance_m)
        self._socket.sendto(message, self._addr)

    # Sends a calibration stop command
    def send_calibrate_stop(self) -> None:
        message = struct.pack(_FMT_CAL_STOP, CMD_CALIBRATE_STOP)
        self._socket.sendto(message, self._addr)


# Defines the CommandReceiver class, which receives commands from the Ground Station on the Jetson Orin NX via UDP
class CommandReceiver:
    # Initializes the CommandReceiver with the given command link configuration
    def __init__(self, command_link_config: CommandLinkConfig):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("0.0.0.0", command_link_config.port))
        self._socket.setblocking(False)

    # Polls for all commands received since the last call.
    # Returns a list of tuples:
    #   ("click", norm_x, norm_y)
    #   ("calibrate_start", distance_m)
    #   ("calibrate_stop",)
    def poll_commands(self):
        commands = []
        while True:
            try:
                data, _addr = self._socket.recvfrom(1024)
            except BlockingIOError:
                break

            if len(data) < 1:
                continue

            cmd_type = data[0]

            if cmd_type == CMD_CLICK and len(data) == _SIZE_CLICK:
                _, norm_x, norm_y = struct.unpack(_FMT_CLICK, data)
                commands.append(("click", norm_x, norm_y))

            elif cmd_type == CMD_CALIBRATE_START and len(data) == _SIZE_CAL_START:
                _, distance_m = struct.unpack(_FMT_CAL_START, data)
                commands.append(("calibrate_start", distance_m))

            elif cmd_type == CMD_CALIBRATE_STOP and len(data) == _SIZE_CAL_STOP:
                commands.append(("calibrate_stop",))

            else:
                print(f"CommandReceiver: unknown command type={cmd_type}, len={len(data)}")

        return commands
