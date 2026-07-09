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
#           CMD_CALIBRATE_CHECK(0x06)   + (no payload)
#
#         Feedback responses sent from Jetson back to Ground Station:
#           CMD_CONFIRM (0x04)          + (no payload)
#           CMD_REJECT (0x05)           + (no payload)
###############################################################################

import socket
import struct
from classes.config import CommandLinkConfig

# Command type constants
CMD_CLICK = 0x01
CMD_CALIBRATE_START = 0x02
CMD_CALIBRATE_STOP = 0x03
CMD_CALIBRATE_CHECK = 0x06

# Response constants
CMD_CONFIRM = 0x04
CMD_REJECT = 0x05

# Struct formats (big-endian)
_FMT_CLICK = ">Bff"           # type + norm_x + norm_y
_FMT_CAL_START = ">Bf"        # type + distance_m
_FMT_CAL_STOP = ">B"          # type only
_FMT_CAL_CHECK = ">B"         # type only
_FMT_RESPONSE = ">B"          # type only

_SIZE_CLICK = struct.calcsize(_FMT_CLICK)
_SIZE_CAL_START = struct.calcsize(_FMT_CAL_START)
_SIZE_CAL_STOP = struct.calcsize(_FMT_CAL_STOP)
_SIZE_CAL_CHECK = struct.calcsize(_FMT_CAL_CHECK)


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

    def _flush_socket(self) -> None:
        """Flushes any stale UDP packets from the receive buffer."""
        self._socket.settimeout(0.0)
        while True:
            try:
                self._socket.recvfrom(1024)
            except (socket.timeout, BlockingIOError, OSError):
                break

    # Checks if calibration can be started (e.g., drone not armed) before prompting for input
    def send_calibrate_check(self) -> bool:
        self._flush_socket()
        message = struct.pack(_FMT_CAL_CHECK, CMD_CALIBRATE_CHECK)
        self._socket.sendto(message, self._addr)
        
        self._socket.settimeout(0.5)
        try:
            data, _ = self._socket.recvfrom(1024)
            if len(data) >= 1:
                resp_type = data[0]
                if resp_type == CMD_CONFIRM:
                    return True
                elif resp_type == CMD_REJECT:
                    print("[Calibration] Rejected: Drone is ARMED. Disarm first!")
                    return False
        except socket.timeout:
            print("[Calibration] Warning: No response from Jetson for check.")
            return True
        except Exception as e:
            print(f"[Calibration] Error receiving check feedback: {e}")
            return True
        finally:
            self._socket.settimeout(None)
        return True

    # Sends a calibration start command and waits for Jetson to confirm or reject.
    # Returns True if calibration started successfully, False if rejected (e.g. drone armed) or timed out.
    def send_calibrate_start(self, distance_m: float) -> bool:
        self._flush_socket()
        message = struct.pack(_FMT_CAL_START, CMD_CALIBRATE_START, distance_m)
        self._socket.sendto(message, self._addr)

        # Wait up to 500ms for a response
        self._socket.settimeout(0.5)
        try:
            data, _ = self._socket.recvfrom(1024)
            if len(data) >= 1:
                resp_type = data[0]
                if resp_type == CMD_CONFIRM:
                    return True
                elif resp_type == CMD_REJECT:
                    print("[Calibration] Rejected by Jetson: Drone is armed or already calibrating!")
                    return False
        except socket.timeout:
            print("[Calibration] Warning: No response from Jetson. Assuming started.")
            return True
        except Exception as e:
            print(f"[Calibration] Error receiving feedback: {e}")
            return True
        finally:
            self._socket.settimeout(None)
        return True

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

    # Sends feedback status back to the sender at the given address
    def send_feedback(self, addr, status_type: int) -> None:
        try:
            message = struct.pack(_FMT_RESPONSE, status_type)
            self._socket.sendto(message, addr)
        except Exception as e:
            print(f"CommandReceiver: failed to send feedback to {addr}: {e}")

    # Polls for all commands received since the last call.
    # Returns a list of tuples:
    #   ("click", norm_x, norm_y)
    #   ("calibrate_start", distance_m, addr)
    #   ("calibrate_stop", addr)
    def poll_commands(self):
        commands = []
        while True:
            try:
                data, addr = self._socket.recvfrom(1024)
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
                commands.append(("calibrate_start", distance_m, addr))

            elif cmd_type == CMD_CALIBRATE_STOP and len(data) == _SIZE_CAL_STOP:
                commands.append(("calibrate_stop", addr))

            elif cmd_type == CMD_CALIBRATE_CHECK and len(data) == _SIZE_CAL_CHECK:
                commands.append(("calibrate_check", addr))

            else:
                print(f"CommandReceiver: unknown command type={cmd_type}, len={len(data)}")

        return commands
