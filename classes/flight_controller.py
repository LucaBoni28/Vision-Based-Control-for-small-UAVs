###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the FlightController class, which handles the MAVLink connection
#         to the ArduPilot flight controller and provides methods to poll attitude and position, as well as send velocity commands
###############################################################################

from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from classes.config import MavlinkConfig


@dataclass
class Attitude:
    roll: float   # radians
    pitch: float  # radians
    yaw: float    # radians, 0 = North, clockwise positive (compass convention)

@dataclass
class GlobalPosition:
    lat: float
    lon: float
    relative_alt_m: float


class FlightController:
    # Initializes the FlightController with the given MAVLink configuration
    def __init__(self, mavlink_config: MavlinkConfig):
        self._config = mavlink_config
        self.master = None
        self._attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
        self._global_position: Optional[GlobalPosition] = None
        self._last_heartbeat = None
        self._last_vel_log = 0.0
        self._last_heartbeat_sent = 0.0

    # Connects to the flight controller via MAVLink, waits for a heartbeat, and requests attitude data at the specified stream rate
    def connect(self) -> None:
        print("Connecting to Flight Controller...")
        self.master = mavutil.mavlink_connection(
            self._config.connection,
            source_system=self._config.source_system,
            source_component=self._config.source_component,
        )

        print("Bridge open. Listening for ArduPilot heartbeat...")
        self.master.wait_heartbeat()
        self._last_heartbeat = self.master.messages.get("HEARTBEAT")

        print("TARGET ACQUIRED: Heartbeat Received!")
        print(f"System ID: {self.master.target_system}")
        print(f"Component ID: {self.master.target_component}")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            self._config.attitude_stream_rate_hz,
            1,
        )

    # Polls for a new HEARTBEAT message from the flight controller, updating the stored heartbeat
    def poll_heartbeat(self) -> None:
        msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if msg:
            self._last_heartbeat = msg

    # Returns True if the drone is currently armed, based on the latest heartbeat
    def is_armed(self) -> bool:
        if self._last_heartbeat is None:
            return False
        return bool(self._last_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def target_system(self):
        return self.master.target_system

    @property
    def target_component(self):
        return self.master.target_component

    # Polls for a new ATTITUDE message from the flight controller, returning the most recent roll, pitch, and yaw values
    def poll_attitude(self) -> Attitude:
        msg = self.master.recv_match(type="ATTITUDE", blocking=False)
        if msg:
            self._attitude = Attitude(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
        return self._attitude

    def poll_pitch(self) -> float:
        return self.poll_attitude().pitch

    # Polls for a new GLOBAL_POSITION_INT message from the flight controller, returning the most recent GPS fix or None if no fix has ever been received
    def poll_global_position(self) -> Optional[GlobalPosition]:
        
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            self._global_position = GlobalPosition(
                lat=msg.lat / 1e7,
                lon=msg.lon / 1e7,
                relative_alt_m=msg.relative_alt / 1000.0,
            )
        return self._global_position

    # Sends a velocity command to the flight controller in the body frame, with the specified velocities in m/s and yaw rate in rad/s
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        import time
        current_time = time.time()
        
        if current_time - self._last_heartbeat_sent > 1.0:
            if self.master:
                # Send heartbeat so Mission Planner registers this companion computer
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            self._last_heartbeat_sent = current_time

        if current_time - self._last_vel_log > 1.0:
            log_msg = f"Vel: Vx:{vx:.1f} Vy:{vy:.1f} Vz:{vz:.1f} Y:{yaw_rate:.1f}"
            if self.master:
                self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, log_msg.encode('ascii')[:50])
            self._last_vel_log = current_time

        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )

    # Sends a stop command to the flight controller, setting all velocities and yaw rate to zero
    def send_stop(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
