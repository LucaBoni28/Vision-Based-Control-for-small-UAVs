###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the FlightController class, which handles the MAVLink connection
#         to the ArduPilot flight controller and provides methods to poll attitude and position, as well as send velocity commands
###############################################################################

from dataclasses import dataclass
from typing import Optional

import time
from pymavlink import mavutil

from classes.config import MavlinkConfig


@dataclass
class Attitude:
    roll: float   # radians
    pitch: float  # radians
    yaw: float    # radians, 0 = North, clockwise positive (compass convention)



class FlightController:
    # Initializes the FlightController with the given MAVLink configuration
    def __init__(self, mavlink_config: MavlinkConfig):
        self._config = mavlink_config
        self.master = None
        self._attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
        self._last_heartbeat = None
        self._last_vel_log = 0.0
        self._last_heartbeat_sent = 0.0
        self._relative_alt_m = None  # Relative altitude above home (meters)

    # Connects to the flight controller via MAVLink, waits for a heartbeat, and requests attitude data at the specified stream rate
    def connect(self) -> None:
        print("Connecting to Flight Controller...")
        self.master = mavutil.mavlink_connection(
            self._config.connection,
            baud=self._config.baud,
            source_system=self._config.source_system,
            source_component=self._config.source_component,
        )

        print("Bridge open. Listening for ArduPilot heartbeat...")
        self.master.wait_heartbeat()
        self._last_heartbeat = self.master.messages.get("HEARTBEAT")

        print("TARGET ACQUIRED: Heartbeat Received!")
        print(f"System ID: {self.master.target_system}")
        print(f"Component ID: {self.master.target_component}")

        print(f"Waiting for Mission Planner to launch the simulation...")
        try:
            self.telemetry_output = mavutil.mavlink_connection(
                self._config.telemetry_output,
                source_system=self._config.source_system,
                source_component=self._config.source_component, 
            )
            print(f"Telemetry output connected to {self._config.telemetry_output}")
        except Exception as e:
            print(f"[WARNING] Telemetry output unavailable ({e}). Continuing without it.")
            self.telemetry_output = None

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            self._config.attitude_stream_rate_hz,
            1,
        )

        if self.telemetry_output and self._config.sitl:
            try:
                # Also request position from the simulation
                self.telemetry_output.mav.request_data_stream_send(
                    1, 1, # Default SITL target system/component
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    self._config.attitude_stream_rate_hz,
                    1,
                )
            except Exception:
                pass

    # Drains the MAVLink socket buffers for both the physical drone and SITL simulation
    def update(self) -> None:
        if self.master:
            while True:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                if msg.get_type() == "HEARTBEAT":
                    if msg.type not in (mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
                        if not getattr(self, '_sitl_heartbeat_active', False):
                            self._last_heartbeat = msg

                if msg.get_type() == "GLOBAL_POSITION_INT":
                    if not getattr(self, '_sitl_alt_active', False):
                        self._relative_alt_m = msg.relative_alt / 1000.0

                if msg.get_type() == "ATTITUDE":
                    self._attitude = Attitude(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)

        if self.telemetry_output and self._config.sitl:
            while True:
                msg = self.telemetry_output.recv_match(blocking=False)
                if not msg:
                    break
                
                if msg.get_type() == "HEARTBEAT":
                    if msg.type not in (mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
                        if not getattr(self, '_sitl_heartbeat_active', False):
                            print("SITL HEARTBEAT DETECTED! Using simulated drone state.")
                        self._sitl_heartbeat_active = True
                        self._last_heartbeat = msg

                if msg.get_type() == "GLOBAL_POSITION_INT":
                    if not getattr(self, '_sitl_alt_active', False):
                        print(f"SITL ALTITUDE DETECTED! Overriding physical drone (SITL Alt: {msg.relative_alt / 1000.0}m)")
                    self._sitl_alt_active = True
                    self._relative_alt_m = msg.relative_alt / 1000.0

    # Polls for a new HEARTBEAT message from the flight controller, updating the stored heartbeat
    def poll_heartbeat(self) -> None:
        if not self.master:
            return
            
        self.update()

        # Send our own companion computer heartbeat at 1Hz continuously
        current_time = time.time()
        if current_time - self._last_heartbeat_sent > 1.0:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            if self.telemetry_output:
                try:
                    self.telemetry_output.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                except Exception:
                    pass
            self._last_heartbeat_sent = current_time

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

    def get_flight_mode(self) -> str:
        if self._last_heartbeat is None:
            return "UNKNOWN"
        if self.master and hasattr(self.master, 'mode_mapping'):
            mapping = self.master.mode_mapping()
            if mapping:
                for name, num in mapping.items():
                    if num == self._last_heartbeat.custom_mode:
                        return name
        return mavutil.mode_string_v10(self._last_heartbeat)

    def set_flight_mode(self, mode: str) -> None:
        if not self.master or not hasattr(self.master, 'mode_mapping'):
            return
        mapping = self.master.mode_mapping()
        if mapping and mode in mapping:
            mode_id = mapping[mode]
            
            # Send to physical drone / primary link using pymavlink's set_mode
            self.master.set_mode(mode_id)

            # Mirror to SITL/Mission Planner telemetry link
            if self.telemetry_output and hasattr(self.telemetry_output, 'set_mode'):
                try:
                    self.telemetry_output.set_mode(mode_id)
                except Exception:
                    pass

            print(f"Requested flight mode change to: {mode} (ID: {mode_id})")
        else:
            print(f"Unknown flight mode: {mode}")

    # Polls for a new ATTITUDE message from the flight controller, returning the most recent roll, pitch, and yaw values
    def poll_attitude(self) -> Attitude:
        self.update()
        return self._attitude

    def poll_pitch(self) -> float:
        return self.poll_attitude().pitch

    # Polls for a new GLOBAL_POSITION_INT message and returns relative altitude in meters (above home). Returns None if no data received yet.
    def poll_relative_alt(self) -> float:
        self.update()
        return self._relative_alt_m


    # Sends a velocity command to the flight controller in the body frame, with the specified velocities in m/s and yaw rate in rad/s
    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        
        self.master.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )

        # Mirror the velocity command to telemetry_output
        if self.telemetry_output:
            try:
                self.telemetry_output.mav.set_position_target_local_ned_send(
                    0, 0, 0, # Target system 0 and component 0 (Broadcast) so MP accepts it
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000011111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, yaw_rate,
                )
            except Exception:
                pass  # Don't crash if telemetry link is down

    # Sends a stop command to the flight controller, setting all velocities and yaw rate to zero
    def send_stop(self) -> None:
        self.send_velocity(0.0, 0.0, 0.0, 0.0)

    # Sends a land command to the flight controller for safety
    def send_land(self) -> None:
        if self.master:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                1, # confirmation
                0, 0, 0, 0, 0, 0, 0
            )
            
        # Mirror the land command to telemetry_output (used by SITL simulation)
        if self.telemetry_output:
            try:
                self.telemetry_output.mav.command_long_send(
                    0, 0, # Target system 0 and component 0 (Broadcast) so MP accepts it
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    1, # confirmation
                    0, 0, 0, 0, 0, 0, 0
                )
            except Exception:
                pass  # Don't crash if telemetry link is down

    # Sends a pitch command to the camera gimbal.
    def send_gimbal_pitch(self, pitch_deg: float) -> None:
        if self.master:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                1, # confirmation
                pitch_deg * 100.0, # param 1 (pitch in centidegrees)
                0, # roll
                0, # yaw
                0, # altitude
                0, # latitude
                0, # longitude
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING # mount_mode
            )
