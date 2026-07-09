"""
mission_controller.py
Phase 5: orchestrates the main loop using injected CameraSource,
FlightController, VideoStreamer, Tracker, TargetSelector, and
DistanceEstimator, with an optional WaypointManager on top for
autonomous multi-waypoint missions.

Two ways to use this class:
  - waypoint_manager=None (default): single-target continuous tracking,
    exactly the Phase 1-4 behavior, unchanged. Used by main.py.
  - waypoint_manager=<WaypointManager>: full mission state machine —
    fly to a waypoint, search for a target, track it, then move on.
    Used by main_mission.py.

The PD control math (_process_frame) is identical in both modes — the
mission state machine only decides WHEN _process_frame runs.

BUGFIX vs. the original script (applied in Phase 1, at your request): the
derivative / PD-control block used to sit at the same indentation as
`if track_id == locked_id:` instead of nested inside it, so it re-ran for
every detected box each frame instead of only the locked target. It's now
correctly nested, so the control math and the MAVLink command only run once
per frame, for the locked target.
"""
###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  Definition of the MissionController class, which orchestrates the main loop of the tracking system,
#         using injected CameraSource, FlightController, VideoStreamer, Tracker, TargetSelector,
#         and DistanceEstimator
###############################################################################

import time
import math
from enum import Enum, auto

import cv2

# Definition of the MissionState enum, which represents the different states of the mission state machine
class MissionState(Enum):
    NAVIGATING_TO_WAYPOINT = auto()
    SEARCHING_FOR_TARGET = auto()
    TRACKING_TARGET = auto()
    MISSION_COMPLETE = auto()

# Definition of the MissionController class, which orchestrates the main loop of the tracking system,
class MissionController:
    # Initializes the MissionController with the given configuration, camera source, flight controller,
    # video streamer, tracker, target selector, distance estimator, and optional waypoint manager
    def __init__(self, config, camera, flight, streamer, tracker, target_selector,
                 distance_estimator, waypoint_manager=None):
        self.config = config
        self.camera = camera
        self.flight = flight
        self.streamer = streamer
        self.tracker = tracker
        self.target_selector = target_selector
        self.distance_estimator = distance_estimator
        self.waypoint_manager = waypoint_manager

        c = config.control
        self._k_p_yaw = c.k_p_yaw
        self._k_d_yaw = c.k_d_yaw
        self._k_p_vz = c.k_p_vz
        self._k_d_vz = c.k_d_vz
        self._k_p_vx = c.k_p_vx
        self._k_d_vx = c.k_d_vx
        self._r_stop = c.r_stop

        self._target_area = distance_estimator.target_area(config.calibration.desired_stopping_distance_m)
        self._max_timeout = config.tracker.max_timeout_frames
        self._prev_time_fps = 0.0

        self._reset_tracking_memory()

        if self.waypoint_manager is not None:
            self._state = (
                MissionState.MISSION_COMPLETE if self.waypoint_manager.is_finished
                else MissionState.NAVIGATING_TO_WAYPOINT
            )
            self._tracking_started_at = None
        else:
            self._state = None  # legacy single-target mode, no state machine

    # Resets the tracking memory, including the locked target ID, timeout frames, previous errors, and previous time
    def _reset_tracking_memory(self) -> None:
        self._locked_id = None
        self._timeout_frames = 0
        self._prev_error_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_area = 0.0
        self._prev_time = time.time()

    # Runs the main loop of the mission controller, reading frames from the camera,
    # processing them according to the current mission state, and streaming the results
    def run(self) -> None:
        while self.camera.is_opened():
            success, frame = self.camera.read()
            if not success:
                break

            if self.waypoint_manager is not None:
                self._process_mission_frame(frame)
            else:
                self._process_frame(frame)

        self.camera.release()
        cv2.destroyAllWindows()

    # Processes a single frame according to the current mission state, handling navigation, searching, and tracking as appropriate
    def _process_mission_frame(self, frame) -> None:
        if self._state == MissionState.MISSION_COMPLETE:
            self.flight.send_stop()
            self._stream(frame)
            return

        if self._state == MissionState.NAVIGATING_TO_WAYPOINT:
            self._navigate_to_waypoint(frame)
            return

        # If we're in SEARCHING_FOR_TARGET or TRACKING_TARGET, we process the frame for tracking
        self._process_frame(frame)

        if self._state == MissionState.SEARCHING_FOR_TARGET and self._locked_id is not None:
            self._state = MissionState.TRACKING_TARGET
            self._tracking_started_at = time.time()
            print("State -> TRACKING_TARGET")

        elif self._state == MissionState.TRACKING_TARGET:
            lost = self._locked_id is None
            timed_out = (time.time() - self._tracking_started_at) > self.config.waypoints.max_tracking_duration_s

            if lost or timed_out:
                reason = "target lost" if lost else "max tracking duration reached"
                print(f"Finished engaging target ({reason}). Advancing to next waypoint.")
                self._reset_tracking_memory()
                self.waypoint_manager.advance()

                if self.waypoint_manager.is_finished:
                    self._state = MissionState.MISSION_COMPLETE
                    print("State -> MISSION_COMPLETE (no more waypoints)")
                else:
                    self._state = MissionState.NAVIGATING_TO_WAYPOINT
                    wp = self.waypoint_manager.current
                    print(f"State -> NAVIGATING_TO_WAYPOINT ({wp.label or wp})")
    
    # Navigates to the current waypoint, sending velocity commands to the flight controller based on the drone's position and orientation, and streams the video frame
    def _navigate_to_waypoint(self, frame) -> None:
        position = self.flight.poll_global_position()

        if position is None:
            # No GPS fix yet — hover and wait rather than fly blind.
            self.flight.send_stop()
            self._stream(frame)
            return

        if self.waypoint_manager.has_reached_current(position.lat, position.lon):
            wp = self.waypoint_manager.current
            print(f"Reached waypoint {self.waypoint_manager.index} ({wp.label or wp}). Searching for target.")
            self._state = MissionState.SEARCHING_FOR_TARGET
            self.flight.send_stop()
            self._stream(frame)
            return

        bearing = self.waypoint_manager.bearing_to_current(position.lat, position.lon)
        current_yaw = self.flight.poll_attitude().yaw
        yaw_error = self._normalize_angle(bearing - current_yaw)

        yaw_rate = self.config.waypoints.transit_yaw_kp * yaw_error
        forward_velocity = self.config.waypoints.transit_forward_velocity

        # If the yaw error is too large, we don't move forward to avoid flying off course.
        if abs(yaw_error) > math.radians(45):
            forward_velocity = 0.0

        self.flight.send_velocity(forward_velocity, 0.0, 0.0, yaw_rate)
        self._stream(frame)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        """Wraps to (-pi, pi] so yaw error always represents the shortest turn direction."""
        return (angle_rad + math.pi) % (2 * math.pi) - math.pi

    # Streams the given frame to the video streamer after resizing it to the configured stream dimensions and applying JPEG compression
    def _stream(self, frame) -> None:
        stream_frame = cv2.resize(
            frame, (self.config.display.stream_width, self.config.display.stream_height)
        )
        self.streamer.send_frame(stream_frame, self.config.display.jpeg_quality)

    # Processes a single frame for tracking, including target selection, PD control, and sending velocity commands to the flight controller
    def _process_frame(self, frame) -> None:
        target_found_this_frame = False

        img_h, img_w = frame.shape[:2]
        center_x = img_w // 2
        center_y = img_h // 2

        current_time = time.time()
        fps = 1.0 / (current_time - self._prev_time_fps)
        self._prev_time_fps = current_time

        tracks = self.tracker.track(frame)

        if tracks and self._locked_id is None:
            selected_id = self.target_selector.select(tracks)
            if selected_id is not None:
                self._locked_id = selected_id
                print(f"LOCKED ONTO TARGET ID: {self._locked_id}")

        if tracks:
            for track in tracks:
                track_id = track.id

                # Everything below only runs for the locked target
                if track_id == self._locked_id:
                    target_found_this_frame = True
                    self._timeout_frames = 0  # Reset the memory decay timer

                    x1, y1, x2, y2 = track.bbox

                    current_pitch_rad = self.flight.poll_pitch()

                    # Calculate the current physical center of the target
                    bb_center_x = int((x1 + x2) / 2)
                    bb_center_y = int((y1 + y2) / 2)

                    # Offset from the camera's true center
                    error_x = bb_center_x - center_x
                    error_y = bb_center_y - center_y

                    print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")

                    # Error normalization in range [-1,1]
                    e_x = error_x / center_x
                    e_y = error_y / center_y
                    e_y_compensated = e_y - math.tan(current_pitch_rad)

                    e_mag = math.sqrt(e_x**2 + e_y_compensated**2)
                    e_mag = min(1.0, e_mag)

                    # Forward velocity from bounding-box area vs. target area (PD control)
                    w = x2 - x1
                    h = y2 - y1
                    current_area = w * h

                    e_area = (self._target_area - current_area) / self._target_area
                    
                    current_time = time.time()
                    dt = current_time - self._prev_time

                    if 0 < dt < self.config.control.max_derivative_dt:
                        derivative_y = (e_y_compensated - self._prev_error_y) / dt
                        derivative_x = (e_x - self._prev_error_x) / dt
                        derivative_area = (e_area - self._prev_error_area) / dt
                    else:
                        derivative_y = 0
                        derivative_x = 0
                        derivative_area = 0

                    # PD controller for omega_z and v_z
                    omega_z = self._k_p_yaw * e_x + self._k_d_yaw * derivative_x
                    v_z = self._k_p_vz * e_y_compensated + self._k_d_vz * derivative_y
                    # PD controller for v_x
                    v_x_request = self._k_p_vx * e_area + self._k_d_vx * derivative_area

                    # Yaw and vertical velocity deadzone
                    if abs(omega_z) < self.config.control.yaw_deadzone: omega_z = 0
                    if abs(v_z) < self.config.control.vz_deadzone: v_z = 0
                    # Area deadzone
                    if abs(e_area) < self.config.control.area_deadzone: v_x_request = 0.0

                    # Speed limit for center alignment
                    if e_mag >= self._r_stop: v_x_limit = 0.0
                    else:
                        e_scaled = e_mag / self._r_stop
                        v_x_limit = self._k_p_vx * (1 - e_scaled**2)

                    # Selection v_x final value
                    if v_x_request > 0:
                        v_x = min(v_x_request, v_x_limit)
                    else:
                        v_x = max(v_x_request, -v_x_limit)

                    print(f"Pitch: {current_pitch_rad*180/3.14:.2f} deg | Vx: {v_x:.2f} m/s | "
                          f"Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")

                    # Distance estimate in meters (from BB area calibration)
                    distance_estimate = self.distance_estimator.distance_to(current_area)

                    self.flight.send_velocity(v_x, 0.0, v_z, omega_z)

                    self._prev_error_y = e_y_compensated
                    self._prev_error_x = e_x
                    self._prev_error_area = e_area
                    self._prev_time = current_time

                    # Visual GUI debugging
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    cv2.arrowedLine(frame, (center_x, center_y), (bb_center_x, bb_center_y),
                                     (0, 0, 255), 5, tipLength=0.05)

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if not target_found_this_frame and self._locked_id is not None:
            self._timeout_frames += 1
            print(f"Target lost... {self._timeout_frames}/{self._max_timeout}")

            self.flight.send_stop()

            if self._timeout_frames > self._max_timeout:
                print("TARGET PURGED FROM MEMORY. SEARCHING FOR NEW TARGET...")
                self._locked_id = None

        stream_frame = cv2.resize(
            frame, (self.config.display.stream_width, self.config.display.stream_height)
        )
        self.streamer.send_frame(stream_frame, self.config.display.jpeg_quality)
