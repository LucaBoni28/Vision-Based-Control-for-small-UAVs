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

from classes.click_command import CMD_CONFIRM
from classes.click_command import CMD_REJECT
from classes.target_selector import ManualClickSelector
from classes.click_command import CommandReceiver
import time
import math
import cv2


# Definition of the MissionController class, which orchestrates the main loop of the tracking system,
class MissionController:
    # Initializes the MissionController with the given configuration, camera source, flight controller,
    # video streamer, tracker, target selector, distance estimator, detector, command receiver, and optional waypoint manager
    def __init__(self, config, camera, flight, streamer, tracker, target_selector,
                 distance_estimator, detector, command_receiver: CommandReceiver):
        self.config = config
        self.camera = camera
        self.flight = flight
        self.streamer = streamer
        self.tracker = tracker
        self.target_selector = target_selector
        self.distance_estimator = distance_estimator
        self.detector = detector
        self.command_receiver = command_receiver


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
        self._calibration_warning_until = 0.0
        
        self._stream_lost_time = None
        self._stream_land_command_sent = False
        self._hover_timeout = 5.0  # seconds to hover before landing on stream loss

        self._reset_tracking_memory()


    # Resets the tracking memory, including the locked target ID, timeout frames, previous errors, and previous time
    def _reset_tracking_memory(self) -> None:
        self._locked_id = None
        self._timeout_frames = 0
        self._prev_error_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_area = 0.0
        self._prev_time = time.time()
        self._last_print_time = 0.0

    # Dispatches commands received from the Ground Station via the CommandReceiver.
    # Forwards click commands to ManualClickSelector, handles calibration start/stop.
    def _dispatch_commands(self) -> None:
        commands = self.command_receiver.poll_commands()
        for cmd in commands:
            if cmd[0] == "click":
                _, norm_x, norm_y = cmd
                if isinstance(self.target_selector, ManualClickSelector):
                    self.target_selector.set_pending_click(norm_x, norm_y)
                    print(f"Click received: ({norm_x:.2f}, {norm_y:.2f})")
                else:
                    print(f"Click ignored: target selection mode is not 'manual'")

            elif cmd[0] == "calibrate_check":
                _, addr = cmd
                if self.flight.is_armed():
                    print("CALIBRATION CHECK REJECTED: drone is ARMED.")
                    self._calibration_warning_until = time.time() + 5.0
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                elif self.distance_estimator.is_recording:
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                else:
                    self.command_receiver.send_feedback(addr, CMD_CONFIRM)

            elif cmd[0] == "calibrate_start":
                _, distance_m, addr = cmd
                if self.flight.is_armed():
                    print("CALIBRATION REJECTED: drone is ARMED. Disarm first.")
                    self._calibration_warning_until = time.time() + 5.0
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                elif self.distance_estimator.is_recording:
                    print("Calibration already in progress.")
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                else:
                    self.distance_estimator.start_recording(distance_m)
                    self.command_receiver.send_feedback(addr, CMD_CONFIRM)

            elif cmd[0] == "calibrate_stop":
                _, addr = cmd
                if self.distance_estimator.is_recording:
                    success = self.distance_estimator.stop_recording()
                    if success:
                        # Update the target area with the new calibration
                        self._target_area = self.distance_estimator.target_area(
                            self.config.calibration.desired_stopping_distance_m
                        )
                        print(f"Target area updated to {self._target_area:.0f} px")
                else:
                    print("No calibration recording in progress.")

    # Runs the main loop of the mission controller, reading frames from the camera,
    # processing them according to the current mission state, and streaming the results
    def run(self) -> None:
        while self.camera.is_opened():
            success, frame = self.camera.read()
            if not success:
                break

            # Poll for commands from the Ground Station (clicks, calibration)
            self._dispatch_commands()

            # Poll heartbeat to keep armed state up to date
            self.flight.poll_heartbeat()

            # Handle video stream disconnection safety
            if not self.streamer.is_connected:
                current_time = time.time()
                if self._stream_lost_time is None:
                    print(f"VIDEO STREAM LOST! Hovering for {self._hover_timeout:.0f} seconds before landing...")
                    self._stream_lost_time = current_time
                    self._stream_land_command_sent = False
                    self.flight.send_stop()

                elapsed = current_time - self._stream_lost_time
                if elapsed > self._hover_timeout and not self._stream_land_command_sent:
                    print(f"VIDEO STREAM STILL LOST AFTER {self._hover_timeout:.0f} SECONDS! Initiating landing...")
                    self.flight.send_land()
                    self._stream_land_command_sent = True

                    # Wait for drone to land (disarm), then exit the script cleanly
                    print("Waiting for drone to land and disarm before exiting...")
                    while True:
                        self.flight.poll_heartbeat()
                        if not self.flight.is_armed():
                            print("Drone disarmed (landed). Shutting down.")
                            break
                        time.sleep(0.5)
                    break  # Exit the main while loop → triggers cleanup below

                elif not self._stream_land_command_sent:
                    # Still in the hover window: try to reconnect and keep the drone stopped
                    self.flight.send_stop()
                    self.streamer.connect()

                # Skip frame processing to prevent blind flight
                time.sleep(0.1)
                continue
            else:
                if self._stream_lost_time is not None:
                    print("VIDEO STREAM RECOVERED! Resuming mission...")
                    self._stream_lost_time = None
                    self._stream_land_command_sent = False

            # Safety check: abort recording immediately if drone is armed
            if self.distance_estimator.is_recording and self.flight.is_armed():
                print("SAFETY INTERLOCK: Drone armed during calibration. Aborting recording!")
                self.distance_estimator.abort_recording()
                self._calibration_warning_until = time.time() + 5.0

            self._process_frame(frame)

        self.camera.release()
        cv2.destroyAllWindows()


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

                    current_time = time.time()
                    should_print = (current_time - self._last_print_time) > 0.5

                    if should_print:
                        print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")

                    # Error normalization in range [-1,1]
                    e_x = error_x / center_x
                    e_y = error_y / center_y
                    
                    pitch_comp_mode = self.config.pitch_compensation.mode
                    if pitch_comp_mode == "software":
                        e_y_compensated = e_y - math.tan(current_pitch_rad)
                    elif pitch_comp_mode == "gimbal_auto":
                        e_y_compensated = e_y
                        self.flight.send_gimbal_pitch(-math.degrees(current_pitch_rad))
                    else:
                        e_y_compensated = e_y

                    e_mag = math.sqrt(e_x**2 + e_y_compensated**2)
                    e_mag = min(1.0, e_mag)

                    # Forward velocity from bounding-box area vs. target area (PD control)
                    w = x2 - x1
                    h = y2 - y1
                    current_area = w * h
                    if self.distance_estimator.is_recording:
                        self.distance_estimator.record_sample(current_area)

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

                    if should_print:
                        print(f"Pitch: {current_pitch_rad*180/3.14:.2f} deg | Vx: {v_x:.2f} m/s | "
                              f"Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")
                        self._last_print_time = current_time

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

        # Draw FPS in the top right (dynamically aligned to prevent cutoff)
        fps_text = f"FPS: {fps:.1f}"
        font_scale = 1.3
        font_thickness = 3
        (fps_w, fps_h), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
        fps_x = frame.shape[1] - fps_w - 30
        cv2.putText(frame, fps_text, (fps_x, 45), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 255), font_thickness)

        # Draw calibration feedback in the top left
        if self.distance_estimator.is_recording:
            # If recording, draw REC overlay
            cv2.circle(frame, (20, 22), 10, (0, 0, 255), -1)
            cv2.putText(frame, "REC", (38, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"Samples: {self.distance_estimator.sample_count}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if self._locked_id is None:
                cv2.putText(frame, "WAITING FOR TARGET LOCK", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            # If not recording, show hint
            cv2.putText(frame, "Press 'c' to calibrate", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Display on-screen warning if calibration was rejected because the drone is armed
        if time.time() < self._calibration_warning_until:
            # Draw semi-transparent dark red full-screen overlay
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 150), -1)
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

            # Draw massive centered text
            text1 = "WARNING: DRONE ARMED!"
            text2 = "DISARM TO CALIBRATE"
            
            font_scale = 1.3
            font_thickness = 3
            (w1, h1), _ = cv2.getTextSize(text1, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            (w2, h2), _ = cv2.getTextSize(text2, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            
            x1 = (frame.shape[1] - w1) // 2
            y1 = (frame.shape[0] - h1) // 2 - 25
            x2 = (frame.shape[1] - w2) // 2
            y2 = (frame.shape[0] - h2) // 2 + 25

            # Text border/shadow (black) for readability
            cv2.putText(frame, text1, (x1+2, y1+2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
            cv2.putText(frame, text2, (x2+2, y2+2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
            
            # Main text (white and red)
            cv2.putText(frame, text1, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness)
            cv2.putText(frame, text2, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        if not target_found_this_frame and self._locked_id is not None:
            self._timeout_frames += 1
            print(f"Target lost... {self._timeout_frames}/{self._max_timeout}")

            self.flight.send_stop()

            if self._timeout_frames > self._max_timeout:
                print("TARGET PURGED FROM MEMORY. SEARCHING FOR NEW TARGET...")
                self._locked_id = None

        self._stream(frame)
