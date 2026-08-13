###############################################################################
# Author: Luca Boninsegna
# Date:   04/07/2026
# Descr:  This class is the "brain" of the drone's visual tracking system.
# It ties together:
#   - Vision (Camera, Tracker, Detector)
#   - Control (FlightController, PD Controller logic)
#   - Communication (VideoStreamer, CommandReceiver)
#   - State Management (Takeoff -> Wait for Guided -> Track -> Pause/Land)
###############################################################################

import sys
import time
import math
import cv2

from classes.click_command import CMD_CONFIRM
from classes.click_command import CMD_REJECT
from classes.target_selector import ManualClickSelector
from classes.click_command import CommandReceiver

class MissionController:
    """
    MissionController orchestrates the main loop of the tracking system.
    It reads frames from the camera, passes them to the tracker to find the target,
    computes the error between the target's position and the camera center,
    and sends velocity commands to the flight controller to keep the target centered.
    """
    
    def __init__(self, config, camera, flight, streamer, tracker, target_selector,
                 distance_estimator, detector, command_receiver: CommandReceiver,
                 mjpeg_server=None):
        # Store all injected dependencies
        self.config = config
        self.camera = camera
        self.flight = flight
        self.streamer = streamer
        self.tracker = tracker
        self.target_selector = target_selector
        self.distance_estimator = distance_estimator
        self.detector = detector
        self.command_receiver = command_receiver
        self.mjpeg_server = mjpeg_server  # Optional MJPEG HTTP server for phone/browser viewing

        # Extract Proportional-Derivative (PD) control gains from config
        c = config.control
        self._k_p_yaw = c.k_p_yaw  # Proportional gain for turning (yaw)
        self._k_d_yaw = c.k_d_yaw  # Derivative gain for turning (dampens oscillation)
        self._k_p_vz = c.k_p_vz    # Proportional gain for vertical movement (altitude)
        self._k_d_vz = c.k_d_vz    # Derivative gain for vertical movement
        self._k_p_vx = c.k_p_vx    # Proportional gain for forward/backward movement (distance)
        self._k_d_vx = c.k_d_vx    # Derivative gain for forward/backward movement
        
        # Radius stop: if the target is too far off-center, don't move forward (safety)
        self._r_stop = c.r_stop

        # Setup Target Tracking Parameters
        # Calculate the ideal bounding box area based on desired stopping distance
        self._target_area = distance_estimator.target_area(config.calibration.desired_stopping_distance_m)
        # How many frames the tracker will try to find a lost target before giving up
        self._max_timeout = config.tracker.max_timeout_frames
        
        # Initialize State Variables for timers and UI
        self._prev_time_fps = 0.0
        self._calibration_warning_until = 0.0
        
        # Video stream failsafe states
        self._stream_lost_time = None
        self._stream_land_command_sent = False
        self._hover_timeout = config.control.hover_timeout  # Seconds to hover before landing if stream is lost

        # Mission State Machine variables
        self._mission_state = "WAITING_TAKEOFF"
        self._hover_alt_min = 9999.0
        self._hover_alt_max = -9999.0
        self._hover_start_time = None

        # Reset the memory variables used for tracking and PD math
        self._reset_tracking_memory()

    def _reset_tracking_memory(self) -> None:
        """
        Clears the current locked target and resets the history needed for derivative calculations.
        This is called when the target is completely lost or the mission is paused.
        """
        self._locked_id = None       # ID of the object currently being tracked
        self._timeout_frames = 0     # Counter for frames where the target wasn't seen
        
        # Previous error values used to calculate the derivative (rate of change)
        self._prev_error_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_dist_m = 0.0
        
        self._prev_time = time.time() # Time of the last frame (for calculating dt)
        self._last_print_time = 0.0   # Timer for console spam reduction

    def _dispatch_commands(self) -> None:
        """
        Reads commands sent from the Ground Station (e.g., clicks on the video stream, calibration triggers)
        and processes them accordingly.
        """
        commands = self.command_receiver.poll_commands()
        for cmd in commands:
            if cmd[0] == "click":
                # Handle user clicking on the video stream to select a target manually
                _, norm_x, norm_y = cmd
                if isinstance(self.target_selector, ManualClickSelector):
                    self.target_selector.set_pending_click(norm_x, norm_y)
                    print(f"Click received: ({norm_x:.2f}, {norm_y:.2f})")
                else:
                    print(f"Click ignored: target selection mode is not 'manual'")

            elif cmd[0] == "calibrate_check":
                # Ground Station asks if it's safe to start calibration
                _, addr = cmd
                if self.flight.is_armed():
                    print("CALIBRATION CHECK REJECTED: drone is ARMED.")
                    self._calibration_warning_until = time.time() + 5.0
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                elif self.distance_estimator.is_recording:
                    # Already calibrating
                    self.command_receiver.send_feedback(addr, CMD_REJECT)
                else:
                    # Safe to calibrate
                    self.command_receiver.send_feedback(addr, CMD_CONFIRM)

            elif cmd[0] == "calibrate_start":
                # Start recording bounding box sizes for distance calibration
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
                # Stop recording and finalize calibration
                _, addr = cmd
                if self.distance_estimator.is_recording:
                    success = self.distance_estimator.stop_recording()
                    if success:
                        # Update the target area with the newly calculated calibration model
                        self._target_area = self.distance_estimator.target_area(
                            self.config.calibration.desired_stopping_distance_m
                        )
                        print(f"Target area updated to {self._target_area:.0f} px")
                else:
                    print("No calibration recording in progress.")

    def run(self) -> None:
        """
        The main loop of the application. It constantly pulls frames from the camera,
        checks for safety conditions (like video stream loss), updates the state machine,
        and optionally processes the frame for target tracking.
        """
        while self.camera.is_opened():
            success, frame = self.camera.read()
            if not success:
                break

            # Process any incoming commands from the ground station
            self._dispatch_commands()

            # Keep the flight controller connection alive and poll telemetry
            self.flight.poll_heartbeat()

            # Handle Video Stream Loss Failsafe
            # If the video stream disconnects, we shouldn't fly blind.
            # We hover for a set time, then auto-land if it doesn't reconnect.
            if not self.streamer.is_connected:
                current_time = time.time()
                
                # First time detecting the disconnect
                if self._stream_lost_time is None:
                    print(f"VIDEO STREAM LOST! Hovering for {self._hover_timeout:.0f} seconds before landing...")
                    print("Launch your Ground Station video receiver script again to reconnect and resume the mission!")
                    self._stream_lost_time = current_time
                    self._stream_land_command_sent = False
                    self.flight.send_stop() # Stop moving immediately

                elapsed = current_time - self._stream_lost_time
                
                # If timeout exceeded, force landing
                if elapsed > self._hover_timeout and not self._stream_land_command_sent:
                    print(f"VIDEO STREAM STILL LOST AFTER {self._hover_timeout:.0f} SECONDS! Initiating landing...")
                    self.flight.send_land()
                    self._stream_land_command_sent = True

                    # Wait in this loop until the drone is firmly on the ground
                    print("Waiting for drone to land before exiting...")
                    _seen_land_mode = False
                    while True:
                        self.flight.poll_heartbeat()
                        alt = self.flight.poll_relative_alt()
                        mode = self.flight.get_flight_mode()

                        if mode == "LAND":
                            _seen_land_mode = True

                        # If the mode changes away from LAND, it means the pilot took over manually.
                        if _seen_land_mode and mode != "LAND" and mode != "UNKNOWN":
                            print(f"Mode changed from LAND to {mode} — pilot took manual control. Restarting service...")
                            sys.exit(1)

                        if alt is not None and alt < 0.5:
                            print(f"Drone on ground (alt={alt:.1f}m). Shutting down.")
                            break
                        if alt is not None:
                            print(f"  Landing... altitude: {alt:.1f}m")
                        time.sleep(1.0)
                    break # Exit the main mission loop

                elif not self._stream_land_command_sent:
                    # While hovering and waiting for reconnection, keep sending the stop command
                    self.flight.send_stop()
                    self.streamer.connect() # Try to reconnect

                # Skip processing this frame since we are flying blind
                time.sleep(0.1)
                continue
            else:
                # Stream is healthy, reset the failsafe timer
                if self._stream_lost_time is not None:
                    print("VIDEO STREAM RECOVERED! Resuming mission...")
                    self._stream_lost_time = None
                    self._stream_land_command_sent = False

            # Calibration Safety Interlock
            # Never allow calibration while the drone is armed (propellers spinning)
            if self.distance_estimator.is_recording and self.flight.is_armed():
                print("SAFETY INTERLOCK: Drone armed during calibration. Aborting recording!")
                self.distance_estimator.abort_recording()
                self._calibration_warning_until = time.time() + 5.0

            # Fetch Flight Data for State Machine
            current_alt = self.flight.poll_relative_alt()
            flight_mode = self.flight.get_flight_mode()

            # Mission State Machine
            # This handles the transition from ground -> hover -> tracking -> paused
            if self._mission_state == "WAITING_TAKEOFF":
                # Condition: Drone must be armed and above minimum altitude to start
                if self.flight.is_armed() and current_alt is not None and current_alt >= self.config.mission_behavior.min_takeoff_alt_m:
                    if self._hover_start_time is None:
                        # Just crossed the altitude threshold, start the timer
                        self._hover_start_time = time.time()
                        self._hover_alt_min = current_alt
                        self._hover_alt_max = current_alt
                    else:
                        # Keep track of altitude variation to ensure stability
                        self._hover_alt_min = min(self._hover_alt_min, current_alt)
                        self._hover_alt_max = max(self._hover_alt_max, current_alt)
                        
                        if (self._hover_alt_max - self._hover_alt_min) > self.config.mission_behavior.hover_stability_threshold_m:
                            # Too much vertical movement, reset the hover timer
                            self._hover_start_time = time.time()
                            self._hover_alt_min = current_alt
                            self._hover_alt_max = current_alt
                        elif (time.time() - self._hover_start_time) >= self.config.mission_behavior.hover_stability_time_s:
                            # Hover is stable for the required duration! We are ready.
                            print("Stable hover detected. Waiting for pilot to switch to GUIDED mode to start mission.")
                            self._mission_state = "WAITING_GUIDED"
                else:
                    self._hover_start_time = None

            elif self._mission_state == "WAITING_GUIDED":
                # Condition: Drone descended back down or disarmed, cancel wait
                if not self.flight.is_armed() or current_alt is None or current_alt < self.config.mission_behavior.min_takeoff_alt_m:
                    print("Drone descended or disarmed while waiting for GUIDED. Returning to WAITING_TAKEOFF.")
                    self._mission_state = "WAITING_TAKEOFF"
                    self._hover_start_time = None
                # Condition: Pilot switched to GUIDED mode, start tracking!
                elif flight_mode == "GUIDED":
                    print("GUIDED mode detected. Starting mission tracking...")
                    self._mission_state = "TRACKING"

            elif self._mission_state == "TRACKING":
                # Detect if the pilot takes manual control (switches out of GUIDED)
                # We ignore this if the stream is lost, because the failsafe handles mode switching
                if self._stream_lost_time is None and flight_mode != "GUIDED" and flight_mode != "UNKNOWN":
                    print(f"Manual override detected (Mode changed to {flight_mode}). Pausing mission.")
                    self._mission_state = "PAUSED"
                    # Forget the current target when paused
                    self._reset_tracking_memory()

            elif self._mission_state == "PAUSED":
                # If pilot switches back to GUIDED, we can resume, but ONLY if we are safely in the air.
                # If they landed the drone, send them back to the takeoff sequence.
                if self._stream_lost_time is None and flight_mode == "GUIDED":
                    if current_alt is not None and current_alt >= self.config.mission_behavior.min_takeoff_alt_m:
                        print("Mode switched back to GUIDED. Resuming mission.")
                        self._mission_state = "TRACKING"
                    else:
                        print("Mode switched to GUIDED but altitude is too low. Returning to WAITING_TAKEOFF.")
                        self._mission_state = "WAITING_TAKEOFF"
                        self._hover_start_time = None

            # Execute Frame Processing based on State
            if self._mission_state == "TRACKING":
                # Process the frame for target detection and flight control
                self._process_frame(frame)
            else:
                # If not tracking, just draw UI elements but DO NOT send velocity commands
                cv2.putText(frame, f"State: {self._mission_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                if current_alt is not None:
                    cv2.putText(frame, f"Alt: {current_alt:.1f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                if self._mission_state == "WAITING_GUIDED":
                    cv2.putText(frame, "Switch to GUIDED to start mission",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Send the un-processed frame to the ground station
                self._stream(frame)

        # Cleanup when the loop ends
        self.camera.release()
        cv2.destroyAllWindows()

    def _stream(self, frame) -> None:
        """
        Resizes the frame to the configured stream dimensions and sends it via TCP
        to the Ground Station (with JPEG compression). Also pushes to the MJPEG
        HTTP server so any browser on the same network can view the stream.
        """
        stream_frame = cv2.resize(
            frame, (self.config.display.stream_width, self.config.display.stream_height)
        )
        self.streamer.send_frame(stream_frame, self.config.display.jpeg_quality)
        if self.mjpeg_server is not None:
            self.mjpeg_server.push_frame(stream_frame, self.config.display.jpeg_quality)

    def _process_frame(self, frame) -> None:
        """
        The core logic for visual tracking and flight control.
        1. Runs the YOLO tracker on the current frame.
        2. Selects a target (or continues following the locked target).
        3. Calculates the error between target position and camera center.
        4. Uses a PD controller to calculate required velocities (Yaw, Z, X).
        5. Sends the velocity commands to the flight controller.
        """
        target_found_this_frame = False

        # Get frame dimensions to find the exact center of the screen
        img_h, img_w = frame.shape[:2]
        center_x = img_w // 2
        center_y = img_h // 2

        # Calculate FPS for display
        current_time = time.time()
        fps = 1.0 / (current_time - self._prev_time_fps)
        self._prev_time_fps = current_time

        # Run the object tracker on the frame
        tracks = self.tracker.track(frame)

        # Target Selection (if we don't have one locked yet)
        if tracks and self._locked_id is None:
            # Let the target selector logic pick one (e.g., biggest box, or manual click)
            selected_id = self.target_selector.select(tracks)
            if selected_id is not None:
                self._locked_id = selected_id
                print(f"LOCKED ONTO TARGET ID: {self._locked_id}")

        # Process the Locked Target
        if tracks:
            for track in tracks:
                track_id = track.id

                # Only process math for the target we are actively tracking
                if track_id == self._locked_id:
                    target_found_this_frame = True
                    self._timeout_frames = 0  # Target seen, reset memory timer

                    # Get bounding box coordinates [Top-Left X, Top-Left Y, Bottom-Right X, Bottom-Right Y]
                    x1, y1, x2, y2 = track.bbox

                    current_pitch_rad = self.flight.poll_pitch()

                    # Find the center of the bounding box
                    bb_center_x = int((x1 + x2) / 2)
                    bb_center_y = int((y1 + y2) / 2)

                    # Calculate the raw error in pixels from the screen center
                    error_x = bb_center_x - center_x
                    error_y = bb_center_y - center_y

                    current_time = time.time()
                    should_print = (current_time - self._last_print_time) > 0.5

                    if should_print:
                        print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")

                    # Normalize errors to a range of [-1, 1]
                    # This makes the controller independent of camera resolution!
                    e_x = error_x / center_x
                    e_y = error_y / center_y
                    
                    # Pitch Compensation
                    # If the drone pitches forward, the camera looks down, artificially moving the target UP in the image.
                    # We compensate for this mechanically (gimbal) or in software (math) so it doesn't cause vertical oscillation.
                    pitch_comp_mode = self.config.pitch_compensation.mode
                    if pitch_comp_mode == "software":
                        e_y_compensated = e_y - math.tan(current_pitch_rad)
                    elif pitch_comp_mode == "gimbal_auto":
                        e_y_compensated = e_y
                        self.flight.send_gimbal_pitch(-math.degrees(current_pitch_rad))
                    else:
                        e_y_compensated = e_y

                    # Calculate total offset magnitude (how far off-center overall)
                    e_mag = math.sqrt(e_x**2 + e_y_compensated**2)
                    e_mag = min(1.0, e_mag)

                    # 5. Distance Calculation (using Bounding Box Area)
                    # We use the area of the box to estimate distance. Bigger box = closer.
                    w = x2 - x1
                    h = y2 - y1
                    current_area = w * h
                    
                    if self.distance_estimator.is_recording:
                        self.distance_estimator.record_sample(current_area)

                    # Calculate absolute physical distance error in meters
                    # Positive error = we are further than target distance = fly forward
                    if current_area > 0:
                        current_dist_m = self.distance_estimator.distance_to(current_area)
                        target_dist_m = self.config.calibration.desired_stopping_distance_m
                        e_dist_m = current_dist_m - target_dist_m
                    else:
                        e_dist_m = 0.0
                    
                    # Derivative (Rate of Change) Calculation
                    # Calculates how fast the error is changing. This helps prevent overshooting the target.
                    current_time = time.time()
                    dt = current_time - self._prev_time

                    if 0 < dt < self.config.control.max_derivative_dt:
                        derivative_y = (e_y_compensated - self._prev_error_y) / dt
                        derivative_x = (e_x - self._prev_error_x) / dt
                        derivative_dist_m = (e_dist_m - self._prev_error_dist_m) / dt
                    else:
                        derivative_y = 0
                        derivative_x = 0
                        derivative_dist_m = 0

                    # Apply the PD Controller Equations
                    # PD Formula: Output = (Proportional_Gain * Error) + (Derivative_Gain * Derivative_Error)
                    
                    # Yaw (Turn): Centers the target horizontally
                    omega_z = self._k_p_yaw * e_x + self._k_d_yaw * derivative_x
                    
                    # Z-Velocity (Altitude): Centers the target vertically
                    v_z = self._k_p_vz * e_y_compensated + self._k_d_vz * derivative_y
                    
                    # X-Velocity (Forward/Backward): Keeps the target at the right distance
                    v_x_request = self._k_p_vx * e_dist_m + self._k_d_vx * derivative_dist_m

                    # Deadzones & Safety Limits
                    # Deadzones prevent the drone from twitching when it's "close enough"
                    if abs(omega_z) < self.config.control.yaw_deadzone: omega_z = 0
                    if abs(v_z) < self.config.control.vz_deadzone: v_z = 0
                    if abs(e_dist_m) < self.config.control.dist_deadzone: v_x_request = 0.0

                    # Speed Limit: If the target is way off center (e_mag >= r_stop), 
                    # stop moving forward until we yaw/climb to center it first.
                    e_scaled = min(1.0, e_mag / self._r_stop)
                    if e_scaled >= 1.0:
                        v_x_limit = 0.0
                    else:
                        v_x_limit = self.config.control.max_vx * (1 - e_scaled**2)

                    # Apply the calculated speed limit to the requested forward velocity
                    if v_x_request > 0:
                        v_x = min(v_x_request, v_x_limit)
                    else:
                        v_x = max(v_x_request, -v_x_limit)

                    # Standard clipping for other axes
                    v_z = max(min(v_z, self.config.control.max_vz), -self.config.control.max_vz)
                    omega_z = max(min(omega_z, self.config.control.max_yaw_rate), -self.config.control.max_yaw_rate)

                    if should_print:
                        print(f"Pitch: {current_pitch_rad*180/3.14:.2f} deg | Vx: {v_x:.2f} m/s | "
                              f"Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")
                        self._last_print_time = current_time

                    # Optional: estimate distance in meters based on calibration data
                    distance_estimate = self.distance_estimator.distance_to(current_area)

                    # Send the final computed velocities to the flight controller
                    self.flight.send_velocity(v_x, 0.0, v_z, omega_z)

                    # Store current errors to use as "previous" errors in the next frame
                    self._prev_error_y = e_y_compensated
                    self._prev_error_x = e_x
                    self._prev_error_dist_m = e_dist_m
                    self._prev_time = current_time

                    # Draw UI Overlays (Bounding Box, ID, Aiming Line)
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    cv2.arrowedLine(frame, (center_x, center_y), (bb_center_x, bb_center_y),
                                     (0, 0, 255), 5, tipLength=0.05)

        # Draw FPS in the top right corner
        fps_text = f"FPS: {fps:.1f}"
        font_scale = 1.3
        font_thickness = 3
        (fps_w, fps_h), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
        fps_x = frame.shape[1] - fps_w - 30
        cv2.putText(frame, fps_text, (fps_x, 45), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 255), font_thickness)

        # Draw Calibration State Overlays
        if self.distance_estimator.is_recording:
            # Show "REC" and sample count
            cv2.circle(frame, (20, 22), 10, (0, 0, 255), -1)
            cv2.putText(frame, "REC", (38, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"Samples: {self.distance_estimator.sample_count}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if self._locked_id is None:
                cv2.putText(frame, "WAITING FOR TARGET LOCK", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            # Show hint when not recording
            cv2.putText(frame, "Press 'c' to calibrate", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Draw Warning if Calibration was rejected due to armed drone
        if time.time() < self._calibration_warning_until:
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 150), -1)
            cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

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

            # Black shadow for readability
            cv2.putText(frame, text1, (x1+2, y1+2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
            cv2.putText(frame, text2, (x2+2, y2+2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
            
            # Red/White text
            cv2.putText(frame, text1, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), font_thickness)
            cv2.putText(frame, text2, (x2, y2), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)

        # Target Loss Handling
        if not target_found_this_frame and self._locked_id is not None:
            self._timeout_frames += 1
            print(f"Target lost... {self._timeout_frames}/{self._max_timeout}")

            # Stop moving immediately if we lose sight of the target
            self.flight.send_stop()

            # If lost for too long, clear the target ID and return to searching phase
            if self._timeout_frames > self._max_timeout:
                print("TARGET PURGED FROM MEMORY. SEARCHING FOR NEW TARGET...")
                self._locked_id = None

        # Stream the final processed frame
        self._stream(frame)
