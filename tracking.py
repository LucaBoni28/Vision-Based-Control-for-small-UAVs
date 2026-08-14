###############################################################################
# Author: Luca Boninsegna
# Date:   25/03/26
# Descr:  Custom DeepSORT Tracking Pipeline (YOLOv8 + MobileNet Re-ID)
#         Calculate pixel errors which need to be converted in drone velocities via MAVLink.
###############################################################################

import cv2
import socket
import struct
import time
import math
from ultralytics import YOLO
import numpy as np
import sys
import select
from deep_sort_realtime.deepsort_tracker import DeepSort
from pymavlink import mavutil
import argparse
from graphs_generation.scripts.thesis_logger import ThesisLogger

# Capture the exact start time to align the graph later
script_start_time = time.time()

# Parse test mode from command line (optional — no flags = normal operation)
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('--test', choices=['combined'], default=None,
                    help='Enable CSV logging: combined (Ch.4 and Ch.6 merged)')
parser.add_argument('--tracker', default='bytetrack',
                    help='Tracker name for benchmark CSV filename')
parser.add_argument('--run-name', default=None,
                    help='Subfolder name for grouping logs (e.g., run_002)')
args, _ = parser.parse_known_args()
logger = ThesisLogger(args.test, tracker_name=args.tracker, run_name=args.run_name) if args.test else None



# Connection to Pixhawk
print("Connecting to Flight Controller...")
# master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551', source_system=255, source_component=191) # 'udp:127.0.0.1:14551',

# gcs_conn = mavutil.mavlink_connection('udpout:172.29.249.199:14550', source_system=255, source_component=191)

# Wait for valid MAVLink heartbeat packet
print("Bridge open. Listening for ArduPilot heartbeat...")
master.wait_heartbeat()

# Connection confirmation 
print("TARGET ACQUIRED: Heartbeat Received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")

# Request the attitude stream
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    10,
    1
)
current_pitch_rad = 0

# Control parameters (must match config.yaml / mission_controller.py)
K_p_yaw = 0.84                   # Proportional gain for yaw rate
K_d_yaw = 0.0                    # Derivative gain for yaw rate
K_p_vz = 2.6                     # Proportional gain for vertical velocity
K_d_vz = 0.0                     # Derivative gain for vertical velocity
K_p_vx = 0.65                     # Proportional gain for forward velocity
K_d_vx = 0.0                     # Derivative gain for forward velocity
R_stop = 0.8                     # Stop ratio for v_x limiting

# Deadzones (must match config.yaml / mission_controller.py)
YAW_DEADZONE = 0.03              # Minimum yaw rate output to act upon
VZ_DEADZONE = 0.03               # Minimum vz output to act upon
DIST_DEADZONE = 0.10             # Minimum distance error to act upon
MAX_DERIVATIVE_DT = 0.5          # Maximum delta time for derivative calculation

# Velocity limits (must match config.yaml / mission_controller.py)
MAX_VX = 1.5                     # Maximum forward velocity limit (m/s)
MAX_VZ = 1.0                     # Maximum vertical velocity limit (m/s)
MAX_YAW_RATE = 1.0               # Maximum yaw rate limit (rad/s)

# Load YOLOv8 compiled as a TensorRT Engine for maximum GPU efficiency
model = YOLO('yolo26n.engine', task='detect')

# Initialize the Custom Tracker (The Visual Memory + Physics Engine)
MAX_TIMEOUT = 50   
tracker = DeepSort(
    max_age=MAX_TIMEOUT,              # Memory your X and Y coordinate pairs duration: Remembers a lost object for max_age frames
    embedder="mobilenet",     # The micro-network uses to extract the visual fingerprint
    half=False,                # Uses FP16 precision to optimize Orin NX Tensor Cores
    max_cosine_distance=0.8,  # Higher value allows for lighting/shadow changes
    n_init=1,                 # Lock onto target immediately after 1 frame
    max_iou_distance=0.8,     # Allows object to move fast between frames
)

# Setting GStreamer pipeline to pull raw data from the CSI Camera
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=960, framerate=30/1, format=NV12 ! "
    "nvvidconv ! "
    "video/x-raw, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink"
)

print("Opening CSI Camera...")
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: OpenCV cannot open the hardware camera stream.")
    exit()

# UI Setup
TCP_IP = "172.29.249.199"
TCP_PORT = 5005

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((TCP_IP,TCP_PORT))

# window_name = "DeepSORT Vision"
# cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
# cv2.resizeWindow(window_name, 1280, 720) 
# cv2.moveWindow(window_name, 100, 100)

# Initialization
locked_id = None   
timeout_frames = 0   
prev_error_y = 0   
prev_error_x = 0   
prev_error_dist_m = 0
prev_time = time.time()
prev_time_fps = 0

# Calibration area-distance parameters
# is_calibrating = False          # Trigger switch
# calibration_areas = []          # Array to store the data
# CALIBRATION_SAMPLES = 100       # Number of frames
OPTICAL_CONSTANT = 108067.2    
DESIRED_STOPPING_DISTANCE = 5.0   # [m]

frame_number = 0

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break
    frame_number += 1
    t_frame_capture = time.time()

    target_found_this_frame = False
    
    # Dynamically calculate the mathematical center of the camera frame
    img_h, img_w = frame.shape[:2]
    CENTER_X = img_w // 2
    CENTER_Y = img_h // 2
    
    # Calculate FPS
    current_time = time.time()
    fps = 1.0 / (current_time - prev_time_fps)
    prev_time_fps = current_time

    t_before_track = time.time()
    
    detected_targets = []
    
    if args.tracker == 'deepsort':
        results = model.predict(frame, classes=[62], conf=0.65, verbose=False)
        
        # Preparation of the data for DeepSORT
        bbs_expected_by_tracker = []
        for box in results[0].boxes:
            raw_x1, raw_y1, raw_x2, raw_y2 = box.xyxy[0].cpu().numpy()
            
            x1 = max(0, int(raw_x1))
            y1 = max(0, int(raw_y1))
            x2 = min(img_w, int(raw_x2))
            y2 = min(img_h, int(raw_y2))
            
            w = x2 - x1
            h = y2 - y1
            if w < 20 or h < 20:
                continue
                
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            bbs_expected_by_tracker.append(([x1, y1, w, h], conf, cls))
            
        tracks = tracker.update_tracks(bbs_expected_by_tracker, frame=frame)
        t_after_track = time.time()
        
        for track in tracks:
            if not track.is_confirmed() or track.time_since_update > 2:
                continue
            ltrb = track.to_ltrb() 
            detected_targets.append((track.track_id, ltrb[0], ltrb[1], ltrb[2], ltrb[3]))
            
    else:
        # ByteTrack or BotSORT
        tracker_yaml = f"{args.tracker}.yaml"
        results = model.track(
            frame,
            classes=[62],
            conf=0.65,
            persist=True,
            tracker=tracker_yaml,
            verbose=False
        )
        t_after_track = time.time()
        
        if results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().numpy()
            for box, track_id in zip(boxes, track_ids):
                detected_targets.append((track_id, box[0], box[1], box[2], box[3]))

    target_found_this_frame = False
    _log_track_id = -1
    _log_bbox_x = 0
    _log_bbox_y = 0
    _log_A_real = 0
    _log_Distance_Est = 0.0
    _log_v_x = 0.0
    _log_v_z = 0.0
    _log_omega_z = 0.0
    _log_pitch_rad = 0.0
    _log_pipeline_latency_ms = 0.0
    
    # Control logic loop and visualization
    for target in detected_targets:
        track_id, x1, y1, x2, y2 = target

        # Grab the first confirmed object we see
        if locked_id is None:
            locked_id = track_id
            print(f"LOCKED ONTO TARGET ID: {locked_id}")
            
        # Error calculation for only the locked target
        if track_id == locked_id:
            target_found_this_frame = True
            timeout_frames = 0 # Reset the memory decay timer

            msg = master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                current_pitch_rad = msg.pitch
                # current_pitch_rad = math.radians(-15)
                        
            # Calculate the current physical center of the target
            bb_center_x = int((x1 + x2) / 2)
            bb_center_y = int((y1 + y2) / 2)
            _log_track_id = int(track_id)
            _log_bbox_x = bb_center_x
            _log_bbox_y = bb_center_y
            
            # Calculate the mathematical offset from the camera's true center
            error_x = bb_center_x - CENTER_X  
            error_y = bb_center_y - CENTER_Y 
            
            print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")
            
            # Error normalization in range [-1,1]
            e_x = error_x / CENTER_X
            e_y = error_y / CENTER_Y
            e_y_compensated = e_y - math.tan(current_pitch_rad)

            # Calculate bounding box area and area error for distance control
            w = x2 - x1
            h = y2 - y1
            current_area = w * h

            # Calculate Distance estimation in meters
            if current_area > 0:
                distance_estimate = math.sqrt(OPTICAL_CONSTANT / current_area)
            else:
                distance_estimate = 0.0
                
            e_dist_m = distance_estimate - DESIRED_STOPPING_DISTANCE

            # Calculate the error magnitude
            e_mag = math.sqrt(e_x**2 + e_y_compensated**2)
            e_mag = min(1.0, e_mag)

            # Derivative (Rate of Change) Calculation
            # Calculates how fast the error is changing. This helps prevent overshooting the target.
            current_time = time.time()
            dt = current_time - prev_time

            if 0 < dt < MAX_DERIVATIVE_DT:
                derivative_y = (e_y_compensated - prev_error_y) / dt
                derivative_x = (e_x - prev_error_x) / dt
                derivative_dist_m = (e_dist_m - prev_error_dist_m) / dt
            else:
                derivative_y = 0
                derivative_x = 0
                derivative_dist_m = 0

            # Apply the PD Controller Equations
            # PD Formula: Output = (Proportional_Gain * Error) + (Derivative_Gain * Derivative_Error)

            # Yaw (Turn): Centers the target horizontally
            omega_z = K_p_yaw * e_x + K_d_yaw * derivative_x

            # Z-Velocity (Altitude): Centers the target vertically
            v_z = K_p_vz * e_y_compensated + K_d_vz * derivative_y

            # X-Velocity (Forward/Backward): Keeps the target at the right distance
            v_x_request = K_p_vx * e_dist_m + K_d_vx * derivative_dist_m

            # Deadzones & Safety Limits
            # Deadzones prevent the drone from twitching when it's "close enough"
            if abs(omega_z) < YAW_DEADZONE:
                omega_z = 0
            if abs(v_z) < VZ_DEADZONE:
                v_z = 0
            if abs(e_dist_m) < DIST_DEADZONE:
                v_x_request = 0.0

            # Speed Limit: If the target is way off center (e_mag >= r_stop),
            # stop moving forward until we yaw/climb to center it first.
            e_scaled = min(1.0, e_mag / R_stop)
            if e_scaled >= 1.0:
                v_x_limit = 0.0
            else:
                v_x_limit = MAX_VX * (1 - e_scaled**2)

            # Apply the calculated speed limit to the requested forward velocity
            if v_x_request > 0:
                v_x = min(v_x_request, v_x_limit)
            else:
                v_x = max(v_x_request, -v_x_limit)

            # Standard clipping for other axes
            v_z = max(min(v_z, MAX_VZ), -MAX_VZ)
            omega_z = max(min(omega_z, MAX_YAW_RATE), -MAX_YAW_RATE)

            print(f"Pitch: {current_pitch_rad*180/3.14:.2f} deg | Vx: {v_x:.2f} m/s | Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")


            # Distance already calculated above

            # Send the MAVLink command
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000011111000111,
                0, 0, 0,
                v_x, 0.0, v_z,
                0, 0, 0,
                0, omega_z
            )
            t_after_mavlink = time.time()

            _log_A_real = current_area
            _log_Distance_Est = distance_estimate
            _log_v_x = v_x
            _log_v_z = v_z
            _log_omega_z = omega_z
            _log_pitch_rad = current_pitch_rad
            _log_pipeline_latency_ms = (t_after_mavlink - t_frame_capture) * 1000

            # Update the memory states
            prev_error_y = e_y_compensated
            prev_error_x = e_x
            prev_error_dist_m = e_dist_m
            prev_time = current_time

            # Visual GUI Debugging
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.arrowedLine(frame, (CENTER_X, CENTER_Y), (bb_center_x, bb_center_y), (0, 0, 255), 5, tipLength=0.05)

    # Combined test logging — one row per frame
    if logger and args.test == "combined":
        logger.log(
            Frame_Number=frame_number,
            Time_Sec=round(t_frame_capture - script_start_time, 3),
            Processing_Time_ms=round((t_after_track - t_before_track) * 1000, 1),
            FPS=round(fps, 1),
            Object_ID=_log_track_id,
            Bbox_X=_log_bbox_x,
            Bbox_Y=_log_bbox_y,
            A_real=_log_A_real,
            Distance_Est=round(_log_Distance_Est, 4),
            v_x=round(_log_v_x, 4),
            v_z=round(_log_v_z, 4),
            omega_z=round(_log_omega_z, 4),
            current_pitch_rad=round(_log_pitch_rad, 6),
            Pipeline_Latency_ms=round(_log_pipeline_latency_ms, 1),
        )

    # UI Overlay
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # If the target is lost increment the timer
    if not target_found_this_frame and locked_id is not None:
        timeout_frames += 1
        print(f"Target lost... {timeout_frames}/{MAX_TIMEOUT}")

        # Brake the drone and hovering until new target is detected
        master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000011111000111,
                0, 0, 0,
                0.0, 0.0, 0.0,
                0, 0, 0,
                0, 0.0
            )
    
        # Reset to find a new target
        if timeout_frames > MAX_TIMEOUT:
            print("TARGET PURGED FROM MEMORY. SEARCHING FOR NEW TARGET...")
            locked_id = None 

    stream_frame = cv2.resize(frame, (640, 480))

    # Compress the frame to JPEG to save network bandwidth
    ret, buffer = cv2.imencode('.jpg', stream_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    frame_data = buffer.tobytes()

    # Pack the size of the frame, then attach the frame data
    message = struct.pack(">L", len(frame_data)) + frame_data

    # Send via TCP
    client_socket.sendall(message)
    
        
cap.release()
cv2.destroyAllWindows()
if logger:
    logger.close()
