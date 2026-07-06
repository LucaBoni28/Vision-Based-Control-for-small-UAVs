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
# from deep_sort_realtime.deepsort_tracker import DeepSort
from pymavlink import mavutil


# # Create a local, failsafe CSV log
# with open('thesis_distance_log.csv', 'w') as f:
#     f.write("Time_Sec,Distance_Est,BB_Area,v_x,v_z,omega_z\n")

# Capture the exact start time to align the graph later
script_start_time = time.time()



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

# Control parameters
K_p_yaw = 1     # Proportional gain for yaw rate
K_d_yaw = 0.05  # Derivative gain for yaw rate
K_p_vz = 1      # Proportional gain for vertical velocity
K_d_vz = 0.05   # Derivative gain for vertical velocity
K_p_vx = 1       # Maximum forward velocity in m/s
R_stop = 0.8    # Radius of stop forward velocity v_x

# Load YOLOv8 compiled as a TensorRT Engine for maximum GPU efficiency
model = YOLO('../yolo26n.engine', task='detect')

# Initialize the Custom Tracker (The Visual Memory + Physics Engine)
MAX_TIMEOUT = 50   
# tracker = DeepSort(
#     max_age=MAX_TIMEOUT,              # Memory your X and Y coordinate pairs duration: Remembers a lost object for max_age frames
#     embedder="mobilenet",     # The micro-network uses to extract the visual fingerprint
#     half=False,                # Uses FP16 precision to optimize Orin NX Tensor Cores
#     max_cosine_distance=0.8,  # Higher value allows for lighting/shadow changes
#     n_init=1,                 # Lock onto target immediately after 1 frame
#     max_iou_distance=0.8,     # Allows object to move fast between frames
# )

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
prev_time = time.time()
prev_time_fps = 0

# Calibration area-distance parameters
# is_calibrating = False          # Trigger switch
# calibration_areas = []          # Array to store the data
# CALIBRATION_SAMPLES = 100       # Number of frames
OPTICAL_CONSTANT = 75000*(1.5)**2    
DESIRED_STOPPING_DISTANCE = 0.6   # [m]
TARGET_AREA = OPTICAL_CONSTANT / (DESIRED_STOPPING_DISTANCE**2)             # Default value


while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    target_found_this_frame = False
    
    # Dynamically calculate the mathematical center of the camera frame
    img_h, img_w = frame.shape[:2]
    CENTER_X = img_w // 2
    CENTER_Y = img_h // 2
    
    # Calculate FPS
    current_time = time.time()
    fps = 1.0 / (current_time - prev_time_fps)
    prev_time_fps = current_time

    # YOLO finds the object independently of tracking
    # class 0 = person, class 62 = monitor
    # results = model.predict(frame, classes=[62], conf=0.65, verbose=False)
    
    # # Preparation of the data for DeepSORT
    # bbs_expected_by_tracker = []
    # for box in results[0].boxes:
    #     # Extract raw floating-point tensors from YOLO
    #     raw_x1, raw_y1, raw_x2, raw_y2 = box.xyxy[0].cpu().numpy()
        
    #     # Clamping coordinates to physical image boundaries to 
    #     # prevents sending negative pixels to MobileNet, which corrupts the Re-ID.
    #     x1 = max(0, int(raw_x1))
    #     y1 = max(0, int(raw_y1))
    #     x2 = min(img_w, int(raw_x2))
    #     y2 = min(img_h, int(raw_y2))
        
    #     # Filter out garbage artifacts or impossibly small boxes at the screen edges
    #     w = x2 - x1
    #     h = y2 - y1
    #     if w < 20 or h < 20:
    #         continue
            
    #     conf = box.conf[0].item()
    #     cls = int(box.cls[0].item())
        
    #     # Package into the strict list format required by DeepSORT
    #     bbs_expected_by_tracker.append(([x1, y1, w, h], conf, cls))

    # # 1. Cropping the image to the bounding box
    # # 2. Running MobileNet to extract the visual fingerprint (Cosine Distance)
    # # 3. Running the Kalman Filter to predict kinematics
    # tracks = tracker.update_tracks(bbs_expected_by_tracker, frame=frame)

    results = model.track(
        frame,
        classes=[62],
        conf=0.65,
        persist=True,
        tracker="bytetrack.yaml", #botsort.yaml
        verbose=False
    )

    target_found_this_frame = False
    
    # # Control logic loop and visualization
    # for track in tracks:
    #     if not track.is_confirmed():
    #         continue
            
    #     # Ignore the "ghost" box, when YOLO doesn't see the object in 2 frames
    #     if track.time_since_update > 2:
    #         continue    
        
    #     track_id = track.track_id 

    if results[0].boxes.id is not None:

        boxes = results[0].boxes.xyxy.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().numpy()

        for box, track_id in zip(boxes, track_ids):

            # Grab the first confirmed object we see
            if locked_id is None:
                locked_id = track_id
                print(f"LOCKED ONTO TARGET ID: {locked_id}")
                
            # Error calculation for only the locked target
            if track_id == locked_id:
                target_found_this_frame = True
                timeout_frames = 0 # Reset the memory decay timer
                
                # # Extract the stabilized, filtered bounding box from DeepSORT
                # ltrb = track.to_ltrb() 
                # x1, y1, x2, y2 = ltrb
                x1, y1, x2, y2 = box

                msg = master.recv_match(type='ATTITUDE', blocking=False)
                if msg:
                    current_pitch_rad = msg.pitch
                    # current_pitch_rad = math.radians(-15)
                            
                # Calculate the current physical center of the target
                bb_center_x = int((x1 + x2) / 2)
                bb_center_y = int((y1 + y2) / 2)
                
                # Calculate the mathematical offset from the camera's true center
                error_x = bb_center_x - CENTER_X  
                error_y = bb_center_y - CENTER_Y 
                
                print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")
                
                # Error normalization in range [-1,1]
                e_x = error_x / CENTER_X
                e_y = error_y / CENTER_Y
                e_y_compensated = e_y - math.tan(current_pitch_rad)

                # Calculate the error magnitude
                e_mag = math.sqrt(e_x**2 + e_y_compensated**2)
                e_mag = min(1.0, e_mag)

                # Calculate the delta t
                current_time = time.time()
                dt = current_time - prev_time

            # Calculate the derivative
            if 0 < dt < 0.5:
                derivative_y = (e_y_compensated - prev_error_y) / dt
                derivative_x = (e_x - prev_error_x) / dt
            else:
                derivative_y = 0
                derivative_x = 0

            # Implementation PD controller for omega_z and v_z
            omega_z = K_p_yaw * e_x + K_d_yaw * derivative_x
            v_z = K_p_vz * e_y_compensated + K_d_vz * derivative_y
            
            # Deadzone for velocities avoiding micro movements
            if abs(omega_z) < 0.03:
                omega_z = 0
            if abs(v_z) < 0.03:
                v_z = 0
            
            # Setting forward velocity
            w = x2 - x1
            h = y2 - y1
            current_area = w*h
            # print(f"Area: {current_area}")

            e_area = (TARGET_AREA - current_area) / TARGET_AREA
            v_x_request = K_p_vx * e_area

            # Distance deadzone: target area
            if abs(e_area) < 0.05:
                v_x_request = 0.0

            # Speed limit for center alignment
            if e_mag >= R_stop:
                v_x_limit = 0.0 # Stop drone if target close to the edge
            else:
                e_scaled = e_mag / R_stop
                v_x_limit = K_p_vx * (1 - e_scaled**2)

            #  Choosing the safest value of velocity
            if v_x_request > 0:
                v_x = min(v_x_request, v_x_limit)
            else:
                v_x = max(v_x_request, -v_x_limit)

            print(f"Pitch: {current_pitch_rad*180/3.14:.2f} deg | Vx: {v_x:.2f} m/s | Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")


            # Calculate Distance estimation in meters
            if current_area > 0:
                distance_estimate = math.sqrt(OPTICAL_CONSTANT / current_area)
            else:
                distance_estimate = 0.0

            # # Log directly to the Jetson's hard drive
            # current_time = time.time() - script_start_time
            # with open('thesis_distance_log.csv', 'a') as f:
            #     f.write(f"{current_time},{distance_estimate},{current_area},{v_x},{v_z},{omega_z}\n")
        
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

            # Update the memory states
            prev_error_y = e_y_compensated
            prev_error_x = e_x
            prev_time = current_time

            # Visual GUI Debugging
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.arrowedLine(frame, (CENTER_X, CENTER_Y), (bb_center_x, bb_center_y), (0, 0, 255), 5, tipLength=0.05)

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
