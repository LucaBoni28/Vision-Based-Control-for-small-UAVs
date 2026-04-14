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
from deep_sort_realtime.deepsort_tracker import DeepSort
from pymavlink import mavutil

# Connection to Pixhawk
print("Connecting to Flight Controller...")
# master = mavutil.mavlink_connection('/dev/ttyACM0',baud=115200)
master = mavutil.mavlink_connection('udp:127.0.0.1:14551', source_system=1, source_component=1)

# Wait for valid MAVLink heartbeat packet
print("Bridge open. Listening for ArduPilot heartbeat...")
master.wait_heartbeat()

# Connection confirmation 
print("TARGET ACQUIRED: Heartbeat Received!")
print(f"System ID: {master.target_system}")
print(f"Component ID: {master.target_component}")

# Control gain parameter
K_yaw = 1.0     # Gain for the yaw angle
k_vz = 0.5      # Gain for vertical velocity
V_max = 1.0     # Maximum forward velocity in m/s

# Load YOLOv8 compiled as a TensorRT Engine for maximum GPU efficiency
model = YOLO('yolov8n.engine', task='detect')

# Initialize the Custom Tracker (The Visual Memory + Physics Engine)
MAX_TIMEOUT = 100   
tracker = DeepSort(
    max_age=MAX_TIMEOUT,              # Memory your X and Y coordinate pairs duration: Remembers a lost object for max_age frames
    embedder="mobilenet",     # The micro-network used to extract the visual fingerprint
    half=True,                # Uses FP16 precision to optimize Orin NX Tensor Cores
    max_cosine_distance=0.8,  # Higher value allows for lighting/shadow changes
    n_init=1,                 # Lock onto target immediately after 1 frame
    max_iou_distance=0.8,     # Allows object to move fast between frames
)

# Setting GStreamer pipeline to pull raw data from the CSI Camera
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1, format=NV12 ! "
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

# State Machine Initialization
locked_id = None            
prev_time = time.time()
timeout_frames = 0


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
    fps = 1.0 / (current_time - prev_time)
    prev_time = current_time

    # YOLO finds the object (Class 0 = Person) independently of tracking
    results = model.predict(frame, classes=[0], conf=0.4, verbose=False)
    
   
    bbs_expected_by_tracker = []
    
    for box in results[0].boxes:
        # Extract raw floating-point tensors from YOLO
        raw_x1, raw_y1, raw_x2, raw_y2 = box.xyxy[0].cpu().numpy()
        
        # Clamping coordinates to physical image boundaries to 
        # prevents sending negative pixels to MobileNet, which corrupts the Re-ID.
        x1 = max(0, int(raw_x1))
        y1 = max(0, int(raw_y1))
        x2 = min(img_w, int(raw_x2))
        y2 = min(img_h, int(raw_y2))
        
        w = x2 - x1
        h = y2 - y1
    
        # Filter out garbage artifacts or impossibly small boxes at the screen edges
        if w < 10 or h < 10:
            continue
            
        conf = box.conf[0].item()
        cls = int(box.cls[0].item())
        
        # Package into the strict list format required by DeepSORT
        bbs_expected_by_tracker.append(([x1, y1, w, h], conf, cls))

    # This single line handles:
    # 1. Cropping the image to the bounding box
    # 2. Running MobileNet to extract the visual fingerprint (Cosine Distance)
    # 3. Running the Kalman Filter to predict kinematics
    tracks = tracker.update_tracks(bbs_expected_by_tracker, frame=frame)


    # Control logic loop
    for track in tracks:
        if not track.is_confirmed():
            continue
            
        # Ignore the "ghost" box, when YOLO doesn't see the object in 2 frames
        if track.time_since_update > 2:
            continue    
        
        track_id = track.track_id 
        
        # Grab the first confirmed object we see
        if locked_id is None:
            locked_id = track_id
            print(f"LOCKED ONTO TARGET ID: {locked_id}")
            
        # Error calculation for only the locked target
        if track_id == locked_id:
            target_found_this_frame = True
            timeout_frames = 0 # Reset the memory decay timer
            
            # Extract the stabilized, filtered bounding box from DeepSORT
            ltrb = track.to_ltrb() 
            x1, y1, x2, y2 = ltrb
            
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

            # Calculate the error magnitude
            e_mag = math.sqrt(e_x**2 + e_y**2)
            e_mag = min(1.0, e_mag)

            # Calculate the sign of the adjustment
            sign_x = 1 if e_x > 1 else -1
            sign_y = 1 if e_y > 1 else -1

            # Adjust the object in the centerwindow_name
            omega_z = K_yaw * sign_x * (e_x**2)
            v_z = k_vz * sign_y * (e_y**2)

            # Setting forward velocity
            R_stop = 0.8
            if e_mag >= R_stop:
                v_x = 0.0 # Stop drone if target close to the edge
            else:
                e_scaled = e_mag / R_stop
                v_x = V_max * (1 - e_scaled**2)

            print(f"Vx: {v_x:.2f} m/s | Vz: {v_z:.2f} m/s | YawRate:{omega_z:.2f} rad/s")

            time_boot_ms = int(time.time() * 1000) % 4294967295

            master.mav.named_value_float_send(time_boot_ms, b'Jetson_Vx',v_x)
            master.mav.named_value_float_send(time_boot_ms, b'Jetson_Vy',v_z)

            # Send the MAVLink command
            master.mav.set_position_target_local_ned_send(
                0, master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,
                0, 0, 0,
                v_x, 0.0, v_z,
                0, 0, 0,
                0, omega_z
            )

            # Visual GUI Debugging
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.arrowedLine(frame, (bb_center_x, bb_center_y), (CENTER_X, CENTER_Y), (0, 0, 255), 5, tipLength=0.05)

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
                0b0000111111000111,
                0, 0, 0,
                0.0, 0.0, 0.0,
                0, 0, 0,
                0, 0.0
            )
    
        # Reset to find a new target
        if timeout_frames > MAX_TIMEOUT:
            print("TARGET PURGED FROM MEMORY. SEARCHING FOR NEW TARGET...")
            locked_id = None 

    # Compress the frame to JPEG to save network bandwidth
    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    frame_data = buffer.tobytes()

    # Pack the size of the frame, then attach the frame data
    message = struct.pack(">L", len(frame_data)) + frame_data

    # Send via TCP
    client_socket.sendall(message)
        
    # cv2.imshow(window_name, frame)
    # if cv2.waitKey(1) & 0xFF == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) == -1:
    #     break

cap.release()
cv2.destroyAllWindows()