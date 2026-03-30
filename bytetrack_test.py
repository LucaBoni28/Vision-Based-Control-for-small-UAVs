#########################

# Author: Luca Boninsegna
# Date:   25/03/26
# Descr:  Adding an ID to object using BYTETrack or DeepSORT 

#########################

import cv2
from ultralytics import YOLO
import time
from deep_sort_realtime.deepsort_tracker import DeepSort


print("--- INITIATING BYTETrack VISION PIPELINE ---")

# model = YOLO('yolov8n.pt')
model = YOLO('yolov8n.engine', task='detect')

# Initialize the Tracker
tracker = DeepSort(max_age=150, embedder="mobilenet", half=True)

# Setting Gstreamer Pipeline
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
    print("Error: OpenCV cannot open the camera.")
    exit()

# Create and configure the UI window
window_name = "BYTETrack Vision"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

# Resize the window to a manageable 720p (does not affect AI accuracy)
cv2.resizeWindow(window_name, 1280, 720) 

# Move the window to coordinate (x=100, y=100) on your screen
cv2.moveWindow(window_name, 100, 100)

# Define the center of the camera frame (640x480 resolution)
CENTER_X = 320
CENTER_Y = 240

# We need to lock onto a specific object ID. 
locked_id = 0 

# Initialize the time variable
prev_time = time.time()

timeout_frames = 0
MAX_TIMEOUT = 30

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Setting the tracking mode
    # persist=True tells the tracker to remember IDs between frames
    # tracker="bytetrack.yaml" forces it to use the BYTETrack algorithm
    results = model.track(frame, persist=True, tracker="bytetrack.yaml", verbose=False)
    target_found_this_frame = False

    # Get the current time
    current_time = time.time()

    # Calculate the difference
    time_delta = current_time - prev_time

    # Calculate the FPS
    fps = 1.0 / time_delta

    # Reset previous time for next loop
    prev_time = current_time

    # Print the FPS result
    print(f"Current FPS: {fps:.1f}")

    if results[0].boxes is not None and results[0].boxes.id is not None:

        # Extract the bounding boxes and their assigned tracking IDs
        boxes = results[0].boxes.xyxy.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().numpy()
        annotated_frame = results[0].plot()
        
        for box, track_id in zip(boxes, track_ids):
            x1, y1, x2, y2 = box
            
            target_found_this_frame = True
            timeout_frames = 0

            # Calculate the bounding box center
            bb_center_x = (x1 + x2) // 2
            bb_center_y = (y1 + y2) // 2
            
            # If no target yet, lock onto the first ID we see
            if locked_id == 0:
                locked_id = track_id
                print(f"LOCKED ONTO TARGET ID: {locked_id}")
                target_found_this_frame = True
                
            # Calculate control errors only for the locked target
            if track_id == locked_id:
                error_x = bb_center_x - CENTER_X  
                error_y = bb_center_y - CENTER_Y 
                
                # Format to 2 decimal places for clean terminal reading
                print(f"ID: {track_id} | Err X: {error_x:6.2f} | Err Y: {error_y:6.2f}")
                
                # Draw a rectangle and the ID on the video frame for visual debugging
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"ID: {track_id}", (int(x1), int(y1) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                # # Draw the direction to center the target object 
                # cv2.arrowedLine(annotated_frame, (bb_center_x, CENTER_Y), (CENTER_X, bb_center_y), (0, 0, 255), 5, tipLength=0.05)
        

    if not target_found_this_frame and locked_id != 0:
        timeout_frames += 1
        print(f"Target lost... {timeout_frames}/{MAX_TIMEOUT}")
    
        if timeout_frames > MAX_TIMEOUT:
            print("TARGET PURGED. SEARCHING FOR NEW TARGET...")
            locked_id = 0 # This allows the script to lock onto a new ID next frame!

    # Display the video
    cv2.imshow(window_name, frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()