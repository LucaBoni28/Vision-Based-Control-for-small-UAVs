from ultralytics import YOLO
import cv2

# Load a YOLO26n PyTorch model
#model = YOLO("yolov8n.pt")

# Export the model to TensorRT
#model.export(format="engine")  # creates 'yolov8n.engine'

# Load the exported TensorRT model
trt_model = YOLO("yolov8n.engine", task="detect")

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
window_name = "Jetson Live YOLOv8"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

# Resize the window to a manageable 720p (does not affect AI accuracy)
cv2.resizeWindow(window_name, 1280, 720) 

# Move the window to coordinate (x=100, y=100) on your screen
cv2.moveWindow(window_name, 100, 100)

print("Starting inference. Press 'q' or click the 'X' to exit.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Run inference silently
    results = trt_model.predict(source=frame, show=False, verbose=False, device=0)

    # Extract the data for the current frame
    frame_results = results[0]
    annotated_frame = results[0].plot()

   # --- FLIGHT CONTROL LOGIC ---
    
    # Determine the camera's true center (The Zero Point)
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    # Variables to track our best target
    best_target_coords = None
    max_area = 0
    confidence_threshold = 0.85 # 85% minimum confidence

    # ilter and Select the Best Target
    for box in results[0].boxes:
        confidence = float(box.conf[0])
        
        # Ignore weak detections
        if confidence < confidence_threshold:
            continue
            
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        
        # Calculate the size of the bounding box
        box_area = (x2 - x1) * (y2 - y1)
        
        # Save the max area Bounding Box coordinates
        if box_area > max_area:
            max_area = box_area
            best_target_coords = (x1, y1, x2, y2)

    # Calculate the Error Vector
    if best_target_coords is not None:
        x1, y1, x2, y2 = best_target_coords
        
        # Find the center of our chosen target
        obj_center_x = (x1 + x2) // 2
        obj_center_y = (y1 + y2) // 2
        
        # Calculate how far off-center the target is
        error_x = obj_center_x - frame_center_x
        error_y = obj_center_y - frame_center_y
        
        # Output the flight command data
        print(f"Target Locked | Error Vector -> X: {error_x}px, Y: {error_y}px")

        # Optional: Draw a line from the targer to the center of the screen
        cv2.arrowedLine(annotated_frame, (obj_center_x, frame_center_y), (frame_center_x, obj_center_y), (0, 0, 255), 5, tipLength=0.05)
        
    # Extract the frame
    # annotated_frame = results[0].plot()
    

    # Push to the window
    cv2.imshow(window_name, annotated_frame)

    # Keyboard 'q' or Window 'X' button to close gStreamer window
    key = cv2.waitKey(10) & 0xFF
    
    # Check if the window was closed by the OS
    if key == ord('q') or cv2.getWindowProperty(window_name, cv2.WND_PROP_AUTOSIZE) == -1:
        print("Termination signal received. Shutting down...")
        break

cap.release()
cv2.destroyAllWindows()