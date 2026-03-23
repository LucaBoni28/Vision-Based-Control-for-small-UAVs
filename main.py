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
    
    # Loop through every single detected object in this frame
    for box in frame_results.boxes:
        
        # 1. Bounding Box Coordinates
        # .xyxy[0] grabs the tensor array [x1, y1, x2, y2]
        # .tolist() pulls it from the GPU into a standard Python list
        coords = box.xyxy[0].tolist()
        
        # Convert the raw floats into integers because pixels are whole numbers
        x1, y1, x2, y2 = map(int, coords)
        
        # 2. Confidence Score
        # .conf[0] grabs the tensor, float() converts it to a standard Python decimal
        confidence = float(box.conf[0])
        
        # 3. Class ID
        # int() converts the tensor ID into a standard Python integer
        class_id = int(box.cls[0])
        
        # Output the mathematical truth to the terminal
        if confidence > 0.90:
            print(f"Class: {class_id} | Conf: {confidence:.2f} | Coords: [{x1}, {y1}, {x2}, {y2}]")

    # Extract the frame
    annotated_frame = results[0].plot()

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