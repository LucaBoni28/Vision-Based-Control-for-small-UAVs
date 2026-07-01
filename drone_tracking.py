import yaml
import cv2
import socket
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
from pymavlink import mavutil

class CameraStream:
    """Handles the video feed from the Jetson CSI camera."""
    def __init__(self, config):
        print("Opening CSI Camera...")
        pipeline = config['camera']['pipeline']
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            raise RuntimeError("Error: OpenCV cannot open the hardware camera stream.")

    def get_frame(self):
        return self.cap.read()

    def close(self):
        self.cap.release()

class VisionPipeline:
    """Handles YOLOv8 detection and DeepSORT tracking."""
    def __init__(self, config):
        print(f"Loading YOLO model: {config['vision']['model_path']}")
        self.detector = YOLO(config['vision']['model_path'], task='detect')
        
        self.tracker_type = config['vision']['tracker_type']
        if self.tracker_type == 'deepsort':
            print("Initializing DeepSORT...")
            self.tracker = DeepSort(
                max_age=config['vision']['deepsort_max_age'],
                embedder="mobilenet", 
                half=False, 
                max_cosine_distance=config['vision']['deepsort_max_cosine_distance'],
                n_init=config['vision']['deepsort_n_init'],
                max_iou_distance=config['vision']['deepsort_max_iou_distance']
            )

    def process_frame(self, frame):
        """Runs YOLO and updates the tracker."""
        # 1. Run YOLO Detection
        results = self.detector(frame, verbose=False)[0]
        
        # 2. Format for DeepSORT: [[left, top, w, h], confidence, class_id]
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            w, h = x2 - x1, y2 - y1
            detections.append([[x1, y1, w, h], conf, cls])
        
        # 3. Update Tracks
        tracks = self.tracker.update_tracks(detections, frame=frame)
        return tracks

class GroundStationClient:
    """Handles TCP socket connection to the UI/Ground Station."""
    def __init__(self, config):
        self.ip = config['network']['tcp_ip']
        self.port = config['network']['tcp_port']
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to Ground Station UI at {self.ip}:{self.port}...")
        # Uncomment the line below once you are ready to test the network
        # self.sock.connect((self.ip, self.port)) 

    def send_data(self, data):
        # Your struct packing logic goes here
        pass

class DroneController:
    """Handles MAVLink connection and velocity commands."""
    def __init__(self, config):
        print("Connecting to Flight Controller via MAVLink...")
        # Initialize your MAVLink connection here
        pass

    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        pass

# ==========================================
# MAIN LOOP
# ==========================================
def main():
    # Load settings
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # Initialize modules
    camera = CameraStream(config)
    vision = VisionPipeline(config)
    # ui_client = GroundStationClient(config)
    # drone = DroneController(config)

    window_name = "DeepSORT Vision"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("Starting Main Loop...")
    try:
        while True:
            ret, frame = camera.get_frame()
            if not ret:
                print("Failed to grab frame.")
                break
            
            # Get tracking data
            tracks = vision.process_frame(frame)
            
            # Draw bounding boxes for confirmed tracks
            for track in tracks:
                if not track.is_confirmed():
                    continue
                
                track_id = track.track_id
                ltrb = track.to_ltrb() # Left, Top, Right, Bottom
                x1, y1, x2, y2 = map(int, ltrb)
                
                # Draw on frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"ID: {track_id}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow(window_name, frame)
            
            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        camera.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()