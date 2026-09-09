# MSc Degree Thesis: Feasibility Investigation of Vision-Based Control Methods for Small-Scale Drones

This thesis investigates the feasibility of vision-based control methods for small-scale drones.
The research initially focuses on laboratory experiments using relative visual positioning.
The system will utilize an NVIDIA Jetson Orin NX computer and a Raspberry Pi Camera V2 to detect objects and determine their positions relative to the drone.

In the final system configuration, the NVIDIA Jetson Orin NX equipped with the camera will act as a companion computer to an ArduPilot open-source autopilot.
Communication between the autopilot and the companion computer will be performed using commands based on the standard MAVLink protocol.
The main task of this project is to control the drone based on visual localization using the MAVLink protocol. The drone will fly in the direction of the mounted
camera (X direction) and adjust its attitude (orientation) so that the detected object remains in the center of the camera image. This ensures that the drone flies
toward the detected target object, enabling vision-based flight control.

Future work may extend the system to outdoor experiments, where GPS positioning from the ArduPilot autopilot system can be combined with visual feedback to enable
more advanced autonomous navigation.

Tasks to be performed by the student:
- Review state-of-the-art vision-based drone control and object detection methods.
- Set up the NVIDIA Jetson Orin NX system and interface it with the Raspberry Pi Camera V2.
- Implement YOLO-based object detection for real-time processing and determine relative object positions.
- Develop a vision-based control algorithm that keeps the detected object at the center of the camera image.
- Generate MAVLink commands to control the autopilot.
- Perform indoor tests on a bench-top model using the NVIDIA Jetson Orin NX and Raspberry Pi Camera V2 system, including the detection of indoor shapes and
    generation of the corresponding MAVLink control commands. 

## How to Run

To run the vision control system, you need to run two scripts: one on the Ground Station (to view the camera feed) and one on the NVIDIA Jetson (to process the tracking and control).

### 1. Ground Station (Video Stream)
Before starting the tracking system, run the video stream server on your Ground Station PC to receive and display the processed frames over TCP:
```bash
python stream_video.py
```

### 2. NVIDIA Jetson (Tracking & Control)
The main execution script is `tracking.py`. 

When executed normally, the script will:
1. Initialize the YOLO object detection model and the DeepSORT tracker.
2. Connect to the flight controller via MAVLink (listening on UDP port 14551 by default).
3. Capture frames from the camera to detect and track the target.
4. Compute the target's pixel error relative to the camera's center.
5. Translate the errors into velocity commands and send them to the drone via MAVLink to keep the target centered.
6. Stream the annotated video frames to the Ground Station PC.

To run the standard tracking loop on the Jetson:
```bash
python tracking.py
```

*(Note: For instructions on how to run the system with data logging enabled to evaluate tracking performance and hardware metrics, please refer to the [graphs_generation/README.md](graphs_generation/README.md) file).*

*(Note: An experimental, modular object-oriented version of the pipeline is located on the `oop` branch).*
