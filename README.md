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
    • Review state-of-the-art vision-based drone control and object detection methods.
    • Set up the NVIDIA Jetson Orin NX system and interface it with the Raspberry Pi Camera V2.
    • Implement YOLO-based object detection for real-time processing and determine relative object positions.
    • Develop a vision-based control algorithm that keeps the detected object at the center of the camera image.
    • Generate MAVLink commands to control the autopilot.
    • Perform indoor tests on a bench-top model using the NVIDIA Jetson Orin NX and Raspberry Pi Camera V2 system, including the detection of indoor shapes and
    generation of the corresponding MAVLink control commands. 
