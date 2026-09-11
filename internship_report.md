# Internship Report: Vision-Based Control for Small UAVs

## Abstract
This report summarizes the work completed during the internship, focused on developing and deploying an autonomous vision-based control system for small unmanned aerial vehicles (UAVs). Originally conceptualized as a feasibility study in a Master's thesis, the project was evolved into a robust, real-world application designed to run on edge hardware (NVIDIA Jetson Orin NX) alongside an ArduPilot-based flight controller. This document outlines the system architecture, control algorithms, testing methodologies (SITL), and deployment strategies implemented throughout the internship.

---

## Chapter 1: Introduction and Project Overview

The objective of this project is to empower a UAV to autonomously identify, track, and physically follow a visual target (such as a person or specific object) using only a monocular camera. 

The software operates entirely on edge-hardware in real-time. By calculating the relative offset of a target from the center of the camera frame, the system translates these visual errors into flight commands (rotational and translational velocities) to keep the drone centered on the target and a safe tracking distance.

### 1.1 Hardware Environment
- **Companion Computer**: NVIDIA Jetson Orin NX.
- **Flight Controller**: Pixhawk running ArduPilot.
- **Camera**: CSI Camera Raspberry Pi Camera Module 2 connected directly to the Jetson.
- **Networking**: Sierra Wireless 4G/5G module utilizing a Tailscale VPN mesh network.

---

## Chapter 2: System Architecture

The codebase was heavily refactored into a modular Object-Oriented Programming (OOP) architecture to decouple vision processing from flight control. This separation of concerns allows for the seamless swapping of vision backends or the transition from simulated inputs to physical hardware without altering the core control logic.

The architecture follows a clear data flow: **Vision -> State Estimation -> Control**.

### 2.1 Core Modules
- **Vision & Perception**: Interfaces with the hardware camera, wraps the YOLO object detector (optimized via TensorRT), and manages object tracking (e.g., ByteTrack/DeepSORT) to maintain consistent target IDs across frames.
- **State Estimation**: Handles the logic for selecting targets and translates 2D bounding boxes into physical 3D distance estimates.
- **Flight & Control**: Manages the MAVLink communication layer and executes the PID control math, computing the required velocity vectors to center the target.
- **Networking**: Manages TCP/UDP video streaming and command routing between the drone and the Ground Station.

### 2.2 MAVLink Network Architecture
A robust triple-connection MAVLink architecture was established to separate flight commands from telemetry inspection and simulation:
1. **`master`**: The primary command link connecting the Jetson to the Pixhawk for real flight commands.
2. **`sitl_connection`**: A dedicated TCP link used exclusively during Software-In-The-Loop (SITL) testing to retrieve simulated state data (Altitude, Attitude).
3. **`telemetry_output`**: A unidirectional UDP broadcast that mirrors the Jetson's internal velocity commands to the Ground Station (Mission Planner), allowing real-time inspection of the control algorithm's output.

---

## Chapter 3: Object-Oriented Architecture and Class Structure

To ensure the system remains maintainable, scalable, and adaptable to future hardware or algorithmic changes, the software was designed using a robust Object-Oriented Programming (OOP) paradigm. The architecture follows a Dependency Injection pattern, where all core modules (vision, control, networking) are instantiated independently and injected into a central orchestrator. 

This design choice decouples the complex subsystems. For example, the control logic does not need to know whether the target coordinates come from a real CSI camera or a simulated video feed, nor does it care if the tracker backend is ByteTrack or DeepSORT.

### 3.1 The Main Entry Point (`main.py`)
The `main.py` script acts as the system bootstrap. It parses the configuration YAML, initializes all hardware interfaces, AI models, and network streams, and injects them into the `MissionController`. Once configured, it hands over the execution loop to the controller.

### 3.2 The Orchestrator (`MissionController`)
Located in `classes/mission_controller.py`, this class is the "brain" of the drone's visual tracking system. It runs the primary continuous loop that:
1. Polls frames from the camera.
2. Manages the system State Machine (WAITING_TAKEOFF $\rightarrow$ WAITING_GUIDED $\rightarrow$ TRACKING $\rightarrow$ PAUSED).
3. Executes the tracking and Proportional-Derivative (PD) control mathematics.
4. Handles safety mechanisms, such as connection timeouts and automatic hover/land sequences if the video stream drops.

### 3.3 Core Class Implementations

#### Vision and Perception
- **`CameraSource` (Abstract Base Class)**: Defines the interface for retrieving frames.
  - **`CSICameraSource`**: A concrete implementation specifically optimized for Jetson's hardware-accelerated GStreamer pipeline to fetch frames from the Raspberry Pi camera with minimal latency.
- **`Detector` & `YoloDetector`**: Wraps the Ultralytics YOLO inference engine. It abstracts the tensor operations and simply returns standardized bounding box data.
- **`Tracker` (Abstract Base Class)**: Defines the interface for assigning persistent IDs to detected objects across consecutive frames.
  - Subclasses like **`ByteTrackTracker`**, **`BotSortTracker`**, and **`DeepSortTracker`** implement specific tracking algorithms. This abstraction allows researchers to swap tracking backends via the config file without modifying the control loop.

#### State Estimation and Target Selection
- **`TargetSelector` (Abstract Base Class)**: Dictates *which* object the drone should follow when multiple objects are detected.
  - **`ManualClickSelector`**: Listens for coordinate clicks from the Ground Station UI to lock onto a user-specified target.
  - **`FirstDetectedSelector`** / **`NearestObjectSelector`**: Autonomous selectors that pick targets based on screen prominence or detection order.
- **`DistanceEstimator`**: Encapsulates the mathematical model used to convert a 2D bounding box area into a physical 3D distance. It also contains the logic for the live calibration recording workflow, keeping calibration math isolated from the main control loop.

#### Flight and Control
- **`FlightController`**: A robust wrapper around the `pymavlink` library. It translates abstract high-level commands (e.g., `send_velocity(vx, vy, vz, yaw_rate)`) into low-level MAVLink messages understood by ArduPilot. It also handles telemetry polling (altitude, pitch, flight mode) and heartbeat maintenance.

#### Communication and Networking
- **`VideoStreamer`**: Handles compressing the raw OpenCV frames into JPEG and transmitting them via a low-latency TCP socket to the Ground Station.
- **`MjpegServer`**: A lightweight HTTP server that streams the video feed as MJPEG, allowing operators to monitor the drone's vision simply by opening a web browser on a phone or tablet connected to the VPN.
- **`CommandReceiver` / `CommandSender`**: Utilizes UDP datagrams to route clicks, calibration triggers, and UI interactions from the remote Ground Station back to the Jetson `MissionController` with almost zero overhead.
- **`AppConfig`**: A series of Python Data Classes that map directly to the `config.yaml` file, providing strongly-typed configuration variables (e.g., `config.control.max_vx`) throughout the codebase.

---

## Chapter 4: Operational Logic, State Machine, and Calibration

The core operational flow of the system is governed by a strict state machine to ensure safe transitions between pilot control and autonomous vision tracking. This chapter details how the software decides its current operational state, how it calibrates distances, and the safety limits imposed on the flight control algorithm.

### 4.1 Mission State Machine
The `MissionController` orchestrates the drone's behavior through four distinct states. This state machine ensures the drone never takes autonomous control unpredictably and always yields to the human pilot when necessary.

1. **`WAITING_TAKEOFF`**: Upon boot, the script enters this state and silently monitors the MAVLink telemetry. It waits for the pilot to manually arm the drone and take off. To proceed, the drone must cross a configured minimum altitude threshold and maintain a stable hover for a set duration, ensuring the physical environment is safe for tracking.
2. **`WAITING_GUIDED`**: Once a stable hover is achieved, the script waits for the pilot to flip the controller switch into `GUIDED` mode. This is the explicit permission for the software to take control. If the pilot lands or disarms before this, the system reverts to `WAITING_TAKEOFF`.
3. **`TRACKING`**: In this active state, the `MissionController` processes frames, executes the Proportional-Derivative (PD) control math, and sends physical velocity commands (`send_velocity`) to the Pixhawk. 
4. **`PAUSED`**: If the pilot flips the flight mode out of `GUIDED` (e.g., to `LOITER` or `STABILIZE`), the system detects this manual override and immediately pauses tracking. It purges the locked target from memory and stops sending commands, allowing the pilot to safely reposition the drone before resuming.

### 4.2 Live Calibration Workflow (Monocular Distance Estimation)
Because a monocular camera lacks native depth perception, the system correlates the bounding box area with physical distance using an inverse-square relationship ($D \propto 1/\sqrt{Area}$). The mathematical constant defining this relationship depends on the camera lens, resolution, and the physical size of the target.

To avoid hardcoding these optical constants, a **live calibration workflow** was developed using the `DistanceEstimator` class:
- **Safety Interlock**: Calibration requires holding the drone at a known physical distance from the target. A strict software interlock rejects any calibration requests from the Ground Station if the drone's motors are armed.
- **Non-blocking Sampling**: Once triggered, the system records the bounding box area over dozens of frames while continuing to stream video normally.
- **Fitting and Persistence**: Upon stopping, the system calculates the optimal optical constant ($K = D^2 \times \text{Average Area}$) and the Root Mean Square (RMS) error. This constant is instantly applied in memory, saved to a JSON file, and automatically written back to the system's `config.yaml` to persist across reboots.

### 4.3 Target Locking and Selection
The system supports both autonomous locking (selecting the most prominent target based on configured criteria) and manual locking. In manual mode, the operator utilizes a custom Ground Station GUI to click on a specific bounding box in the video stream, establishing a persistent tracking lock on that object ID via UDP datagrams.

### 4.4 Camera Pitch Compensation
When the drone pitches forward to accelerate, the camera points downward, artificially shifting the target in the video frame. This is addressed via two modes:
- **Software Compensation**: Applies custom trigonometric math to offset the visual error based on the drone's IMU pitch.
- **Gimbal Integration**: Detects if an active hardware gimbal is maintaining camera level, automatically disabling the software compensation.

### 4.5 Coupled Forward Velocity Limiter
A critical safety feature implemented in the control law is the Coupled Forward Velocity Limiter. The system monitors the visual centering error; if the target is outside a defined safe radius, forward velocity is clamped to zero. As the drone yaws to re-center the target, the forward velocity limit quadratically increases. This ensures the drone prioritizes rotating to face the target before physically approaching it.

### 4.6 Failsafe Protocols and Video Loss Recovery
A comprehensive fail-safe mechanism was integrated to handle video stream or network loss:
- **Hover Timeout**: If the `VideoStreamer` detects a broken TCP connection, the drone immediately halts its trajectory by sending zero-velocity commands and enters a grace period (e.g., 5 seconds) to hover in place while the socket attempts to reconnect.
- **Autonomous Landing**: If the connection is not restored within the timeout, the system issues a MAVLink land command, polling the relative altitude to perform a safe, clean software shutdown only after physical touchdown is confirmed.

---

## Chapter 5: Deployment and System Configuration

To transition the software from a development environment to a headless, autonomous edge device, several system-level configurations were established:

### 5.1 Headless Auto-Start (systemd)
To ensure the drone is instantly ready to fly upon powering up in the field, the system was configured to boot completely headlessly (without a monitor, keyboard, or mouse).
- **`tailscaled.service`**: Utilized the official background daemon provided by Tailscale to automatically establish the VPN mesh network on boot, ensuring the Jetson is instantly reachable by the Ground Station.
- **`mavproxy.service`**: A custom service that connects to the ArduPilot flight controller via serial (`/dev/ttyUSB0`) on boot. Running in daemon mode, it multiplexes the single serial connection into multiple UDP streams, allowing both the onboard Python script and the remote Ground Station to receive telemetry simultaneously.
- **`yolov26.service`**: A custom service that launches the main vision-control loop. It is configured to run under a specific user and preload critical memory libraries (`LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0`) to resolve known GStreamer TLS allocation crashes on Jetson architecture. Systemd's `journalctl` is utilized to allow operators to live-stream debug logs over the network.

---

## Chapter 6: Results and SITL Testing

Testing vision-based flight algorithms on a test bench with a physical flight controller induces Extended Kalman Filter (EKF) variance errors because the motors spin while the physical IMU detects zero movement. Therefore, all control tuning was conducted using ArduPilot SITL.

### 6.1 Inner Loop Verification (Test 1)
Verified that velocity commands sent by the companion computer translate correctly into physical movement by the simulated ArduPilot controller, minimizing actuator lag and inner-loop steady-state error.

### 6.2 Outer Loop Step Response & The D-Gain Dilemma (Test 2)
Used step responses to tune the vision-based PID loops. A critical discovery involved the Derivative (D) gain:
- **Noise Amplification**: Raw bounding box pixel coordinates contain frame-to-frame jitter. Differentiating this noisy signal for the D-gain resulted in severe motor command spikes.
- **Axis-Specific Tuning**: For the Yaw axis, overshoot is acceptable, making a well-tuned Proportional (P-only) controller optimal. For Altitude and Distance, severe overshoot could theoretically cause physical collisions, meaning damping (D-gain) is generally mandatory. However, due to the severe noise amplification caused by differentiating raw vision signals, we practically opted to carefully tune the P-gains to accept a small, safe amount of overshoot rather than implementing complex low-pass filters or non-linear Square-Root controllers at this stage.

### 6.3 Frequency Response (Test 3)
Bode plots were generated to visualize the bandwidth and phase margin of the control loops, highlighting the classic trade-off between tracking speed (bandwidth) and overshoot (resonance). This testing also exposed the severe non-linearity of using raw bounding box Area for distance control. Because Area is inversely proportional to the square of the Distance ($A \propto 1/D^2$), a small movement at close range causes a massive change in Area, while the same movement at long range causes almost no change. Consequently, a linear PID controller tuned for close-range stability becomes entirely sluggish at long ranges. This proves that the distance error signal must be mathematically linearized (e.g., converting Area to a linear "Virtual Distance" using $1/\sqrt{Area}$) to achieve consistent tracking performance across all ranges.

### 6.4 Trajectory Simulation and Noise Injection (Test 4)
Unlike previous tests that isolated individual axes, this test evaluated the full 3-axis tracking performance of the PD controller by simulating a target following predefined 2D and 3D walking trajectories. This approach captured real-world coupling dynamics (e.g., yaw-induced altitude drift and forward velocity affecting centering).
- **Scenarios Evaluated**: Tested pure 2D scenarios (like continuous circular walking) and complex 3D scenarios (like L-shape climbs, stop-and-go staircases, and approach/retreat on slopes) to force extreme coupling between lateral, forward, and vertical velocities.
- **Vision Noise Injection**: A virtual camera wrapper was developed to simulate real-world YOLO imperfections, including bounding box jitter, processing latency (frame delays), target dropouts, and tracker ID switches. By running identical trajectories with and without deterministic or random noise, we were able to objectively evaluate how visual degradation impacts tracking smoothness and validate the control algorithm's safety margins before physical flight.

---

## Conclusion

This internship successfully transformed a proof-of-concept feasibility study into a robust, deployment-ready software stack for autonomous vision-based UAV tracking on the NVIDIA Jetson Orin NX. By systematically refactoring the codebase into a modular Object-Oriented architecture, the core vision, state estimation, and control pipelines are now fully decoupled. This separation of concerns ensures the system is highly maintainable and ready for future sensor or algorithmic upgrades.

A significant achievement of this project was the development of a strict mission state machine and a live, non-blocking calibration workflow. These features ensure that the drone safely manages transitions between human pilot control and autonomous tracking, while dynamically adapting its monocular distance estimation to novel targets in the field.

Furthermore, through extensive Software-In-The-Loop (SITL) simulations, the control logic was rigorously validated across complex 3D scenarios. This testing yielded critical insights into the limitations of purely linear PID controllers when driven by noisy visual inputs, specifically highlighting the non-linear relationship between bounding box area and physical distance, and the danger of amplifying frame-to-frame detector noise through derivative gains. 

The codebase is now fully documented and features a robust triple-connection MAVLink architecture, automated headless `systemd` startup scripts, and integrated Tailscale VPN networking. With this groundwork firmly laid and structured for an easy handover, the foundation is set. For the next phase of development, the incoming team can confidently transition to physical flight tests and explore advanced Image-Based Visual Servoing (IBVS) techniques to further linearize tracking control and mitigate the inherent noise of object detection algorithms.
