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

## Chapter 3: Key Features and Control Logic

### 3.1 Target Locking and Selection
The system supports both autonomous locking (selecting the most prominent target based on configured criteria) and manual locking. In manual mode, the operator utilizes a custom Ground Station GUI to select a specific bounding box, establishing a persistent tracking lock on that object ID.

### 3.2 Monocular Distance Estimation
Because a monocular camera lacks native depth perception, the system correlates the bounding box area with physical distance using an inverse-square relationship ($D \propto 1/\sqrt{Area}$). To avoid hardcoding optical constants, a **live calibration workflow** was developed. Operators can command the drone to record area samples at a known physical distance, allowing the system to automatically compute the optical constant for novel targets.

### 3.3 Camera Pitch Compensation
When the drone pitches forward to accelerate, the camera points downward, artificially shifting the target in the video frame. This is addressed via two modes:
- **Software Compensation**: Applies custom trigonometric math to offset the visual error based on the drone's IMU pitch.
- **Gimbal Integration**: Detects if an active hardware gimbal is maintaining camera level, automatically disabling the software compensation.

### 3.4 Coupled Forward Velocity Limiter
A critical safety feature implemented in the control law is the Coupled Forward Velocity Limiter. The system monitors the visual centering error; if the target is outside a defined safe radius, forward velocity is clamped to zero. As the drone yaws to re-center the target, the forward velocity limit quadratically increases. This ensures the drone prioritizes rotating to face the target before physically approaching it.

### 3.5 Fail-Safe Protocols
A comprehensive fail-safe mechanism was integrated to handle video stream or network loss:
- **Hover Timeout**: Upon losing connection, the drone immediately halts its trajectory and enters a 5-second hover grace period while attempting socket reconnections.
- **Autonomous Landing**: If the connection is not restored, the system issues a MAVLink land command, polling the relative altitude to perform a safe, clean software shutdown only after physical touchdown is confirmed.

---

## Chapter 4: Software-In-The-Loop (SITL) Testing

Testing vision-based flight algorithms on a test bench with a physical flight controller induces Extended Kalman Filter (EKF) variance errors because the motors spin while the physical IMU detects zero movement. Therefore, all control tuning was conducted using ArduPilot SITL.

### 4.1 Inner Loop Verification (Test 1)
Verified that velocity commands sent by the companion computer translate correctly into physical movement by the simulated ArduPilot controller, minimizing actuator lag and inner-loop steady-state error.

### 4.2 Outer Loop Step Response & The D-Gain Dilemma (Test 2)
Used step responses to tune the vision-based PID loops. A critical discovery involved the Derivative (D) gain:
- **Noise Amplification**: Raw bounding box pixel coordinates contain frame-to-frame jitter. Differentiating this noisy signal for the D-gain resulted in severe motor command spikes.
- **Axis-Specific Tuning**: For the Yaw axis, overshoot is acceptable, making a well-tuned Proportional (P-only) controller optimal. For Altitude and Distance, severe overshoot could theoretically cause physical collisions, meaning damping (D-gain) is generally mandatory. However, due to the severe noise amplification caused by differentiating raw vision signals, we practically opted to carefully tune the P-gains to accept a small, safe amount of overshoot rather than implementing complex low-pass filters or non-linear Square-Root controllers at this stage.

### 4.3 Frequency Response (Test 3)
Bode plots were generated to visualize the bandwidth and phase margin of the control loops, highlighting the classic trade-off between tracking speed (bandwidth) and overshoot (resonance). This testing also exposed the severe non-linearity of using raw bounding box Area for distance control. Because Area is inversely proportional to the square of the Distance ($A \propto 1/D^2$), a small movement at close range causes a massive change in Area, while the same movement at long range causes almost no change. Consequently, a linear PID controller tuned for close-range stability becomes entirely sluggish at long ranges. This proves that the distance error signal must be mathematically linearized (e.g., converting Area to a linear "Virtual Distance" using $1/\sqrt{Area}$) to achieve consistent tracking performance across all ranges.

### 4.4 Trajectory Simulation and Noise Injection (Test 4)
Unlike previous tests that isolated individual axes, this test evaluated the full 3-axis tracking performance of the PD controller by simulating a target following predefined 2D and 3D walking trajectories. This approach captured real-world coupling dynamics (e.g., yaw-induced altitude drift and forward velocity affecting centering).
- **Scenarios Evaluated**: Tested pure 2D scenarios (like continuous circular walking) and complex 3D scenarios (like L-shape climbs, stop-and-go staircases, and approach/retreat on slopes) to force extreme coupling between lateral, forward, and vertical velocities.
- **Vision Noise Injection**: A virtual camera wrapper was developed to simulate real-world YOLO imperfections, including bounding box jitter, processing latency (frame delays), target dropouts, and tracker ID switches. By running identical trajectories with and without deterministic or random noise, we were able to objectively evaluate how visual degradation impacts tracking smoothness and validate the control algorithm's safety margins before physical flight.

---

## Chapter 5: Deployment and System Configuration

To transition the software from a development environment to a headless, autonomous edge device, several system-level configurations were established:

### 5.1 Headless Auto-Start (systemd)
To ensure the drone is instantly ready to fly upon powering up in the field, the system was configured to boot completely headlessly (without a monitor, keyboard, or mouse).
- **`tailscaled.service`**: Utilized the official background daemon provided by Tailscale to automatically establish the VPN mesh network on boot, ensuring the Jetson is instantly reachable by the Ground Station.
- **`mavproxy.service`**: A custom service that connects to the ArduPilot flight controller via serial (`/dev/ttyUSB0`) on boot. Running in daemon mode, it multiplexes the single serial connection into multiple UDP streams, allowing both the onboard Python script and the remote Ground Station to receive telemetry simultaneously.
- **`yolov26.service`**: A custom service that launches the main vision-control loop. It is configured to run under a specific user and preload critical memory libraries (`LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0`) to resolve known GStreamer TLS allocation crashes on Jetson architecture. Systemd's `journalctl` is utilized to allow operators to live-stream debug logs over the network.

---

## Conclusion
This internship successfully transformed a proof-of-concept feasibility study into a robust, deployment-ready software stack for autonomous vision-based UAV tracking on the NVIDIA Jetson Orin NX. By systematically refactoring the codebase into a modular Object-Oriented architecture, the core vision, state estimation, and control pipelines are now fully decoupled. This separation of concerns ensures the system is highly maintainable and ready for future sensor or algorithmic upgrades.

Through extensive Software-In-The-Loop (SITL) simulations, the control logic was rigorously validated across complex 3D scenarios. This testing yielded critical insights into the limitations of purely linear PID controllers when driven by noisy visual inputs, specifically highlighting the non-linear relationship between bounding box area and physical distance, and the danger of amplifying frame-to-frame detector noise through derivative gains. 

The codebase is now fully documented, featuring a robust triple-connection MAVLink architecture, automated systemd startup scripts, and integrated Tailscale VPN networking. With this groundwork firmly laid and structured for an easy handover, the foundation is set. For the next phase of development, the incoming team can confidently transition to physical flight tests and explore advanced Image-Based Visual Servoing (IBVS) techniques to further linearize tracking control and mitigate the inherent noise of object detection algorithms.
