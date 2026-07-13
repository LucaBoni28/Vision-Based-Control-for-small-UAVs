# Internship & Thesis Journal
**Project**: Vision-Based Control for small UAVs

This journal is used to track daily progress, challenges, and solutions to help with the final internship/thesis report.

---

## Week 1 (July 6 - July 12, 2026)

### 🎯 Weekly Goals
- [ ] Define control logic improvements
- [ ] 

### 📝 Daily Log

#### Monday, July 6
- Restructured the codebase into an Object-Oriented Programming (OOP) architecture.
- Created core class modules (`camera`, `detector`, `tracker`, `flight_controller`, `mission_controller`, etc.) in a new `classes/` folder.
- Migrated hard-coded scripts into the new OOP framework.

#### Tuesday, July 7
- **Target Selection Logic**: Removed the old Euclidean distance-based threshold (`max_click_distance_px`). Now, the system simply checks if the operator's click coordinates fall inside a detected bounding box (`x1 <= click_x <= x2` and `y1 <= click_y <= y2`). If multiple boxes overlap, it selects the one with the smallest area.
- **Live Calibration Preview**: Overhauled the calibration process in `distance_estimator.py` to use a multithreaded live preview. The camera now streams continuously on a background thread while the main thread handles user input (`ENTER` to start/stop recording), saving per-frame area samples instead of just an average.

#### Wednesday, July 8
- **Drone Simulator**: Created `pc_test/drone_simulator.py`, an interactive Tkinter/Matplotlib GUI application. It models drone kinematics, actuator lag, virtual camera perspective projection, and visual tracking errors. This allows for rapid testing and tuning of PD, PID, and Sliding Mode Control (SMC) strategies without flying the actual hardware.

#### Thursday, July 9
- *Started journal to track progress*
- **Calibration Safety Interlocks**: Added robust safety checks to prevent calibrating while the drone is flying. Created a new network command (`CMD_CALIBRATE_CHECK`, `0x06`) so the Ground Station can ask the Jetson if calibration is allowed. If the drone is `ARMED`, it rejects the request and displays a massive red "WARNING: DRONE ARMED! DISARM TO CALIBRATE" overlay on the video feed.
- **Target Lock Requirement**: The calibration process now waits for a target to be locked (`self._locked_id`) before recording samples. It also correctly uses the tracked target's specific area rather than just blindly picking the largest detection in the frame.
- **Config Fixes**: Fixed a naming mismatch in the configuration (`fallback_optical_constant` was changed to `optical_constant` in `config.yaml` and `config.py`).
- **Local Testing Environment**: Implemented a complete local testing pipeline in the untracked `pc_test/` folder.
  - `test_local.py`: Runs the full vision pipeline on a PC using a webcam (`WebcamCameraSource`) and a dummy flight controller (`DummyFlightController`), bypassing the need for a Jetson Orin NX or MAVLink connection.
  - `stream_video_local.py`: A local version of the Ground Station streaming script that receives the processed video stream on `localhost`, processes mouse clicks for target selection, and handles remote calibration.

#### Friday, July 10
- Set up the hardware on a wooden sheet in order to test the system around the lab with a real Pixhawk flight controller as soon as the router/wifi module is ready.
- Already verified the setup using an Ethernet connection, and it works successfully.

### 💡 Notes for Final Report
*Jot down architectural decisions, key learnings, or figures you might want to include in the LaTeX report later.*
- 

---

## Week 2 (July 13 - July 19, 2026)

### 🎯 Weekly Goals
- [x] **Headless Jetson Logging**: Forward console logs/outputs from the Jetson to the Ground Station software since the Jetson will run headless in the real application.
- [ ] **Camera Pitch Control (Servo)**: Implement two modes for camera pitch:
  - *Manual*: Controlled via a potentiometer on the RC controller.
  - *Automatic*: System autonomously reads the current pitch and sends a correction signal through the flight controller to a self-leveling servo motor.
- [ ] **Simulation & Ground Station**: Connect a ground station software (Mission Planner) and set up a smooth, fluent 3D drone simulation in Gazebo. Optimize the Gazebo setup to run better on the struggling PC.

### 📝 Daily Log

#### Monday
- Successfully connected Mission Planner (Ground Station) to the real Pixhawk on the Jetson over the Wi-Fi network using UDP (after temporarily disabling Windows Firewall).
- Resolved serial port access conflicts by routing MAVLink telemetry through MAVProxy on the Jetson. The Python control script now connects to a local UDP stream (`udpin:0.0.0.0:14551`) instead of locking `/dev/ttyACM0`, allowing Mission Planner and the vision-control script to run simultaneously.

#### Tuesday
- 

#### Wednesday
- 

#### Thursday
- 

#### Friday
- 

### 💡 Notes for Final Report
- **Physical Hardware vs. Simulation (The EKF Conflict)**: We initially attempted to simulate flight movements (faking a GPS signal) while the physical Pixhawk was sitting on a test bench. However, this is fundamentally flawed due to the flight controller's Extended Kalman Filter (EKF). When the control algorithm commands a velocity, the motors spin up and the fake GPS reports movement. BUT, the physical IMU (accelerometer/gyro) on the bench reports zero acceleration. The EKF instantly detects this contradiction as a massive sensor variance error, triggering a failsafe and rejecting further autonomy commands.
- **The Solution (SITL)**: The correct approach to simulate flight paths and test vision-control logic without a real flight is to use **Software-In-The-Loop (SITL)**. SITL entirely replaces the physical Pixhawk with a virtual one. Because the virtual Pixhawk simulates *all* sensors (GPS, IMU, Baro, Compass) in perfect unison based on the physics engine, the EKF remains stable. We can run this lightweight SITL directly inside Mission Planner and route the Jetson's MAVLink connection to it via TCP/UDP over the network, allowing full system testing without hardware conflicts.
