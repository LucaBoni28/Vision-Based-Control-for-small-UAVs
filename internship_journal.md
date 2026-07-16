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
- [x] **Camera Pitch Control (Servo)**: Implement two modes for camera pitch:
  - *Manual*: Controlled via a potentiometer on the RC controller.
  - *Automatic*: System autonomously reads the current pitch and sends a correction signal through the flight controller to a self-leveling servo motor.
- [ ] **Simulation & Ground Station**: Connect a ground station software (Mission Planner) and set up a smooth, fluent 3D drone simulation in Gazebo. Optimize the Gazebo setup to run better on the struggling PC.
- [ ] **Network Stability**: Set up Tailscale to establish a fixed, static IP address connection between the Ground Station and the Jetson, avoiding the need to dynamically change IP addresses in the configuration files whenever the local network changes.

### 📝 Daily Log

#### Monday
- Successfully connected Mission Planner (Ground Station) to the real Pixhawk on the Jetson over the Wi-Fi network using UDP (after temporarily disabling Windows Firewall).
- Resolved serial port access conflicts by routing MAVLink telemetry through MAVProxy on the Jetson. The Python control script now connects to a local UDP stream (`udpin:0.0.0.0:14551`) instead of locking `/dev/ttyACM0`, allowing Mission Planner and the vision-control script to run simultaneously.

#### Tuesday
- **Video Stream Stability**: 
  - **The Issue**: The entire main vision-control loop was crashing or stalling when the Ground Station disconnected unexpectedly or network latency spiked.
  - **The Cause**: Video frames were being encoded and transmitted synchronously on the main thread. If the network socket blocked (e.g., waiting for an acknowledgment or hitting a broken pipe like `WinError 10053`), the flight control loop was completely halted, preventing the drone from updating its velocities.
  - **The Solution**: Refactored the video streaming logic to use an asynchronous background thread with a bounded queue. This completely decouples the network transmission from the flight loop, ensuring the drone's control logic always runs at its target frequency without waiting on the network.
- **Camera Pitch Control (Servo)**: Completed the software implementation of the automatic camera pitch compensation. Fixed a bug where the `MAV_CMD_DO_MOUNT_CONTROL` pitch parameter was sent in degrees instead of the centidegrees required by the ArduPilot MAVLink specification. *(Note: Still needs physical testing and validation, as the physical hardware is currently unavailable).*
- **MAVLink Telemetry Routing & Mission Planner Integration**:
  - **The Problem**: Discovered a critical behavior in MAVProxy's loop-prevention routing. When attempting to send custom `NAMED_VALUE_FLOAT` telemetry (for graphing commanded velocities like `CmdVx` against actual velocity in Mission Planner), MAVProxy was silently dropping the packets.
  - **The Cause**: MAVProxy automatically blocks packets that originate from a Ground Station/UDP port but claim to be from the drone (System ID 1), as it considers them invalid loopback packets.
  - **The Solution**: Bypassed MAVProxy's filtering entirely by establishing a dedicated secondary MAVLink connection (`self.telemetry_output`) specifically for telemetry output.
  - **Result**: The primary MAVLink connection remains dedicated to flight control (reading attitude, sending velocity targets), while the new secondary connection cleanly transmits the `NAMED_VALUE_FLOAT` custom data directly to Mission Planner for live tuning.
- **Firewall Configuration**: Created a permanent PowerShell script (`setup_firewall.ps1`) to automatically configure Windows Defender Firewall rules for the UAV application. This permanently opens the required UDP and TCP ports (UDP 14550, 14551, 14560 for MAVLink; TCP/UDP 5005, 5006 for video and command streams), eliminating the need to completely disable the Windows Firewall during field testing and improving overall system security.
#### Wednesday, July 15
- **Video Stream Loss Safety Protocol & Landing Logic**: 
  - **The Problem**: Previously, if the video stream dropped, the main control loop would block or continue flying blindly without operator feedback. 
  - **The Solution**: Implemented a comprehensive fail-safe mechanism in `mission_controller.py`. The stream connection check (`streamer.is_connected`) is now evaluated in a non-blocking manner within the main 30Hz loop. 
  - **Hover Timeout**: If the connection drops, the drone immediately halts its current trajectory by sending zero-velocity commands (`send_stop()`). It enters a 5-second grace period where it hovers in place while the `VideoStreamer` background thread attempts socket reconnections (with a `0.5s` socket timeout to prevent thread blocking).
  - **Autonomous Landing**: If the connection is not restored within 5 seconds, the `MissionController` issues a `MAV_CMD_NAV_LAND` command (via `command_long_send` with confirmation) to the flight controller. 
  - **Safe Shutdown Sequence**: Crucially, the script does not exit immediately after sending the land command, as this would terminate the control loop mid-air. Instead, it enters a while-loop that polls the drone's `GLOBAL_POSITION_INT` MAVLink message. It waits until the relative altitude (`msg.relative_alt / 1000.0`) drops below `0.5m`, confirming the drone has physically touched the ground, before performing a clean shutdown of all threads and exiting.
- **SITL Simulation Overrides for Lab Testing**: 
  - **The Problem**: Testing the new altitude-based landing protocol inside the lab is difficult because the physical Pixhawk (sitting on a desk) constantly reports a relative altitude of `0m`. When the land command is issued, the script immediately detects `alt < 0.5m` and exits, making it impossible to verify the descent logic. 
  - **The Workaround**: Configured the script to interact simultaneously with the physical Pixhawk (`master` on UDP) and Mission Planner's SITL simulation (`telemetry_output` on TCP). 
  - **Command Mirroring**: Modified `flight_controller.py` so that `send_velocity()` (sending `SET_POSITION_TARGET_LOCAL_NED`) and `send_land()` are mirrored to both the physical drone and the SITL connection. This allows the simulated drone in Mission Planner to physically move in response to the script's commands.
  - **Altitude Hijacking**: Upgraded the `poll_relative_alt()` method to request the `MAV_DATA_STREAM_POSITION` stream from the SITL connection as well. It now aggressively drains the message buffer of the `telemetry_output` socket using `recv_match(type="GLOBAL_POSITION_INT", blocking=False)` in a `while True` loop to ensure we always have the freshest data without lag. If it successfully reads a simulated altitude, it sets a persistent flag (`_sitl_alt_active = True`). From that point forward, the script completely ignores the physical drone's `0m` altitude and bases all landing checks on the SITL's altitude. This allows the entire 5-second hover and descent sequence to be verified visually and safely in the Mission Planner simulator.
  - **Recovery Validation**: Tested and verified that if the video stream drops but is manually restarted *before* the 5-second hover timeout expires, the system successfully aborts the landing sequence and seamlessly resumes normal flight tracking.
#### Thursday
- 

#### Friday
- 

### 💡 Notes for Final Report
- **Physical Hardware vs. Simulation (The EKF Conflict)**: We initially attempted to simulate flight movements (faking a GPS signal) while the physical Pixhawk was sitting on a test bench. However, this is fundamentally flawed due to the flight controller's Extended Kalman Filter (EKF). When the control algorithm commands a velocity, the motors spin up and the fake GPS reports movement. BUT, the physical IMU (accelerometer/gyro) on the bench reports zero acceleration. The EKF instantly detects this contradiction as a massive sensor variance error, triggering a failsafe and rejecting further autonomy commands.
- **The Solution (SITL)**: The correct approach to simulate flight paths and test vision-control logic without a real flight is to use **Software-In-The-Loop (SITL)**. SITL entirely replaces the physical Pixhawk with a virtual one. Because the virtual Pixhawk simulates *all* sensors (GPS, IMU, Baro, Compass) in perfect unison based on the physics engine, the EKF remains stable. We can run this lightweight SITL directly inside Mission Planner and route the Jetson's MAVLink connection to it via TCP/UDP over the network, allowing full system testing without hardware conflicts.
