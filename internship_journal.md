# Internship & Thesis Journal
**Project**: Vision-Based Control for small UAVs

This journal is used to track daily progress, challenges, and solutions to help with the final internship/thesis report.

---

## Week 1 (July 6 - July 12, 2026)

### 🎯 Weekly Goals
- [x] Restructure codebase into a modular OOP architecture.
- [x] Implement target selection logic and live calibration preview.
- [x] Develop a 2D Kinematic Drone Simulator for control logic validation.

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
- **OOP Refactoring Benefits**: Transitioning from linear scripting to a modular OOP architecture (`CameraSource`, `Detector`, `Tracker`, `FlightController`, `MissionController`) decoupled vision processing from flight control. This separation of concerns allows hot-swapping vision backends (e.g. switching from CPU PyTorch to TensorRT ONNX) or switching from a physical camera stream to simulated inputs without altering any control logic.
- **Simulator-in-the-Loop Validation**: Developing a standalone simulator prior to hardware integration was critical. It allowed validation of control laws (PD/SMC) against simulated physical lag and tracking error before dealing with flight controller EKF constraints.

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
#### Friday, July 17
- **Mission Planner Telemetry Integration**: Successfully configured the system to make velocity commands (`SET_POSITION_TARGET_LOCAL_NED`) permanently visible in the Mission Planner MAVLink inspector.
- **MAVLink Routing Fixes**: Identified that Mission Planner drops point-to-point commands addressed to the autopilot. Solved this by configuring the `telemetry_output` connection to broadcast the velocity commands (`target_system=0`, `target_component=0`), ensuring Mission Planner receives and displays them without the physical drone executing them twice.
- **SITL Configuration Toggle**: Added a `sitl` boolean flag to `config.yaml` to decouple the SITL simulation overrides from the Mission Planner telemetry connection. The Ground Station can now receive mirrored commands for inspection even when the simulation is fully disabled.
- **MAVProxy Heartbeat Resolution**: Diagnosed an issue where `wait_heartbeat()` was latching onto Component `0` instead of `1`. This was caused by MAVProxy generating local UDP keep-alive heartbeats that arrive milliseconds before the Pixhawk's serial heartbeat.

### 💡 Notes for Final Report
- **Physical Hardware vs. Simulation (The EKF Conflict)**: We initially attempted to simulate flight movements (faking a GPS signal) while the physical Pixhawk was sitting on a test bench. However, this is fundamentally flawed due to the flight controller's Extended Kalman Filter (EKF). When the control algorithm commands a velocity, the motors spin up and the fake GPS reports movement. BUT, the physical IMU (accelerometer/gyro) on the bench reports zero acceleration. The EKF instantly detects this contradiction as a massive sensor variance error, triggering a failsafe and rejecting further autonomy commands.
- **The Solution (SITL)**: The correct approach to simulate flight paths and test vision-control logic without a real flight is to use **Software-In-The-Loop (SITL)**. SITL entirely replaces the physical Pixhawk with a virtual one. Because the virtual Pixhawk simulates *all* sensors (GPS, IMU, Baro, Compass) in perfect unison based on the physics engine, the EKF remains stable. We can run this lightweight SITL directly inside Mission Planner and route the Jetson's MAVLink connection to it via TCP/UDP over the network, allowing full system testing without hardware conflicts.
- **MAVLink Network Architecture (Identities & Connections)**: 
  A crucial aspect of multi-device UAV networks is the distinction between identities and connections. In MAVLink, physical devices and connections are two different concepts.

  #### 1. The Identities (System ID & Component ID)
  In MAVLink, every piece of hardware or software is a "node" identified by a `(System_ID, Component_ID)` pair. 
  *   **System ID** represents a single vehicle or a completely separate entity.
  *   **Component ID** represents a specific subsystem *inside* that vehicle.

  For our physical setup:
  1.  **Pixhawk (Autopilot)**
      *   **System ID:** `1` (It is the core of Vehicle 1)
      *   **Component ID:** `1` (Standard ID for the main flight controller)
  2.  **Jetson (Companion Computer)**
      *   **System ID:** `1` (Because it is physically mounted on and part of Vehicle 1)
      *   **Component ID:** `191` (Standard ID for an onboard companion computer)
      *   *Note: Any message the Python script generates is stamped with `1:191` so everyone knows who sent it.*
  3.  **PC (Mission Planner)**
      *   **System ID:** `255` (The standard ID reserved for Ground Control Stations)
      *   **Component ID:** `190` or `0` (Standard ID for Mission Planner)

  #### 2. The Connections (`master` and `telemetry_output`)
  Connections are just the "wires" (or TCP/UDP sockets) connecting these identities together. The Jetson is currently talking on two different wires:

  *   **`master` Connection (Jetson <--> Pixhawk)**
      This wire connects the Python script (1:191) to the Pixhawk (1:1). When we want the drone to move, we send `mav.set_position_target_local_ned_send` over this wire, specifically addressed to **Target 1:1**. The Pixhawk reads it and executes the movement.
  *   **`telemetry_output` Connection (Jetson <--> PC/Mission Planner)**
      This wire is a direct TCP tunnel between the Python script (1:191) and Mission Planner (255:190). Because we want to visualize the velocities in Mission Planner, the Python script sends a *copy* of the velocity command over this wire. By addressing the copy to **Target 0:0** (Broadcast), Mission Planner happily receives it and displays it in the MAVLink inspector under the Jetson's identity (Source 1:191). Additionally, the Jetson must send `HEARTBEAT` messages over this link to prevent Mission Planner from marking the `1:191` component as disconnected.

---

## Week 3 (July 20 - July 26, 2026)

### 🎯 Weekly Goals
- [x] Analyze flight controller communication sequence.
- [x] Prepare for upcoming flight tests.

### 📝 Daily Log

#### Monday, July 20
- **Flight Controller Communication Analysis**: Conducted a deep dive into the MAVLink connection sequence inside `flight_controller.py`. Specifically mapped out the control flow immediately following a successful heartbeat from the physical drone:
  1. The system attempts to establish a secondary `telemetry_output` connection for Ground Station/SITL routing.
  2. It explicitly requests the `MAV_DATA_STREAM_EXTRA1` message stream from the physical drone at the configured `attitude_stream_rate_hz` to receive continuous roll, pitch, and yaw updates.
  3. If SITL simulation is enabled (`self._config.sitl`), it separately requests the `MAV_DATA_STREAM_POSITION` stream from the telemetry connection, bypassing the physical drone's GPS/Barometer to rely purely on simulated altitude/position data.
- **Dynamic Telemetry Routing Refactoring**: Refactored `config.py` and `config.yaml` to dynamically generate the `telemetry_output` connection string. It now uses a centralized `ground_station_ip` property alongside the `sitl` flag to automatically construct either a TCP connection (`tcp:{ip}:5762` for SITL) or a UDP broadcast (`udpout:{ip}:14560` for standard telemetry).

#### Tuesday, July 21
- **Configuration Documentation**: Added comprehensive descriptions to parameters in `config.yaml` to clarify the purpose and impact of camera settings, MAVLink connections, tracker configurations, and PID control values.
- **Drone Automation & Safety Logic**: Implemented robust safety preconditions for mission initialization in `mission_controller.py` and `flight_controller.py`. The control loop will now only start if the drone is in GUIDED mode and has been stably hovering at an altitude over 3 meters for at least 3 seconds.
- **SITL Telemetry Stability**: Improved the secondary MAVLink connection (`telemetry_output`) handling in `flight_controller.py`. Added logic to detect dead TCP sockets to prevent terminal spam (`EOF on TCP socket`) when Mission Planner is closed. Implemented a 5-second auto-reconnect loop so the Python script can seamlessly resume telemetry and position syncing as soon as Mission Planner is restarted.
- **Override Log Refinement**: Reworded the log message in `mission_controller.py` from "Emergency stop" to "Manual override detected" when a flight mode change pauses the mission, accurately reflecting that the system is simply yielding control back to the operator rather than triggering an emergency fail-safe.

#### Thursday, July 23
- **Headless & Auto-Start Configuration Documentation**: Added configuration instructions in `README.md` referencing systemd configuration for screenless and internet-free auto-starting. Highlighted the role of `fake-hwclock` (crucial for Tailscale startup when offline) and setting up udev rules to prevent `ModemManager` from locking the `/dev/ttyUSB0` serial line.
- **Interactive Mission Start & Pilot Control**: Refactored the takeoff/mission transition flow to improve safety. Instead of automatically commanding GUIDED mode upon detecting a stable hover, the controller now displays a status prompt to the pilot ("Switch to GUIDED to start mission") and enters a `WAITING_GUIDED` state. The controller begins tracking only after the pilot manually toggles the mode switch.
- **Flight Mode Safety Logic & Error Handling**:
  - Implemented fallbacks: if the drone descends below minimum takeoff altitude or disarms during the `WAITING_GUIDED` state, the mission controller reverts to `WAITING_TAKEOFF`.
  - Guarded tracking/paused override logic to ignore flight mode changes if the video stream is lost, preventing the state machine from getting stuck in `PAUSED` when a landing sequence is already active.
  - Refined the landing loop to detect when the pilot manually takes back control (switching out of `LAND` mode to a non-unknown mode after landing mode is active), causing the mission controller to exit with code `1` to trigger a clean `systemd` restart.
- **Robust Telemetry EOF Detection & Reconnection**: Improved TCP/UDP telemetry reconnection by implementing low-level socket checks (`select` + `MSG_PEEK` to test for socket EOF / TCP disconnect) to prevent `recv_match` from silently hanging. Expanded the 5-second auto-reconnection logic to handle all telemetry configurations, resetting SITL override flags during reconnection to prevent stale state usage.

### 💡 Notes for Final Report
- **Control Law Architecture & Coupled Velocity Limiting**: A critical detail for the thesis is the final control logic implemented in `mission_controller.py`. While the Python simulator (built in Week 1) was used to test SMC and PID, the actual edge deployment utilizes a multi-axis **PD Controller**. A vital safety and tracking feature is the **Coupled Forward Velocity Limiter** (`v_x_limit`). The system continuously monitors the visual centering error (`e_mag`). If the target is outside a defined safe radius (`r_stop = 0.8`), the drone zeroes its forward velocity (`v_x = 0`). As the drone yaws to re-center the target, the forward velocity limit quadratically increases (`1 - e_scaled**2`). This ensures the drone *prioritizes rotating to face the target* before it is allowed to physically approach it, preventing fly-bys. Action deadzones (`yaw_deadzone`, `vz_deadzone`, `area_deadzone`) were also added to eliminate micro-oscillations caused by bounding box pixel-jitter.

---

## Week 4 (July 27 - August 2, 2026)

### 🎯 Weekly Goals
- [x] Improve safety during state transitions.
- [x] Clean up and heavily document the main control logic for the thesis report.

### 📝 Daily Log

#### Monday, July 27
- **Pause State Altitude Safety Check**: Modified the `PAUSED` state logic in `mission_controller.py`. When the pilot resumes the mission by switching back to `GUIDED` mode, the system now checks if the drone is still above the minimum takeoff altitude. If it is too low (e.g., if the drone was manually landed while paused), it refuses to track and safely falls back to the `WAITING_TAKEOFF` state.
- **Mission Controller Documentation**: Performed a comprehensive cleanup of `mission_controller.py`. Added extensive inline comments, Python docstrings, and a numbered breakdown of the `_process_frame` PD math logic. This restructuring will make it significantly easier to present and explain the system architecture in the final thesis defense.

#### Wednesday, July 29
- **MJPEG HTTP Streaming Support**: Integrated a new `MjpegServer` class into `video_streamer.py` and `main.py` which runs a background HTTP server. This allows for low-latency streaming of the drone's camera feed directly to a phone or web browser over a local network.
- **Initialization Order Fixes**: Refactored the camera and streamer initialization order in `main.py`. The `CSICameraSource` is now initialized last to prevent GStreamer buffer overflow crashes during startup.
- **Git Repository Recovery**: Successfully recovered the local git repository and uncommitted index changes after a local git object file corruption.

---

## Week 5 (August 3 - August 9, 2026)

### 🎯 Weekly Goals
- [x] Tune the PID gains for the UAV's visual tracking using SITL step response tests.
- [x] Analyze automated grid search and manual tuning data to optimize tracking performance.

### 📝 Daily Log

#### Sunday, August 9
- **Yaw PID Tuning & Step Response Analysis**: 
  - Ran extensive step response tests (both automated grid sweeps and manual tuning) to find the optimal PID gains for the yaw axis.
  - Developed analysis tools to evaluate Rise Time, Overshoot, Settling Time, and Steady-State Error.
- **The D-Gain Problem (Noise Amplification)**:
  - **Observation**: Discovered that adding any derivative (D) gain to the yaw controller resulted in highly erratic and "spiky" yaw rate commands (rapidly saturating at ±1.0 rad/s), which is infeasible for actual hardware and would cause severe motor chatter.
  - **Root Cause**: The controller computes the derivative as a raw finite difference `(e_x - prev_e_x) / dt` on the vision bounding box position. Vision detection inherently has frame-to-frame pixel jitter (detector/tracker noise). Differentiating this raw, unfiltered noisy signal drastically amplifies high-frequency noise.
  - **Decision**: Concluded that the D-gain is actually detrimental to yaw control in the current implementation without a low-pass filter. Since yaw is the least safety-critical axis (slight overshoot horizontally is acceptable), we opted for a purely Proportional (P-only) control strategy.
- **Altitude (Vz) & Distance (Vx) PID Tuning**:
  - Transitioned from Yaw tuning to Altitude and Distance tuning using the SITL step response environment.
  - Unlike Yaw, P-only controllers for Altitude and Distance resulted in unacceptable and dangerous overshoot (e.g., oscillating past the target altitude or flying past the target distance).
- **The D-Gain Dilemma (Safety vs. Noise)**:
  - **Yaw (Horizontal)**: Overshoot is acceptable because passing the target horizontally simply requires a turn to correct; it doesn't cause a crash. Therefore, P-only is fine and avoids noise amplification.
  - **Altitude (Vertical) & Distance (Forward)**: Overshoot is catastrophic. An altitude overshoot means crashing into the ground, and a distance overshoot means ramming into the tracked target. Therefore, **D-gain is absolutely mandatory** to provide the necessary damping and braking force as the drone approaches the target state.
  - **The Problem**: Using D-gain on these axes re-introduces the severe noise amplification problem caused by raw vision bounding box jitter, creating "spiky" velocity commands that will wear out motors or destabilize the flight controller.
- **Future Mitigation Strategies Discussed**:
  - To safely utilize D-gain without amplifying noise, two primary solutions were identified:
    1.  **Low-Pass Filter (LPF)**: Implementing a software low-pass filter (like a first-order IIR or moving average) on the vision error signals *before* calculating the derivative, effectively smoothing the frame-to-frame quantization noise.
    2.  **Square-Root Controller**: Replacing the linear P-controller with a square-root controller (similar to ArduPilot's internal `sqrt_controller`). This provides high gain when far away for speed, but naturally tapers off the gain as the error approaches zero, providing a smooth, critically damped deceleration without relying on a noisy derivative term.

### 💡 Notes for Final Report
- **PID Tuning for Vision-Based Control**: When tuning PID loops that rely on computer vision for error estimation, the derivative term (D) cannot be used naively. Because vision bounding boxes inherently jitter from frame to frame, a raw derivative `de/dt` acts as a high-pass filter, amplifying this noise into massive command spikes. To use D-gain effectively in vision applications, the derivative term must be processed through a low-pass filter (e.g., a first-order IIR filter) to smooth out frame-to-frame quantization noise before it enters the control equation. For less safety-critical axes like yaw, a well-tuned P-only controller often provides a smoother, more robust, and hardware-friendly response than an unfiltered PD controller.
- **Axis-Specific Control Strategies**: The severity of this noise problem dictates different control strategies per axis. For less safety-critical axes like Yaw, where overshoot does not cause collisions, a well-tuned P-only controller provides a smoother, more robust response. However, for critical axes like Altitude and Distance, overshoot implies a physical collision (ground or target). Here, the damping effect of the D-gain is mandatory. To achieve this safely on hardware, the raw vision signal must either be passed through a Low-Pass Filter prior to differentiation, or the entire control law must be swapped to a non-linear approach like a Square-Root Controller to achieve natural braking without a derivative term.

---

## Week 6 (August 10 - August 16, 2026)

### 🎯 Weekly Goals
- [x] Perform frequency response (Bode plot) analysis on the tracking loops to determine bandwidth and phase margin.
- [x] Correlate frequency-domain metrics (bandwidth/resonance) with time-domain metrics (rise time/overshoot).

### 📝 Daily Log

#### Monday, August 10
- **Frequency Response Simulation Fixes**: 
  - **Yaw Target Motion**: Discovered that moving the target along the global Y-axis caused purely forward/backward motion if the drone spawned facing West. Fixed the simulator to move the target laterally relative to the drone's actual heading using a rotation matrix.
  - **Distance Target Motion**: Found that the drone was saturating its velocity because it was trying to reach a default 0.6m stopping distance while the target spawned at 10m. Temporarily overrode the stopping distance to 10m during distance tests to allow it to track the sine wave properly.
- **Bode Plot Generation Pitfalls**:
  - Fixed a "negative amplitude" bug in the SciPy curve fitting script that caused artificial 180-degree jumps in the phase plot.
  - Implemented phase unwrapping to prevent the phase from teleporting when physical lag drops below -180 degrees.
- **Velocity Saturation (Non-Linearity)**:
  - Realized that running a 3.0m amplitude frequency sweep for the distance axis at 0.5 Hz requires a tracking speed of >9.0 m/s. 
  - Since the drone's `max_vx` is capped at 1.5 m/s, it heavily saturated, invalidating the Bode plot. We learned that frequency response tests must use small amplitudes (e.g., 0.5m for distance) to keep the drone operating in its linear region.
  - *Note*: Yaw was unaffected by the 3.0m amplitude because a 3m lateral shift at 10m distance is only 0.29 radians, keeping angular velocity well below the 1.0 rad/s limit.

### 💡 Notes for Final Report
- **The Damping vs. Bandwidth Trade-off**: The frequency response tests beautifully visualized the classic control theory trade-off between speed and stability. In the time-domain (Step Response), a low Proportional gain (e.g., $K_p = 0.3$ for distance) provided excellent damping and prevented overshoot. However, the frequency-domain (Bode Plot) revealed the cost of this damping: extreme sluggishness. The system had a DC gain of -13 dB, meaning it severely under-responded to continuous target movement. If we increase $K_p$ to achieve a wider bandwidth (faster tracking), we will inevitably see a **Resonance Peak** in the Bode plot, which is the exact frequency-domain equivalent of **Overshoot** in the step response. 
- **Linearity in Bode Plots**: When evaluating control loops via simulation, it is critical to ensure the system remains in its linear region. If the test stimulus (e.g., sine wave amplitude) demands a physical response that exceeds the hardware's saturation limits (like maximum velocity or acceleration), the resulting Bode plot will falsely indicate terrible phase margin and low bandwidth. This is not a failure of the PID tuning, but a failure of the test methodology. Stimulus amplitudes must be calculated mathematically to stay under saturation limits prior to testing.
- **The Non-Linearity of Vision-Based Distance**: A fundamental discovery was made regarding the use of bounding box Area as a proxy for Distance. Because Area is inversely proportional to the square of the Distance ($A \propto 1/D^2$), the mapping between physical distance error and area error is highly non-linear. At a close hovering distance (e.g., 0.6m), a 1.0m movement causes a massive 86% change in area. At a far hovering distance (10.0m), a 0.5m movement causes only a 10% change in area. Consequently, a linear PID controller tuned perfectly for close-range stability ($K_p = 0.3$) becomes 100x mathematically weaker and entirely sluggish at long ranges. This non-linearity proves that for advanced vision-based control, the distance signal must be linearized (e.g., estimating actual meters using $\sqrt{Area}$) rather than passing raw Area into a linear PID loop.
- **Future Work - Image-Based Visual Servoing (IBVS)**: To solve the non-linearity of distance tracking without knowing the true physical size of the target, a "Virtual Distance" metric can be introduced. By taking the inverse square root of the bounding box area ($D_v = 1/\sqrt{Area}$), we obtain a signal that is perfectly linear with physical distance (i.e., $D_v \propto D_{physical}$). Calculating the error based on this virtual distance ($e = D_{v,target} - D_{v,current}$) ensures the tracking controller behaves uniformly at all distances. This elegantly eliminates the need for complex gain scheduling or noisy derivative (D) gains for non-linear stabilization, allowing a simple Proportional controller to track targets safely and consistently across the entire operating range.
