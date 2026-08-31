# Vision-Based Control for UAVs

![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)
![Status](https://img.shields.io/badge/status-active-success.svg)

> **Project Evolution:** This repository originated as an MSc Degree Thesis focusing on the feasibility of vision-based control methods for *small-scale drones* (the original thesis work is preserved in the `master` branch). During an ongoing internship, the project has evolved on this branch into a robust, real-world application designed to be deployed and tested on a larger, custom-developed UAV built by a separate team.

This repository contains the software implementation for an autonomous vision-based control system. The system enables real-time target tracking and flight control using relative visual positioning. It runs on an NVIDIA Jetson Orin NX companion computer alongside an ArduPilot-based flight controller.

---

## What the Program Does

This software empowers a UAV to autonomously identify, track, and physically follow a visual target (such as a person or a specific object). It operates in real-time entirely on edge-hardware (NVIDIA Jetson). 

The system calculates the relative offset of the target from the center of the camera frame and translates these visual offsets into flight commands (rotational and translational velocities) to keep the drone centered on the target.

---

## Architecture & Repository Structure

The project features a highly modular Object-Oriented Programming (OOP) architecture for real-world deployment. The core logic flows from **Vision** (seeing the target) -> **State Estimation** (understanding where it is) -> **Control** (moving the drone).

- **`main.py`**: The primary application running on the **NVIDIA Jetson**. It initializes hardware, runs the real-time YOLO detection/control loop, and dispatches MAVLink flight commands.
- **`stream_video.py`**: The Ground Station client application. It receives the live video feed, provides a GUI for operators, handles mouse clicks for target locking, and manages remote distance calibration.

### Core Modules (`classes/`)
- **Configuration**: `config.yaml` & `config.py` - Centralized parameters for PID gains, network IPs, camera settings, and pitch compensation modes.
- **Vision & Perception**: 
  - `camera.py`: Interface for the hardware CSI camera.
  - `detector.py`: YOLO object detection wrapper.
  - `tracker.py`: Object tracking (ByteTrack/DeepSORT) to maintain IDs across frames.
- **State Estimation**: 
  - `target_selector.py`: Logic for deciding which tracked object to follow (e.g., operator manual click).
  - `distance_estimator.py`: Translates 2D bounding boxes into physical 3D distance estimates using a calibrated optical constant.
- **Flight & Control**:
  - `flight_controller.py`: Manages MAVLink communication (arming, takeoff, telemetry polling).
  - `mission_controller.py`: The main "Brain". Reads vision data, runs PID control math, applies pitch compensation, and commands the `flight_controller`.
- **Networking**: `video_streamer.py` & `click_command.py` - Manages TCP video streaming and UDP command routing.

---

## Software-In-The-Loop (SITL) Testing

Before deploying to the physical drone, you can evaluate the control algorithms using the **`sitl_tests/`** directory. This allows for rapid testing and tuning of PID controllers without hardware risk. 

The `sitl_tests/` folder is structured into specific test suites:
- **`test_1_inner_loop/`**: Validates the low-level ArduPilot flight controller responses, ensuring that velocity commands sent by the companion computer translate correctly into physical movement without excessive actuator lag.
- **`test_2_step_response/`**: Time-domain analysis for the vision-based PID loops (Altitude, Distance, Yaw). Used extensively to tune gains, evaluate rise time/overshoot, and analyze the impact of vision-signal noise on Derivative (D) gains.
- **`test_3_frequency_response/`**: Bode plot generation to evaluate the system's bandwidth and phase margin. This visualizes the trade-off between tracking speed and resonance, while also highlighting the non-linear math of using bounding box area for distance estimation.
- **`test_4_trajectory/`**: Evaluates how smoothly and accurately the drone can track a target moving through complex, multi-axis paths over time (e.g., sine waves).
- **`utils/`**: Shared helper scripts, plotting functions, and math utilities used across all the test suites to visualize simulation data.

*Note: Detailed execution commands and explanations for each test are provided in dedicated `README.md` files located inside each specific test folder.*

---

## Key Features & Operations

### 1. Target Locking Modes
Target selection logic is governed by the `config.yaml` file, supporting both autonomous and manual modes:
- **Auto-Lock**: The system automatically locks onto the most prominent target as soon as it enters the frame, based on configurable criteria.
- **Manual Lock**: The system detects objects but waits for operator confirmation. The operator uses the `stream_video.py` Ground Station GUI to click on a specific bounding box, telling the Jetson to lock onto that specific ID and begin tracking.

### 2. Monocular Distance Estimation
Because a single 2D camera cannot natively determine depth, the system relies on an `optical_constant` to correlate the bounding box pixel area with physical distance ($D = \sqrt{K/Area}$). 
While you can hardcode this constant in the configuration, the system features a robust **live calibration workflow**. Through the Ground Station, an operator can command the drone to record area samples at a known physical distance. The system will automatically calculate the precise optical constant and sync it to `config.yaml`. This is essential because it allows the controller to accurately track novel objects of various sizes without requiring the developer to do manual math.

### 3. Camera Pitch Compensation
When the drone pitches forward to fly, the camera points downward, artificially shifting the target in the image. The system handles this via `pitch_compensation` in `config.yaml`:
- **`software`**: Applies a custom mathematical approximation (trigonometry) to offset the visual error based on the drone's IMU pitch. This is a custom software fallback used when the physical gimbal's hardware stabilization is disabled.
- **`gimbal_auto` / `gimbal_manual`**: Tells the software *not* to apply math, because a physical gimbal (controlled by Pixhawk or RC) is already keeping the camera level.

---

## Setup & Execution

### Hardware & Environment Setup
- Connect your camera to the Jetson Orin NX CSI port.
- Connect the Jetson to the **ArduPilot** flight controller via a UART serial connection or USB.
- Ensure **Python 3.8+** is installed on the Jetson.

### System Dependencies
Install all required Python libraries for the vision pipeline, tracking, and telemetry routing:
```bash
pip install MAVProxy ultralytics pymavlink opencv-python pyyaml
```

### Tailscale VPN (Networking)
To ensure reliable communication between the drone and the Ground Station, **Tailscale** is used to create a secure VPN mesh network. This allows seamless video streaming and command routing from anywhere.
1. Install Tailscale on both devices (`curl -fsSL https://tailscale.com/install.sh | sh`).
2. Authenticate by running `sudo tailscale up`.
3. Obtain the Tailscale IP of each device (`tailscale ip -4`) for use in `config.yaml`.

### MAVProxy & Telemetry Routing
To allow both the Python script AND Mission Planner to read drone telemetry simultaneously, use **MAVProxy** as a central router on the Jetson.

**Run MAVProxy:**
```bash
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14551 --out=udp:100.x.x.x:14550
```
*(Replace `100.x.x.x` with the Ground Station Tailscale IP, and `/dev/ttyACM0` with the flight controller port).*

### Generate TensorRT Engine File
For real-time performance on the Jetson, convert the PyTorch YOLO model to TensorRT:
```bash
yolo export model=yolo26n.pt format=engine
```

### Option 1: Manual Execution
1. **Configure**: Update `classes/config.yaml` with the correct IPs, ports, and `.engine` model path.
2. **Start Ground Station**: Run `python stream_video.py` on your remote PC.
3. **Start Drone**: Run `python main.py` on the Jetson.

### Option 2: Headless / Boot Auto-Start Setup (systemd)
To run the telemetry router and the mission code automatically at boot without a screen, keyboard, mouse:
1. Refer to the `systemd/README.md` for full setup instructions.
2. It includes guides on setting up `fake-hwclock` (critical for Tailscale offline startup) and configuring `ModemManager` udev rules so it doesn't conflict with the flight controller `/dev/ttyUSB0` serial line.
