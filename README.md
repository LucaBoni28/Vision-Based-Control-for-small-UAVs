# Vision-Based Control for UAVs

![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)
![Status](https://img.shields.io/badge/status-active-success.svg)

> **Project Evolution:** This repository originated as an MSc Degree Thesis focusing on the feasibility of vision-based control methods for *small-scale drones* (the original thesis work is preserved in the `master` branch). During an ongoing internship, the project has evolved on this branch into a robust, real-world application designed to be deployed and tested on a larger, custom-developed UAV built by a separate team.

This repository contains the software implementation for an autonomous vision-based control system. The system enables real-time target tracking and flight control using relative visual positioning. It runs on an NVIDIA Jetson Orin NX companion computer alongside an ArduPilot-based flight controller.

---

## What the Program Does

This software empowers a UAV to autonomously identify, track, and physically follow a visual target (such as a person or a specific object). It operates in real-time entirely on edge-hardware (NVIDIA Jetson). 

The system calculates the relative offset of the target from the center of the camera frame and then translates these visual offsets into flight commands (rotational and traslational velocities) to keep the drone centered on the target.

---

## Repository Structure

The project has been refactored into a highly modular Object-Oriented Programming (OOP) architecture for real-world deployment:

- **`main.py`**: The primary application running on the **NVIDIA Jetson** (the drone). It initializes the hardware, runs the real-time YOLO detection and control loop, and sends MAVLink flight commands.
- **`stream_video.py`**: The client application running on the **Ground Station PC** (the operator). It receives the live video feed, provides a GUI, handles mouse clicks for manual target locking, and manages distance calibration.
- **`classes/`**: The core Object-Oriented folder containing all the modular building blocks for the system:
  - `config.yaml` & `config.py`: Centralized configuration (PID values, network IPs, camera settings).
  - `camera.py`: CSI camera interface.
  - `detector.py` & `tracker.py`: YOLO detection and tracking wrappers.
  - `distance_estimator.py`: Calculates distance from bounding box area.
  - `flight_controller.py`: Manages MAVLink connection and command dispatch.
  - `mission_controller.py`: The main control loop that glues all components together.
  - `target_selector.py` & `click_command.py`: Target locking logic and UDP command reception.
  - `video_streamer.py`: TCP video streaming to the ground station.

---

## Steps to Run

### Hardware & Environment Setup
- Connect your camera to the Jetson Orin NX CSI port.
- Connect the Jetson to the **ArduPilot** flight controller via a UART serial connection or USB.
- Ensure **Python 3.8+** is installed on the Jetson.

### Tailscale VPN Setup (Networking)
To ensure reliable communication between the drone and the Ground Station, **Tailscale** is used to create a secure, peer-to-peer VPN mesh network. This is critical because the drone and the PC may not be on the same local network (e.g., using 4G/LTE modems). Tailscale assigns fixed `100.x.y.z` IP addresses to both devices, allowing seamless video streaming and command routing from anywhere without port forwarding.
1. Install Tailscale on both the Jetson and the Ground Station PC. *(On Jetson: `curl -fsSL https://tailscale.com/install.sh | sh`)*
2. Authenticate both devices to the same network by running `sudo tailscale up`.
3. Obtain the Tailscale IP of each device (run `tailscale ip -4`) to use in the Configuration step below.

### MAVProxy & Telemetry Routing
A physical serial connection (UART/USB) between the ArduPilot flight controller (Pixhawk) and the Jetson is exclusive, meaning it can only be accessed by one program at a time. If the Python script connects directly to this serial port, you lose the ability to simultaneously use Ground Station software (like Mission Planner or QGroundControl) to monitor telemetry.

To solve this, use **MAVProxy** as a central router running on the Jetson:
1. **MAVProxy** connects to the physical Pixhawk serial port (the master).
2. It multiplexes and forwards the MAVLink traffic to multiple virtual network endpoints.
3. The Python script (`main.py`) connects to a local UDP endpoint (e.g., `udpin:0.0.0.0:14551`).
4. Mission Planner (on the remote PC) connects to another endpoint routed securely over the Tailscale VPN.

**Install Dependencies & MAVProxy:**
MAVProxy is a standalone application that is installed as a Python package via pip (it utilizes `pymavlink` under the hood). You can install it alongside the other project dependencies on the Jetson:
```bash
pip install MAVProxy ultralytics pymavlink opencv-python pyyaml
```

**Run MAVProxy:**
To start MAVProxy, run the following command on the Jetson:
```bash
mavproxy.py --master=/dev/ttyACM0 --out=udp:127.0.0.1:14551 --out=udp:100.x.x.x:14550
```
*Note: Replace `100.x.x.x` with the Tailscale IP address of your Ground Station PC, and `/dev/ttyACM0` with the actual serial port of your flight controller. The UDP ports (`14551` and `14550`) can be arbitrarily chosen, but they must be consistent. For example, if the Python script (`config.yaml`) is configured to connect to port `14551`, MAVProxy must have a corresponding `--out=udp:127.0.0.1:14551` endpoint.*

This architecture allows our vision system to send flight commands to the drone while you simultaneously monitor real-time telemetry on the Ground Station.

### Generate TensorRT Engine File
For real-time performance on the Jetson, you must convert the PyTorch YOLO model into a TensorRT `.engine` format.
```bash
yolo export model=yolov8n.pt format=engine
```

### Configuration
Open `classes/config.yaml` and configure it for your environment:
- **`mavlink.connection`**: Set this to your flight controller's port (e.g., `/dev/ttyUSB0` for direct serial, or `udpin:0.0.0.0:14551` if using MAVProxy).
- **`video_link` / `command_link`**: Set the `host` IP addresses to match the IP of the PC acting as your Ground Station.
- **`model.path`**: Ensure this points to your generated `.engine` file.
- **`control`**: You can fine-tune the PID constants (`k_p`, `k_d`) here during testing.

### Start the Ground Station Client
Before running the main mission loop, you **must** start the ground station script on your remote PC. This script receives the video stream and allows you to send target locking and calibration commands.
```bash
python stream_video.py
```
*(Note: While running `stream_video.py` is mandatory, opening a full Ground Station software like Mission Planner is optional.)*

### Run the Mission Loop
Once the ground station client is running and listening, execute the main script on the Jetson to start the autonomous control loop:
```bash
python main.py
```

---

## Hardware Requirements
- **Companion Computer**: NVIDIA Jetson Orin NX
- **Flight Controller**: ArduPilot compatible board (e.g., Pixhawk, Cube Orange)
- **Camera**: Raspberry Pi Camera V2 or similar CSI camera
- **Drone Frame**: Custom-developed UAV platform
