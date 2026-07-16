# Vision-Based Control for Small-Scale Drones

![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)
![Status](https://img.shields.io/badge/status-active-success.svg)

> **MSc Degree Thesis:** Feasibility Investigation of Vision-Based Control Methods for Small-Scale Drones

This repository contains the software implementation for a vision-based control system for small-scale drones. The system enables autonomous target tracking and flight control using relative visual positioning, utilizing an NVIDIA Jetson Orin NX companion computer alongside an ArduPilot-based flight controller.

## 🚀 Key Features

- **Real-Time Object Detection**: Uses YOLO-based object detection optimized for NVIDIA Jetson Orin NX (via TensorRT/`.engine`).
- **Visual Servoing**: Calculates relative object positions and keeps the detected target centered in the camera frame.
- **MAVLink Integration**: Translates visual feedback into MAVLink attitude and velocity commands for ArduPilot.
- **Hardware Optimized**: Interfaces directly with a Raspberry Pi Camera V2 for low-latency video capture.

## 🛠️ Hardware Requirements

- **Companion Computer**: NVIDIA Jetson Orin NX
- **Flight Controller**: ArduPilot compatible board (e.g., Pixhawk)
- **Camera**: Raspberry Pi Camera V2
- **Drone Frame**: Small-scale quadcopter

## 📁 Repository Structure

- `classes/`: Object-oriented implementations for drone control, detection, and tracking.
- `hard_code_version/`: Baseline sequential implementations and prototyping scripts.
- `main.py`: Main entry point for the vision-based control loop.

## ⚙️ Setup & Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/LucaBoni28/Vision-Based-Control-for-small-UAVs.git
   cd Vision-Based-Control-for-small-UAVs
   ```

2. **Install dependencies**
   *(Ensure you have Python 3.8+ installed on your Jetson Orin NX)*
   ```bash
   # Make sure TensorRT and Ultralytics YOLO are installed
   pip install ultralytics pymavlink opencv-python
   ```

3. **Generate TensorRT Engine Files**
   Developers generally do not commit large binary models (`.engine`, `.pt`) to the repository. Generate them locally on the Jetson Orin NX:
   ```bash
   yolo export model=yolov8n.pt format=engine
   ```

## 🎯 Usage

Run the main vision-based control loop:
```bash
python main.py
```

## 📈 Future Work

- Extension to outdoor environments combining GPS positioning with visual feedback.
- Advanced autonomous navigation algorithms.

## 📝 Tasks & Objectives

- [x] Review state-of-the-art vision-based control and object detection.
- [x] Interface NVIDIA Jetson Orin NX with Raspberry Pi Camera V2.
- [x] Implement YOLO-based real-time object detection.
- [x] Develop visual servoing control algorithm.
- [x] Generate MAVLink control commands.
- [ ] Perform indoor bench-top tests and validate flight behavior.
