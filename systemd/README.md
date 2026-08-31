# Systemd Services

Service unit files for running the UAV vision system headlessly on the Jetson Orin NX.

## Prerequisites

### 1. Offline Clock Restoration (`fake-hwclock`)
Because the Jetson devkit lacks an RTC battery (or it is depleted), it loses time on boot. Tailscale's security handshake (WireGuard/TLS) will reject connections if the system clock is off. We use `fake-hwclock` to save time on shutdown and restore it on boot:

```bash
sudo apt install fake-hwclock
sudo timedatectl set-ntp true
```

### 2. Configure ModemManager (Ignore Flight Controller)
ModemManager conflicts with the Pixhawk/ArduPilot serial connection by trying to send AT commands to `/dev/ttyUSB0` (which is your FTDI chip `0403:6001`). However, you **need** ModemManager for your stacked Sierra Wireless 4G/5G module.

To allow both to work simultaneously, configure a udev rule to ignore the flight controller FTDI chip:

```bash
echo 'ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-ignore-ftdi-modemmanager.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl enable ModemManager
sudo systemctl restart ModemManager
```

## Installation

Copy the services to the system directory, reload systemd, and enable them:

```bash
# Copy service files
sudo cp /home/rtd/luca_boni/yolov26/systemd/*.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable services to run automatically on boot
sudo systemctl enable mavproxy.service yolov26.service
```

## Usage & Monitoring

```bash
# Start/Stop/Restart services
sudo systemctl restart mavproxy.service
sudo systemctl restart yolov26.service

# Check status (snapshot)
sudo systemctl status yolov26.service

# Live-stream logs in real-time (highly recommended for headless debugging)
sudo journalctl -u yolov26.service -f
```

## Service Architectures

- **`mavproxy.service`**: Connects to ArduPilot via `/dev/ttyUSB0` at `921600` baud and runs in `--daemon` mode (non-interactive) to prevent exiting on headless boot. Streams UDP telemetry locally (`127.0.0.1:14551`) and to the Ground Station Tailscale IP.
- **`yolov26.service`**: Runs the main vision control loop (`main.py`) under the local user `rtd`. Preloads the GStreamer memory library (`LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0`) to avoid the TLS allocation issue on Jetson.
