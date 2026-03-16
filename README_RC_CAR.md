# Autonomous RC Car

A self-driving RC car platform built on a Jetson Nano, featuring LIDAR-based mapping, camera vision, and motor control via a VESC motor controller. The system is capable of real-time environment sensing, obstacle detection, and autonomous navigation.

---

## How It Works

The Jetson Nano acts as the central computer, processing sensor data from the LIDAR and camera in real time. A VESC motor controller receives drive commands from the Jetson Nano and handles throttle and steering. Together these components allow the car to sense its surroundings and navigate autonomously.

```
[ LIDAR ] ──┐
            ├──► [ Jetson Nano ] ──► [ VESC ] ──► [ Motors ]
[ Camera ] ─┘
```

---

## Hardware

| Component | Notes |
|---|---|
| RC car chassis | Any 1/10 scale chassis with servo steering |
| NVIDIA Jetson Nano | Onboard computer — runs all perception and control code |
| LIDAR sensor | Environment scanning and obstacle detection |
| Camera | Visual input for lane following or object detection |
| VESC motor controller | Receives commands from Jetson Nano, drives the motors |
| LiPo battery | Powers motors and VESC |
| DC-DC buck converter | Steps down battery voltage to power Jetson Nano safely |
| Jumper wires | Signal connections between components |
| USB cables | Camera and peripheral connections |

---

## Wiring

### Jetson Nano → VESC

| Jetson Nano | VESC |
|---|---|
| UART TX | UART RX |
| UART RX | UART TX |
| GND | GND |

### LIDAR → Jetson Nano

| LIDAR Pin | Jetson Nano |
|---|---|
| VCC | 5V |
| GND | GND |
| TX | UART RX |
| RX | UART TX |

### Camera → Jetson Nano

| Connection | Notes |
|---|---|
| USB or CSI ribbon cable | Depends on camera model |

> **Note:** Power the Jetson Nano from a dedicated buck converter off the main LiPo — do not power it directly from the VESC 5V rail.

---

## Software

### Prerequisites

- Jetson Nano with **JetPack SDK** installed (includes Ubuntu, CUDA, cuDNN, TensorRT)
- Python 3.6+
- ROS (Robot Operating System) — optional but recommended for sensor integration

### Setting Up the Jetson Nano

Flash JetPack to a microSD card using the [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager) or the [Balena Etcher method](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit).

Boot the Jetson Nano, complete the initial Ubuntu setup, then run:

```bash
sudo apt update && sudo apt upgrade -y
```

### Installing Python Dependencies

```bash
pip3 install pyserial numpy opencv-python
```

### Installing the VESC Python Library

```bash
pip3 install pyvesc
```

### Installing ROS (Optional)

```bash
sudo apt install ros-melodic-desktop
source /opt/ros/melodic/setup.bash
```

---

## Project Structure

```
autonomous-rc-car/
├── src/
│   ├── vesc_control.py       ← sends throttle and steering commands to VESC
│   ├── lidar_reader.py       ← reads and processes LIDAR scan data
│   ├── camera_stream.py      ← captures and processes camera frames
│   └── main.py               ← main control loop
├── config/
│   └── settings.py           ← UART ports, speed limits, sensor config
└── README.md
```

---

## Setup Instructions

### 1. Flash and Boot Jetson Nano

1. Download the latest JetPack image from NVIDIA
2. Flash to a 32GB+ microSD card using Balena Etcher
3. Insert card, connect monitor and keyboard, power on
4. Complete the Ubuntu first-boot setup

### 2. Connect Hardware

1. Connect LIDAR to Jetson Nano via UART or USB
2. Connect camera via USB or CSI port
3. Connect VESC to Jetson Nano via UART
4. Connect VESC to motors and LiPo battery
5. Power Jetson Nano via buck converter from LiPo

### 3. Configure Serial Ports

Check which ports your devices are on:

```bash
ls /dev/tty*
```

Update `config/settings.py` with the correct port names:

```python
VESC_PORT   = "/dev/ttyTHS1"   # VESC UART port
LIDAR_PORT  = "/dev/ttyUSB0"   # LIDAR port
```

### 4. Run the System

```bash
cd autonomous-rc-car
python3 src/main.py
```

---

## Usage

Once everything is connected and configured:

1. Place the car on the ground with clear space around it
2. Power on the LiPo battery
3. SSH into the Jetson Nano or use a connected monitor
4. Run `python3 src/main.py`
5. The car will begin scanning its environment and driving autonomously

### Stopping the Car

Press `Ctrl+C` in the terminal to stop. The VESC will cut throttle immediately.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| VESC not responding | Check UART TX/RX wiring — TX on one side connects to RX on the other |
| LIDAR not detected | Run `ls /dev/tty*` before and after plugging in to identify the port |
| Camera not found | Run `ls /dev/video*` to confirm the camera is detected |
| Jetson Nano won't boot | Check microSD card is fully seated and JetPack is correctly flashed |
| Car not moving | Confirm VESC is armed and LiPo battery is charged |
| Overheating | Attach heatsinks and a fan to the Jetson Nano module |

---

## Limitations

- Designed for **indoor or smooth outdoor surfaces** — large bumps may affect LIDAR accuracy
- LIDAR provides **2D scanning only** — cannot detect obstacles above or below the scan plane
- Autonomous navigation accuracy depends on environment complexity and sensor quality
- Jetson Nano requires a few seconds to boot before the system is ready

---

## Expanding the System

Possible upgrades and additions:

- Add **IMU (e.g. MPU6050)** for orientation and tilt sensing
- Integrate **GPS** for outdoor waypoint navigation
- Replace camera pipeline with a **neural network** for object detection (YOLO, SSD)
- Add **ROS SLAM** (Simultaneous Localisation and Mapping) using the LIDAR for full map building
- Use **Donkeycar** or **ROS Navigation Stack** as a higher-level framework

---

## License

MIT License — free to use, modify, and distribute.
