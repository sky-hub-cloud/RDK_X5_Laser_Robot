# RDK_X5_Laser_Robot
## README.md (English Translation)

```markdown
# Autonomous Laser Weeding Robot

An autonomous laser-weeding robot based on **RDK X5** and **STM32F407VET6** heterogeneous dual-core architecture. It employs a lightweight YOLO26 model for real-time weed detection, drives a 2-DOF stepper pan-tilt for automatic targeting, and fires a high-power laser to burn weeds — all without human intervention.

![Overview](docs/overview.jpg) <!-- Replace with actual image -->

---

## 📖 Introduction

Traditional weeding relies on chemical herbicides or manual labor, causing residue pollution and high costs. This project designs a fully autonomous laser weeding robot that uses purely physical weed control, eliminating pesticide residues and enabling green agriculture.

- **Heterogeneous Dual-MCU**: RDK X5 handles AI vision inference, while STM32 manages motion control, pan-tilt servo, and laser safety logic.
- **End-to-End Weed Detection**: Lightweight YOLO26 model adapted for outdoor lighting, outputting weed pixel coordinates in real time.
- **Auto-Tracking Pan-Tilt**: Pixel-to-angle mapping with PID closed-loop control for stable two-axis targeting.
- **Safe Laser Activation**: Multiple interlock protections (target loss, overheating, obstacle) ensure laser fires only when pan-tilt is stable.
- **Fully Automatic**: Powers on and starts line-following, detecting, aiming, and weeding automatically — no remote control or buttons needed.
- **Remote Monitoring**: Real-time video stream can be transmitted to a mobile phone via Wi-Fi for supervision.

---

## 🧱 Hardware Architecture

| Component             | Model/Description                           |
|-----------------------|---------------------------------------------|
| Vision MCU            | D-Robotics RDK X5                           |
| Motion MCU            | STM32F407VET6 (Lichuang SkyStar)            |
| Camera                | USB webcam (640×480)                        |
| Pan-Tilt              | 2-DOF stepper pan-tilt (±90° horizontal, 90°~180° vertical) |
| Stepper Driver        | A4988 (dual-axis)                           |
| Chassis               | 4-wheel differential, DC motors + AT8236 drivers |
| Laser Module          | 130W high-power laser with opto-isolated switch |
| Navigation Sensor     | Infrared line-following sensors             |
| Power Supply          | Li-ion battery pack, isolated DC-DC (5V/3.3V) |
| Communication         | TTL UART (RDK X5 ↔ STM32)                   |

> Mechanical: 3D-printed pan-tilt bracket with coaxial camera and laser.

---

## 💻 Software Architecture

### 1. RDK X5 Side (Ubuntu + ROS2)
- **Inference Node** (`inference_node.cpp`):
  - Captures USB camera frames
  - Runs YOLO26 inference via Horizon BPU acceleration
  - Outputs weed bounding boxes and center pixel coordinates
  - Sends an 8‑byte custom frame over UART (`/dev/ttyUSB0`, 9600 bps) to STM32
  - Publishes ROS2 Image topic `/image_with_detections`
- **Model**: YOLO26 (Anchor-Free), input 640×640, 1 class (weed)
- **Dependencies**: ROS2 Humble, Horizon `hb_dnn`, OpenCV, cv_bridge

### 2. STM32 Side (Bare-metal HAL)
- **UART Interrupt RX**: Parses 8‑byte frame (header `AA 55`, X/Y coordinates, confidence)
- **Line-Following**: Infrared sensors, PID differential speed control (`Set_4_Speed`)
- **Pan-Tilt Servo**: Pixel → angle mapping, stepper positioning (`Emm_V5_Pos_Control`), PID regulation for both axes
- **Laser Safety**: Activates laser (PWM `htim9`) only when pan-tilt is stable (within error threshold) and no faults
- **IMU**: JY901S 9‑axis module (UART3) for yaw compensation (optional)
- **OLED Display**: Shows real-time angles and status (debug)

---

## 🚀 Quick Start

### 1. Hardware Connections
- Connect the USB camera to RDK X5.
- Cross‑connect RDK X5's UART2 (PD5/PD6) to STM32's USART2 (PD5/PD6) — TX↔RX.
- Connect motor drivers, stepper drivers, and laser control signals to STM32 GPIO.
- Power: Provide separate supplies for RDK X5 (5V/2A), STM32 (5V), motors (12V), and laser (12V).

### 2. Model Deployment
- Convert your trained YOLO26 model to Horizon `.bin` format using the Horizon toolchain.
- Place the model file under `/home/sunrise/ros2_ws_10_YOLO26_weed/` (or modify `model_path` in `inference_node.cpp`).

### 3. Build & Run
#### RDK X5 Side
```bash
cd ~/ros2_ws_10_YOLO26_weed
colcon build --packages-select inference_node
source install/setup.bash
ros2 run inference_node inference_node --ros-args -p model_path:=/path/to/model.bin -p camera_id:=0 -p serial_port:=/dev/ttyUSB0 -p baudrate:=9600