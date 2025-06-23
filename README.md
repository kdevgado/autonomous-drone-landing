# 🛸 Autonomous Drone Navigation & Landing System

This project showcases my contribution to an autonomous drone landing system developed as part of the **RoboCup Flying Robot League** under the **RMIT UAV Research Team**.

> **Note:** This is a public summary repository. The original source code is private and maintained by the [flying-robot-league](https://github.com/flying-robot-league) organization at RMIT.

## Combined ROS 2 Workspace

This repository contains a combined ROS 2 workspace used for developing and testing autonomous drone features including ArduPilot integration, landing pad detection, localization, and mapping using LiDAR and camera-based inputs.

---

## 📌 Project Overview

Our goal was to enable a drone to autonomously navigate and land on designated targets without relying on GPS. The project was part of Phase 1 of the RoboCup Flying Robot League competition.

The workspace includes the following components:

- **ROS 2 Humble**
- **ArduPilot ROS Integration** (`ardupilot_ros`, `ardupilot_cartographer`)
- **Landing Pad Detector**
- **Localization Server**
- **Map Server**

These packages are intended to work together in a containerized environment using Docker.

---

### 🔧 Key Features
- GPS-free navigation using **Cartographer SLAM** and **MAVROS**
- Hybrid detection system integrating **YOLOv8**, **Hough Transform**, and custom object filters
- Real-time landing pad recognition with >98% detection accuracy in simulation
- Seamless return-to-origin logic using onboard visual odometry

## 📦 Package Descriptions

### 🛩 `ardupilot_ros`

This package bridges ROS 2 and ArduPilot SITL or hardware, providing MAVLink-based telemetry and control.

### 🗺 `ardupilot_cartographer`

Used for integrating Cartographer SLAM with ArduPilot data, enabling map generation and pose estimation from LiDAR.

To launch:
```bash
ros2 launch ardupilot_cartographer cartographer.launch.py
```

### 🎯 `landing_pad_detector`

A custom package for detecting visual landing pads in the arena using camera inputs and CV models.

To launch:
```bash
ros2 launch landing_pad_detector detector
```

### 🧭 `localisation_server`

Provides localization services using fused sensor inputs (e.g. IMU, vision, GPS). Used for estimating the current drone pose.

_No launch script provided in this example — you must launch any node(s) manually._

### 🗃 `map_server`

Handles loading and serving saved maps for reuse in localization and navigation. Works with SLAM outputs from Cartographer.

_No launch script provided in this example — you must launch any node(s) manually._

---

## 🐳 Docker Instructions

### 🔨 Build the Docker Image

From the root of the workspace:
```bash
sudo docker build -t img .
```

### 🚀 Run the Docker Container

```bash
sudo docker run -it \
  --rm \
  --privileged \
  --net=host \
  --device=/dev/bus/usb \
  --volume="/dev/bus/usb:/dev/bus/usb" \
  --volume="$PWD:/ros2_ws" \
  --name drone \
  img
```

Once inside the container:

### 🧱 Build the Workspace

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## ⚙️ Running the System

Start each required node manually in separate terminals or backgrounded processes.

1. **Start RPLIDAR A3 Driver**
   ```bash
   ros2 launch rplidar_ros rplidar_a3_launch.py
   ```

2. **Start SLAM with Cartographer**
   ```bash
   ros2 launch ardupilot_cartographer cartographer.launch.py
   ```

3. **Start Landing Pad Detector**
   ```bash
   ros2 launch landing_pad_detector detector
   ```

4. **Start the control Node**
   ```bash
   python3 src/guide_arm_takeoff/scripts/guide_arm_takeoff.py

### 🧩 Open a New Bash Terminal in a Running Container
```bash
sudo docker exec -it drone bash
```
