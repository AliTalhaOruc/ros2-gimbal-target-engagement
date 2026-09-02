# 🎯 Autonomous ROS 2 Gimbal Tracking & Engagement System

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)](https://gazebosim.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF.svg)](https://docs.ultralytics.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

## Overview
This project is a fully autonomous target tracking and engagement system developed using **ROS 2 Humble**, **Gazebo Classic**, **OpenCV**, and a custom-trained **YOLOv8** model.

The system detects an aerial target drone in real time, predicts its motion using a **Kalman Filter-based tracking pipeline**, automatically aims a 2-axis gimbal, and launches simulated projectiles through a **custom Gazebo C++ physics plugin**.

The entire perception, tracking, prediction, aiming, firing, and hit-detection loop operates autonomously inside the simulation environment.

---

## Highlights
- **Custom-trained YOLOv8 drone detector**
- **Kalman Filter motion prediction**
- **Autonomous pan-tilt target tracking**
- **Real-time projectile engagement**
- **Custom Gazebo C++ physics plugin**
- **Full ROS 2 integration**

---

## 🎬 Demo & Test Scenarios

### 1. Smooth Sinusoidal Trajectory Tracking & Hit Logging
Smooth target engagement along a sinusoidal path. The terminal on the right logs real-time hit events published by the Gazebo C++ physics plugin.

<video src="https://github.com/user-attachments/assets/https://github.com/user-attachments/assets/6017e9e7-201e-44c0-adbb-5a3b202057a5.mp4" 
       controls 
       autoplay 
       loop 
       muted 
       playsinline 
       width="100%">
</video>


---

### 2. Horizontal Motion & RQT Angle Tracking Analysis
Comparative view showing the horizontal trajectory tracking alongside RQT plots for real-time target position and error analysis.

<table>
  <tr>
    <td width="50%" align="center">
      <b>Horizontal Trajectory Tracking</b><br/><br/>
      <video src="https://github.com/user-attachments/assets/d7941584-5c04-4a3d-ad03-fa75ec5622d3.mp4" autoplay loop muted playsinline width="100%"></video>
    </td>
    <td width="50%" align="center">
      <b>RQT Target Tracking Analysis</b><br/><br/>
      <video src="https://github.com/user-attachments/assets/cebad52a-2c46-4ade-aae5-0cb5a90a1814.mp4" autoplay loop muted playsinline width="100%"></video>
    </td>
  </tr>
</table>

---

### 3. High-Speed Stress Test (Square Pattern)
Stress-testing the tracking algorithm with rapid high-speed square maneuvers. Demonstrates system limits, prediction behavior, and recovery during sharp trajectory shifts.

<video src="https://github.com/user-attachments/assets/https://github.com/user-attachments/assets/https://github.com/user-attachments/assets/732c370e-9d20-4eab-9f47-4006c1a61659.mp4" 
       controls 
       autoplay 
       loop 
       muted 
       playsinline 
       width="100%">
</video>

---


### 4. 3D Figure-8 & Altitude Tracking (RQT Plot Analysis)
Evaluates the tracking system during complex 3D maneuvers, including simultaneous altitude changes (climb/descent) and continuous Figure-8 trajectories. The embedded RQT plot verifies dynamic joint response, minimal pitch/yaw error, and steady-state tracking performance.

<video src="https://github.com/user-attachments/assets/c65a5ebd-29d2-4f15-a2a0-d5035b1d32c1.mp4" 
       controls 
       autoplay 
       loop 
       muted 
       playsinline 
       width="100%">
</video>
---

## ✨ Main Features

### 1. Custom YOLOv8 Target Detector
Instead of relying on simple color segmentation, the final system uses a custom-trained YOLOv8 model.
- **Dataset:** 500+ manually collected drone images, multiple drone colors, different viewing angles, various target distances, and simulated aerial footage generated from Gazebo.
- **Output:** The detector publishes target center pixels (c_x, c_y), bounding box coordinates, and detection confidence through ROS 2 topics.

### 2. YOLOv8 Training Pipeline
The original prototype relied on HSV color segmentation. To improve robustness, a custom YOLOv8 model was trained. 

**Training process:**
- Dataset collection from Gazebo simulation
- Manual annotation
- Multiple target colors
- Different scales and distances
- Data augmentation
- YOLOv8 fine-tuning

**Result:** The detector became independent of target color and can reliably detect the drone under different visual conditions.

### 3. Kalman Filter Motion Prediction
Raw detections are inherently noisy and delayed. To obtain smooth tracking performance, the target position is processed using a Kalman Filter.
- **State Estimation:** Target pixel position (x, y) and Target velocity (vx, vy).
- **Benefits:** The Kalman Filter predicts future target locations between image frames, providing smoother tracking and more stable gimbal motion.

### 4. 3D Pan-Tilt Gimbal Tracking
The turret continuously calculates Yaw and Pitch angles from target position information. The **PD-Based Gimbal Controller** automatically rotates the Pan Joint and Tilt Joint to keep the target centered in the camera frame.

### 5. Automatic Fire Control
Once the target is inside the firing window, the system automatically triggers engagement:
```
Target Detected ──► Prediction Updated ──► Gimbal Aligned ──► Projectile Spawned
```
6. Custom Projectile Physics Plugin

A custom Gazebo C++ plugin (BulletPlugin.cc) was developed for realistic ballistics simulation:

    Dynamic projectile spawning

    Initial velocity assignment

    Lifetime management & cleanup

    Collision checking & hit event publishing

    Built using Gazebo C++, ROS 2, and gazebo_ros.

7. Hit Detection System

Projectile collisions are monitored in simulation. When a collision occurs between a projectile and the target, a collision event is registered and a ROS 2 message is published to notify the tracking system.
8. Moving Target Simulation

The target drone is controlled through libgazebo_ros_planar_move, allowing constant velocity motion, dynamic trajectory testing, and repeatable experiments.
📡 ROS 2 Interfaces

Published Topics

    /target/pixel_position

    /camera/process_image

    /bullet_hits

Subscribed Topics

    /camera/image_raw

    /target/cmd_vel

🏗️ System Architecture
Plaintext

                 Gazebo Camera
                       │
                       ▼
              ┌─────────────────┐
              │ YOLOv8 Detector │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │  Kalman Filter  │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ Target Tracker  │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ Fire Controller │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │  Bullet Plugin  │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │  Hit Detection  │
              └─────────────────┘

📂 Repository Structure
Plaintext

gimbal_ws
│
├── src
│   ├── gimbal_description
│   │   ├── config/              # ROS 2 parameter configurations
│   │   ├── launch/              # Launch files for simulation
│   │   ├── media/               # Demo videos, GIFs, and screenshots
│   │   ├── urdf/                # Xacro / URDF robot models
│   │   ├── weights/             # Custom YOLOv8 model weights (best.pt)
│   │   ├── worlds/              # Gazebo world definitions
│   │   ├── target_detector.py   # YOLOv8 detection node
│   │   ├── target_tracker.py    # Kalman Filter & gimbal control node
│   │   └── target_mover.py      # Target trajectory node
│   │
│   └── gimbal_plugins
│       ├── BulletPlugin.cc      # C++ physics & hit detection plugin
│       └── CMakeLists.txt
│
├── .gitignore
├── requirements.txt             # Python dependencies
└── README.md

🛠️ Installation & Prerequisites

Prerequisites

    OS: Ubuntu 22.04 LTS

    ROS Version: ROS 2 Humble

    Simulator: Gazebo Classic 11

    Python: 3.10+

Setup Instructions

    Clone the Repository:

Bash

git clone [https://github.com/AliTalhaOruc/ros2-gimbal-target-engagement.git](https://github.com/AliTalhaOruc/ros2-gimbal-target-engagement.git)
cd ros2-gimbal-target-tracker

    Install ROS 2 Dependencies:

Bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y

    Install Python Packages:

Bash

pip install -r requirements.txt

    Build the Workspace:

Bash

colcon build --symlink-install
source install/setup.bash

🚀 Usage Guide

Launch Simulation & Gimbal:
Bash

ros2 launch gimbal_description display.launch.py

Start Target Movement:
Bash

ros2 run gimbal_description target_mover

Start Detection & Tracking Pipeline:
Bash

ros2 run gimbal_description target_detector
ros2 run gimbal_description target_tracker

💻 Performance

Tested on: Ubuntu 22.04 LTS | ROS 2 Humble | NVIDIA RTX 3060

    Real-time object detection and tracking

    Low-latency Kalman Filter prediction

    Autonomous trajectory locking & dynamic moving-target interception

🙋‍♂️ Personal Contribution

This project was designed, implemented, trained, tested, and integrated entirely by me.

Implemented components:

    Dataset collection, annotation, and custom YOLOv8 model training.

    ROS 2 perception pipeline using cv_bridge and standard ROS messages.

    Kalman Filter algorithm implementation for target motion prediction.

    Pan-Tilt tracking controller for gimbal joint positioning.

    Projectile spawning logic and autonomous fire control.

    Gazebo C++ plugin for projectile dynamics and hit-detection.

    Simulation world design and end-to-end system integration testing.

🛠️ Technologies Used

    Robotics: ROS 2 Humble, TF2, Gazebo Classic, URDF / Xacro

    Computer Vision: OpenCV, YOLOv8, cv_bridge

    Tracking & Control: Kalman Filter, Motion Prediction, PD-Based Gimbal Control

    Languages: Python, C++

    Machine Learning: PyTorch, Ultralytics

🔮 Future Improvements

    Multi-object tracking (SORT / DeepSORT integration)

    Target classification and prioritization

    Advanced ballistic lead compensation for dynamic gravity/wind effects

    Drone swarm engagement scenarios

    TensorRT acceleration for higher inference rates

    Real-world hardware deployment on physical pan-tilt hardware

✍️ Author

Ali Talha Oruç Computer Engineering Student | Konya Technical University

Interests: Autonomous Robotics, ROS 2, Computer Vision, Navigation Systems, Intelligent Control Systems
