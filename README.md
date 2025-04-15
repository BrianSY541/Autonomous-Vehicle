# Autonomous Vehicle: SLAM, Planning & Manipulation

[![Demo Videos](https://img.shields.io/badge/YouTube-Demo_Videos-red)](https://www.youtube.com/playlist?list=PLsdWuvXTaeK0HYMaCTsfsfi2ttjFZUhXJ)

## 🚗 Project Overview

This project demonstrates a comprehensive autonomous mobile robot system implemented on the Qualcomm RB5 robotics platform, integrating motor calibration, visual feedback control, Extended Kalman Filter (EKF) based SLAM, and intelligent path planning (minimizing distance and maximizing safety).

## 🛠️ Technical Details

### 1️⃣ Motor Calibration and Navigation
- **Motor Kinematic Calibration** using Least Squares optimization to compute an accurate kinematic model.
- **Navigation Algorithm** converts global waypoints into motor commands, achieving precise directional control.

### 2️⃣ Visual Feedback Control
- **Camera Calibration** using chessboard patterns for intrinsic parameters estimation.
- **Visual Anchoring** using April Tags for robust visual localization.
- **Closed-Loop Feedback Control** ensuring consistent navigation even when visual markers are temporarily lost.

### 3️⃣ EKF-based SLAM
- **Sensor Fusion** using Extended Kalman Filter to simultaneously estimate robot pose and landmark positions.
- **Robust Mapping** with accurate localization, achieving landmark position errors averaging below 0.05 m.

### 4️⃣ Intelligent Path Planning
- **Configuration Grid** used as the world representation for efficient path planning.
- **Minimum Distance Path** planning achieved via Breadth-First Search (BFS).
- **Maximum Safety Path** ensured using Brushfire algorithm and Voronoi diagrams.

## 📂 Project Structure

```
.
├── reports/                   # Project reports documenting methodologies and results
│   ├── Introduction_to_robotics_report1.pdf
│   ├── Introduction_to_robotics_report2.pdf
│   ├── Introduction_to_robotics_report3.pdf
│   └── Introduction_to_robotics_report4.pdf
├── src/                       # Source code for various project components
│   ├── hw1_sol.py             # Motor calibration and navigation
│   ├── hw2_sol                # Visual feedback control
│   │   ├── rb5_control
│   │   │   ├── include/
│   │   │   ├── launch/
│   │   │   └── src/
│   │   │       ├── hw2_sol.py
│   │   │       ├── mpi_control.py
│   │   │       └── mpi_twist_control_node.py
│   │   └── ros2_april_detection
│   │       ├── include/
│   │       ├── launch/
│   │       ├── msg/
│   │       └── src/
│   │           ├── april_detection_node.cpp
│   │           └── april_detection.cpp
│   ├── hw3_sol                # EKF-based SLAM
│   │   ├── hw3
│   │   │   ├── control_node.py
│   │   │   ├── kalman_filter.py
│   │   │   └── mpi_twist.py
│   │   ├── include/
│   │   └── launch/
│   ├── hw4_sol.py             # Path planning solutions
│   └── pathPlanner.py         # Path planning algorithms
└── README.md
```

## 🎥 Demo Videos
Explore the full project demonstrations through our YouTube playlist:

➡️ [Autonomous Vehicle Demo Videos](https://www.youtube.com/playlist?list=PLsdWuvXTaeK0HYMaCTsfsfi2ttjFZUhXJ)

## 🧰 Technologies Used
- **Languages & Frameworks**: Python, ROS 2, C++
- **Libraries**: NumPy, OpenCV, GTSAM
- **Hardware Platform**: Qualcomm RB5 robotics platform, Mbot Mega
- **Sensors**: IMU, RGB Camera, Encoders, April Tags

## 📈 Performance Highlights
- Precise motor control through kinematic calibration.
- Robust visual localization ensuring consistent navigation.
- Accurate landmark mapping with EKF-based SLAM.
- Effective path planning balancing shortest distance and maximum safety.

## 🚩 Acknowledgments
This project was completed as part of the CSE 276A coursework at the University of California, San Diego.

---

📫 **Contact**: [Brian (Shou Yu) Wang](https://github.com/BrianSY541) | briansywang541@gmail.com | www.linkedin.com/in/sywang541

