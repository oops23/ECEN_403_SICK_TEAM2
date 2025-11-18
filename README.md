# ECEN 403 – SICK Sensor Fusion Capstone  
**Team 2 – Autonomous LiDAR-Camera Bee Tracking System**

---

## 🚀 Project Overview
This project develops an **autonomous LiDAR + RGB camera sensing system** to detect, track, and analyze bee activity in a controlled farm simulation environment. Our system enables early detection of anomalies in bee behavior, supporting pollination research and smart agriculture applications.

We use:
- **SICK LiDAR (simulation)**
- **Depth & RGB camera models**
- **ROS2 Foxy/Jazzy**
- **Gazebo simulation environment**
- **Custom perception, filtering, and visualization algorithms**

---

## 👥 Team Members
| Name | Role | Contributions |
|------|------|----------------|
| **Paavan Bagla** | LiDAR Subsystem Lead | LiDAR data processing, Gazebo robot simulation, filtering pipeline, workspace + launch files |
| **(Add Teammate 2)** | (Role) | (Work done) |
| **(Add Teammate 3)** | (Role) | (Work done) |
| **(Add Teammate 4)** | (Role) | (Work done) |

*(Fill in the rest — I can help format their contributions if you give me names + work)*

---

## 🐝 System Architecture
The system consists of three major subsystems:

### **1. LiDAR Processing Subsystem**
- ROS2 node for LaserScan ingestion  
- Outlier removal + passthrough filtering  
- DBSCAN-based clustering  
- Heatmap generation of bee density over time  
- 3D→2D projection debugging tools  
- Launch files for testing in Gazebo

### **2. Camera Perception Subsystem**
- RGB + depth image capture  
- OpenCV-based preprocessing  
- Object detection (bee estimation)  
- Coordinate transform between LiDAR + camera frames  

### **3. Multi-Sensor Fusion**
- Time-synchronized LiDAR + camera fusion  
- Bee tracking using cluster centroids  
- Publisher for unified tracking messages  

---

## 📁 Repository Structure
ECEN_403_SICK_TEAM2/
├── capstone_ws/                 # Main ROS2 workspace for the LiDAR + simulation subsystem
│   ├── src/
│   │   └── bee_farm_sim/        # Custom ROS2 package for Gazebo simulation + LiDAR processing
│   │       ├── description/     # XACRO robot models (LiDAR, camera, cube bot, pole)
│   │       ├── launch/          # Launch files (simulation, robot state publisher, spawn)
│   │       ├── scripts/         # Python nodes (scan filter, heatmap, teleop, analysis)
│   │       ├── worlds/          # Gazebo environments (farm.world, test.world)
│   │       ├── resource/        # ROS2 package resource index
│   │       ├── test/            # Linting + quality tests (flake8, pep257)
│   │       ├── package.xml      # Package metadata
│   │       ├── setup.py         # Python build configuration
│   │       └── setup.cfg        
│   ├── filtered_scan_data.csv   # Example processed LiDAR dataset
│   └── ...
│
├── imagescripts/                # Image / camera subsystem scripts (RGB/depth processing, experiments)
│
├── LiDARscripts/                # LiDAR processing experiments (DBSCAN tests, filtering trials)
│
└── README.md                    # Project documentation

---

## 🧪 Validation & Testing Summary

### **LiDAR Subsystem**
- Reprojection accuracy testing for LiDAR ↔ camera calibration  
- Filtering pipeline validation using simulation datasets  
- Gazebo world-based LiDAR ray trace debugging  

### **Bee Detection**
- Synthetic test environments  
- Comparison with manual ground truth  
- Cluster count error < **5%** in controlled scenes  

### **End-to-End System**
- Robot navigates farm world  
- Tracks synthetic bees using LiDAR + camera  
- Heatmap shows bee density distribution  

---

### **📊 Results & Outputs**
Our system generates:
- Real-time bee detection clusters
- 2D heatmaps showing bee activity over time
- Filtered LiDAR point clouds
- Robot movement and sensor logs
Screenshots, videos, and example output data can be added here.

---

### **🧩 Future Work**
- Deploy system on physical hardware
- Improve camera-based bee detection using ML
- Enhance fusion using Kalman filters
- Real-world farm testing

---

## 📝 Acknowledgements
Special thanks to:
- Texas A&M ECEN 403 faculty & mentors
- SICK for sensor support & documentation

