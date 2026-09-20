# Autonomous Mobile Robot with Computer Vision & LiDAR Fusion (ROS 2 Lyrical)

[![ROS 2](https://img.shields.io/badge/ROS%202-Lyrical-blue.svg)](https://docs.ros.org)
[![Gazebo](https://img.shields.io/badge/Gazebo%20Sim-10.5.0-orange.svg)](https://gazebosim.org)
[![CUDA](https://img.shields.io/badge/CUDA-13.0%20%2F%2013.2-green.svg)](https://developer.nvidia.com/cuda-zone)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.14.0%2Bcu130-red.svg)](https://pytorch.org)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Real--time%20Inference-yellow.svg)](https://ultralytics.com)

A comprehensive university-grade autonomous robotics and computer vision project built specifically for **ROS 2 Lyrical** on **Ubuntu 26.04 LTS**. The robot navigates autonomously through a complex 25m × 25m Gazebo simulation environment using 2D LiDAR, an RGB camera, YOLOv8 deep learning on an NVIDIA RTX 3050 GPU, sensor fusion, and the Nav2 navigation stack.

---

## 1. Problem Statement & Objectives
Autonomous mobile robots (AMRs) operating in dynamic environments (warehouses, hospitals, offices) require more than geometric obstacle detection. A robot must understand **what** an obstacle is (semantic context) to adapt its clearance and behavior, while relying on **LiDAR** for precise metric distances.

### Key Objectives:
1. **Autonomous Navigation**: Navigate through multi-room indoor environments with dead ends, narrow corridors, and moving obstacles.
2. **Deep Learning Vision**: Detect objects (`person`, `chair`, `table`, `box`, `cone`) at >150 FPS on an NVIDIA RTX 3050 Laptop GPU.
3. **Camera-LiDAR Fusion**: Associate 2D bounding boxes with LiDAR angular sectors to accurately estimate physical obstacle distance and bearing.
4. **Perception-Aware Navigation**: Flag critical dynamic obstacles for safety slowdown and path replanning.
5. **Clean ROS 2 Lyrical Architecture**: Standard modular packages targeting modern ROS 2 Lyrical and Gazebo Sim 10.5.0.

---

## 2. System Architecture

```
                                +-------------------+
                                | Gazebo Sim 10.5.0 |
                                +---------+---------+
                                          |
                +-------------------------+-------------------------+
                | /camera/image_raw                                 | /scan, /odom, /tf
                v                                                   v
+-------------------------------+                   +-------------------------------+
|  YOLOv8 Object Detection Node |                   |  Nav2 Navigation Stack        |
|  (NVIDIA RTX 3050 CUDA)       |                   |  - Navfn Global Planner       |
|  -> /vision/detections        |                   |  - Regulated Pure Pursuit     |
+---------------+---------------+                   |  - 2D Costmaps (Static/Obst)  |
                |                                   +---------------+---------------+
                v                                                   ^
+-------------------------------+                                   |
|   LiDAR-Camera Fusion Node    |-----------------------------------+
|   -> /vision/objects          |  (Semantic Obstacle Alerts & Replanning)
|   -> /vision/obstacles        |
+-------------------------------+
```

---

## 3. Package Structure

```
~/CV_Autonomous_Navigation/
├── ros2_ws/src/
│   ├── autonomous_robot_description/    # URDF, Xacro, wheel odometry, sensor macros
│   ├── autonomous_robot_gazebo/         # Complex 25mx25m world, maze, bridge config
│   ├── autonomous_robot_navigation/     # Nav2 params, SLAM config, pre-built map
│   ├── autonomous_robot_perception/     # YOLOv8 node, LiDAR fusion, safety node
│   ├── autonomous_robot_interfaces/     # Custom Detection2D and ObstacleWarning msgs
│   └── autonomous_robot_bringup/        # Master launch file and RViz configuration
├── datasets/                            # Dataset storage for custom training
├── models/                              # Pretrained YOLOv8n.pt (CUDA-ready)
├── results/                             # Benchmark logs and experiment outputs
├── documentation/                       # Detailed architectural and user guides
├── experiments/                         # Evaluation scripts
├── scripts/                             # Benchmark and helper scripts
├── .venv/ -> ~/venvs/ai                 # Dedicated Python AI virtual environment
└── README.md
```

---

## 4. Hardware & Software Requirements

| Component | Specification |
| :--- | :--- |
| **Operating System** | Ubuntu 26.04.1 LTS (Resolute Raccoon) |
| **Processor** | AMD Ryzen 7 5800H (8 Cores, 16 Threads) |
| **GPU** | NVIDIA GeForce RTX 3050 Laptop GPU (4 GB VRAM) |
| **NVIDIA Driver** | 595.84 (Proprietary) |
| **CUDA Version** | 13.0 (PyTorch runtime) / 13.2 (Driver API) |
| **ROS Distribution** | ROS 2 Lyrical |
| **Simulator** | Gazebo Sim 10.5.0 (`gz sim`) |
| **Deep Learning** | PyTorch 2.14.0+cu130, Ultralytics YOLOv8n |

---

## 5. Quick Start: How to Run

### 1. Source Environments
```bash
source /opt/ros/lyrical/setup.bash
source ~/CV_Autonomous_Navigation/ros2_ws/install/setup.bash
```

### 2. Launch Complete Simulation & Vision Pipeline
```bash
# Launch Gazebo world, robot, sensor bridges, Nav2, perception, and RViz
ros2 launch autonomous_robot_bringup bringup.launch.py
```

### 3. Launch with SLAM Mapping Mode
```bash
ros2 launch autonomous_robot_bringup bringup.launch.py slam:=true
```

### 4. Run GPU Benchmark
```bash
~/CV_Autonomous_Navigation/scripts/benchmark_gpu.py
```

---

## 6. Experimental Results

* **Inference Throughput**: **178.0 FPS** on RTX 3050 GPU (5.62 ms average latency)
* **VRAM Consumption**: **36.2 MB** (Leaves >3.7 GB VRAM free for simulation and system)
* **Sensor Fusion Accuracy**: 1.5 cm distance estimation error against ground truth in simulation
* **Dynamic Obstacle Reaction**: Successfully detected moving obstacle in transit corridor and initiated safety alert.

---

## 7. Future Work
* Integration of 3D depth camera point clouds with RTAB-Map for full 3D semantic SLAM.
* Custom fine-tuning of YOLOv8 on Gazebo-specific object meshes using domain randomization.
