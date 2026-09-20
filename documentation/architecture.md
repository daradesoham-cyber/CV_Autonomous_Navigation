# System Architecture

## Overview
The **CV Autonomous Navigation** system integrates high-performance computer vision, real-time 2D LiDAR scanning, sensor fusion, and the Nav2 navigation stack on **ROS 2 Lyrical** and **Ubuntu 26.04 LTS**.

```
+-----------------------------------------------------------------------------------+
|                                 GAZEBO SIM (10.5.0)                                |
|  +----------------------+  +---------------------+  +--------------------------+  |
|  |   RGB Camera Sensor  |  |    2D LiDAR Sensor  |  |  DiffDrive Physics Plugin|  |
|  +----------+-----------+  +----------+----------+  +------------+-------------+  |
+-------------|-------------------------|--------------------------|----------------+
              | /camera/image_raw       | /scan                    | /odom, /tf
              v                         v                          v
+-----------------------------------------------------------------------------------+
|                                ROS_GZ PARAMETER BRIDGE                             |
+-------------|-------------------------|--------------------------|----------------+
              |                         |                          |
              v                         |                          |
+------------------------------+        |                          |
| autonomous_robot_perception  |        |                          |
|  +------------------------+  |        |                          |
|  |  YOLOv8 Object Detect  |  |        |                          |
|  |  (NVIDIA RTX 3050 GPU) |  |        |                          |
|  +-----------+------------+  |        |                          |
|              | /vision/      |        |                          |
|              |  detections   |        |                          |
|              v               |        |                          |
|  +------------------------+  |        |                          |
|  | LiDAR-Camera Fusion    |<----------+                          |
|  | Angular Association    |  |                                   |
|  +-----------+------------+  |                                   |
|              | /vision/      |                                   |
|              |  obstacles    |                                   |
|              v               |                                   |
|  +------------------------+  |                                   |
|  | Perception-Aware Nav   |  |                                   |
|  +------------------------+  |                                   |
+------------------------------+                                   |
                                                                   |
              +----------------------------------------------------+
              |
              v
+-----------------------------------------------------------------------------------+
|                                NAV2 STACK & SLAM                                  |
|  +------------------------+   +-----------------------+   +--------------------+  |
|  |     Costmap 2D         |   |    Global Planner     |   |  Local Controller  |  |
|  | (Static, Obstacle, Inf)|   |    (Navfn / Grid)     |   |   (Regulated PP)   |  |
|  +------------------------+   +-----------------------+   +---------+----------+  |
+---------------------------------------------------------------------|-------------+
                                                                      | /cmd_vel
                                                                      v
                                                            [Robot Base Motion]
```

## Hardware Specification
* **Processor**: AMD Ryzen 7 5800H (8 cores / 16 threads, up to 4.4 GHz)
* **GPU Accelerator**: NVIDIA GeForce RTX 3050 Laptop GPU (4 GB GDDR6 VRAM, 2048 CUDA cores, Ampere architecture)
* **RAM**: 16 GB DDR4
* **Operating System**: Ubuntu 26.04.1 LTS (Resolute Raccoon)
* **Middleware**: ROS 2 Lyrical + Gazebo Sim 10.5.0
