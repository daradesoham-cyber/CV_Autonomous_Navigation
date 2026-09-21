# Third-Party Dependencies and Open-Source Asset Attribution

This document provides complete attribution, licensing, and source information for third-party software, libraries, and simulation assets integrated into the **Vision-LiDAR Semantic Autonomous Navigation** system.

---

## 1. Core Frameworks and Libraries

| Framework / Dependency | Version / Distribution | Source Repository / Website | License | Purpose in Project |
| :--- | :--- | :--- | :--- | :--- |
| **ROS 2** | Lyrical (Ubuntu 26.04) | [github.com/ros2](https://github.com/ros2) | Apache-2.0 | Middleware, node communication, parameter management, and hardware abstraction. |
| **Gazebo Sim** | 10.5.0 (`gz sim`) | [gazebosim.org](https://gazebosim.org) | Apache-2.0 | High-fidelity physics simulation, sensor rendering (RGB camera, 2D LiDAR), and collision checking. |
| **Nav2 (Navigation2)** | Lyrical release | [github.com/ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | Apache-2.0 / BSD-3-Clause | 2D costmaps, path planning (Navfn), path following (Regulated Pure Pursuit), and collision avoidance. |
| **Ultralytics YOLOv8** | 8.x | [github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | AGPL-3.0 / Enterprise | Real-time object detection architecture fine-tuned on custom indoor obstacle classes. |
| **OpenCV** | 5.0 / 4.x | [opencv.org](https://opencv.org) | Apache-2.0 | Image preprocessing, contour analysis, directional sign template matching, and color thresholding. |
| **PyTorch** | 2.14.0+cu130 | [pytorch.org](https://pytorch.org) | BSD-style | Deep learning inference backend with NVIDIA CUDA GPU acceleration (`cuda:0`). |
| **NetworkX** | 3.x | [networkx.org](https://networkx.org) | BSD-3-Clause | Spatial graph modeling, Dijkstra/A* routing, and dynamic edge penalty manipulation. |
| **SQLite3** | 3.x | [sqlite.org](https://sqlite.org) | Public Domain | Persistent relational storage for topological nodes, directed edges, and navigation history. |
| **NumPy & SciPy** | 1.26+ / 1.12+ | [numpy.org](https://numpy.org) | BSD-3-Clause | Numerical matrix operations, sensor coordinate transformations, and geometric computations. |

---

## 2. Simulation 3D Models and Environment Assets

The realistic facility environment (`worlds/realistic_facility_world.sdf`) incorporates open-source 3D models and materials derived from OpenRobotics Gazebo Fuel and AWS RoboMaker indoor simulation repositories:

| Asset Name / Category | Original Source | Upstream License | Purpose in Project |
| :--- | :--- | :--- | :--- |
| **Warehouse Racks & Shelving** | OpenRobotics Fuel / AWS RoboMaker Small Warehouse | Apache-2.0 | Structural obstacles and visual landmarks in the Industrial Warehouse wing. |
| **Hospital Beds & Clinical Equipment** | AWS RoboMaker Hospital World (`aws-robotics/aws-robomaker-hospital-world`) | Apache-2.0 | Furnishings and detection targets in the Hospital Ward and Triage area. |
| **Office Furniture (Desks, Chairs, Bookshelves)** | OpenRobotics Fuel / AWS RoboMaker Small House World | CC BY 4.0 / Apache-2.0 | Interior obstacles and spatial geometry for Executive Office and Reception zones. |
| **Cafeteria Tables and Chairs** | OpenRobotics Fuel Models | CC BY 4.0 | Dining hall furniture clusters for dynamic navigation testing. |
| **Directional Signs (Hospital, Warehouse, etc.)** | Custom Procedurally Generated (PBR Albedo) | Apache-2.0 (Original Project) | Semantic visual cues for sign detection node (`/vision/signs`). |
| **Pallet & Box Clusters** | OpenRobotics / AWS RoboMaker Warehouse | Apache-2.0 | Corridors obstacles used for dynamic replanning and blockage detection. |

---

## 3. Attribution Statement

- The original robotics software packages (`autonomous_robot_bringup`, `autonomous_robot_description`, `autonomous_robot_gazebo`, `autonomous_robot_interfaces`, `autonomous_robot_navigation`, `autonomous_robot_perception`) and decision engine are original work developed for this project.
- No third-party assets are claimed as original designs. All third-party models, textures, and libraries retain their respective original copyrights and open-source licenses as documented above.
