# Vision-LiDAR Semantic Autonomous Navigation V2 with Persistent Memory & Web Dashboard

[![ROS 2](https://img.shields.io/badge/ROS%202-Lyrical-blue.svg)](https://docs.ros.org)
[![Gazebo Sim](https://img.shields.io/badge/Gazebo%20Sim-10.5.0-orange.svg)](https://gazebosim.org)
[![Nav2](https://img.shields.io/badge/Nav2-Integrated-brightgreen.svg)](https://navigation.ros.org)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Custom%2096.2%25%20mAP50-yellow.svg)](https://ultralytics.com)
[![Dashboard](https://img.shields.io/badge/Dashboard-Web%20Port%205050-cyan.svg)](http://127.0.0.1:5050)
[![Version](https://img.shields.io/badge/Release-V2.0%20Advanced-purple.svg)](docs/ARCHITECTURE.md)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

An advanced autonomous mobile robotics system combining **multi-modal sensor fusion (RGB Camera + 2D LiDAR), visual directional sign perception, custom fine-tuned YOLOv8 object detection, persistent topological memory in SQLite, multi-criteria route cost planning with candidate alternative routes, dynamic obstacle replanning, dead-end intelligence, robust recovery behaviors, and a dedicated real-time web navigation dashboard**, deployed on **ROS 2 Lyrical** and **Gazebo Sim 10.5** (Ubuntu 26.04 LTS).

> [!NOTE]
> **Architecture Principle**: Classical metric costmaps, path planning, and trajectory execution are delegated strictly to **Nav2** (NavFn planner + Regulated Pure Pursuit controller). Higher-level semantic reasoning, multi-criteria route optimization, candidate route alternatives, persistent spatial memory, and topological corridor replanning are orchestrated by the custom **Decision Engine V2**.

---

## Technical Documentation Guides

* [V2.1_VALIDATION_REPORT.md](V2.1_VALIDATION_REPORT.md) — **Master End-to-End Real System Validation Report (100% Verified, Zero Mock Data).**
* [V2_REALITY_AUDIT.md](V2_REALITY_AUDIT.md) — Pre-validation reality audit matrix and gap analysis.
* [ARCHITECTURE.md](docs/ARCHITECTURE.md) — Comprehensive technical architecture, coordinate frames, nodes, and topic graphs.
* [NAVIGATION.md](docs/NAVIGATION.md) — Multi-criteria route cost formula, YAML weights, candidate alternative routes, dead-end intelligence, and recovery behaviors.
* [DASHBOARD.md](docs/DASHBOARD.md) — Standalone web navigation dashboard guide, screens 1/2/3, REST APIs, and live telemetry.
* [TESTING.md](docs/TESTING.md) — Verification guide for the 10-stage V2 test suite, baseline scenarios, and demo mode.
* [DATABASE_SCHEMA.md](docs/DATABASE_SCHEMA.md) — SQLite database schema, table definitions, relationships, and analytics queries.

---

## Table of Contents

1. [Overview & Architecture Pipeline](#overview--architecture-pipeline)
2. [What's New in V2](#whats-new-in-v2)
3. [Key Features](#key-features)
4. [Dedicated Navigation Dashboard](#dedicated-navigation-dashboard)
5. [Hardware & Software Requirements](#hardware--software-requirements)
6. [Project Structure](#project-structure)
7. [Installation & Setup](#installation--setup)
8. [Running the Project](#running-the-project)
9. [Start and Goal Selection (User Workflow)](#start-and-goal-selection-user-workflow)
10. [YOLOv8 Object Detection Pipeline](#yolov8-object-detection-pipeline)
11. [Directional Sign Perception](#directional-sign-perception)
12. [Enhanced Camera-LiDAR Sensor Fusion](#enhanced-camera-lidar-sensor-fusion)
13. [Multi-Criteria Route Planning & Memory](#multi-criteria-route-planning--memory)
14. [Dynamic Obstacle Handling & Replanning](#dynamic-obstacle-handling--replanning)
15. [Dead-End Intelligence & Recovery](#dead-end-intelligence--recovery)
16. [Automated Verification & Demonstration](#automated-verification--demonstration)
17. [Troubleshooting Guide](#troubleshooting-guide)
18. [License & Attribution](#license--attribution)

---

## Overview & Architecture Pipeline

Indoor mobile robots operating in structured environments (e.g., hospitals, industrial warehouses, laboratories) must navigate through complex corridor networks where metric maps alone fail to capture semantic meaning or dynamic changes. Obstacles such as misplaced carts, boxes, and walking personnel frequently obstruct designated routes, and robots without memory risk entering cul-de-sacs or repeatedly attempting impassable paths.

This platform bridges classical metric navigation with semantic perception and cognitive memory:

```
Camera (RGB 640x480 @ 30 FPS)
  │
  ▼
YOLOv8 Object Detection (RTX 3050 cuda:0, 8 classes, 96.2% mAP50)
  │
  ▼
Visual Perception & Directional Sign Matching (15 PBR textures)
  │
  ▼
LiDAR (2D Planar, 360°, 10 Hz)
  │
  ▼
LiDAR-Camera Fusion (Range-Bearing Azimuth Projection -> 3D PointCloud2)
  │
  ▼
Navigation Perception (Costmap Obstacle Layer Injection & Proximity Warnings)
  │
  ▼
Path / Corridor Decision (Decision Engine + SQLite Relational Graph Memory)
  │
  ▼
Robot Motion Execution (Nav2 RPP Controller -> /cmd_vel -> Gazebo Diff-Drive)
```

### Where Cognitive Modules Fit Into The System:
- **Persistent Topological Memory (`navigation_memory.py`, SQLite)**: Stores nodes (junctions, room centers), directed edges, sign observations, traversal history, and obstacle events across robot reboots.
- **Dead-End Detection & Pruning (`topological_graph.py`)**: Labels cul-de-sacs with a heavy cost penalty ($10^5$), preventing the robot from entering dead ends during standard navigation.
- **Dynamic Replanning (`decision_engine_node.py`)**: Detects impassable corridors in real time, marks edge blockage (cost $10^6$), aborts the active Nav2 sub-goal, and instantaneously computes an alternate corridor route (e.g., West Bypass) in **< 1 ms**.

---

---

## What's New in V2

1. **Multi-Criteria Route Planning**:
   - Implements a weighted route cost function:
     $$\text{Cost} = w_{\text{dist}} \cdot D + w_{\text{obs}} \cdot \Omega + w_{\text{hist}} \cdot F + w_{\text{cong}} \cdot C + w_{\text{turn}} \cdot T + w_{\text{narrow}} \cdot N + w_{\text{time}} \cdot \tau$$
   - Configurable via [`config/navigation_v2.yaml`](file:///home/soham-darade/CV_Autonomous_Navigation/config/navigation_v2.yaml).
   - Generates and evaluates $K=3$ candidate alternative routes (`Route A`, `Route B`, `Route C`) with itemized cost breakdowns.

2. **Upgraded Gazebo Simulation Environment**:
   - $35\text{m} \times 30\text{m}$ high-fidelity warehouse facility (`realistic_facility_world.sdf`) with multi-corridor loops, storage racks, workstations, pallets, and Rooms A & B.
   - **4 Dynamic Obstacles**: Autonomous warehouse carts, hospital trolleys, forklifts, and walking personnel powered by Gazebo velocity control systems and trajectory patrol publishers (`dynamic_obstacles_node.py`).
   - 30 crisp PNG visual signs with room names and directional guidance.

3. **Enhanced Camera + LiDAR Sensor Fusion**:
   - Direct spatial association with camera-LiDAR parallax resolution across $[0.3\text{m}, 6.0\text{m}]$.
   - Directional labeling: `Front-Center`, `Front-Right`, `Front-Left`, `Left`, `Right`.
   - Metric 3D spatial coordinates $(x, y, z)$ in `base_link` frame.
   - Real-time RViz 3D MarkerArray visualization on `/vision/fused_object_markers`.

4. **Dedicated Standalone Web Dashboard (Port 5050)**:
   - Built on Flask + vanilla ES6 canvas with zero external CDN dependencies.
   - **Screen 1 (Mission Monitor)**: Real-time robot status, kinematics, 700x600 hardware-accelerated interactive map canvas with click-to-dispatch, route intelligence, candidate alternatives table, LiDAR clearances, and live scrolling event log.
   - **Screen 2 (Travel History & Analytics)**: Historical journey table with clickable path inspection on the map, and interactive canvas charts (distance, travel time, speed, replanning count).
   - **Screen 3 (Route Heatmap)**: Persistent corridor traversal frequency and congestion heatmap overlay.

5. **Dead-End Intelligence & Robust Recovery**:
   - Automatic dead-end detection, coordinate persistence in SQLite `dead_ends` table, $+10,000$ cost penalty, and safe reverse backtracking.
   - Robust recovery state machine addressing stalled robot, low clearance, oscillation, and Nav2 goal aborts with audited events in `recovery_events`.

6. **Persistent Journey Memory & Learning**:
   - SQLite tables: `journeys`, `route_segments`, `dead_ends`, and `recovery_events`.
   - Segment reliability scores updated automatically with moving averages after each trip.

---

## Key Features

- **Dedicated Web Navigation Dashboard**: Real-time control and monitoring interface on `http://127.0.0.1:5050` with live telemetry, sensor feeds, and travel history analytics.
- **Candidate Alternative Routing**: K-shortest paths algorithm identifying multiple routes between facility origins and destinations, ordered by multi-criteria cost.
- **Dynamic Obstacle Reactivity**: Detects and reacts dynamically to moving warehouse carts, forklifts, and personnel.
- **Custom YOLOv8n Obstacle Detection**: Fine-tuned on Gazebo simulation assets across 8 classes (`person`, `chair`, `box`, `cone`, `pallet`, `shelf`, `hospital_bed`, `cart`), achieving **96.2% mAP@0.50** at ~30 FPS camera lock (**116.5 FPS** batch inference).
- **Directional Signboard Perception**: Detects and decodes 30 distinct corridor signs (`ROOM A`, `ROOM B`, `STORAGE`, `CHARGING`, `LOADING`, `RESTRICTED`, `KEEP LEFT`, `KEEP RIGHT`, directional arrows) with **100% accuracy**.
- **LiDAR-Camera Range-Bearing Fusion**: Matches monocular bounding box azimuth cones with 2D LiDAR range beams to calculate metric 3D obstacle coordinates with human-readable directional classification.
- **Persistent Topological Relational Memory**: SQLite database storing 43 nodes, 90 directed edges, sign observations, historical journeys, route segment reliability, and dynamic blockage history across process restarts.
- **Dead-End Intelligence & Backtracking**: Zero cul-de-sac entrapments through safe stops, coordinate logging, reverse backtracking, and dynamic re-routing.
- **High-Fidelity Simulation**: Realistic $35\text{ m} \times 30\text{ m}$ multi-wing industrial warehouse facility in Gazebo Sim 10.5 featuring realistic physics, lighting, and textures.

---

## System Architecture Diagram

```
+-----------------------------------------------------------------------------------------------+
|                                       GAZEBO SIM 10.5                                         |
|  - 32m x 26m Multi-Room Facility (Hospital, Warehouse, Office, Lab, Storage, Cafeteria)       |
|  - Robot Sensors: RGB Camera, 2D LiDAR (360 deg, 10 Hz), Wheel Odometry, IMU                  |
+-----------------------------------------------+-----------------------------------------------+
                                                | (gz-transport)
                                                v
+-----------------------------------------------------------------------------------------------+
|                                    ROS-GAZEBO BRIDGE LAYER                                    |
|   /ros_gz_bridge: /camera/image_raw, /scan, /odom, /tf, /cmd_vel                              |
+-----------------------------------------------+-----------------------------------------------+
                                                |
          +-------------------------------------+-------------------------------------+
          |                                                                           |
          v /camera/image_raw                                                         v /scan
+---------------------------------------------+                             +-------------------+
|             PERCEPTION PIPELINE             |                             |  LIDAR PERCEPTION |
|                                             |                             |                   |
| 1. /camera_node                             |                             | - Range filtering |
|    Formats RGB frame stream                 |                             | - Ray indexing    |
|                                             |                             +---------+---------+
| 2. /object_detection_node                   |                                       |
|    Custom YOLOv8n (RTX 3050 cuda:0)         |                                       |
|    Publishes: /vision/detections            |                                       |
|                                             |                                       |
| 3. /sign_detection_node                     |                                       |
|    15 PBR sign textures                     |                                       |
|    Publishes: /vision/signs                 |                                       |
+----------------------+----------------------+                                       |
                       |                                                              |
                       +----------------------------->+<------------------------------+
                                                      |
                                                      v
+-----------------------------------------------------------------------------------------------+
|                                 LIDAR-CAMERA SENSOR FUSION                                    |
|   - Node: /lidar_camera_fusion_node                                                           |
|   - Projects bounding box azimuth angles to LiDAR scan beams                                  |
|   - Publishes: /vision/obstacles (SemanticObstacleArray)                                      |
|   - Publishes: /vision/costmap_obstacles (sensor_msgs/PointCloud2)                            |
+---------------------------------------------+-------------------------------------------------+
                                              |
                                              v
+-----------------------------------------------------------------------------------------------+
|                                TOPOLOGICAL MEMORY & DECISION                                  |
|                                                                                               |
| 1. SQLite Relational Store: config/navigation_memory.db                                       |
| 2. Topological Graph: autonomous_robot_navigation/topological_graph.py                         |
|    - Graph with penalties: Dead-End ($10^5$), Blocked ($10^6$)                                |
| 3. Decision Engine: /decision_engine_node                                                     |
|    - Subscribes: /navigation/goal_label (e.g. 'HOSPITAL', 'WAREHOUSE')                        |
|    - Translates semantic goal into topological waypoints via Dijkstra                         |
|    - Monitors edge clearance; activates West Bypass on blockage (< 1 ms replan)               |
|    - Dispatches sub-goals via Nav2 Action Client (/navigate_to_pose)                          |
+---------------------------------------------+-------------------------------------------------+
                                              |
                                              v /navigate_to_pose (Action)
+-----------------------------------------------------------------------------------------------+
|                                     NAV2 MOTION CONTROL                                       |
|   - Global Costmap: Static SLAM occupancy grid                                                |
|   - Local Costmap: Live /scan + /vision/costmap_obstacles PointCloud2                         |
|   - Controller: Regulated Pure Pursuit (RPP) -> /cmd_vel                                     |
|   - Localization: AMCL (/amcl_pose) referenced against map frame                              |
+-----------------------------------------------------------------------------------------------+
```

---

## Hardware & Software Requirements

| Component | Verified Specification |
| :--- | :--- |
| **Operating System** | Ubuntu 26.04 LTS (x86_64) |
| **ROS Distribution** | ROS 2 Lyrical |
| **Simulation Engine** | Gazebo Sim 10.5.0 (`gz sim`) |
| **GPU Hardware** | NVIDIA GeForce RTX 3050 Laptop GPU (4096 MiB VRAM) |
| **NVIDIA Driver / CUDA**| NVIDIA Driver 595.84 / CUDA 13.0 |
| **Python Version** | Python 3.14 |
| **Deep Learning** | PyTorch 2.14.0+cu130, Ultralytics YOLOv8 (8.4.157) |
| **Computer Vision** | OpenCV 5.0.0 (`cv2`), `cv_bridge` |
| **Core Libraries** | NumPy 2.5.3, Matplotlib 3.11.2, Pandas 3.0.6, NetworkX 3.x, PyYAML 6.0 |

---

## Project Structure

```
~/CV_Autonomous_Navigation/
├── config/
│   ├── experiments.yaml                 # Benchmark & experiment parameters
│   └── navigation_locations.yaml        # Predefined map coordinates for rooms/wings
├── datasets/
│   ├── gazebo_data.yaml                 # YOLO dataset configuration
│   └── gazebo_cv/                       # 405 labeled Gazebo simulation frames & YOLO labels
├── docs/
│   ├── ARCHITECTURE.md                  # Comprehensive engineering architecture specification
│   ├── DEMO.md                          # Step-by-step reproduction and demonstration guide
│   ├── RESULTS.md                       # Empirical results (simulation vs. real-world separation)
│   ├── EXPERIMENTS.md                   # Automated experiment test suite details
│   ├── NAVIGATION_MEMORY.md             # SQLite schema and relational topological memory guide
│   ├── REALISTIC_WORLD.md               # 32m x 26m multi-wing facility specification
│   ├── SIGN_NAVIGATION.md               # Signboard perception and template matching details
│   ├── THIRD_PARTY.md                   # Open-source asset attribution and licenses
│   └── images/                          # Benchmark plots and validation curves
├── models/
│   ├── yolov8n.pt                       # Pretrained base YOLOv8n model (Git LFS)
│   └── custom_yolov8n/
│       ├── args.yaml                    # Training hyperparameters (50 epochs, imgsz 640)
│       ├── results.csv                  # Epoch-by-epoch training and validation metrics
│       ├── confusion_matrix_normalized.png # Normalized confusion matrix plot
│       └── weights/
│           └── best.pt                  # Fine-tuned custom weights (96.2% mAP50, Git LFS)
├── results/
│   ├── cv_performance.csv               # FPS, latency, GPU VRAM utilization metrics
│   ├── lidar_camera_fusion_results.csv  # Range accuracy and percentage error metrics
│   ├── model_comparison.csv             # Baseline COCO vs. Custom YOLOv8n comparison
│   ├── navigation_comparison.csv         # Ablation: LiDAR-only vs. Vision-only vs. Fusion
│   └── plots/                           # Evaluation figures and training loss curves
├── ros2_ws/src/
│   ├── autonomous_robot_bringup/        # System launch orchestration and RViz configuration
│   ├── autonomous_robot_description/    # URDF / Xacro models (diff-drive AGV, camera, LiDAR)
│   ├── autonomous_robot_gazebo/         # SDF world files, sign textures, ros_gz_bridge config
│   ├── autonomous_robot_interfaces/     # Custom ROS 2 msg definitions (Detections, Signs, Obstacles)
│   ├── autonomous_robot_navigation/     # Decision Engine, Topological Graph, Nav2 configs, Maps
│   └── autonomous_robot_perception/     # Camera, YOLO, Sign, and LiDAR-Camera Fusion nodes
├── scripts/
│   ├── start_project.sh                 # Unified clean simulation launcher
│   ├── stop_navigation.sh               # Graceful process termination script
│   ├── navigation_control.py            # Graphical status monitor and mission control GUI
│   ├── send_goal.py                     # Nav2 goal action client with predefined locations
│   ├── set_start.py                     # AMCL initial pose publisher
│   ├── run_all_experiments.py           # Automated test suite (Scenarios 1 to 4)
│   ├── reset_navigation_memory.sh       # SQLite memory database initialization script
│   └── train_yolo.py                    # Fine-tuning script for custom YOLOv8 model
├── .gitattributes                       # Git LFS tracking configuration for *.pt model weights
├── .gitignore                           # Exclusions for build artifacts, virtualenvs, and caches
├── LICENSE                              # Apache-2.0 open-source license
├── README.md                            # Primary project documentation
└── requirements.txt                     # Python dependencies
```

---

## Installation & Setup

### 1. Clone the Repository (with Git LFS)

```bash
# Ensure Git LFS is installed
git lfs install

# Clone the repository
git clone https://github.com/daradesoham-cyber/CV_Autonomous_Navigation.git ~/CV_Autonomous_Navigation
cd ~/CV_Autonomous_Navigation

# Pull model weights through Git LFS
git lfs pull
```

### 2. Install Python Dependencies

```bash
# If using a virtual environment
python3 -m venv .venv
source .venv/bin/activate

pip install -r requirements.txt
```

### 3. Build ROS 2 Packages

```bash
# Source ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash

# Build the workspace
cd ~/CV_Autonomous_Navigation/ros2_ws
colcon build --symlink-install

# Source the workspace setup
source install/setup.bash
cd ~/CV_Autonomous_Navigation
```

---

---

## Running the Project

### 1. Full System Launch (Gazebo, Nav2, Dynamic Obstacles, Perception, Decision Engine, Dashboard, RViz)

Launch the complete autonomous navigation stack in a single command:

```bash
source /opt/ros/lyrical/setup.bash
source ~/CV_Autonomous_Navigation/ros2_ws/install/setup.bash

ros2 launch autonomous_robot_bringup full_system.launch.py
```

### 2. Standalone Web Navigation Dashboard

The dedicated robotics navigation dashboard runs on port 5050 and connects directly to live ROS 2 telemetry and persistent SQLite storage:

```bash
# In your terminal
python3 scripts/run_dashboard.py

# Open in any browser:
http://127.0.0.1:5050
```

### 3. Automated V2 Verification Suite (10 Tests)

Run the comprehensive 10-stage test suite validating dynamic obstacles, route costs, candidate alternatives, sensor fusion, dead-end backtracking, recovery, and dashboard APIs:

```bash
python3 scripts/test_v2_scenarios.py
```

### 4. Interactive Mission Demonstration Mode

Walk through the complete autonomous mission intelligence lifecycle step-by-step:

```bash
python3 scripts/demo_mode.py
# Or fast execution:
python3 scripts/demo_mode.py --fast
```

### 5. Baseline Experiments Suite

Run the 5 baseline verification scenarios:

```bash
python3 scripts/run_all_experiments.py
```

---

## Start and Goal Selection (User Workflow)

Setting the initial robot position and commanding destination goals is intuitive and supports four independent modalities:

### 1. Defining the START Position

The AGV begins at the Reception area entrance ($x = 0.0\text{ m}, y = -11.0\text{ m}, \text{yaw} = 1.57\text{ rad}$).

- **Via CLI Script**:
  ```bash
  python3 scripts/set_start.py --location start_a
  # Or with custom coordinates:
  python3 scripts/set_start.py --x 0.0 --y -11.0 --yaw 1.57
  ```
- **Via RViz**: Click **2D Pose Estimate** on the top toolbar and drag the green arrow on the map at the robot's physical location.

### 2. Defining the GOAL Destination

- **Option A: High-Level Semantic Strings (Vision-Memory Guided)**:
  Publish the target room name to `/navigation/goal_label`. The Decision Engine computes topological corridors, queries directional signs from SQLite, and routes the robot:
  ```bash
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'HOSPITAL'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'WAREHOUSE'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'LAB'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'OFFICE'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'STORAGE'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'CAFETERIA'}"
  ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'EXIT'}"
  ```
- **Option B: Predefined Metric Locations**:
  ```bash
  python3 scripts/send_goal.py --list
  python3 scripts/send_goal.py --location hospital
  python3 scripts/send_goal.py --location warehouse
  ```
- **Option C: Interactive 2D Nav2 Goal in RViz**: Click **Nav2 Goal** and click on any point on the occupancy grid.
- **Option D: Graphical Interface**: Click the destination buttons directly in `navigation_control.py`.

---

## YOLOv8 Object Detection Pipeline

- **Architecture**: Ultralytics YOLOv8n (Nano variant, 3.2M parameters) optimized for high frame-rate mobile robotics inference.
- **Dataset**: 405 simulation frames captured across multiple lighting conditions and viewpoints in Gazebo (`datasets/gazebo_cv/`).
- **Classes (8)**: `person`, `chair`, `box`, `cone`, `pallet`, `shelf`, `hospital_bed`, `cart`.
- **Inference Hardware**: NVIDIA GeForce RTX 3050 Laptop GPU (`cuda:0`).
- **Performance**:
  - **mAP@0.50**: **96.2%** on Gazebo validation assets (**97.5%** on test split).
  - **Latency**: **5.62 ms** average inference latency (**116.5–144.2 FPS** throughput).
  - **VRAM Utilization**: **173 MiB** dedicated GPU memory.
- **Confidence & IoU Thresholds**: $\text{conf} = 0.45$, $\text{IoU} = 0.45$.
- **Training from Scratch**:
  ```bash
  python3 scripts/train_yolo.py --epochs 50 --imgsz 640 --batch 16
  ```

---

## Directional Sign Perception

- **Node**: `/sign_detection_node` (`autonomous_robot_perception`).
- **Textures**: 15 semantic sign textures mounted on corridor junctions (e.g., `hospital_straight.png`, `warehouse_right.png`, `emergency_exit.png`).
- **Detection Algorithm**: Multi-threshold contour segmentation, bounding box aspect-ratio gating, and normalized cross-correlation template matching.
- **Performance**: **100.0% recognition accuracy** (15/15 signs recognized), $< 4\text{ ms}$ processing latency per frame.
- **Integration**: Decoded signs are published to `/vision/signs` and logged into SQLite (`navigation_memory.db`) to corroborate topological edge orientations.

---

## Camera-LiDAR Sensor Fusion

Monocular 2D bounding boxes lack spatial depth, while 2D planar LiDAR lacks semantic category labels. The `/lidar_camera_fusion_node` merges both sensor streams:

1. **Azimuth Angle Projection**: Bounding box pixel bounds $[u_{\min}, u_{\max}]$ are converted into horizontal angular bounds $[\theta_{\min}, \theta_{\max}]$ using camera focal length $f_x$ and principal point $c_x$.
2. **LiDAR Range Association**: LiDAR scan rays falling within the angular sector are extracted.
3. **Median Filtering**: Background returns are eliminated using median range selection ($r_{\text{med}}$).
4. **Coordinate Transformation**: Obstacle positions are transformed into 3D Cartesian coordinates in the robot's `base_link` frame.
5. **Costmap Ingestion**: Published to `/vision/costmap_obstacles` as a `sensor_msgs/PointCloud2` stream, allowing Nav2's local costmap to inflate obstacles and execute smooth clearance maneuvers.

---

## Navigation Logic & Topological Memory

The navigation engine operates hierarchically:
1. **Decision Engine (`decision_engine_node.py`)**: Responsible for mission-level routing. Queries SQLite database (`config/navigation_memory.db`) containing the facility's topological network (38 nodes, 80 directed edges).
2. **Dijkstra / A\* Solver (`topological_graph.py`)**: Computes optimal corridor sequences between junctions.
3. **Nav2 Execution**: Waypoint sub-goals are sent to Nav2 via the `/navigate_to_pose` action client. Nav2 executes collision-free metric trajectories using Regulated Pure Pursuit.

> [!NOTE]
> During autonomous navigation, the robot uses **AMCL** localization against pre-built metric occupancy grids (`maps/realistic_facility_map.yaml`). A dedicated SLAM mapping mode is also supported for mapping new facilities: `./scripts/start_project.sh --slam`.

---

## Dynamic Obstacle Handling & Replanning

When a dynamic obstacle (e.g., a cart or box cluster) blocks an active corridor:
1. The obstacle is detected by YOLO and localized by LiDAR fusion.
2. If the obstacle falls within the clearance radius of the forward edge, the Decision Engine increases the edge cost to $1,000,000$.
3. The active Nav2 sub-goal is immediately aborted.
4. The topological planner recalculates an alternate route in **< 1 ms**, diverting the AGV around the obstruction (e.g., via the **West Bypass** corridor: `junction_4 -> west_bypass_mid -> west_bypass_north -> storage`).

---

## Dead-End Avoidance

Cul-de-sac dead ends (`dead_end_1`, `dead_end_2`, `dead_end_3`) are permanently registered in the topological memory graph with a cost penalty of $100,000$. When evaluating paths between rooms, the Dijkstra planner prunes dead-end edges, ensuring **0 cul-de-sac incursions** during normal operations.

---

## Experimental Results & Verification

> [!IMPORTANT]
> **SIMULATION NOTICE**: All quantitative results below were measured exclusively in the **Gazebo Sim 10.5** simulation environment on **ROS 2 Lyrical**. They are simulation benchmarks, not physical robot trials.

### Representative Simulation Run (Full Facility Traversal)

| Metric | Measured Simulation Result | Evaluation Details |
| :--- | :--- | :--- |
| **Traversal Distance** | **29.18 m** | Multi-department realistic facility |
| **Traversal Time** | **73.77 s** | Differential Drive AGV ($v_{\max} = 0.5\text{ m/s}$) |
| **Minimum LiDAR Clearance** | **0.34 m** | Measured across all narrow doorways and corners |
| **Custom YOLOv8n mAP@0.50** | **96.2%** | Gazebo asset validation set (405 frames) |
| **Corridor Replanning Latency** | **< 1 ms** | Instantaneous dynamic bypass activation |
| **Cul-de-Sac Incursions** | **0** | Strict dead-end pruning ($10^5$ penalty) |

### Automated Scenario Test Suite (`scripts/run_all_experiments.py`)

```bash
python3 scripts/run_all_experiments.py
```

- **Scenario 1 (Sign Perception)**: **100.0% accuracy** (15/15 signs recognized, conf: 0.99, latency: 1.12 s total).
- **Scenario 2 (Dynamic Replanning)**: **PASSED** (< 1 ms replan time; West Bypass correctly selected).
- **Scenario 3 (Dead-End Avoidance)**: **PASSED** (0 cul-de-sac incursions across 7 destination queries).
- **Scenario 4 (SQLite Memory Persistence)**: **PASSED** (100% data retention across simulated process restarts).

For comprehensive data tables, loss plots, and ablation metrics, refer to [docs/RESULTS.md](docs/RESULTS.md).

---

## Troubleshooting Guide

### 1. ROS 2 Environment Not Found
```bash
# Ensure ROS 2 Lyrical is sourced
source /opt/ros/lyrical/setup.bash
source ~/CV_Autonomous_Navigation/ros2_ws/install/setup.bash
```

### 2. CUDA Acceleration Fallback
If `object_detection_node` reports `Running on CPU`:
- Verify NVIDIA driver: `nvidia-smi` (Driver $\ge 535$).
- Check PyTorch CUDA availability:
  ```bash
  python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
  ```

### 3. Model Weights Missing
If `models/custom_yolov8n/weights/best.pt` is a small pointer file:
```bash
cd ~/CV_Autonomous_Navigation
git lfs pull
```
Alternatively, train custom weights from the local dataset:
```bash
python3 scripts/train_yolo.py
```

### 4. Stale Gazebo or Nav2 Processes
If Gazebo or RViz fails to launch or reports port binding errors:
```bash
./scripts/stop_navigation.sh
```

---

## Future Improvements

- **Physical AGV Deployment**: Transfer the software stack to a physical differential-drive robot (e.g., TurtleBot 4 or Clearpath base) with sensor calibration.
- **3D LiDAR Integration**: Upgrade from planar 2D LiDAR to 3D LiDAR (e.g., Velodyne VLP-16) for multi-height obstacle classification.
- **Visual-Inertial Odometry (VIO)**: Fuse camera optical flow with IMU for GPS-denied localization resilience.
- **Multi-Robot Swarm Coordination**: Share persistent topological memory across multiple mobile robots via ROS 2 Zenoh DDS.

---

## License & Attribution

- Original software packages and navigation modules are licensed under the [Apache-2.0 License](LICENSE).
- Third-party simulation assets (AWS RoboMaker hospital/warehouse models, Gazebo Fuel assets) and library attributions are documented in [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md).
