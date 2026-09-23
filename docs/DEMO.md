# Demonstration & Reproducibility Guide: Autonomous Navigation

This guide provides a step-by-step walkthrough for reproducing the complete **Vision-LiDAR Semantic Autonomous Navigation** system demonstration. It is structured for academic review, technical presentations, and faculty evaluations.

---

## 1. System Requirements & Environment

| Component | Specification |
| :--- | :--- |
| **Operating System** | Ubuntu 26.04 LTS (x86_64) |
| **ROS 2 Distribution** | ROS 2 Lyrical |
| **Simulator** | Gazebo Sim 10.5.0 (`gz sim`) |
| **Python Environment** | Python 3.14 with PyTorch 2.14.0+cu130, Ultralytics YOLOv8, OpenCV 5.0 |
| **GPU Acceleration** | NVIDIA GeForce RTX 3050 Laptop GPU (CUDA 13.0, `cuda:0`) |

---

## 2. Terminal Environment Setup

Open a terminal and source the core ROS 2 and workspace environments:

```bash
# Source ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash

# Navigate to project repository
cd ~/CV_Autonomous_Navigation

# Source Python virtual environment (if using .venv)
if [ -d ".venv" ]; then
    source .venv/bin/activate
fi

# Source ROS 2 Colcon workspace
source ros2_ws/install/setup.bash
```

> [!TIP]
> You can also launch the full stack using the unified startup script `./scripts/start_project.sh`, which automatically cleans up any stale simulation processes and performs workspace validation.

---

## 3. Starting the Demonstration

### Option A: Standard Full System Launch (Recommended)

Launch the complete stack (Gazebo simulation, robot description, ROS-Gazebo bridge, AMCL localization, Nav2 navigation stack, perception pipeline, decision engine, visualizer, and RViz):

```bash
ros2 launch autonomous_robot_bringup bringup.launch.py world:=realistic
```

Or using the launcher script:

```bash
./scripts/start_project.sh --world realistic_facility_world
```

### Option B: Interactive Graphical Control Panel

For live demonstrations and committee presentations, an interactive control GUI is available:

```bash
python3 scripts/navigation_control.py
```

This panel displays real-time GPU/CUDA acceleration status, active ROS nodes, one-click start/stop buttons, and predefined mission dispatchers.

---

## 4. Understanding the Display Windows

When launch completes, two primary windows will appear:

1. **Gazebo Sim 10.5 Window**:
   - Renders the full 32 m × 26 m multi-department indoor facility with physical lighting and rigid-body dynamics.
   - Contains 6 functional zones: **Hospital Ward**, **Industrial Warehouse**, **Executive Office**, **Research Laboratory**, **Storage Depot**, and **Cafeteria**.
   - Contains 15 directional signboards with procedural PBR albedo textures mounted on corridor junctions.
   - Renders the differential-drive AGV equipped with front RGB camera, 2D LiDAR (10 Hz, 360°), and wheel encoders.

2. **RViz2 Visualization Window**:
   - **Robot Model**: Shows live 3D robot articulation and coordinate frames (`base_footprint`, `base_link`, `camera_link`, `lidar_link`).
   - **Costmaps**: Global costmap (static facility geometry) and Local costmap (dynamic 360° obstacle inflation).
   - **Topological Graph Overlay**: Color-coded nodes and edge networks representing traversable indoor corridors.
   - **Active Path**: Cyan trajectory visualizing the topological route planned by the Decision Engine.
   - **Camera Streams**:
     - `/camera/image_raw`: Live monocular camera feed.
     - `/vision/annotated_image`: YOLOv8 bounding boxes with class labels and confidence.
     - `/vision/annotated_signs`: Bounding boxes for recognized directional signs (e.g., `HOSPITAL -> STRAIGHT`).

---

## 5. Setting Initial Robot Pose (START)

The robot starts by default near the facility entrance (Reception area: $x = 0.0$, $y = -11.0$, $\text{yaw} = 1.57\text{ rad}$).

If AMCL localization requires initialization:

### Method 1: Using the CLI Script
```bash
python3 scripts/set_start.py --location start_a
```
Or with custom map coordinates:
```bash
python3 scripts/set_start.py --x 0.0 --y -11.0 --yaw 1.57
```

### Method 2: Using RViz 2D Pose Estimate
1. Click the **2D Pose Estimate** button in the RViz top toolbar.
2. Click on the map near the robot's visible starting location and drag the green orientation arrow in the direction the robot is facing.
3. Observe the AMCL particle cloud converge onto the robot footprint.

---

## 6. Dispatching Navigation Missions (GOAL)

Navigation goals can be dispatched through multiple user-facing interfaces:

### Interface 1: High-Level Semantic Goals (Vision-Memory Guided)

Send semantic room destination strings directly to the Decision Engine:

```bash
# Navigate to Hospital Ward
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'HOSPITAL'}"

# Navigate to Industrial Warehouse
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'WAREHOUSE'}"

# Navigate to Research Laboratory
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'LAB'}"

# Navigate to Executive Office
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'OFFICE'}"

# Navigate to Storage Depot
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'STORAGE'}"

# Navigate to Cafeteria
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'CAFETERIA'}"

# Navigate to Emergency Exit
ros2 topic pub --once /navigation/goal_label std_msgs/msg/String "{data: 'EXIT'}"
```

### Interface 2: Predefined Metric Waypoints

Use the target dispatcher script to select validated facility coordinates:

```bash
# List all predefined locations
python3 scripts/send_goal.py --list

# Dispatch robot to predefined location
python3 scripts/send_goal.py --location hospital
python3 scripts/send_goal.py --location warehouse
```

### Interface 3: Interactive RViz Goal

Click the **Nav2 Goal** tool in the RViz toolbar and click any target position on the 2D map.

---

## 7. Demonstrating Key Autonomous Behaviors

### Scenario 1: Semantic Sign Guidance
- **Observation**: As the robot approaches corridor intersections, `sign_detection_node` detects directional signs (e.g., `hospital_straight.png` or `warehouse_right.png`).
- **Mechanism**: The detected sign is logged into SQLite (`navigation_memory.db`) with node ID, text, direction, and confidence.
- **Verification**: In RViz, watch the `/vision/annotated_signs` image panel highlight detected signs with blue bounding boxes.

### Scenario 2: Dynamic Obstacle Detection & Sensor Fusion
- **Observation**: Place or observe an indoor hazard (pallet, chair, box, or cart) in the robot's path.
- **Mechanism**:
  1. `object_detection_node` runs YOLOv8n at ~30 FPS on `cuda:0` and detects the obstacle class.
  2. `lidar_camera_fusion_node` extracts LiDAR scan rays aligned with the horizontal pixel bounding box, computing radial distance and bearing.
  3. The obstacle is projected as a 3D point into the robot frame and published to `/vision/costmap_obstacles`.
  4. Nav2's local costmap inflates the obstacle, and Regulated Pure Pursuit steers around it smoothly.

### Scenario 3: Dead-End Avoidance & Topological Pruning
- **Observation**: Dispatch a goal to a location situated beyond a cul-de-sac corridor (e.g., `dead_end_1` or `dead_end_2`).
- **Mechanism**: The Decision Engine queries `topological_graph.py`. Dead-end edges carry a cost penalty of $10^5$. The Dijkstra planner prunes all cul-de-sac paths, preventing the robot from ever entering dead ends.
- **Verification**: Run `python3 scripts/test_scenario_3_dead_end_avoidance.py` to inspect the path costs across all 7 destinations.

### Scenario 4: Dynamic Corridor Blockage & Instant Replanning
- **Observation**: When a main corridor is fully blocked by a dynamic cart or large pallet cluster (e.g., corridor `junction_4 -> storage`):
- **Mechanism**:
  1. `decision_engine_node` detects that the forward corridor is impassable.
  2. The active edge penalty is increased to $10^6$.
  3. The engine immediately aborts the active Nav2 sub-goal.
  4. The engine invokes Dijkstra/A* replanning and selects the **West Bypass** (`junction_4 -> west_bypass_mid -> west_bypass_north -> storage`) in **< 1 ms**.
  5. The new waypoint sequence is streamed to Nav2, and the robot completes the mission without getting stuck.
- **Verification**: Run `python3 scripts/test_scenario_2_dynamic_replanning.py` to verify replanning logic in isolation.

### Scenario 5: Persistent Topological Memory Across Restarts
- **Observation**: Stop the simulation and restart the stack.
- **Mechanism**: `navigation_memory.py` queries `config/navigation_memory.db`. All previously observed signs, edge traversal counts, and dynamic blockage histories are retained.
- **Verification**: Run `python3 scripts/test_scenario_4_memory_persistence.py` to verify cross-session retention.

---

## 8. Executing Automated Test Suite

To demonstrate all scenarios automatically with pass/fail metrics:

```bash
python3 scripts/run_all_experiments.py
```

Expected Output:
```
======================================================================
AUTONOMOUS NAVIGATION SYSTEM - EXPERIMENT SUITE
Environment: ROS 2 Lyrical | Gazebo Sim 10.5 | NVIDIA RTX 3050 (cuda:0)
======================================================================
[INFO] Executing Scenario 1: Semantic Sign Perception & Guidance...
[PASS] All 15 directional signs detected and classified with 100% accuracy.
[INFO] Executing Scenario 2: Dynamic Obstacle Blockage & Replanning...
[PASS] Replanning latency < 1 ms; West Bypass activated.
[INFO] Executing Scenario 3: Dead End Avoidance & Pruning...
[PASS] 0 cul-de-sac incursions across all destination queries.
[INFO] Executing Scenario 4: Navigation Memory & SQLite Persistence...
[PASS] Cross-session relational data verified.
======================================================================
EXPERIMENT SUITE SUMMARY REPORT
======================================================================
Scenario Name                                      | Status   | Duration (s)
----------------------------------------------------------------------
Scenario 1: Semantic Sign Perception & Guidance    | PASSED   | 1.124
Scenario 2: Dynamic Obstacle Blockage & Replanning | PASSED   | 0.500
Scenario 3: Dead End Avoidance & Pruning           | PASSED   | 0.525
Scenario 4: Navigation Memory & SQLite Persistence | PASSED   | 0.525
======================================================================
[ALL EXPERIMENTS COMPLETED SUCCESSFULLY - 100% PASS RATE]
```

---

## 9. Clean Shutdown

To stop all ROS 2 nodes, Nav2 servers, Gazebo processes, and RViz:

```bash
./scripts/stop_navigation.sh
```

Or press `Ctrl+C` in the terminal running `start_project.sh` (the trap handler will cleanly terminate all background child processes).
