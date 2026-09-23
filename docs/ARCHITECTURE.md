# System Architecture: Semantic Memory & Vision-LiDAR Autonomous Navigation

This document specifies the technical architecture, data pipeline, coordinate frames, node-topic graph, and algorithmic decision models of the **CV Autonomous Navigation** platform at an engineering specification level.

---

## 1. System Overview & Layered Architecture

The system augments classical metric 2D navigation (Nav2) with an active cognitive perception layer combining:
1. **GPU-accelerated visual obstacle detection** (Ultralytics YOLOv8n fine-tuned on Gazebo assets).
2. **Deterministic signboard recognition** (Multi-channel OpenCV template matching).
3. **Planar LiDAR-Camera range-bearing sensor fusion**.
4. **Relational topological memory** implemented in SQLite with Dijkstra/A* heuristic routing.
5. **Dynamic corridor replanning** and **cul-de-sac dead-end avoidance**.

```
+---------------------------------------------------------------------------------------+
|                                    GAZEBO SIM 10.5                                    |
|   - 32m x 26m Multi-Room Facility (Hospital, Warehouse, Office, Lab, Storage, Cafe)   |
|   - Diff-Drive AGV with Sensors: RGB Camera, 2D LiDAR (360 deg, 10Hz), IMU, Encoders  |
+-------------------------------------------+-------------------------------------------+
                                            | (GZ Transport)
                                            v
+---------------------------------------------------------------------------------------+
|                             ROS-GAZEBO BRIDGE LAYER                                   |
|   - Node: /ros_gz_bridge (parameter_bridge)                                            |
|   - Bridges: /camera/image_raw, /scan, /odom, /tf, /cmd_vel                            |
+-------------------------------------------+-------------------------------------------+
                                            |
         +----------------------------------+----------------------------------+
         |                                                                     |
         v /camera/image_raw                                                   v /scan
+-------------------------------------+                               +-----------------+
|        PERCEPTION PIPELINE          |                               |  LIDAR PIPELINE |
|                                     |                               |                 |
| 1. /camera_node                     |                               | - Range gating  |
|    Formats & bridges RGB stream     |                               | - Angle indexing|
|                                     |                               +--------+--------+
| 2. /object_detection_node           |                                        |
|    YOLOv8n (RTX 3050 cuda:0)        |                                        |
|    Publishes: /vision/detections    |                                        |
|                                     |                                        |
| 3. /sign_detection_node             |                                        |
|    15 PBR sign templates            |                                        |
|    Publishes: /vision/signs         |                                        |
+------------------+------------------+                                        |
                   |                                                           |
                   +------------------------->+<-------------------------------+
                                              |
                                              v
+---------------------------------------------------------------------------------------+
|                           CAMERA-LIDAR SENSOR FUSION LAYER                            |
|   - Node: /lidar_camera_fusion_node                                                   |
|   - Projects bounding box azimuth angles to LiDAR scan beams                          |
|   - Computes metric 3D obstacle coordinates in base_link                              |
|   - Publishes: /vision/obstacles (SemanticObstacleArray)                              |
|   - Publishes: /vision/costmap_obstacles (sensor_msgs/PointCloud2)                    |
+---------------------------------------------+-----------------------------------------+
                                              |
                                              v
+---------------------------------------------------------------------------------------+
|                           TOPOLOGICAL MEMORY & DECISION LAYER                         |
|                                                                                       |
| 1. Persistent Storage (SQLite): config/navigation_memory.db                           |
|    Tables: nodes, edges, signs, traversals, obstacles                                 |
|                                                                                       |
| 2. Topological Graph: autonomous_robot_navigation/topological_graph.py                 |
|    NetworkX digraph with cost penalties: Dead-end ($10^5$), Blocked ($10^6$)          |
|                                                                                       |
| 3. Decision Engine: /decision_engine_node                                             |
|    - Subscribes: /navigation/goal_label (e.g. 'HOSPITAL', 'WAREHOUSE')                |
|    - Translates semantic room queries into topological waypoint sequences             |
|    - Dynamically monitors forward edge clearance; triggers West Bypass on blockage   |
|    - Dispatches sub-goals via Nav2 Action Client (/navigate_to_pose)                  |
+---------------------------------------------+-----------------------------------------+
                                              |
                                              v /navigate_to_pose (Action)
+---------------------------------------------------------------------------------------+
|                                    NAV2 MOTION EXECUTION                              |
|   - Global Costmap: Static SLAM map + Obstacle Inflation Layer                        |
|   - Local Costmap: Live /scan + /vision/costmap_obstacles PointCloud2                 |
|   - Controller: Regulated Pure Pursuit (RPP) -> /cmd_vel                              |
|   - Localization: AMCL (/amcl_pose) referenced against map frame                     |
+---------------------------------------------------------------------------------------+
```

---

## 2. Verified ROS 2 Node Architecture

The production stack deploys the following verified ROS 2 nodes:

| Node Name | Package | Executable / Script | Description |
| :--- | :--- | :--- | :--- |
| **`/ros_gz_bridge`** | `ros_gz_bridge` | `parameter_bridge` | Bridges sensor data and command velocities between Gazebo Sim and ROS 2 DDS. |
| **`/robot_state_publisher`** | `robot_state_publisher` | `robot_state_publisher` | Broadcasts static and kinematic coordinate frame transforms from URDF Xacro models. |
| **`/camera_node`** | `autonomous_robot_perception` | `camera_node.py` | Formats and timestamps raw optical frames from the simulated camera. |
| **`/object_detection_node`** | `autonomous_robot_perception` | `object_detection_node.py` | Runs GPU-accelerated custom YOLOv8n inference on `/camera/image_raw`. |
| **`/sign_detection_node`** | `autonomous_robot_perception` | `sign_detection_node.py` | Extracts, verifies, and decodes 15 directional signboard textures. |
| **`/lidar_camera_fusion_node`**| `autonomous_robot_perception` | `lidar_camera_fusion_node.py` | Fuses visual bounding boxes with 2D LiDAR range rays to determine 3D spatial obstacle vectors. |
| **`/navigation_perception_node`** | `autonomous_robot_perception` | `navigation_perception_node.py` | Ingests obstacle detections and generates synthetic costmap points and safety warnings. |
| **`/decision_engine_node`** | `autonomous_robot_navigation` | `decision_engine_node.py` | High-level executive coordinating semantic goal parsing, topological routing, and Nav2 dispatch. |
| **`/navigation_visualizer_node`** | `autonomous_robot_navigation` | `navigation_visualizer_node.py` | Publishes RViz MarkerArrays representing topological graph nodes, edges, and active trajectories. |
| **`/amcl`** | `nav2_amcl` | `amcl` | Adaptive Monte Carlo Localization against the static facility map. |
| **`/controller_server`** | `nav2_controller` | `controller_server` | Regulated Pure Pursuit local path tracker publishing velocities to `/cmd_vel`. |
| **`/planner_server`** | `nav2_planner` | `planner_server` | NavFn global Dijkstra/A* path planner computing metric paths between topological waypoints. |

---

## 3. Topic Architecture & Interface Definitions

```
+---------------------------------------------------------------------------------------------+
|                                    ACTIVE TOPIC GRAPH                                       |
+---------------------------------------------------------------------------------------------+
Topic Name                     | Message Type                        | Publisher -> Subscriber
-------------------------------+-------------------------------------+------------------------
/camera/image_raw              | sensor_msgs/msg/Image               | gz_bridge -> camera_node, object_detection, sign_detection
/camera/camera_info            | sensor_msgs/msg/CameraInfo          | gz_bridge -> perception nodes
/scan                          | sensor_msgs/msg/LaserScan           | gz_bridge -> fusion_node, nav2_costmaps
/odom                          | nav_msgs/msg/Odometry               | gz_bridge -> amcl, nav2_controller
/tf, /tf_static                | tf2_msgs/msg/TFMessage              | rsp, amcl -> nav2, visualizer
/vision/detections             | interfaces/msg/Detection2DArray     | object_detection -> fusion_node
/vision/annotated_image        | sensor_msgs/msg/Image               | object_detection -> rviz2
/vision/signs                  | interfaces/msg/SignDetectionArray   | sign_detection -> decision_engine
/vision/annotated_signs        | sensor_msgs/msg/Image               | sign_detection -> rviz2
/vision/obstacles              | interfaces/msg/SemanticObstacleArray| fusion_node -> decision_engine
/vision/costmap_obstacles      | sensor_msgs/msg/PointCloud2         | fusion_node -> nav2_local_costmap
/vision/obstacle_warning       | interfaces/msg/ObstacleWarning      | nav_perception -> decision_engine
/navigation/goal_label         | std_msgs/msg/String                 | user/cli -> decision_engine
/navigation/active_path        | visualization_msgs/msg/MarkerArray  | decision_engine -> rviz2
/navigation/topological_graph  | visualization_msgs/msg/MarkerArray  | visualizer_node -> rviz2
/amcl_pose                     | geometry_msgs/PoseWithCovariance    | amcl -> decision_engine
/navigate_to_pose (Action)     | nav2_msgs/action/NavigateToPose     | decision_engine -> nav2_bt_navigator
/cmd_vel                       | geometry_msgs/msg/Twist             | nav2_controller -> gz_bridge
```

---

## 4. Sensor Data Flow & Processing Pipelines

### 4.1 Monocular Camera Pipeline
- **Resolution**: $640 \times 480$ pixels @ 30 FPS.
- **Horizontal FOV**: $\text{HFOV} = 1.089\text{ rad} \approx 62.4^\circ$.
- **Camera Intrinsics**:
  $$f_x = \frac{W / 2}{\tan(\text{HFOV} / 2)} = \frac{320}{\tan(0.5445)} \approx 528.2\text{ px}, \quad c_x = 320.0, \quad c_y = 240.0$$

### 4.2 YOLOv8 Inference Pipeline
1. Incoming RGB frame converted from ROS `sensor_msgs/Image` via `cv_bridge` (`bgr8`).
2. Letterbox pre-processing to $640 \times 640$ tensor.
3. FP16 tensor loaded into GPU VRAM on `cuda:0` (NVIDIA RTX 3050 Laptop).
4. Non-Maximum Suppression (NMS) with $\text{conf\_threshold} = 0.45$ and $\text{iou\_threshold} = 0.45$.
5. Outputs: Class ID, class name, bounding box $[x_{\min}, y_{\min}, x_{\max}, y_{\max}]$, and confidence score $\in [0, 1]$.

### 4.3 LiDAR Range-Bearing Extrusion & Fusion
For each detected 2D bounding box with horizontal bounds $[u_{\min}, u_{\max}]$:
1. Compute the angular aperture bounds in camera optical frame:
   $$\theta_{\min} = -\arctan\left(\frac{u_{\max} - c_x}{f_x}\right), \quad \theta_{\max} = -\arctan\left(\frac{u_{\min} - c_x}{f_x}\right)$$
2. Map angles to LiDAR ray indices:
   $$i = \left\lfloor \frac{\theta - \theta_{\text{start}}}{\Delta \theta} \right\rfloor$$
3. Filter valid returns: $r_i \in [r_{\min}, r_{\max}]$.
4. Compute median range $r_{\text{med}}$ to reject background clutter.
5. Compute Cartesian position in robot `base_link` frame:
   $$x = r_{\text{med}} \cos(\theta_{\text{mid}}) + \Delta x_{\text{sensor}}, \quad y = r_{\text{med}} \sin(\theta_{\text{mid}}) + \Delta y_{\text{sensor}}$$
6. Pack obstacle coordinates into a `sensor_msgs/PointCloud2` and inject directly into Nav2's local costmap observation buffer.

---

## 5. Cognitive Memory & Decision Engine

### 5.1 SQLite Relational Schema (`config/navigation_memory.db`)
- **`nodes`**: Node ID (`TEXT`), name, $(x, y, \theta)$, node type (`junction`, `room`, `dead_end`), semantic label (`HOSPITAL`, `WAREHOUSE`, etc.).
- **`edges`**: `from_node`, `to_node`, Euclidean distance, effective cost, traversal count, blockage flag (`is_blocked`), dead-end flag (`is_dead_end`).
- **`signs`**: Observed signboard ID, text, direction (`STRAIGHT`, `LEFT`, `RIGHT`), confidence, timestamp.
- **`traversals`**: Historical edge traversal outcome, duration, timestamp.
- **`obstacles`**: Dynamic blockage history with timestamp and obstacle class.

### 5.2 Routing Algorithms & Penalty Weighting
The graph planner runs Dijkstra and A* across the topological graph $G = (V, E)$ with edge weight function:
$$W(u, v) = D(u, v) + C_{\text{dead\_end}} \cdot \mathbb{I}_{\text{dead\_end}}(v) + C_{\text{blocked}} \cdot \mathbb{I}_{\text{blocked}}(u, v)$$
Where:
- $D(u, v)$ is Euclidean corridor length.
- $C_{\text{dead\_end}} = 100,000$ ensures cul-de-sacs are strictly avoided during normal transit.
- $C_{\text{blocked}} = 1,000,000$ immediately diverts the planner away from obstructed corridors.

### 5.3 Dynamic Replanning State Machine
```
[IDLE / AT_REST]
       |
       | Goal received (/navigation/goal_label)
       v
[COMPUTE_TOPOLOGICAL_ROUTE]
       |
       | Waypoints generated
       v
[EXECUTE_SUBGOAL (Nav2 Action)] <---------------+
       |                                        | Next waypoint
       | Live sensor check                      |
       +---> Forward corridor blocked?          |
       |        |                               |
       |        | YES                           |
       |        v                               |
       |     [ABORT_NAV2_SUBGOAL]               |
       |        |                               |
       |        v                               |
       |     [SET_EDGE_BLOCKED (Cost 10^6)]     |
       |        |                               |
       |        v                               |
       |     [REPLAN_DIJKSTRA (West Bypass)] ---+
       |
       +---> Waypoint reached? ---> Destination reached? ---> [MISSION_SUCCESS]
```

---

## 6. Simulation & Coordinate Frame Architecture

- **World Frame (`map`)**: Fixed coordinate origin established by AMCL and static SLAM occupancy grid.
- **Odometry Frame (`odom`)**: Continuous, drift-accumulating dead-reckoning reference from wheel encoders.
- **Robot Base (`base_footprint` & `base_link`)**: Center of the differential drive wheelbase at ground level.
- **Sensor Frames**:
  - `lidar_link`: Planar LiDAR mounting point ($z = +0.20\text{ m}$).
  - `camera_link` & `camera_optical_link`: Optical axis ($+Z$ forward in optical frame, $+X$ forward in robot frame).

All frames are maintained and broadcast continuously via `robot_state_publisher` and AMCL with a maximum transform latency $< 10\text{ ms}$.
