# System Architecture: Semantic Memory & Vision-LiDAR Autonomous Navigation V2

This document specifies the technical architecture, data pipeline, coordinate frames, node-topic graph, and algorithmic decision models of the **Autonomous Navigation V2** platform.

---

## 1. System Overview & Layered Architecture

The system augments classical metric 2D navigation (Nav2) with an active cognitive perception layer, multi-criteria route optimization, and dedicated web teleoperation:
1. **GPU-Accelerated Visual Perception**: Ultralytics YOLOv8 fine-tuned on warehouse and facility assets.
2. **Deterministic Signboard Recognition**: Multi-channel OpenCV template matching with associative memory.
3. **Synchronized LiDAR-Camera Sensor Fusion**: Range-bearing association, human-readable directional classification (`Front-Center`, `Front-Right`, `Front-Left`, `Left`, `Right`), and 3D metric bounding boxes.
4. **Multi-Criteria Topological Route Planner**: Heuristic optimization based on distance, obstacle density, history failure penalties, turn counts, narrow corridor penalties, and past traversal durations.
5. **Candidate Alternative Routes**: Dynamic generation of $K=3$ alternative corridors with real-time cost breakdown.
6. **Dead-End Intelligence & Backtracking**: Safe stop, coordinate persistence in SQLite, and reverse backtracking.
7. **Robust Recovery Behaviors**: Audited recovery sequences for stalled, oscillating, or low-clearance conditions.
8. **Persistent SQLite Journey Memory**: Long-term storage of trips, route segment reliability, and performance analytics.
9. **Dedicated Web Navigation Dashboard**: Professional real-time interface (port 5050) with interactive HTML5 canvas map, live event logs, and analytical charts.

```
+---------------------------------------------------------------------------------------+
|                                    GAZEBO SIM 10.5                                    |
|   - 35m x 30m Realistic Facility (Storage Racks, Pallets, Workstations, Rooms A/B)   |
|   - 4 Dynamic Obstacles: Warehouse Cart, Hospital Trolley, Forklift, Moving Person    |
|   - AGV with Sensors: RGB Camera (30 FPS), 2D LiDAR (360 deg, 20Hz), IMU, Encoders    |
+-------------------------------------------+-------------------------------------------+
                                            | (GZ Transport)
                                            v
+---------------------------------------------------------------------------------------+
|                             ROS-GAZEBO BRIDGE LAYER                                   |
|   - Node: /ros_gz_bridge (parameter_bridge)                                            |
|   - Bridges: /camera/image_raw, /scan, /odom, /tf, /cmd_vel                            |
|   - Dynamic Bridges: /cart/cmd_vel, /trolley/cmd_vel, /forklift/cmd_vel, /person/cmd_vel|
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
|    YOLOv8n Inference                |                                        |
|    Publishes: /vision/detections    |                                        |
|                                     |                                        |
| 3. /sign_detection_node             |                                        |
|    30 Directional & Room Signs      |                                        |
|    Publishes: /vision/signs         |                                        |
+------------------+------------------+                                        |
                   |                                                           |
                   +------------------------->+<-------------------------------+
                                              |
                                              v
+---------------------------------------------------------------------------------------+
|                           CAMERA-LIDAR SENSOR FUSION LAYER                            |
|   - Node: /lidar_camera_fusion_node                                                   |
|   - Resolves camera-LiDAR baseline parallax across depth range [0.3m, 6.0m]           |
|   - Computes 3D coordinates (x, y, z) and direction label in robot base_link          |
|   - Publishes: /vision/semantic_obstacles, /fused_objects (SemanticObstacleArray)     |
|   - Publishes: /vision/fused_object_markers (visualization_msgs/MarkerArray)          |
+---------------------------------------------+-----------------------------------------+
                                              |
                                              v
+---------------------------------------------------------------------------------------+
|                           TOPOLOGICAL MEMORY & DECISION ENGINE V2                     |
|                                                                                       |
| 1. Persistent Memory (SQLite): config/navigation_memory.db                            |
|    Tables: nodes, edges, signs, traversals, obstacles, journeys,                      |
|            route_segments, dead_ends, recovery_events                                 |
|                                                                                       |
| 2. Topological Graph: autonomous_robot_navigation/topological_graph.py                 |
|    - Multi-criteria route cost function with configurable YAML weights                |
|    - K-Shortest alternative routes generator with itemized cost breakdown             |
|                                                                                       |
| 3. Decision Engine: /decision_engine_node                                             |
|    - 10-State Navigation State Machine                                                |
|    - Dynamic obstacle replanning around blocked corridors                             |
|    - Dead-end intelligence: Stop -> Log coords -> Safe reverse -> Re-route            |
|    - Recovery system: Stalled timeout, critical clearance, Nav2 abort recovery        |
|    - Telemetry & Event stream: /navigation/mission_status, /navigation/events         |
+-----------------------+---------------------+-----------------------------------------+
                        |                     |
     /navigate_to_pose  v                     v /navigation/mission_status
+-----------------------------------+     +---------------------------------------------+
|        NAV2 MOTION EXECUTION      |     |       DEDICATED NAVIGATION DASHBOARD        |
| - Global & Local Costmaps         |     | - Backend Node: /dashboard_backend (Flask)  |
| - Regulated Pure Pursuit (/cmd_vel|     | - Port: 5050 (http://127.0.0.1:5050)        |
| - AMCL Localization (/amcl_pose)  |     | - Screen 1: Real-time Mission Monitor       |
+-----------------------------------+     | - Screen 2: Travel History & Analytics      |
                                          | - Screen 3: Route Heatmap & Topology        |
                                          +---------------------------------------------+
```

---

## 2. Core ROS 2 Nodes

| Node Name | Package | Executable | Description |
| :--- | :--- | :--- | :--- |
| **`/ros_gz_bridge`** | `ros_gz_bridge` | `parameter_bridge` | Bridges simulation sensors and velocity commands. |
| **`/dynamic_obstacles_node`** | `autonomous_robot_navigation` | `dynamic_obstacles_node.py` | Controls moving warehouse carts, trolleys, forklifts, and walking people. |
| **`/camera_node`** | `autonomous_robot_perception` | `camera_node.py` | Formats and timestamps camera frames. |
| **`/object_detection_node`** | `autonomous_robot_perception` | `object_detection_node.py` | Runs fine-tuned YOLOv8 custom object detection. |
| **`/sign_detection_node`** | `autonomous_robot_perception` | `sign_detection_node.py` | Detects visual signs and decodes navigation markers. |
| **`/lidar_camera_fusion_node`** | `autonomous_robot_perception` | `lidar_camera_fusion_node.py` | Synchronizes LiDAR rays with bounding boxes; publishes `/fused_objects` and RViz 3D markers. |
| **`/decision_engine_node`** | `autonomous_robot_navigation` | `decision_engine_node.py` | Multi-criteria planner, state machine, replanner, dead-end backtrack, and recovery coordinator. |
| **`/navigation_visualizer_node`**| `autonomous_robot_navigation` | `navigation_visualizer_node.py` | Publishes topological graph, nodes, and active path lines for RViz. |
| **`/dashboard_backend`** | `autonomous_robot_navigation` | `dashboard_backend.py` | Serves the professional Web Navigation Dashboard on port 5050. |
| **`/amcl`** | `nav2_amcl` | `amcl` | Adaptive Monte Carlo Localization on static facility map. |
| **`/controller_server`** | `nav2_controller` | `controller_server` | Regulated Pure Pursuit path tracking to `/cmd_vel`. |
| **`/planner_server`** | `nav2_planner` | `planner_server` | NavFn global Dijkstra/A* path planner. |

---

## 3. Active Topic Graph & Interfaces

| Topic Name | Message Type | Publisher $\rightarrow$ Subscriber |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `ros_gz_bridge` $\rightarrow$ perception nodes |
| `/scan` | `sensor_msgs/msg/LaserScan` | `ros_gz_bridge` $\rightarrow$ fusion, decision, dashboard, costmaps |
| `/odom` | `nav_msgs/msg/Odometry` | `ros_gz_bridge` $\rightarrow$ AMCL, decision engine, dashboard |
| `/vision/detections` | `interfaces/msg/Detection2DArray` | `object_detection_node` $\rightarrow$ fusion, dashboard |
| `/fused_objects` | `interfaces/msg/SemanticObstacleArray` | `lidar_camera_fusion_node` $\rightarrow$ decision engine, dashboard |
| `/vision/fused_object_markers` | `visualization_msgs/msg/MarkerArray` | `lidar_camera_fusion_node` $\rightarrow$ RViz2 |
| `/vision/signs` | `interfaces/msg/SignDetectionArray` | `sign_detection_node` $\rightarrow$ decision engine |
| `/navigation/goal_label` | `std_msgs/msg/String` | dashboard / CLI $\rightarrow$ decision engine |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz / dashboard $\rightarrow$ decision engine |
| `/navigation/control` | `std_msgs/msg/String` | dashboard $\rightarrow$ decision engine |
| `/navigation/mission_status` | `std_msgs/msg/String` (JSON) | decision engine $\rightarrow$ dashboard |
| `/navigation/current_state` | `std_msgs/msg/String` | decision engine $\rightarrow$ dashboard |
| `/navigation/alternative_paths` | `std_msgs/msg/String` (JSON) | decision engine $\rightarrow$ dashboard |
| `/navigation/events` | `std_msgs/msg/String` (JSON) | decision engine $\rightarrow$ dashboard |
| `/navigation/active_path` | `std_msgs/msg/String` | decision engine $\rightarrow$ visualizer, dashboard |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | controller / decision engine $\rightarrow$ `ros_gz_bridge` |
| `/cart/cmd_vel`, `/trolley/cmd_vel` | `geometry_msgs/msg/Twist` | `dynamic_obstacles_node` $\rightarrow$ `ros_gz_bridge` |
| `/forklift/cmd_vel`, `/person/cmd_vel` | `geometry_msgs/msg/Twist` | `dynamic_obstacles_node` $\rightarrow$ `ros_gz_bridge` |

---

## 4. Enhanced Sensor Fusion

For every detected visual object:
1. Projects bounding box bounds into LiDAR angular envelope, accounting for the 0.12m longitudinal baseline between the camera and LiDAR.
2. Extracts range from the closest consistent obstacle cluster using 20th percentile filtering.
3. Computes human-readable direction:
   - Bearing $> +45^\circ$: `Left`
   - $+10^\circ \le$ Bearing $\le +45^\circ$: `Front-Left`
   - $-10^\circ \le$ Bearing $\le +10^\circ$: `Front-Center`
   - $-45^\circ \le$ Bearing $< -10^\circ$: `Front-Right`
   - Bearing $< -45^\circ$: `Right`
4. Calculates 3D metric coordinates $(x, y, z)$ in the robot's `base_link` frame:
   $$x = 0.10 + d \cdot \cos(\theta), \quad y = 0.00 + d \cdot \sin(\theta), \quad z = 0.25\text{m}$$
5. Publishes interactive RViz 3D markers with color coding (Red = dynamic obstacle, Amber = static obstacle) and floating text tags.
