# System Architecture: Semantic Memory-Based Autonomous Navigation

## 1. High-Level Architecture Overview

The system enhances standard 2D mobile robot navigation with **semantic memory, computer vision-based sign interpretation, and topological graph decision-making**, while retaining Nav2 as the sole local and global motion execution authority.

```mermaid
flowchart TD
    subgraph Simulation ["Gazebo Sim 10.5 Environment"]
        GZ_WORLD["Complex Maze World<br/>(Junctions, Rooms, Cul-de-sacs)"]
        GZ_ROBOT["Robot Sensors<br/>(RGB Camera, 2D LiDAR, Odom)"]
        GZ_SIGNS["Visual Semantic Signs<br/>(PBR Materials & Directions)"]
    end

    subgraph Perception ["Perception Pipeline (autonomous_robot_perception)"]
        CAM_NODE["Camera Bridge Node"]
        YOLO_NODE["Custom YOLOv8n (RTX 3050 cuda:0)<br/>best.pt (8 classes, 29.5 FPS)"]
        SIGN_NODE["Sign Detection Node<br/>(Contour + Template + Color Verification)"]
        FUSION_NODE["LiDAR-Camera Fusion Node<br/>(Range-Bearing Projection)"]
    end

    subgraph Memory_Decision ["Cognitive & Memory Layer (autonomous_robot_navigation)"]
        SIGN_MEM["Sign Observations<br/>(Node, Text, Direction, Conf)"]
        SQLITE_DB[("SQLite Persistent Memory<br/>navigation_memory.db")]
        TOPO_GRAPH["Topological Graph<br/>(Nodes, Edges, Costs, Dead-Ends)"]
        DECISION_ENG["Decision Engine Node<br/>- Mission Goal Parsing<br/>- Dijkstra/A* Path Planner<br/>- Dead-End Pruning<br/>- Dynamic Replanning Trigger"]
    end

    subgraph Motion_Execution ["Nav2 Navigation Stack"]
        AMCL["AMCL / Map Server<br/>(/amcl_pose, /map)"]
        GLOBAL_COSTMAP["Global Costmap<br/>(Static + Scan + Vision Obstacles)"]
        LOCAL_COSTMAP["Local Costmap<br/>(Dynamic Inflation & Obstacles)"]
        CONTROLLER["RPP / Controller Server<br/>(/cmd_vel)"]
        NAV2_ACTION["/navigate_to_pose Action Server"]
    end

    subgraph UI ["User Interface & Visualization"]
        RVIZ["RViz2 Display<br/>- Topological Graph & Active Path<br/>- Camera & Annotated Signs<br/>- Costmaps & Robot Model"]
    end

    %% Sensor data flow
    GZ_ROBOT -->|/camera/image_raw| CAM_NODE
    GZ_ROBOT -->|/scan| FUSION_NODE
    GZ_ROBOT -->|/scan| LOCAL_COSTMAP
    GZ_ROBOT -->|/odom, /tf| AMCL

    %% Perception flow
    CAM_NODE --> YOLO_NODE
    CAM_NODE --> SIGN_NODE
    YOLO_NODE -->|/vision/detections| FUSION_NODE
    SIGN_NODE -->|/vision/signs| DECISION_ENG
    FUSION_NODE -->|/vision/semantic_obstacles| DECISION_ENG
    FUSION_NODE -->|/vision/costmap_obstacles| LOCAL_COSTMAP

    %% Memory and Decision flow
    SIGN_NODE --> SIGN_MEM
    SIGN_MEM --> SQLITE_DB
    SQLITE_DB <--> TOPO_GRAPH
    AMCL -->|/amcl_pose| DECISION_ENG
    DECISION_ENG <--> TOPO_GRAPH
    DECISION_ENG -->|NavigateToPose.Goal| NAV2_ACTION
    NAV2_ACTION --> CONTROLLER
    CONTROLLER -->|/cmd_vel| GZ_ROBOT

    %% Visualizer
    TOPO_GRAPH -->|/navigation/topological_graph_markers| RVIZ
    DECISION_ENG -->|/navigation/active_path| RVIZ
    SIGN_NODE -->|/vision/annotated_signs| RVIZ
```

---

## 2. Component Breakdown

### 2.1 Perception Pipeline
- **`sign_detection_node`**:
  - Subscribes: `/camera/image_raw`
  - Detects 15 semantic sign textures using multi-threshold contour extraction, aspect-ratio gating, and normalized cross-correlation template matching.
  - Publishes: `/vision/signs` (`SignDetectionArray`) and `/vision/annotated_signs` (`sensor_msgs/Image`).
- **`object_detection_node`**:
  - Runs custom-trained YOLOv8n weights (`best.pt`) on NVIDIA RTX 3050 (`cuda:0`).
  - Detects 8 classes: `person`, `chair`, `box`, `cone`, `pallet`, `shelf`, `hospital_bed`, `cart`.
  - Achieves 0.975 mAP50 at ~30 FPS.
- **`lidar_camera_fusion_node`**:
  - Fuses bounding boxes with 2D LiDAR scan rays to compute metric 3D obstacle locations.
  - Publishes semantic obstacle array and point cloud costmap obstacle layer.

### 2.2 Cognitive Layer & Memory System
- **`topological_graph.py`**:
  - Encapsulates nodes (junctions, rooms, dead ends) and edges.
  - Computes optimal paths using Dijkstra / A* with penalty weights for dead ends ($10^5$) and blocked edges ($10^6$).
- **`navigation_memory.py`**:
  - SQLite database at `config/navigation_memory.db`.
  - Persists nodes, edges, traversal counts, detected signs, and obstacle blockage history across reboots.
- **`decision_engine_node.py`**:
  - High-level orchestrator.
  - Translates user mission goals (`/navigation/goal_label`) into topological waypoints.
  - Commands Nav2 via `/navigate_to_pose` action client.
  - Reacts to corridor blockages by cancelling Nav2 goal, updating memory, and computing alternate routes (e.g. West Bypass).

### 2.3 Motion Control & Safety
- **Nav2 Stack**:
  - Retains exclusive authority over local trajectory generation and collision avoidance.
  - Dual obstacle layer (`/scan` + `/vision/costmap_obstacles`) ensures both geometry and semantic obstacles are respected.
