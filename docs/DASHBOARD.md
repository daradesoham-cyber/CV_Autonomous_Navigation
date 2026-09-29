# Dedicated Robotics Navigation Dashboard V2

Autonomous Navigation V2 includes a standalone, professional web dashboard separate from RViz. The dashboard connects directly to live ROS 2 topics and persistent SQLite storage, with zero mocked or synthetic metrics.

URL: **`http://127.0.0.1:5050`**

Launch command:
```bash
python3 scripts/run_dashboard.py
```

---

## 1. Interface Screens & Layout

### 1.1 Screen 1: Mission Monitor
The real-time mission monitor is structured into a 3-column operational layout with a bottom telemetry drawer:

```
+-----------------------------------------------------------------------------------------+
| [BRAND] AUTONOMOUS NAV V2     ● SYSTEM SYNCHRONIZED       [MONITOR] [HISTORY] [HEATMAP] |
+-----------------------+---------------------------------------+-------------------------+
| LEFT PANEL            | CENTER: LIVE INTERACTIVE MAP          | RIGHT PANEL             |
| - State Banner        | - Realistic Facility Map Canvas       | - Route Cost Breakdown  |
| - Robot Position X/Y  | - Active Planned Path (Neon Glow)     | - Candidate Routes Table|
| - Heading & Velocity  | - Traveled History Path               | - Safety Clearances     |
| - Destination & ETA   | - Camera-LiDAR Fused 3D Obstacles     | - Live Event Stream Log |
| - Goal Dispatch Chips | - LiDAR Ray Samples                   |                         |
| - Journey Accumulator | - Interactive Click-to-Navigate       |                         |
+-----------------------+---------------------------------------+-------------------------+
| BOTTOM DRAWER                                                                           |
| - Live Sensor Feeds: Camera FPS, Resolution, YOLO Latency, LiDAR Freq, Min Range        |
| - System Health: ROS 2 Nodes (Status, Freq, Latency), CPU %, RAM %                      |
+-----------------------------------------------------------------------------------------+
```

#### Left Panel: Robot Status & Direct Controls
* **Navigation State Banner**: Emits current state machine state with color coding (`NAVIGATING`, `REPLANNING`, `DEAD_END`, `RECOVERY`, `IDLE`).
* **Pose & Kinematics**: Displays $(X, Y)$ in metric map frame, heading in degrees, and linear/angular velocity ($v, \omega$).
* **Active Mission Target**: Displays current topological node, active waypoint, goal destination, distance remaining, and estimated time of arrival (ETA).
* **Facility Goal Chips**: One-click dispatch for `ROOM A`, `ROOM B`, `STORAGE`, `CHARGING`, `LOADING`, `RESTRICTED`, `HOSPITAL`, `WAREHOUSE`, `EXIT`, and `EXPLORE`.
* **Journey Metrics**: Real-time accumulator for current distance travelled, elapsed duration, replans, and recoveries.

#### Center Panel: Hardware-Accelerated Map Canvas
* **Warehouse Floor Map**: 700x600 high-resolution metric map of the facility.
* **Active Planned Path**: Glowing purple line strip showing the currently selected path.
* **Alternative Routes**: Dotted amber and cyan corridors showing candidate routes considered by the planner.
* **Fused Obstacles**: 3D bounding boxes with direction labels (e.g. `CART (1.8m, Front-Right)`).
* **Robot Sprite**: Heading arrow tracking the real robot position and orientation.
* **Click-to-Navigate**: Click anywhere on the map to dispatch a 2D navigation goal.

#### Right Panel: Route Intelligence & Event Stream
* **Multi-Criteria Scoring**: Displays total route cost, obstacle density factor, turn count penalty, and reliability score.
* **Candidate Alternatives Table**: Compares Candidate Options (Route A, Route B, Route C) with distance, cost, and reliability percentage.
* **LiDAR Clearances**: Color-coded gauges for Minimum, Front, Left, and Right obstacle clearances.
* **Live Event Stream**: Scrolling timestamped log with severity coloring (`INFO`, `WARN`, `ERROR`, `SUCCESS`).

#### Bottom Drawer: Sensors & System Health
* **Camera Telemetry**: Live FPS, resolution (640x480), and YOLOv8 inference latency.
* **LiDAR Telemetry**: 360-degree scan frequency, point count, and minimum range.
* **ROS 2 Node Monitor**: Real-time status, execution frequency (Hz), and message age for all core nodes:
  - `camera_node`
  - `object_detection_node`
  - `lidar_camera_fusion_node`
  - `decision_engine_node`
  - `robot_state_publisher`
  - `ros_gz_bridge`
  - `dynamic_obstacles_node`
* **Host Resources**: Real-time CPU and RAM utilization.

---

### 1.2 Screen 2: Travel History & Analytics
A dedicated screen for historical audits and performance evaluation:
* **KPI Header Cards**: Total Journeys, Success Rate %, Average Distance, Average Travel Time, Total Replans, Total Recoveries.
* **Historical Journeys Table**: Complete log of every trip (`Journey ID`, `Start`, `Goal`, `Distance`, `Travel Time`, `Avg Speed`, `Min Clearance`, `Replans`, `Result`).
* **Path Inspection**: Click any journey row in the table to display its historical trajectory overlaid on the map canvas.
* **Analytical Graphs**:
  - Distance Travelled per Journey (m)
  - Travel Time per Journey (s)
  - Average Translation Speed (m/s)
  - Replanning Count per Journey

---

### 1.3 Screen 3: Route Heatmap & Topology
Visual corridor analysis based on historical traversals:
* **Green / Emerald**: High-frequency, reliable corridors.
* **Blue**: Normal, lower-frequency routes.
* **Amber / Orange**: Corridors with high obstacle density.
* **Crimson / Red**: Blocked corridors or detected dead ends.

---

## 2. REST API Specification

The dashboard backend exposes standard REST endpoints on port 5050:

| Method | Endpoint | Description |
|---|---|---|
| `GET` | `/api/telemetry` | Consolidated live mission state, robot kinematics, clearances, and events |
| `GET` | `/api/sensors` | Camera FPS/latency, LiDAR frequencies, and latest fused objects |
| `GET` | `/api/system_health` | 8-stage architecture pipeline status, ROS node frequencies, CPU %, RAM % |
| `GET` | `/api/topology` | Full topological graph nodes and edges |
| `GET` | `/api/history` | List of all recorded journeys from SQLite |
| `GET` | `/api/history/<id>` | Full journey record with coordinates |
| `GET` | `/api/journey_compare` | Side-by-side comparison of two journeys (`?j1=...&j2=...`) |
| `GET` | `/api/corridor_details` | Deep segment statistics for clicked corridor (`?from=...&to=...`) |
| `GET` | `/api/replay/<id>` | Reconstructs historical journey coordinates and timed event sequence |
| `GET` | `/api/analytics` | Summary KPIs and timeseries data for charts |
| `GET` | `/api/heatmap` | Edge traversal counts and reliability scores |
| `GET` | `/api/dead_ends` | Recorded dead ends with metric coordinates |
| `GET` | `/api/recoveries` | Log of autonomous recovery maneuvers |
| `POST` | `/api/goal` | Dispatch goal (`{"goal": "ROOM A"}` or `{"x": 6.5, "y": -9.5}`) |
| `POST` | `/api/control` | Send mission command (`{"action": "PAUSE" | "RESUME" | "ABORT"}`) |
| `POST` | `/api/reset_memory`| Reset dynamic traversals while preserving topology |

---

## 3. Mission Replay Mode (V2.1)

The Mission Replay mode allows auditing any historical journey stored in SQLite:
1. User selects a journey from the Travel History table.
2. The dashboard fetches `/api/replay/<id>`, which reconstructs the actual recorded $(x, y)$ coordinate points.
3. An animated robot marker traverses the historical trajectory path.
4. A synchronized event timeline highlights key milestones:
   - `START`: Mission initialization.
   - `OBSTACLE`: Location and time of dynamic obstacle encounter.
   - `REPLAN`: Corridor blockage and alternative route activation.
   - `RECOVERY`: Autonomous recovery maneuvers (stop, reverse, pivot).
   - `GOAL`: Arrival at destination or termination state.

---

## 4. System Overview Architecture Panel (V2.1)

Screen 1 features a compact real-time pipeline panel showing the status and frequency of each subsystem:

```
CAMERA (14.8 Hz)
  ↓
YOLO (10.5 Hz)
  ↓
OBJECT DETECTION (10.5 Hz)
  ↓
CAMERA-LIDAR FUSION (23.8 Hz)
  ↓
PERCEPTION (8.1 Hz)
  ↓
GLOBAL PLANNER (ONLINE)
  ↓
LOCAL NAVIGATION (2.4 Hz)
  ↓
MOTOR COMMAND (19.8 Hz)
```

Each subsystem indicates live status (`ONLINE`, `STANDBY`, or `OFFLINE`) derived from actual ROS 2 topic message heartbeats.
