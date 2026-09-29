# V2 Reality Audit: Verification of Claims vs. Actual Implementation

**Date:** September 28, 2026  
**Auditor:** Autonomous Navigation Engineering Team  
**Scope:** Complete Codebase, ROS 2 Lyrical, Gazebo Sim 10.5, Python 3.14 Environment  

---

## 1. Executive Summary

This audit assesses the actual state of the **CV Autonomous Navigation V2** codebase prior to any modifications. Every feature, architectural pipeline, node, topic, database schema, and dashboard screen was examined against the actual source code and runtime behavior.

While the software-level 10-stage test suite (`scripts/test_v2_scenarios.py`) reports a 10/10 pass rate, that suite strictly executes offline Python unit tests with mock data. In an end-to-end running ROS 2 + Gazebo environment, several critical discrepancies and integration gaps were identified:
1. **CUDA Crash Risk in Perception:** `object_detection_node.py` unconditionally raised a fatal `RuntimeError` if CUDA was unavailable, crashing the robot perception stack when running on systems where the NVIDIA kernel driver is unlinked or offline. Graceful CPU fallback is required.
2. **Reverse Safety in Recovery & Dead-End Backtracking:** `_execute_backtrack()` in `decision_engine_node.py` blindly published reverse linear velocity (`-0.18 m/s`) for 2.0 seconds without verifying rear LiDAR clearance, creating a risk of reversing into obstacles or walls.
3. **Route Cost Breakdown Itemization:** `calculate_route_metrics()` only provided partial cost terms (`distance_cost`, `obstacle_cost`), omitting itemized costs for history, congestion, turns, narrow corridors, and time.
4. **Dashboard Baseline Seeding:** `dashboard_backend.py` injected synthetic mock journeys (`J-001` through `J-005`) into SQLite whenever the database was empty, and hardcoded `robot_state_publisher` and `dynamic_obstacles_node` health statuses.
5. **Dashboard Visualizations & Analytics:** The live map lacked zoom/pan controls, coordinates readout, scale bar, heading indicator, and multi-journey comparison tools. Screen 3 route heatmap lacked interactive corridor click breakdown. Mission Replay and System Overview architecture pipeline were missing.

---

## 2. Feature Verification & Audit Table

| Feature | Code Exists | Runtime Works | Data Is Real | Tested | Status | Notes |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Realistic 35×30m Warehouse Environment** | YES | YES | YES | YES | **PASS** | Validated in `realistic_facility_world.sdf`. Bounding walls span [-16, 16]m on X, [-13, 13]m on Y (~32×26m interior, 35×30m structural). 30 visual signs, shelves, storage bays. |
| **4 Dynamic Obstacles** | YES | YES | YES | YES | **PASS** | Cart, trolley, forklift, and walking person modeled in SDF with `gz-sim-velocity-control-system`. Driven by `dynamic_obstacles_node.py` via `/cart/cmd_vel`, `/trolley/cmd_vel`, `/forklift/cmd_vel`, `/person/cmd_vel`. |
| **Multi-Criteria Route-Cost Planning** | YES | YES | PARTIAL | YES | **PARTIAL** | Core formula implemented in `topological_graph.py` `Edge.calculate_cost()`. Configurable via `navigation_v2.yaml`. However, `calculate_route_metrics()` lacked itemized breakdown of history, turns, congestion, time, and narrow corridor penalties. |
| **K=3 Alternative Routes** | YES | YES | YES | YES | **PASS** | `find_alternative_routes()` calculates $K=3$ candidate paths using edge penalization and returns paths sorted by total multi-criteria cost. |
| **Camera + LiDAR 3D Fusion** | YES | YES | YES | YES | **PASS** | `lidar_camera_fusion_node.py` projects YOLO bounding box azimuths to LiDAR scans, resolves camera-LiDAR parallax across [0.3m, 6.0m], and computes metric 3D `(x, y, z)` in `base_link`. |
| **Directional Object Detection** | YES | YES | YES | YES | **PASS** | Computes direction labels (`Front-Center`, `Front-Right`, `Front-Left`, `Left`, `Right`) from effective bearing in `lidar_camera_fusion_node.py`. |
| **Custom YOLOv8 Object Detection** | YES | PARTIAL | YES | YES | **PARTIAL** | Fine-tuned weights exist at `models/custom_yolov8n/weights/best.pt` (8 classes, 96.2% mAP50). However, `object_detection_node.py` hard-crashes if `torch.cuda.is_available()` is False rather than gracefully falling back to CPU (~26 FPS). |
| **Directional Sign Perception** | YES | YES | YES | YES | **PASS** | `sign_detection_node.py` loads 30 PNG sign templates from Gazebo materials and performs multi-scale template matching with contour filtering. |
| **Persistent SQLite Journey Memory** | YES | YES | PARTIAL | YES | **PARTIAL** | Relational schema in `navigation_memory.db` tracks `journeys`, `route_segments`, `dead_ends`, `recovery_events`, `signs`, `traversals`. Real missions record accurately, but empty DB was auto-seeded with 5 synthetic journeys (`J-001` - `J-005`). |
| **Dead-End Detection & Backtracking** | YES | PARTIAL | YES | YES | **PARTIAL** | Cul-de-sac detected, stored in SQLite `dead_ends` table, edge penalized ($10^4$). However, `_execute_backtrack()` blind reverses without checking rear LiDAR clearance. |
| **Recovery Behaviors** | YES | PARTIAL | YES | YES | **PARTIAL** | State machine triggers on stuck robot, low clearance, goal abort. Saves to `recovery_events` table. However, recovery reverse maneuver lacks rear LiDAR obstacle verification. |
| **Nav2 Stack & Localization** | YES | YES | YES | YES | **PASS** | AMCL localization, map server, NavFn global planner, and Regulated Pure Pursuit (RPP) controller integrated in `navigation.launch.py`. |
| **Web Dashboard (Port 5050)** | YES | YES | YES | YES | **PASS** | Flask server with REST APIs in `dashboard_backend.py`. Telemetry, sensors, topology, and history endpoints functional. |
| **Screen 1 — Mission Monitor** | YES | YES | YES | YES | **PASS** | Real-time robot pose, velocity, goal dispatch, active path, alternative paths table, clearances, and live scrolling event log. |
| **Screen 2 — Travel History & Analytics** | YES | YES | YES | YES | **PARTIAL** | Displays journey table and canvas charts. Missing detailed single-mission summary overlay and side-by-side journey comparison tool. |
| **Screen 3 — Route Heatmap** | YES | YES | YES | YES | **PARTIAL** | Displays corridor segments with traversal colors. Missing interactive corridor click details (traversal count, avg clearance, failure count, reliability score). |
| **Mission Replay Mode** | NO | NO | NO | NO | **DOCUMENTATION ONLY** | Not implemented in original V2 dashboard. Required in Phase 13. |
| **System Overview Architecture Panel** | PARTIAL | NO | PARTIAL | NO | **PARTIAL** | Some node health indicators exist in `/api/system_health`, but `robot_state_publisher` and `dynamic_obstacles_node` were hardcoded to `ONLINE`. Pipeline flow diagram missing. |
| **Automated V2 Test Suite** | YES | YES | PARTIAL | YES | **PARTIAL** | `scripts/test_v2_scenarios.py` passes 10/10 tests, but tests are pure unit tests with mock data rather than end-to-end Gazebo simulation runs. |
| **Desktop Launcher & One-Click Runner** | NO | NO | NO | NO | **DOCUMENTATION ONLY** | No `.desktop` file or unified desktop runner existed. Required in Phase 20. |

---

## 3. Discrepancies & Findings Requiring Action

1. **Object Detection Hard Crash on Non-CUDA Devices:**
   - In `object_detection_node.py` (lines 44-46 and 56-58), `raise RuntimeError("CUDA is required on cuda:0...")` forces an immediate abort if CUDA is unavailable.
   - *Fix:* Check `torch.cuda.is_available()`. If False, log a descriptive warning and seamlessly fall back to CPU inference, maintaining 26 FPS without crashing the autonomous stack.
2. **Blind Reverse Hazard in Backtracking & Recovery:**
   - `_execute_backtrack()` sends `linear.x = -0.18 m/s` for 2.0s without checking rear LiDAR clearance.
   - *Fix:* Add rear LiDAR sector check (angles $-135^\circ$ to $+135^\circ$ / rear azimuth). Abort reverse if rear clearance $< 0.35$ m.
3. **Itemized Route Cost Transparency:**
   - Multi-criteria weights influence route selection in `calculate_cost()`, but `calculate_route_metrics()` only recorded distance and obstacle costs.
   - *Fix:* Fully itemize each cost term (distance, obstacle, history, congestion, turns, narrow, time, familiarity) so the dashboard and user can inspect exactly why Route B was selected over Route A.
4. **Synthetic Data Removal:**
   - `seed_baseline_journeys_if_empty()` in `dashboard_backend.py` seeded 5 artificial journeys.
   - *Fix:* Rely purely on actual real-world missions recorded from ROS 2 and Gazebo. Display `DATA UNAVAILABLE` or empty state when no actual missions have run.
5. **Dashboard Enhancements:**
   - Implement Phase 5: Map legend, zoom/pan, coordinate HUD, scale indicator, heading indicator, and historical journey overlay.
   - Implement Phase 6: Mission Summary card and historical journey comparison tool.
   - Implement Phase 7: Interactive corridor click inspection with actual segment reliability, traversal counts, and clearance statistics.
   - Implement Phase 13: Mission Replay feature animating recorded robot trajectory with event timeline.
   - Implement Phase 14: System Overview panel with live pipeline status (Camera → YOLO → Object Detection → Fusion → Perception → Global Planner → Local Navigation → Motor Command).
6. **Desktop Launcher (Phase 20):**
   - Create `start_cv_navigation_v2.sh`, `stop_cv_navigation_v2.sh`, `install_desktop_launcher.sh`, desktop shortcut `~/Desktop/CV_Navigation_V2.desktop`, and local project icon.

---
