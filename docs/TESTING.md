# Testing & Verification Guide V2

This document details the automated test suites, verification procedures, and reproduction commands for **Autonomous Navigation V2**.

---

## 1. Test Suite Summary

The codebase provides three primary automated test suites:
1. **10-Stage V2 Verification Suite** (`scripts/test_v2_scenarios.py`): Validates all V2 features.
2. **5-Scenario Baseline Suite** (`scripts/run_all_experiments.py`): Preserves existing functionality.
3. **16-Point Facility Evaluation** (`scripts/test_realistic_facility.py`): Comprehensive facility-wide verification.

---

## 2. Running the 10-Stage V2 Verification Suite

Command:
```bash
python3 scripts/test_v2_scenarios.py
```

### Verified Stages
| Stage | Test Name | Focus & Verification |
|---|---|---|
| **Test 1** | Gazebo Environment & Obstacles | Validates `realistic_facility_world.sdf` includes corridors, junctions, signs, furniture, and 4 dynamic obstacles with velocity controllers. |
| **Test 2** | Dynamic Obstacle Controllers | Checks trajectory publishers on `/cart/cmd_vel`, `/trolley/cmd_vel`, `/forklift/cmd_vel`, and `/person/cmd_vel`. |
| **Test 3** | Multi-Criteria Route Cost | Validates cost formula: distance, obstacle density, narrow corridors, turns, and travel time. |
| **Test 4** | Candidate Alternative Routes | Verifies K-shortest paths algorithm, metric calculation, and cost-based route ordering. |
| **Test 5** | Camera-LiDAR Fusion | Checks directional classification (`Front-Center`, `Front-Right`, `Left`, etc.), 3D coordinates, and RViz markers. |
| **Test 6** | Visual Signs Guidance | Verifies sign texture assets (30 PNGs) and associative memory in SQLite. |
| **Test 7** | SQLite Journey Memory | Validates recording of complete journeys with coordinates, travel time, clearance, replans, and recoveries. |
| **Test 8** | Dead-End Intelligence | Verifies dead-end coordinate logging in SQLite, backtracking maneuvers, and topological cost penalties. |
| **Test 9** | Recovery Behaviors | Tests detection of stuck/low-clearance conditions and recovery event logging. |
| **Test 10** | Web Dashboard & REST APIs | Exercises all 10 Flask REST endpoints with test client (HTTP 200 OK). |

---

## 3. Running the Baseline Experiment Suite

Command:
```bash
python3 scripts/run_all_experiments.py
```

### Verified Scenarios
1. **Scenario 1**: Semantic Sign Perception & Guidance (100% recognition).
2. **Scenario 2**: Dynamic Obstacle Blockage & Replanning (West Bypass engaged and restored).
3. **Scenario 3**: Dead End Avoidance & Pruning (All 4 dead ends pruned from destination routes).
4. **Scenario 4**: Navigation Memory & SQLite Persistence (Traversal counts and blockages persisted across restarts).
5. **Scenario 5**: 16-Point Facility Evaluation (SDF, map alignment, YOLOv8 weights, shortest routes, memory).

---

## 4. Running the Demonstration Mode

To run a guided, automated walkthrough of the complete mission intelligence lifecycle:

```bash
# Interactive mode (realistic pacing with step pauses)
python3 scripts/demo_mode.py

# Fast verification mode
python3 scripts/demo_mode.py --fast
```

The demonstration exercises:
1. Candidate route evaluation (Route A, Route B, Route C).
2. Dynamic obstacle encounter with camera-LiDAR fusion.
3. Corridor blockage and dynamic replanning via the optimal bypass.
4. Dead-end detection and safe reverse backtracking.
5. Autonomous recovery behavior activation.
6. Destination arrival and SQLite journey record creation.
7. Verification of analytics summary and route heatmap data.

---

## 5. Live Simulation Testing in Gazebo

To run a live mission in the Gazebo simulation:

1. Launch the full system:
```bash
ros2 launch autonomous_robot_bringup full_system.launch.py
```

2. Open the Dedicated Navigation Dashboard:
```
http://127.0.0.1:5050
```

3. In the dashboard, click any goal chip (e.g. `ROOM A`, `STORAGE`, `CHARGING`) to dispatch the mission.

---

## 6. End-to-End Real Simulation Validation (V2.1)

To run automated end-to-end missions directly against Gazebo simulation physics and capture live ROS 2 and SQLite ground truth:

```bash
# Execute a single mission (e.g., Room A -> Storage)
python3 scripts/run_real_missions.py --start room_a --goal storage

# Execute all 5 standard validation missions
python3 scripts/run_real_missions.py --all
```

---

## 7. Performance & Latency Benchmarks (V2.1)

To measure real-time node frequencies, CPU/RAM utilization, global planning latency, and REST API response times:

```bash
python3 scripts/measure_performance_metrics.py
```

Outputs live frequencies for all 8 architecture pipeline stages, sub-10ms REST latencies, and multi-criteria A* execution speeds.
