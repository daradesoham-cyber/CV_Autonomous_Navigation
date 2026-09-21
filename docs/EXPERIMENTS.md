# Experiment Evaluation Suite & Benchmarks

## 1. Overview

The experiment suite systematically validates the performance of the semantic memory, computer vision pipeline, topological decision-making, and dynamic obstacle avoidance in the ROS 2 Lyrical + Gazebo Sim 10.5 environment.

---

## 2. Automated Scenarios

### Scenario 1: Semantic Sign Perception & Guidance
- **Script**: `scripts/test_scenario_1_sign_navigation.py`
- **Objective**: Verify that all 15 directional sign textures are accurately detected, classified, and decoded by `sign_detection_node`.
- **Result**:
  - Detection Accuracy: **100.0%** (15/15 signs recognized)
  - Mean Confidence: **0.99**
  - Processing Latency: **< 4 ms** per frame
  - Status: **PASSED**

### Scenario 2: Dynamic Corridor Blockage & Replanning
- **Script**: `scripts/test_scenario_2_dynamic_replanning.py`
- **Objective**: Test route replanning when a primary corridor is obstructed by a dynamic obstacle.
- **Results**:
  - Primary corridor (`junction_4` -> `storage`) blocked.
  - Decision engine automatically activates **West Bypass** (`junction_4` -> `west_bypass_mid` -> `west_bypass_north` -> `storage`).
  - Replanning Latency: **< 1 ms**
  - Unblocking corridor immediately restores optimal path.
  - Status: **PASSED**

### Scenario 3: Dead End Avoidance & Pruning
- **Script**: `scripts/test_scenario_3_dead_end_avoidance.py`
- **Objective**: Verify that the topological planner strictly avoids cul-de-sacs (`dead_end_1`, `dead_end_2`, `dead_end_3`) across all destinations.
- **Results**:
  - Paths evaluated to 7 destinations (`hospital`, `warehouse`, `office`, `lab`, `storage`, `cafeteria`, `exit`).
  - Zero dead-end incursions.
  - Dead-end cost penalty verified at $10^5$.
  - Status: **PASSED**

### Scenario 4: Navigation Memory & SQLite Persistence
- **Script**: `scripts/test_scenario_4_memory_persistence.py`
- **Objective**: Test initialization, updates, and persistence across process restarts using SQLite (`navigation_memory.db`).
- **Results**:
  - 22 nodes and 48 edges initialized and validated.
  - Sign observations, edge traversals, and dynamic blockages successfully persisted across independent memory instances.
  - Reset function clears dynamic observations while preserving topological structure.
  - Status: **PASSED**

---

## 3. YOLOv8 Custom Model Performance Benchmark

Evaluated on **NVIDIA GeForce RTX 3050 Laptop GPU** (`cuda:0`) using custom weights `models/custom_yolov8n/weights/best.pt`:

| Class | Precision | Recall | mAP50 | Latency (Inference) |
| :--- | :--- | :--- | :--- | :--- |
| **Person** | 0.962 | 1.000 | 0.995 | 3.2 ms |
| **Chair** | 0.947 | 1.000 | 0.995 | 3.2 ms |
| **Box** | 0.962 | 0.833 | 0.835 | 3.2 ms |
| **Cone** | 0.966 | 1.000 | 0.995 | 3.2 ms |
| **Pallet** | 1.000 | 0.920 | 0.995 | 3.2 ms |
| **Shelf** | 0.945 | 1.000 | 0.995 | 3.2 ms |
| **Hospital Bed** | 0.943 | 1.000 | 0.995 | 3.2 ms |
| **Cart** | 0.948 | 1.000 | 0.995 | 3.2 ms |
| **All Classes** | **0.959** | **0.969** | **0.975** | **29.5 FPS** |

---

## 4. How to Run All Experiments

To execute the complete benchmark suite:
```bash
./scripts/run_all_experiments.py
```
Output:
```
======================================================================
AUTONOMOUS NAVIGATION SYSTEM - EXPERIMENT SUITE
Environment: ROS 2 Lyrical | Gazebo Sim 10.5 | NVIDIA RTX 3050 (cuda:0)
======================================================================
...
======================================================================
EXPERIMENT SUITE SUMMARY REPORT
======================================================================
Scenario Name                                 | Status   | Duration (s)
----------------------------------------------------------------------
Scenario 1: Semantic Sign Perception & Guidance | PASSED   | 1.124       
Scenario 2: Dynamic Obstacle Blockage & Replanning | PASSED   | 0.500       
Scenario 3: Dead End Avoidance & Pruning      | PASSED   | 0.525       
Scenario 4: Navigation Memory & SQLite Persistence | PASSED   | 0.525       
======================================================================
[ALL EXPERIMENTS COMPLETED SUCCESSFULLY - 100% PASS RATE]
```
