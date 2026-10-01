# V3 Hospital Logistics — Completion Status

Status as of 2026-10-01. **Not committed**, as instructed: no git commit, push, tag or merge was made.
Details and evidence: `V3/V3_FINAL_VALIDATION_REPORT.md`. Operation: `V3/V3_FINAL_RUNBOOK.md`.

## Components

| Component | File(s) | Status | Validated by |
|---|---|---|---|
| Gazebo hospital world plus robot | `V3/worlds/hospital_logistics_world.sdf`, `launch/v3_world.launch.py` | Done | Phase 1 evidence, every mission |
| YOLOv8n V3 detector (9 classes) and signs | `hospital_logistics/v3_detection_node.py`, `V3/models_trained/v3_yolov8n_from_v25.pt` | Done | Phase 2 eval; 28 Hz live, 8 ms inference |
| Camera–LiDAR spatial fusion, world-frame tracker | `hospital_logistics/v3_spatial_fusion_node.py`, `V3/perception/spatial_fusion.py` | Done | Phases 3 and 7 (0 false TTC on static objects) |
| Costmap obstacles (odom-anchored) | `hospital_logistics/v3_costmap_obstacle_node.py` | Done; limitation in scenario F | Phase 7 |
| Decision engine (V2.6 code plus V3 adapter) | `hospital_logistics/v3_decision_engine.py` | Done; 2 V2.6 bugs fixed by the adapter | Phases 5 and 6, adapter unit test |
| Mission manager (state machine, SQLite history) | `hospital_logistics/v3_mission_manager_node.py` | Done | T01–T12; 40 batch missions |
| Web UI (control, map, camera, detections, routes, events, history) | `hospital_logistics/v3_web_server.py`, `V3/ui/index.html` | Done | T02, T03, T10, T11 |
| Hospital traffic | `hospital_logistics/v3_traffic_node.py` (+ V2.6 dynamic_obstacles_node) | Done; interaction coverage limited | Phase 8 |
| Path memory (V3 database) | `V3/scripts/init_v3_memory.py`, `V3/data/v3_navigation_memory.db` | Done | Phase 5, live route convergence |
| Full-system launch | `launch/v3_full_system.launch.py` | Done (staggered start) | READY in 17–23 s |
| Start / stop / restart / health | `V3/scripts/start_v3.sh`, `stop_v3.sh`, `restart_v3.sh`, `health_check.sh` | Done | Health PASS; no processes left after stop |
| Desktop launchers | `V3/scripts/install_desktop_launchers.sh`, `v3_desktop_session.sh` → `~/Desktop/V3_Hospital_Logistics.desktop`, `~/Desktop/V3_Hospital_Logistics_Stop.desktop` | Done | Phase 21 (gio launch) |
| Documentation | `V3/V3_FINAL_VALIDATION_REPORT.md`, `V3/V3_FINAL_RUNBOOK.md`, this file | Done | — |

## Access

| Item | Value |
|---|---|
| Start | Double-click **V3 Hospital Logistics** on the desktop, or `V3/scripts/start_v3.sh [--headless] [--traffic] [--no-browser] [--port N]` |
| Stop | Enter in the launcher terminal, the **V3 Hospital Logistics (Stop)** launcher, or `V3/scripts/stop_v3.sh` |
| Restart | `V3/scripts/restart_v3.sh` |
| Health check | `V3/scripts/health_check.sh` |
| Port | 8080 (`--port` / `V3_PORT` to change) |
| Local URL | http://localhost:8080 |
| LAN URL | http://192.168.1.42:8080 (private and loopback clients only; not exposed to the internet) |
| Mission database | `V3/data/v3_missions.db` |
| Path memory database | `V3/data/v3_navigation_memory.db` |
| Logs | `V3/logs/latest/launch.log` |
| Evidence | `V3/docs/evidence/phase*/` |

## Statistics (measured)

| Measure | Value |
|---|---|
| System tests (final build) | 12/12 PASS |
| Missions, final build | 10/10 complete without traffic + 5/5 with traffic. Without traffic: 160.4 ± 25.0 s, 66.5 ± 10.4 m |
| Missions, previous build | 15/15 complete (10 without traffic: 144.2 ± 0.4 s, 60.8 ± 0.1 m) |
| Failures / timeouts in batch missions | 0 / 0 (40 missions) |
| Replans per mission (final build, no traffic) | 0.6 mean, all DYNAMIC_OBSTACLE; 0 once history converged (missions 6–10) |
| Closest LiDAR range in missions | 0.256 m |
| Sign decision tests | 9/9 (raw V2.6: 8/9) |
| False TTC < 1.8 s on static objects | V3: 0 in 6 scenarios; V2.6: 3–89 per scenario |
| Resource use | about 4.6 CPU cores, 3.55 GB RAM, GPU 17% / 163 MB |
| Rates | camera 28 Hz, YOLO 28 Hz, fusion 28 Hz, LiDAR 20 Hz |
| Startup to READY | 17–23 s; stop about 20–25 s |

## Known limitations

1. In scenario F (close pass of a trolley), 36% of costmap points lie more than 1 m from the object. This comes from the ring geometry inherited from V2.6 plus fusion range jumps at the image edge.
2. Traffic interaction is thinly sampled: close person encounters in 1 of 5 final-build traffic missions, and no TTC yield occurred during any mission.
3. Sign direction is not used by the V2.6 gating (map route is the authority).
4. Path history recovers slowly after failures (Bayesian prior).
5. Nav2 controller and planner ignore SIGINT; the stop script escalates to SIGTERM/SIGKILL.
6. The desktop test used `gio launch`, not a physical mouse click. Simulation only.

## Git state (V3 work NOT committed)

- `git diff --name-only` shows tracked changes only in the V3 package:
  - `ros2_ws/src/hospital_logistics/{launch/v3_world.launch.py, package.xml, setup.py}`
  - 23 lines added, 2 removed.
- **No V2.6 source changed.** No tracked file outside `ros2_ws/src/hospital_logistics/` or `V3/` differs, and nothing is staged.
- **New V3 files (untracked):**
  - V3 package: 8 node files (`hospital_logistics/v3_*.py`) and `launch/v3_full_system.launch.py`
  - `V3/` scripts, tests, ui, config, perception, training, datasets metadata, models_trained, docs/evidence
  - Reports: `V3/V3_FINAL_*.md`, `V3/V3_COMPLETION_STATUS.md`
  - Earlier V3 reports at the project root: `V3_*.md`
  - About 75 MB across 260 non-ignored files under `V3/`. Generated data, logs and run files are git-ignored.
- **Pre-existing unrelated untracked files (left untouched, not deleted):** all dated 2026-09-30 10:33–13:55, before the V3 work.
  - `V2.6_GAZEBO_MANUAL_REPAIR_REPORT.md`, `V2.6_SIGN_TOPOLOGY_AUDIT.{json,md}`
  - `config/navigation_locations.yaml.bak_20260930`, `recovery/`
  - `ros2_ws/src/autonomous_robot_navigation/autonomous_robot_navigation/navigation_memory.py.bak_20260930`
  - `scripts/build_realistic_facility.py.bak_20260930`
  - `scripts/{run_sign_regression_test.py, run_targeted_recovery_tests.py, test_office_hospital_signs.py, validate_sign_topology.py}`
- All ROS / Gazebo processes are stopped.
