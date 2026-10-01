# V3 Hospital Logistics — Final Validation Report

Date: 2026-10-01.

Platform:
- ROS 2 Lyrical, Gazebo Sim 10.5, Nav2/AMCL
- YOLOv8n V3 model (9 classes) on an RTX 3050
- 16-core CPU

Every number here comes from a test that actually ran. The raw output is under `V3/docs/evidence/`.
Injected test inputs (sign messages, path-history outcomes) are labelled as such in their evidence files.
Timeouts are reported as timeouts. Nothing in V2.6 was modified; every fix is a V3-local file or adapter.

## 1. Summary

| Area | Result | Evidence |
|---|---|---|
| System tests T01–T12 (final build) | **12/12 PASS** | `phase16/system_tests.json` |
| Repeated missions, final build, no traffic | **10/10 COMPLETE**. 160.4 ± 25.0 s, 66.5 ± 10.4 m, 0.6 replans per mission, closest LiDAR range 0.256 m | `phase17/final/` |
| Repeated missions, previous build (before the decision-engine adapter) | 10/10 COMPLETE. 144.2 ± 0.4 s, 60.8 ± 0.1 m | `phase17/repeated/` |
| Traffic missions, final build (V3 traffic node) | **5/5 COMPLETE**. The walking person came within 1.05 m of the robot in 1 of 5; in the other 4 the route stayed about 4 m away | `phase8/traffic_missions_v3_traffic_node.json` |
| Path history changes route choice | PASS offline (injected outcomes) and **observed live**: route converged from the blocked east corridor to the east bypass | `phase5/`, `phase17/final/` |
| Sign / semantic decisions | **9/9 PASS** with the V3 adapter; unmodified V2.6 code passes 8/9 (bug found) | `phase6/sign_gating*.json` |
| Tracker / costmap (Phase 7, V3 vs V2.6) | No false TTC warnings under 1.8 s in any static scenario (V2.6: 3–89); costmap points away from real objects 0–0.9% (V2.6: 0–7.8%), except scenario F | `phase7/phase7_summary.md` |
| Desktop launcher (gio launch, as on double-click) | PASS: terminal opened, READY in 23 s, Gazebo GUI and browser opened, health check PASS; Stop launcher left no processes running | §8 |
| Ground-truth collisions | 0 in every scenario | `phase7/*` |

## 2. Mission manager and UI (Phases 4, 9–11, 16)

The mission sequence is:
READY → GO_TO_COLLECTION → AT_COLLECTION → COLLECTING (3 s) → PAYLOAD_SECURED → GO_TO_LAB → AT_LAB → DELIVERING (3 s) → MISSION_COMPLETE.
From any of these, the mission can go to PAUSED, CANCELLED, FAILED (leg timeout 300 s, paused time excluded) or EMERGENCY_STOP.

System tests on the final build (`test_v3_system.py`):

| Test | Observed |
|---|---|
| T01 | All 10 subsystems up; READY conditions: AMCL, decision engine, Nav2 action server, fresh YOLO output, fresh fusion output |
| T02 | UI page, map PNG, camera JPEG, MJPEG stream (5 frames in under 3 s) |
| T03 | Unknown command, a command containing a shell string, an `exec` field, a path-traversal location and source = destination: all HTTP 400; mission still READY |
| T04–T06 | Start → robot moves; Pause → max speed 0.0 m/s and 0.0 m drift over 3 s; Resume → robot moves |
| T07–T08 | Cancel → speed 0.0; Reset → READY |
| T09 | E-stop → speed 0.0 for 4 s; latched; Start refused while latched; Reset clears it |
| T10 | LAN URL (192.168.1.42, private address) serves the UI and API |
| T11–T12 | History lists the cancelled, e-stopped and completed missions; a full mission completes |

Defects found and fixed during these tests:
1. **Pause did not stop the robot.** V2.6 PAUSE sends one zero Twist and leaves the Nav2 goal alive. Measured: the robot drove on for about 3.3 s and about 1 m. The V3 mission manager now cancels the NavigateToPose goal itself. Measured after the fix: stopped within 0.5 s.
2. **E-stop could fail** when a Nav2 goal was accepted just after the ABORT. It now cancels all goals immediately, and again every second while latched.
3. **READY came before YOLO had loaded** (T01 failed with fusion and YOLO down). Fresh perception output is now a READY condition.
4. The web server now rejects a start where source = destination.

## 3. Path history (Phase 5)

- **Offline** (`test_v3_path_history_influence.py`): the unmodified V2.6 NavigationMemory and graph were run on a database copy.
  - Recording 4 failures on the selected route's own segments switched the selection to the alternative.
  - Recording 12 later successes left the history penalty still elevated, so recovery is slow.
- **Bug found — learning was failure-only.** In practice waypoints are reached by the proximity ("seamless") check, which only calls `record_traversal`. So `route_segments`, from which route costs are computed, never recorded a success.
  - Measured after about 20 completed missions: 0 successes on every used segment, and 7 failures on the only ward-entry segment.
  - Fix: the V3 adapter `v3_decision_engine.py` also records successes, guarded against double counting. Tested in `test_v3_decision_engine_adapter.py` (PASS).
  - The pre-fix database is archived in `phase5/failure_only_history_before_adapter/`.
- **Live, final build**, starting from reset history:

| Missions | Route | Replans | Duration |
|---|---|---|---|
| 1, 2, 5 | East corridor, the shortest; blocked by the parked trolley | 1 | about 196 s |
| 3, 4 | Other routes | 1–2 | about 145 s |
| 6–9 | East bypass, planned from the start | 0 | 144–145 s |

  On the east bypass, the route reliability reported by the decision engine rose from 91.7% to 94.3% over missions 6–9. That increase is the newly recorded successes taking effect.

## 4. Signs (Phase 6)

`test_v3_sign_gating.py` constructs the real decision-engine node, sets the goal to `lab` with the real route, and passes sign messages to the node's own callback.

| Case | Result |
|---|---|
| correct goal sign | CONFIRMS route |
| irrelevant sign | logged; route preserved |
| goal sign pointing a conflicting direction | route not changed |
| off-route destination at a branch | route preserved |
| multiple signs in one frame | 1 confirms + 2 preserved |
| no sign | nothing |
| confidence 0.40 | ignored |
| CLOSED sign off the route | edges ×10, no replan |
| CLOSED sign on the active route | edges ×10 **and** replan |

- **V2.6 bug:** the on-route CLOSED case calls `ReplanReason.OBSTACLE_BLOCKED`, which does not exist. The AttributeError was swallowed, so the replan never happened (8/9 with raw V2.6). The V3 adapter defines it as ROUTE_BLOCKED.
- **Observation:** sign direction is never used for gating. The map route is the authority.
- **Live missions:** 10 de-duplicated sign decisions per mission, stored in each mission record.

## 5. Tracker and costmap (Phase 7)

Each scenario ran on both chains: V3 (world-frame least-squares tracker plus odom-anchored costmap) and V2.6, with the same detector. Full table: `phase7/phase7_summary.md`.

| Scenario (static object) | V3 TTC < 1.8 s | V2.6 TTC < 1.8 s | V3 costmap points > 1 m from a real object | V2.6 |
|---|---|---|---|---|
| S3 robot toward bed | 0 | 3 | 0.0% | 0.0% |
| S4 robot past bed | 0 | 14 | 0.9% | 7.8% |
| E close pass, bed | 0 | 89 | 0.7% | 5.8% |
| B robot toward static trolley | 0 | 7 | 0.7% | 0.0% |
| F close pass, trolley | 0 | 10 | **36.1%** | 49.7% |
| H multiple obstacles | 0 | 13 | 0.0% | 0.0% |

- **True motion still detected:** S2, trolley approaching at 0.3 m/s. V3 reported MOVING_TOWARDS with a minimum TTC of 8.8 s at about 2.3 m, consistent with distance ÷ speed. V2.6 lost the trolley entirely.
- **Cause of the V2.6 false TTC:** V2.6 estimated velocity as raw minus smoothed position in the robot frame, which inflated apparent motion about 2.9× while the robot moved. V3 uses a 1 s least-squares fit in the odom frame. **Thresholds were not changed.**
- **Scenario F remains high:** the V2.6 ring geometry (centre = range + r, ring radius r = 0.7 m for a trolley) puts ring points up to about 0.8 m behind a 0.6 m-deep trolley. Fusion range also jumps up to 0.82 m as the trolley leaves the image edge. This is a limitation, not hidden by changing the radius.
- **S1 (person crossing):** the person is outside the camera view for most of the run in both chains, so S1 gives no evidence either way.
- **Doorway test G:** not run as a separate scenario. Doorways are traversed in every mission: 15 batch missions on the final build, 0 collisions.

## 6. Traffic (Phase 8)

- **First measurement**, V2.6 traffic driver (`phase8/traffic_exposure.json`):
  - The person did not move at all (0.0 m travelled).
  - The trolley jittered in place (23 m of wheel travel, 0.29 m extent).
  - Cart and forklift patrols stayed at least 2.2 m and 6.5 m from the route.
  - The earlier 5 "traffic" missions (`phase17/traffic/`) therefore **do not test traffic**, and are not counted as such.
- **Fix:** the V3 traffic node walks the person across the east bypass corridor (0.4 m/s, set_pose at 10 Hz, waiting while the robot is within 1.2 m).
- **First trial batch:** all 5 completed. The person came within 1.0–1.4 m of the robot in 3 of 5, with up to 3 replans and a closest LiDAR range of 0.264 m.
- **Final build:** 5/5 complete. The person came within 1.05 m in 1 of 5. In the other 4, the learned route took the robot past the crossing line at a different time, so the person stayed at least 3.9 m away.
- **Interaction coverage is therefore limited** (see §10).

## 7. Performance (Phase 19)

Sampled for 60 s during a mission (`phase19/performance.json`):

| Measure | Value |
|---|---|
| Total CPU | about 4.6 cores of 16 (gz-sim 68%, detection 55%, web server 48%, mission manager 46%, decision engine 45%, fusion 35%) |
| Total RAM | 3.55 GB (detection node 1.6 GB) |
| GPU | 16.6% average, 163 MB of 4096 MB |
| Topic rates | camera 28.0 Hz, YOLO 28.3 Hz, fusion 28.3 Hz, LiDAR 19.9 Hz, odom 28.5 Hz, costmap obstacles 10.0 Hz, mission state 2 Hz |
| YOLO time per frame | preprocess 0.9 ms, inference 8.1 ms, postprocess 1.4 ms |
| Fusion time | 1.04 ms mean |
| Startup to READY (final build) | 17–23 s |
| Stop | about 20–25 s; Nav2 controller and planner need SIGTERM/SIGKILL (§10) |

## 8. Desktop and LAN (Phases 12–15, 21)

- `install_desktop_launchers.sh` generates `~/Desktop/V3_Hospital_Logistics.desktop` and `…_Stop.desktop`, plus the application-menu copies.
  - Both pass `desktop-file-validate` and are marked trusted.
  - The Exec paths are generated at install time, so the repository has no hard-coded paths.
- **Launch test:** run with `gio launch` (the desktop's own launch mechanism). It is not a physical mouse click.
  - A ptyxis terminal opened and ran `v3_desktop_session.sh`.
  - READY was reached after 23 s, with the Gazebo GUI client running and Firefox opened on the UI.
  - `health_check.sh` gave HEALTH: PASS: all nodes, Nav2 lifecycle active, TF, rates, LAN API, databases.
- **Stop test:** the Stop launcher removed the process-ID file within 23 s; 0 V3, Gazebo or Nav2 processes remained and port 8080 was closed.
- **LAN:** reachable at `http://192.168.1.42:8080`. Non-private clients are refused by the server. No firewall or port-forward changes were made.

## 9. Failure analysis (Phase 18)

From `phase18/failure_analysis.json` and the investigations above:

- **Mission outcomes:**
  - 0 failures and 0 timeouts in 15 recorded batch missions on the final build (10 without traffic + 5 with traffic).
  - Also 0 in the 15 batch missions on the previous build, and in the 3 full missions inside the system tests.
  - The cancelled and e-stopped missions were deliberate test actions.
- **Replans:** every replan trigger was DYNAMIC_OBSTACLE.
  - Main cause: the parked surgical trolley in the east corridor. It is classed as a dynamic object, so it is treated as a moving obstacle, not a permanent block.
  - The others came from the walking person (traffic batch).
  - Learned history removes the corridor-blockage replans after about 5 missions.
- **Closest LiDAR range:** 0.256–0.42 m across missions. The minimum occurred while replanning around the trolley. The robot's LiDAR never read below 0.25 m. Ground-truth contact was measured only in the Phase 7 scenarios (0 collisions), not during missions.
- **Defects found and fixed (all V3-local):**

| # | Defect | Fix |
|---|---|---|
| 1 | Pause did not stop the robot | Mission manager cancels the Nav2 goal |
| 2 | E-stop race with goal acceptance | Cancel all goals, repeated while latched |
| 3 | READY before perception was running | Fresh perception output required |
| 4 | Missing ReplanReason (V2.6) | Adapter defines it |
| 5 | Failure-only path learning (V2.6) | Adapter records successes |
| 6 | Robot-frame tracker velocity (Phase 7) | World-frame least squares |
| 7 | Costmap drift (Phase 7) | Points anchored in odom |
| 8 | Static traffic | V3 traffic node |
| 9 | Phase 7 harness defects (scenario filter; leftover objects contaminating later scenarios) | Fixed, and all scenarios rerun |

## 10. Known limitations

1. **Scenario F:** 36% of costmap points are more than 1 m from the trolley, because of the ring geometry inherited from V2.6 plus fusion range jumps at the image edge. The extra margin is conservative (more caution, not less), but it is still a phantom-obstacle source.
2. **Traffic interaction is thinly sampled:** close person encounters in 1 of 5 final-build missions (3 of 5 in the trial batch). There are no TTC-triggered yields in any mission (ttc_events = 0); TTC behaviour is validated in Phase 7 only.
3. **Sign direction is ignored** by the V2.6 gating. Signs are only confirm, ignore, or close.
4. **Path history recovers slowly:** after failures, many successes are needed before a penalised segment's cost returns (Bayesian prior α = 5, β = 1).
5. **Shutdown:** Nav2 `controller_server` / `planner_server` ignore SIGINT, so the stop script escalates to SIGTERM/SIGKILL (named in its output). The V3 Python nodes now exit cleanly.
6. **Desktop test:** done with `gio launch`, not a physical mouse click (no input automation in this session).
7. **Simulation only;** no real hardware.
