# V3 Reference Compatibility Report (Phase 0)

Date: 2026-09-30
Scope: read-only audit of the reference repository `TommasoVandermeer/Hospitalbot-Path-Planning` (shallow clone, HEAD of default branch) against the V2.6 baseline at `~/CV_Autonomous_Navigation`.
I changed no V2.6 code during this phase.

Target environment (verified on this machine):
- ROS 2: `/opt/ros/lyrical`
- Gazebo Sim: `gz sim --version` → 10.5.0
- Project venv: `gymnasium` and `stable_baselines3` are **not installed**

---

## A. Reference repository architecture

It has one ament_python package, `hospital_robot_spawner`:

| File | Role |
|---|---|
| `hospitalbot_env.py`, `hospitalbot_simplified_env.py` | OpenAI Gym environment. Observation is robot position plus 61 laser ranges. The action is continuous (v, ω). The reward is shaped by goal distance and collisions. |
| `start_training.py` | Stable-Baselines3 PPO training and Optuna tuning |
| `trained_agent.py` | Loads a PPO `.zip` policy and publishes `cmd_vel` |
| `robot_controller.py` | ROS node that bridges the Gym env to `/cmd_vel`, `/scan`, `/odom` and resets through `gazebo_msgs` services |
| `spawn_demo.py` | Spawns the robot through `gazebo_msgs/srv/SpawnEntity` on `/spawn_entity` |
| `launch/*.launch.py` | Starts `gazebo` / `gzserver` with `libgazebo_ros_init.so` and `libgazebo_ros_factory.so` |
| `worlds/hospital*.world` | SDF 1.6 hospital (about 183 `<include>`s), with `libgazebo_ros_state.so` in the world |
| `models/` | About 86 models: Pioneer 3AT, Hokuyo, and the AWS RoboMaker hospital/residential/warehouse assets (beds, trolleys, carts, wheelchairs, patients, visitors, IV stands, etc.) |
| `rl_models/*.zip` | Six pretrained PPO policies |

It has no camera, no computer vision, no Nav2, no map, and no semantic layer. Navigation is end-to-end RL on LiDAR alone.

## B. V2.6 architecture (baseline)

The ROS 2 workspace is `ros2_ws/src`:

| Package | Contents |
|---|---|
| `autonomous_robot_description` | xacro robot (core, camera, LiDAR, IMU, gz plugins), `rsp.launch.py` |
| `autonomous_robot_gazebo` | `realistic_facility_world.sdf`, `complex_world.sdf`, `maze_world.sdf`, `ros_gz_bridge.yaml`, sign textures |
| `autonomous_robot_interfaces` | `Detection2D(Array)`, `SemanticObstacle(Array)`, `SignDetection(Array)`, `ObstacleWarning` |
| `autonomous_robot_perception` | `camera_node`, `object_detection_node` (YOLOv8, `models/yolov8n_v24.pt`), `sign_detection_node`, `lidar_camera_fusion_node` (ego-motion-compensated TTC from V2.6 Phase 1), `navigation_perception_node`, `navigation_controller_node` |
| `autonomous_robot_navigation` | `decision_engine_node` (1421 LOC), `topological_graph`, `navigation_memory` (SQLite: nodes, edges, signs, traversals, obstacles, journeys, route_segments, dead_ends, recovery/oscillation/replan events), `dynamic_obstacles_node`, `navigation_visualizer_node`, `dashboard_backend` (port 5050) plus `web/`, Nav2 params and BT XML, maps |
| `autonomous_robot_bringup` | `bringup.launch.py`, `full_system.launch.py`, RViz config |

The launcher is `start_cv_navigation_v2.sh`, run from `~/Desktop/CV_Navigation_V2.desktop`. The dashboard is at `http://127.0.0.1:5050`.

`config/semantic_map.yaml` defines these locations: room_a ("Advanced Robotics Lab", category `lab`), storage, hospital, loading, charging, warehouse, exit, office, room_b, reception, utility_service. It also defines junction signs, including `sign_j4_lab`.

**Git state at audit time:** the branch is `v2.6-validation-recovery` (HEAD `5312068`). **The branch has 24 uncommitted modified files**, including the world SDF, `semantic_map.yaml`, `decision_engine_node.py`, `object_detection_node.py`, sign textures, and the map PGM. Someone has to decide whether these belong to the V2.6 baseline before I branch V3.

## C. ROS / Gazebo incompatibilities

| Reference dependency | Status on Lyrical + Gazebo Sim 10.5 |
|---|---|
| `gazebo` / `gzserver` binaries (Gazebo Classic 11) | **Incompatible.** Classic is EOL and not packaged for Ubuntu 26.04. |
| `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`, `libgazebo_ros_state.so` (`gazebo_ros_pkgs`) | **Incompatible.** Replaced by `ros_gz_sim` (`create`) and `ros_gz_bridge`. |
| `gazebo_msgs/srv/SpawnEntity`, `SetEntityState` | **Incompatible.** Use `ros_gz_interfaces` and `/world/<w>/create` or `set_pose` gz services. |
| Classic sensor/diff-drive plugins in `pioneer3at/model.sdf` and `hokuyo` | **Incompatible.** Needs the `gz-sim-*-system` plugins, which V2.6 already uses. |
| SDF 1.6 worlds, `model://` URIs, `~/.gazebo/models` | Partially loadable. Needs `GZ_SIM_RESOURCE_PATH`, and Classic-specific material scripts (OGRE 1 `.material`) will not render in gz-rendering. |
| ROS 2 Humble rclpy API usage | Mostly source-compatible with Lyrical. It's the Gazebo coupling that blocks reuse. |
| OpenAI `gym` (legacy) | Not installed. The `gym` package is unmaintained. It would need a port to `gymnasium`. |
| Stable-Baselines3, Optuna, Tensorboard | Not installed. Heavy extra dependencies that V3 does not need. |

## D. Reusable algorithms / concepts

- **LiDAR downsampling to a fixed-size range vector** (61 beams over 180°). This is useful as a compact feature for the history/risk scoring, not for control.
- **Episode bookkeeping** (success, collision, timeout outcome per run). It maps directly onto V3 mission outcomes in path history.
- **Randomized start/goal curriculum** (`_randomize_env_level`). This is a good idea for a V3 *test harness* that samples source→destination pairs, and it's deterministic given a seed.
- **Target visualization marker.** This is the same idea as the V3 payload/destination marker.

## E. Reusable robot / world assets

- **Worth considering (props only):** the AWS RoboMaker hospital prop models, namely `TrolleyBed`, `MalePatientBed`, `PatientWheelChair`, `InstrumentCart1/2`, `SurgicalTrolley`, `BPCart`, `IVStand`, `XRayMachine`, `aws_robomaker_hospital_nursesstation`, `aws_robomaker_hospital_hospitalsign`. These are mesh-only SDF models. Each one must be checked individually for Classic-only material scripts before use. The upstream AWS RoboMaker hospital world is MIT-0 licensed. The reference repo has no LICENSE file, so V3 should source these from the upstream AWS repo, not from this repo.
- **Not reusable:** the Pioneer 3AT and Hokuyo models, which use Classic plugins, and V2.6 already has a validated robot. The full `hospital.world` is not reusable either: it has 183 includes, a Classic state plugin, and a different map from the V2.6 Nav2 map.

## F. Reusable path-planning ideas

- The reference repo has none at the global-planning level. It is purely reactive RL.
- Its reward-shaping terms (progress toward goal, a collision penalty, a proximity penalty) give a useful **vocabulary for the V3 route cost**: length, historical failure rate, minimum clearance, dynamic-obstacle density. V3 should apply them as explainable edge weights in the topological planner, not as a learned policy.

## G. RL components worth considering

**None for V3.0.** Reasons:
1. The best reported agents succeed about 80% of the time. V2.6 plus Nav2 already has validated navigation, so swapping it in would lower reliability.
2. The policies were trained on a different robot, sensor layout, and world. They would need retraining against Gazebo Sim, which requires a Gazebo-Sim Gym bridge that doesn't exist here.
3. The policies are LiDAR-only, while V3's contribution is CV.

If RL is explored later, it should be an **optional, isolated** `V3/planning/rl_experimental/` module, evaluated only against the deterministic V3 planner baseline under the same seeds. It is out of scope for V3.0.

## H. Components that should NOT be imported

- `hospitalbot_env.py`, `hospitalbot_simplified_env.py`, `start_training.py`, `trained_agent.py`, `robot_controller.py`, and `rl_models/`, because they are RL and Gym-specific
- `spawn_demo.py` and all launch files, because they depend on `gazebo_ros` and Classic
- `worlds/*.world`, and the `pioneer3at` and `hokuyo` models
- `backup/`, `test/`
- Dependencies: `gym`, `stable_baselines3`, `optuna`, `tensorboard`, `gazebo_ros_pkgs`

## I. Recommended V3 architecture

V3 stays **inside the existing ROS 2 workspace**. It adds one new package plus a `V3/` folder for docs, config, and scripts, and reuses the V2.6 packages through their topics.

```
V3/                              (docs, config, launch scripts, tests, evidence)
  config/  logistics_locations.yaml, missions/*.yaml, v3_params.yaml
  worlds/  hospital_logistics_world.sdf  (copy of V2.6 facility + minimal markers)
  scripts/ start_v3.sh, stop_v3.sh, test_v3_stageNN_*.py
  docs/    reports, evidence matrix
ros2_ws/src/hospital_logistics/  (NEW ament_python package)
  mission_manager_node      WAITING→PICKUP→NAVIGATING→OBSTACLE_EVENT→REPLANNING→ARRIVED→DELIVERY→COMPLETE
  logistics_planner_node    named location → semantic/topological route, cost = f(length, history, clearance, blockages)
  route_history             SQLite (extends V2.6 navigation_memory schema: missions, mission_routes, mission_events)
  fused_object_tracker      (consumes V2.6 fusion output; adds pose-frame tracking + static/dynamic label)
  v3_dashboard_backend      new port (5060) so V2.6 dashboard on 5050 is untouched
  launch/v3_full_system.launch.py  (includes V2.6 bringup with V3 world arg)
```

The data flow:

```
camera → object_detection (YOLOv8, reused) ─┐
                                            ├→ lidar_camera_fusion (reused, ego-motion TTC) → fused_object_tracker (V3)
/scan ──────────────────────────────────────┘                                          │
sign_detection (reused) → destination-aware sign reasoning (reused from decision_engine) │
                                                                                          ▼
mission_manager → logistics_planner (topology + history cost) → Nav2 NavigateToPose waypoints → controller
                        ▲                                                        │
                        └──────────── route_history (SQLite) ◄───────────────────┘
v3_dashboard_backend subscribes to all of the above → http://localhost:5060
```

**World:** the V2.6 facility already has lab, reception, storage, office, and charging areas plus lab signage. V3 therefore needs only a **copy** of it with a collection-point marker, a lab/test-point marker, and optionally 2–4 hospital props from section E. The V2.6 world file stays untouched.

**Blocking decision before Phase 1:** the 24 uncommitted V2.6 changes. Either commit them to the V2.6 branch as part of the baseline, or stash them and branch V3 from `5312068`.
