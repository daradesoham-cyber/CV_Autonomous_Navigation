# V3 New Hospital: Status (2026-10-01)

Source: `V3/V3_NEW_HOSPITAL_SOURCE.md` (AWS RoboMaker Hospital World, MIT-0, plus OpenRobotics Fuel models, CC-BY 4.0).

## Runtime files

| Item | Path |
|---|---|
| World | `V3/worlds/v3_hospital_world.sdf` (built by `V3/scripts/build_aws_hospital_world.py` + `v3_hospital_additions.py`) |
| Map | `V3/maps/v3_hospital_map.yaml` / `.pgm` (27.1 × 59.0 m, 0.05 m; sliced from the collision meshes by `generate_hospital_map.py`) |
| Layout (single source) | `V3/scripts/v3_hospital_layout.py`: spawn, 33 nodes, 37 edges, signs, plaques, traffic |
| Topology DB | `V3/data/v3_hospital_navigation_memory.db` (built by `build_v3_hospital_topology.py`, fresh history) |
| Mission / path history DB | `V3/data/v3_hospital_missions.db` |
| Semantic map (signs) | `V3/config/v3_hospital_semantic_map.yaml` |
| Logistics locations | `V3/config/logistics_locations.yaml` |
| Nav2 / AMCL parameters | `V3/config/v3_nav2_params.yaml` |
| Validation | `V3/scripts/validate_hospital_topology.py`: 33/33 nodes free, 37/37 edges connected at robot radius 0.27 m |

**Spawn:** main entrance lobby (0, 13), facing south.
**Collection point:** West Ward aisle (−8.5, −15.6), with the bedside table, payload and COLLECTION plaque.
**Laboratory / test point:** NE laboratory (8.6, 14.6), with lab benches and the LABORATORY TEST POINT plaque.
**Route:** West Ward → west corridor → intersections → main lobby → laboratory. The east corridor and the cross corridors are alternative routes.

## Fixes made for the new environment

- **Collada units:** the map generator now honours `<unit>`; the AWS meshes are in centimetres.
- **Fallen props:** three upstream props at z = −7.6e8 were restored to the floor; the trolley bed was re-parked.
- **Model names:** Fuel model folders were renamed to match their `model://` URIs, and the portrait photo paths were fixed.
- **Floor collision:** the AWS floor is now visual-only and the robot drives on the ground plane. Wheel slip on the floor mesh had made odometry read 1.52× the true distance; after the fix it reads 1.00×.
- **AMCL:** random-particle recovery is disabled. In the symmetric wings it caused wrong-wing relocalisation jumps.
- **Decision-engine adapter:** non-moving obstacles that lie on the static map no longer trigger topology replans. The first mission had 7 replans and 154 m; after the change it had 0 replans and 107 m.
- **Traffic:** every crossing line now crosses a north–south corridor that carries no east–west edge. Models yield, step back from a close robot, and turn back after 6 s of mutual yielding.
- **E-stop latch:** Nav2 cancel every 0.2 s and ABORT every 1 s while latched.
- **UI:** the tall map is shown rotated 90° (north to the left), and the traffic label was updated.

## Results (measured, `V3/docs/evidence/new_hospital/`)

- **Visual check** (`views/*.png`): it reads as a hospital. Reception desk with visitors, waiting seating, corridors with handrails and signs, curtained wards with patients, staff in scrubs, wheelchair, IV stand, furnished laboratory.
- **YOLO** (existing model, 9,314 frames during a mission): every class was detected (person 0.80, sign 0.76, IV stand 0.74, wheelchair 0.73, surgical trolley 0.69, cart 0.65, bed 0.64, collection marker 0.46, lab marker 0.44 mean confidence). No retraining was needed.
- **System tests** (`system_tests.json`): 12/12 PASS (start, UI/camera/map, command whitelist, start, pause, resume, cancel, reset, E-stop, LAN, history, full mission).
  - T09 includes a 0.5 s braking allowance. Measured directly: 0.31 → 0.0 m/s within 0.5 s, and no non-zero `/cmd_vel` after the E-stop.
- **Restart** (`restart_v3.sh`): READY in 48 s. A normal start reaches READY in 27–28 s.
- **Final missions** (`missions/final_no_traffic`, `missions/final_traffic`): each mission drives to the collection point from where the previous one ended, then to the laboratory.

| Run | Complete | Time s (mean, min–max) | Distance m (mean, min–max) | Replans (mean, max) | TTC events (mean, max) | Min LiDAR clearance m (min) |
|---|---|---|---|---|---|---|
| No traffic | 10/10 | 289.3 (270.0–327.9) | 124.0 (114.6–142.3) | 0.9, 2 | 0.2, 1 | 0.264 |
| With traffic | 5/5 | 319.6 (262.5–394.2) | 140.0 (114.8–171.4) | 1.4, 3 | 0.8, 2 | 0.177 |

- **Localisation** (ground truth vs AMCL at 2 Hz, `missions/final_truth_vs_amcl.jsonl`, valid segment t ≥ 3100 s, which covers the traffic missions): mean 0.12 m, p95 0.23 m, max 0.37 m. Closest robot-to-traffic distance: 0.36 m.
  - The earlier part of that log is invalid: the monitor's `/amcl_pose` feed was stale. Missions completed normally during that window, which could not happen with the logged 8–18 m "errors".

## Limitations

- One `ros_gz_bridge` segfault at startup in about 12 starts. `start_v3.sh` reports it as not READY; a restart fixes it.
- The marker plaques are detected at about 0.45 confidence, lower than other classes. They are informational only; the map pose decides arrival.
- Sign gating uses the V2.6 decision engine as-is; the sign direction is not used for routing.
- The AWS ceiling is left out so the plan is visible from above; ceiling lights are point lights.
- The dataset and training scripts in `V3/datasets` and `V3/training` still reference the old world. They are historical: the current YOLO model was trained with them. They aren't used at runtime.
