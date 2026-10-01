# V3 Dataset Generation Report (Phase 2)

Date: 2026-09-30 · Branch: `v3.0-dev` (uncommitted working tree)

## 1. Why a new dataset

`scripts/generate_v25_dataset.py` produced the whole V2.5 dataset (2,000 images, 10 classes, 1,400/400/200 split) **without Gazebo**. It draws scenes with OpenCV: perspective polygons for walls and floor, flat shapes for objects, and pasted sign textures. The V2.5 model had never seen a rendered 3D mesh. That explains the Phase 1 transfer failure: flat grey quadrilaterals were labelled `hospital_bed`, so real beds were missed and benches and desks fired as beds. Details are in `V3_CV_DOMAIN_TRANSFER_REPORT.md`.

The V3 dataset uses **only actual Gazebo Sim 10.5 renders of the V3 hospital world**. Labels come from Gazebo's own scene metadata.

## 2. Class set

`V3/perception/v3_classes.yaml`

| id | class | Gazebo models labelled | Notes |
|---|---|---|---|
| 0 | person | Scrubs, FemaleVisitor, MaleVisitorOnPhone, dynamic_person | |
| 1 | hospital_bed | MalePatientBed, TrolleyBed | |
| 2 | cart | BPCart, InstrumentCart1, BloodPressureMonitor | BloodPressureMonitor is a monitor on a wheeled stand and is visually near-identical to BPCart, so labelling one and not the other would be inconsistent |
| 3 | wheelchair | PatientWheelChair | new |
| 4 | iv_stand | IVStand | new |
| 5 | surgical_trolley | SurgicalTrolley, dynamic_hospital_trolley | new |
| 6 | directional_sign | V2.6 `sign_*` models | |
| 7 | collection_point_marker | v3_sign_collection_point | new semantic landmark |
| 8 | lab_test_point_marker | v3_sign_laboratory_test_point | new semantic landmark |

**V2.5 classes not carried over:**
- `forklift`, `pallet`, `box` and `charging_station` exist in the scene only as untextured box primitives. Labelling them would re-teach the "box = object" shortcut behind the Phase 1 failures. They remain LiDAR obstacles and serve as hard negatives.
- `obstacle` has no visual referent.
- `door` has nothing to label, because doorways are open gaps.

**Deliberately unlabelled (hard negatives):** lab benches, reception desk, office desks, cafeteria tables, chairs, drawers, metal cabinet, storage racks, adjustable table, forklift/cart/pallet primitives, pillars and walls.

**Occupants:** MalePatientBed and PatientWheelChair are single meshes that include a person. The occupant is treated as part of the object, and evaluation ignores `person` predictions that lie at least 70% inside such a ground-truth box.

## 3. Rendering setup

- **Dataset world:** `V3/worlds/hospital_logistics_dataset_world.sdf`, built by `V3/datasets/build_dataset_world.py` from the V3 world. The V3 runtime world and V2.6 are unchanged. It adds:
  - Gazebo `Label` system plugins (label = class_id + 1) on the labelled models
  - `dynamic_*` models made static, so they can be repositioned
  - an instance pool of 29 extra props, parked at z = −30 until placed
  - the camera rig described below
- **Camera rig `v3ds_camera_rig`:**
  - one link with an RGB camera, a `boundingbox_camera` (2D) and a `segmentation` camera (instance)
  - intrinsics identical to the robot camera: 640 × 480, horizontal FOV 1.15, clip 0.08–25 m, Gaussian noise 0.005
  - posed at the robot-camera pose: base + 0.22 m forward, 0.41 m above the ground, pitch −0.05 ± 0.02
  - a level `gpu_lidar` identical to the robot LiDAR, at the robot's LiDAR mount relative to the camera (used only for fusion validation)
- **Scene control:**
  - in-process Gazebo transport: `/world/<w>/set_pose_vector/blocking` and `/world/<w>/light_config`
  - a frame is accepted only when the RGB image, boxes and segmentation carry the **same sim timestamp**, are at least 0.5 s of sim time newer than the last frame seen before the pose change, two consecutive frames have identical segmentation, and every label passes the per-frame verification (§7)
  - service requests run in a child process, because in the gz-transport Python binding a blocking request made in a process with active Python subscriptions timed out every time (5.0 s, reproducible)

## 4. Ground truth (no manual or fabricated labels)

1. **Source.** The instance-segmentation camera gives, for every labelled model, its visible pixels in exactly the rendered frame. An instance is the (instance id, label) pair: Gazebo instance ids are unique only within a label, and grouping by id alone merged objects of different classes. A validation check caught this and it was fixed.
2. **Box.** The tight box around the visible pixels. It is identical to Gazebo's `boundingbox_camera` 2D box, which is kept as an independent cross-check.
3. **Occlusion rule.** A tight box around the scattered visible slivers of a mostly hidden object spans its occluder. For example, the pole tip and a wheel sliver of a wheelchair behind the forklift primitive produced a wheelchair box drawn over a plain yellow box, which is exactly the "box = object" shortcut to avoid.
   - `calibrate_visibility.py` renders every labelled model alone (30 poses each) and measures its median unoccluded fill ratio (visible pixels ÷ box area).
   - Per frame, estimated visibility = observed fill ÷ calibrated fill.
   - An instance is labelled if it has at least 40 visible pixels, both box sides are at least 6 px, and estimated visibility is at least 0.35.
   - Otherwise it is recorded as `dropped_small` or `dropped_occluded` in the metadata. Evaluation treats dropped instances as ignore regions.
4. **Independent verification.** `validate_dataset.py` projects the known 3D visual bounds of every placed object through the rig pose and pinhole intrinsics, and requires every label box to lie inside the projection of a same-class object (8 px tolerance). This doesn't depend on Gazebo's labelling path.

## 5. Splits and scene generation

`V3/datasets/generate_dataset.py`, orchestrated by `generate_all.sh`, with fixed seeds.

| Split | Seed | Scene content |
|---|---|---|
| train | 101 | Randomized scenes, with the mode chosen per frame (below). Props relocated around the camera; class-balanced object choice; point lights randomized (global 0.45–1.25 × per-light jitter 0.85–1.15 × warm/neutral/cool tint). |
| val | 202 | Same distribution as train, disjoint seed. |
| test | 303 | The **canonical V3 world layout** (props at their real V3 positions, pool parked). The camera is at robot-camera poses along the real Phase 1 Nav2 routes (70%) or random free poses (30%), and the first 10 frames are the Phase 1 failure viewpoints. The moving person and trolley are placed in view with probability 0.5. Default lighting for 80% of frames, dim or bright for 20%. |
| subsets A–J | 404 | Targeted challenge sets (§6), evaluation only. |

Train and val frame modes:
- **random (45%)**: 1–4 labelled objects and 0–3 negatives in view
- **hard_negative (20%)**: the camera faces fixed furniture, a pillar or a movable negative; half the time a confusable positive is placed next to it (bed beside a bench or desk, trolley beside the forklift, wheelchair or cart beside chairs, IV stand beside a pillar)
- **sign_marker (15%)**: the camera faces a V2.6 sign, or a V3 marker plaque placed on a random wall face
- **empty (10%)**: no movable objects
- **clutter (10%)**: 5–9 labelled objects plus 1–4 negatives

Leakage control:
- The canonical layout (test) never occurs in train or val, where every prop is relocated each frame.
- The subsets use their own seed and placements.
- Shared across splits, and unavoidable in simulation: the building, fixed furniture and the object meshes themselves. Generalization to *other* wheelchair or bed models is therefore **not** measured.

## 6. Challenge subsets

The canonical layout plus controlled placements; 40 frames each. For B, C, D, E and G, even-numbered frames contain the confusable positive and odd-numbered frames contain only the negative. Positive frames are re-rendered (up to 25 attempts) until the target class is visibly labelled, and failures are recorded.

| Subset | Content |
|---|---|
| A hospital beds | canonical ward beds or a pool bed in another room |
| B bed vs lab bench | camera facing lab benches; bed beside them in even frames |
| C bed vs reception desk | camera facing the reception or office desks; bed beside them in even frames |
| D chair vs cart | 2–4 chairs; a medical cart among them in even frames |
| E surgical trolley vs forklift/cart | forklift and warehouse-cart primitives in view; surgical trolley in even frames |
| F wheelchair | wheelchair among 0–3 chairs |
| G IV stand | camera facing a pillar; IV stand beside it in even frames and 40% of odd frames |
| H person | 1–3 people (static models and the moving nurse) |
| I empty hallway | no movable objects within 8 m (signs may be visible) |
| J cluttered corridor | 6–9 random labelled and negative objects |

## 7. Scene/render synchronization failure found and fixed

The first full generation looked correct in the smoke tests. On the full set, however, `validate_dataset.py` reported **6,883 label boxes inconsistent with the scene metadata**.

- Frames 5–400 verified at 100%. From about frame 1,030 on, `random`-mode frames failed while other modes still passed.
- 549 of 2,999 frames were near-identical to their predecessor.
- Frame 1,030 showed a wheelchair and a bed that the frame had not placed.

Probing the live server showed that pose changes of static models were acknowledged (`data: true`) but not reflected in later renders once the long-running server had degraded: 91% CPU after 2 h 42 min, and frame time rising from 0.43 s to 1.2 s. Because the RGB and segmentation renders could also disagree in these frames, **the entire first dataset and the two models trained on it were discarded**. They were moved out of the project and are not reported.

Fixes, all in the committed pipeline:
1. **Freshness.** Capture waits for frames stamped at least 0.5 s of sim time after the last frame seen before the change.
2. **Settling.** Two consecutive synchronized frames must have identical segmentation.
3. **Per-frame verification.** Every kept box must pass the independent projection check, and no labelled instance may be unexplained by the metadata. Otherwise the poses are re-sent and the frame retried up to 4 times, then the scene is discarded and resampled.
4. **Fresh server.** A new Gazebo server is started for every 500-frame chunk and every subset.

Retry and discard statistics are in `meta/sync_stats_*.jsonl`:

| Stage | Render calls | Retries | Discarded (resampled) |
|---|---|---|---|
| train 0–1,616 (before chunking; last logged at frame 1,600) | 1,631 | 205 | 30 |
| train 1,617–2,999 | 1,383 | 0 | 0 |
| val | 600 | 0 | 0 |
| test (one 500-frame chunk) | 669 | 936 | 169 |
| subsets A–J (fresh server each) | 525 | 0 | 0 |

The test chunk degraded after about frame 370. All 38 resampled test frames have index ≥ 371. A fresh-server rerun of the first 60 test scenes produced 0 retries, so no scene type fails systematically. Resampled test frames are still canonical-layout scenes, just with a different camera pose.

## 8. Results

`V3/datasets/v3_hospital_cv/dataset_validation.json`, reproduced in `V3/docs/phase2_result_tables.md`.

| Split | Images | Empty (negative) images | Labelled instances | Dropped small / occluded | Label verification |
|---|---|---|---|---|---|
| train | 3,000 | 759 | 6,471 | 781 / 449 | 6,363 verified + 108 partial, **0 unverified** |
| val | 600 | 155 | 1,295 | 154 / 101 | 1,275 + 20, **0 unverified** |
| test (canonical) | 500 | 127 | 1,504 | 120 / 109 | 1,365 + 139, **0 unverified** |
| subsets A–J | 10 × 40 | 84 | 870 | 136 / 52 | all verified, **0 unverified** |

"Partial" means the object is partly behind the camera plane; the box is checked against the projection clipped to the image border.

Instances per class in train, val and test:

| Class | train | val | test |
|---|---|---|---|
| person | 875 | 185 | 186 |
| hospital_bed | 557 | 109 | 47 |
| cart | 843 | 178 | 39 |
| wheelchair | 461 | 93 | **3** |
| iv_stand | 610 | 113 | **12** |
| surgical_trolley | 547 | 119 | 178 |
| directional_sign | 2,285 | 438 | 1,003 |
| collection_point_marker | 134 | 26 | **4** |
| lab_test_point_marker | 159 | 34 | 32 |

**Coverage limits:**
- The canonical test layout contains only one wheelchair, one IV stand and one collection plaque. The test split therefore has too few of these classes (3, 12 and 4) for reliable per-class metrics, and challenge subsets F and G are the primary evidence for them.
- The marker classes are the smallest in train (134 and 159).
- Every Gazebo cross-check mismatch is on a box the dataset didn't keep: the per-frame check verifies all kept labels, and the remaining mismatches are sub-40-pixel slivers that the small-object rule drops.

Size: 4,500 images, 160 MB. The data is excluded from git by `V3/.gitignore` and is reproducible with `run_dataset_world.sh` and `generate_all.sh`, which use fixed seeds. The sync retries make the exact camera poses of the affected frames depend on timing.

## 9. Reproduction

```
cd ~/CV_Autonomous_Navigation
LOG_DIR=/tmp V3/datasets/generate_all.sh     # starts/stops the dataset world itself, resumable
.venv/bin/python V3/datasets/validate_dataset.py
```
