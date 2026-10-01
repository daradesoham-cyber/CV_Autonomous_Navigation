# V3 Fusion Validation Report (Phase 3: camera–LiDAR spatial perception)

Date: 2026-10-01 · Branch `v3.0-dev`, uncommitted working tree · YOLO model unchanged from Phase 2 (`V3/models_trained/v3_yolov8n_from_v25.pt`, sha256 `2ec5cd9f…`).
All numbers are measured; the source file is given with each table. The success-criteria verdict is in §18.

## 1. Baseline reproduction

The Phase 2 fusion test (`V3/tests/test_v3_fusion.py`, 300 frames, seed 505, conf 0.35) was rerun unchanged before any change was made.

| | Phase 2 report | Phase 3 rerun |
|---|---|---|
| Detected associations / measurable | 894 / 428 | 894 / 428 |
| Range-OK rate | 0.367 | 0.367 |
| ON-object rate (detections) | 0.512 | 0.514 |
| ON-object rate (GT boxes) | 0.468 | 0.470 |
| Detections unmatched to GT | 76 | 76 |

The baseline reproduces. The ≤ 0.002 differences come from the LiDAR's Gaussian noise (σ 0.01 m).

**Test-harness defect found.** The Phase 2 test, and the first Phase 3 corpora, passed a freshly seeded `random.Random(0)` to the renderer every frame. As a result, every frame used a camera pitch of −0.0362 rad instead of the robot's calibrated −0.05 rad: a constant **0.0138 rad (0.79°) calibration offset**.
- The V2.6 association does not use pitch, so the Phase 2 baseline stands.
- The new method uses pitch. All primary results below therefore use corpora recorded at the **exact robot calibration**.
- The offset corpora are kept as a **miscalibration robustness test** (§12).

## 2. Evaluation method

1. **Fusion corpora** (`V3/tests/record_fusion_corpus.py`, `record_all_fusion_corpora.sh`). Each frame stores what a runtime fusion node sees: the RGB image, the raw LaserScan from a `gpu_lidar` identical to the robot's at the robot's LiDAR mount, and V3 YOLO detections. It also stores Gazebo ground truth, used **only for scoring**: instance boxes with object identity and all object poses. A fresh Gazebo server is used per set, and no frames were skipped.

   | Set | Frames | Purpose |
   |---|---|---|
   | `dev` (seed 606) | 300 | the only set used to design and tune the new method |
   | `eval` (seed 505) | 300 | the exact Phase 2 baseline frames; held out |
   | `controlled` | 120 | 12 scene types × 5 distances (1.5–5.5 m) × 2 mirrored layouts; held out |
   | `*_pitchoffset` | 300 / 300 / 120 | the same scenes with the 0.79° offset |

2. **Scoring** (`V3/tests/eval_fusion.py`):
   - Each detection is matched to a Gazebo object (same class, IoU ≥ 0.5). Unmatched detections are counted and excluded. The same evaluation also runs on GT boxes ("oracle").
   - An object is **measurable** if its visual geometry crosses the LiDAR plane (z = 0.235 m).
   - **ON:** the fused LiDAR point lies within 0.25 m of the object's collision footprint.
   - **OFF:** a false association.
   - **GP_ON / GP_OFF:** the V3 camera-only ground-plane fallback, on or off the object.
   - **ABSTAIN:** no estimate.
   - Objects that are not measurable either return VISUAL_ONLY (correct) or FALSE_RANGE.
   - Every scan beam is attributed to the object or structure it hit, for the failure taxonomy.

3. **Parameters were frozen after the dev runs.** Algorithm checksums are in `V3/docs/evidence/phase3_fusion/frozen_algorithm_sha256.txt`. No parameter was changed after looking at `eval` or `controlled`.

## 3. Failure taxonomy of the V2.6 association (measured, `eval` set)

Source: `V3/docs/evidence/phase3_fusion/fusion_eval.json` → `eval.taxonomy_detected` and `taxonomy_oracle`.

The V2.6 association gives 204 OFF (wrong-object) results out of 428 measurable detected objects.

**By category:**

| Category (from per-beam attribution) | Count | Main classes |
|---|---|---|
| Nearer surface selected (foreground occluder or other object in the window) | 83 | person 23, hospital_bed 21, trolley 16, cart 14 |
| Farther surface selected (background seen through gaps) | 72 | **surgical_trolley 45**, cart 19 |
| No LiDAR return on the object inside the window at all | 49 | iv_stand 13, cart 13, person 11 |

**Selected surface:** fixed structure (wall or furniture) 118, another object 85, the object itself 1.

**Other measured factors:**

| Factor | Measurement |
|---|---|
| Window too wide | In **113 of 204** OFF cases, the majority of the selected returns project *outside* the object's image box |
| Sparse lower geometry | Median LiDAR returns per object (bed 37 for reference): iv_stand **1**, cart 2, trolley 3, person 7, wheelchair 17. Objects with **zero** returns: iv_stand 13/27, cart 12/79, trolley 12/107, person 11/159 |
| Box vs. LiDAR-visible surface | The camera sees the whole object, but the LiDAR plane cuts only legs, wheels and poles, so the box's angular span mostly contains background |
| Above-plane objects | 468 signs and markers are not LiDAR-measurable. V2.6 returned a false range for **435** of them |
| Depth ambiguity | "Closest consistent cluster" has no notion of which depth belongs to the box. The oracle GT boxes do no better (ON 0.466), so the cause is the association rule, not detection |
| Calibration | V2.6 extrinsics (dx 0.12 m) match the URDF. Its linear column-to-bearing mapping differs from the exact pinhole by 1.11° median, 1.50° max: minor compared with the window width |
| Timing / robot motion | Not present in the corpora (image and scan share a stamp). Measured at runtime: scan–image offset 10–12.5 ms median, 30–35 ms max (§10) |

## 4. The V2.6 association

`lidar_camera_fusion_node.py` (unchanged), ported verbatim to `V3/perception/lidar_fusion.py`:
1. The box column is mapped to a camera bearing linearly: b = (0.5 − u/W)·HFOV.
2. The angular window spans the whole box, widened over depths {0.4, 0.8, 1.5, 3, 5} m for the 0.12 m baseline, plus 0.03 rad.
3. All ranges in the window are sorted and split into clusters at gaps > 0.35 m.
4. The **closest** cluster with ≥ 3 points is chosen, and its 20th percentile is the range.

## 5. The new V3 association

`V3/perception/spatial_fusion.py` (pure numpy, no ROS). Gazebo ground truth is never an input.

1. **Projection instead of a bearing window.** Each LiDAR return p_L is transformed to the camera optical frame with the calibrated extrinsics and projected with the intrinsics:
   - p_B = p_L + t_LB
   - p_C = R_BCᵀ (p_B − t_CB)
   - p_O = R_OC p_C
   - u = c_x + f_x X/Z, v = c_y + f_y Y/Z

   A return is a candidate for a box only if u lies inside the box **and** v lies inside it vertically, which is a depth-consistency test. Near foreground returns project below the box. If the box is cut off by an image border, the test is open on that side.
2. **Segments, then depth hypotheses.** Candidates are split into angle-contiguous segments (range jump > 0.25 m or angular gap > 3.5 beams). Segments whose median ranges are within 0.40 m are merged into one hypothesis. The separate legs, wheels or poles of a sparse object therefore form one hypothesis instead of being out-voted.
3. **Ground-contact prior (camera only).** The ray through the box's bottom-centre pixel is intersected with the floor plane z_B = −0.06 m, giving r_g, with σ(r_g) = 0.15 + 0.03·r_g² m.
   - If the box touches the bottom image border, the floor contact is *below* the image. The object is then nearer than where the bottom-row ray meets the floor, which gives an **upper-bound prior** r ≤ r_b + 0.25 m (Gaussian decay σ = 0.20 m beyond it).
4. **Scoring instead of "closest".** For hypothesis h at range r_h (its 20th percentile):
   - S = S_prior · (0.4 + 0.6·coverage) · (0.5 + 0.5·support) · S_bg
   - S_prior = exp(−½((r_h − r_g)/σ)²), or the bound term, or 0.5 without any prior
   - coverage = the fraction of the box's 8-px columns that contain a return of h
   - support = min(1, n/n_expected), with n_expected = 1 for iv_stand, 2 for wheelchair, cart, trolley and person, 3 for bed
   - S_bg = 0.5 if the same surface continues within 18 px beyond **both** box edges (walls, large background)

   The best hypothesis with S ≥ 0.15 is taken.
5. **Outputs:** range, bearing, base_link x/y, association confidence S, and `range_source`:
   - `lidar`
   - `ground_plane`: camera-only fallback with low confidence, **not** fed to the costmap or TTC
   - `visual_only`: the box lies entirely above the LiDAR plane
   - `none`

**Parameters chosen on `dev` only:**
- The first version regressed below 1 m (0.545 vs 1.000 ON). Two geometric causes were found: truncated boxes had no prior, and very near returns project below the image. They were fixed with the truncation bound and the open vertical test.
- An inward box margin was swept over {0.10, 0.03, 0.00}. 0.00 was best, because the legs of carts and trolleys sit at the box edges.

## 6. Calibration used

| Item | Value | Source |
|---|---|---|
| Camera intrinsics | f_x = f_y = 493.792, c_x = 320, c_y = 240, 640 × 480 | Gazebo `camera_info` |
| camera_link in base_link | (0.22, 0, 0.35), rpy (0, −0.05, 0); optical = z forward | `camera.xacro` |
| laser_link in base_link | (0.10, 0, 0.175), level | `lidar.xacro` |
| base_link height | 0.06 m above the floor | `robot_core.xacro` |

## 7. Timestamp handling and motion compensation (runtime node)

- **Synchronisation:** detections carry the image stamp, and the closest scan within 100 ms is used (V2.6 rule, kept). Measured scan–image offset: median 10–12.5 ms, max 30–35 ms.
- **Motion compensation**, in `ros2_ws/src/hospital_logistics/hospital_logistics/v3_spatial_fusion_node.py`:
  - LiDAR points are moved from the scan time to the image time with `lookup_transform_full(base_link@t_img ← base_link@t_scan, fixed frame odom)`.
  - When the odom TF is not yet available at t_img, a constant-twist fallback is used with the latest `/odom` (v_x, ω_z) over Δt.
  - The method used is published per frame. Across the final runtime runs, TF was used on 762 frames and the twist fallback on 325; no frame was uncompensated.

## 8. Results by distance (`eval`, held out, exact calibration)

Source: `fusion_eval.json` → `eval.detected`.

| True range | n | V2.6 ON | V2.6 OFF | **V3 ON (LiDAR)** | **V3 OFF** | V3 ON incl. ground-plane fallback |
|---|---|---|---|---|---|---|
| 0–1 m | 46 | 0.978 | 0.022 | 0.978 | **0.000** | 1.000 |
| 1–2 m | 114 | 0.711 | 0.289 | **0.904** | 0.018 | 0.965 |
| 2–3 m | 73 | 0.479 | 0.521 | **0.863** | 0.055 | 0.932 |
| 3–4 m | 82 | 0.280 | 0.707 | **0.756** | 0.098 | 0.890 |
| 4+ m | 113 | 0.319 | 0.655 | **0.637** | 0.150 | 0.726 |
| **all** | 428 | 0.514 | 0.477 | **0.806** | **0.072** | 0.886 |

**Controlled set (held out):**

| Range | V2.6 ON | V3 ON |
|---|---|---|
| 1–2 m | 0.879 | 0.939 |
| 2–3 m | 0.786 | 0.952 |
| 3–4 m | 0.738 | 0.905 |
| 4+ m | 0.697 | 0.829 |
| all | 0.761 (OFF 0.239) | **0.893 (OFF 0.036)** |

## 9. Results by class (`eval`, held out)

| Class | n | V2.6 ON / OFF | **V3 ON / OFF** | V3 abstain | V3 ON incl. fallback |
|---|---|---|---|---|---|
| person | 159 | 0.761 / 0.233 | **0.887 / 0.057** | 0.050 | 0.893 |
| hospital_bed | 45 | 0.489 / 0.511 | **0.867 / 0.133** | 0.000 | 0.867 |
| cart | 79 | 0.418 / 0.582 | **0.747 / 0.089** | 0.025 | 0.886 |
| wheelchair | 11 | 0.455 / 0.545 | **0.818 / 0.182** | 0.000 | 0.818 |
| iv_stand | 27 | 0.185 / 0.815 | **0.519 / 0.074** | 0.259 | 0.667 |
| surgical_trolley | 107 | 0.318 / 0.654 | **0.776 / 0.047** | 0.009 | 0.944 |

**Controlled set by class (V2.6 ON → V3 ON):**
- bed 0.849 → **0.977** (OFF 0.151 → 0.012)
- cart 0.70 → **0.90**
- iv_stand 0.444 → **0.778** (OFF 0.556 → 0)
- surgical_trolley 0.692 → **0.885** (OFF 0.308 → 0)
- wheelchair 1.0 → 1.0
- person 0.655 → 0.655, with OFF 0.345 → 0.172

## 10. Controlled scenes (12 scene types)

Source: `fusion_eval.json` → `controlled.controlled_per_scene`. Counts are of measurable detected objects.

| Scene | n | V2.6 | V3 | V3 median abs. range error (ON) |
|---|---|---|---|---|
| 01 bed alone | 10 | ON 10 | ON 9, ground-plane ON 1 | 0.143 m |
| 02 bed + trolley | 18 | ON 18 | ON 18 | 0.112 m |
| 03 bed + wheelchair | 18 | ON 18 | ON 18 | 0.138 m |
| 04 bed + IV stand | 18 | ON 18 | ON 18 | 0.110 m |
| 05 cart + chair | 10 | ON 10 | ON 10 | 0.130 m |
| 06 trolley + forklift primitive | 10 | ON 6 / **OFF 4** | ON 9, GP-ON 1 | 0.056 m |
| 07 person near bed | 20 | ON 20 | ON 20 | 0.118 m |
| 08 person behind bed | 20 | ON 10 / **OFF 10** | ON 10, GP-ON 3, **OFF 4**, GP-OFF 1, abstain 2 | 0.130 m |
| 09 multiple depths | 30 | ON 15 / **OFF 15** | ON 28, OFF 1, GP-ON 1 | 0.125 m |
| 10 IV stand, 1.5–5.5 m | 10 | **OFF 10** | ON 6, GP-ON 4 | 0.201 m |
| 11 wheelchair, 1.5–5.5 m | 10 | ON 10 | ON 10 | 0.166 m |
| 12 partial occlusion | 23 | ON 15 / **OFF 8** | ON 20, GP-ON 2, GP-OFF 1 | 0.132 m |

The per-frame debug images (§14) show the projected points, the selected hypothesis and the estimate for each detection.

## 11. Dynamic objects (real runtime: robot camera 30 Hz, LiDAR 20 Hz, motion)

Source: `V3/docs/evidence/phase3_runtime/runtime_v3_S*.json` (V3 node), with V2.6 scored offline on the *same* recorded detection/scan pairs (`v26_offline_same_frames`).

| Scenario | True range | V3 ON when detected | V3 false | V3 lost | V2.6 ON (same frames) | Range error median, jitter (σ) | Frame-to-frame excess jump p95 / max | First detection → first ON |
|---|---|---|---|---|---|---|---|---|
| S1 person crossing FOV (0.4 m/s) | 2.65–2.91 m | **1.000** (51/51) | 0 | 0 | n/a (below its tracking gate) | 0.319 m*, σ 0.030 | 0.040 / 0.050 m | 0.0 s |
| S2 trolley approaching (0.3 m/s) | 1.77–3.95 m | **0.808** (215/266) | 0 | 0.192 | **0.070** (lost 0.93) | 0.037 m, σ 0.008 | 0.018 / 0.155 m | 0.595 s |
| S3 robot → static bed (0.25 m/s) | 1.50–3.76 m | **1.000** (282/282) | 0 | 0 | 1.000 | 0.141 m, σ 0.011 | 0.013 / 0.017 m | 0.0 s |
| S4 robot past bed (0.3 m/s) | 0.86–2.84 m | **0.940** (281/299) | 0 | 0.060 | 1.000 | 0.573 m†, σ 0.242 | 0.028 / 0.517 m | 0.0 s |

\* The person's GT footprint is the 0.25 m collision cylinder, but the LiDAR hits the thinner body mesh, so this is a constant offset (jitter 0.03 m).
† While the robot passes the bed, the bed's nearest face leaves the camera FOV. The fused range correctly refers to the *visible* part, while the GT uses the nearest face overall. ON (fused point on the bed) is 94%.

- No false association occurred in any dynamic scenario.
- Lost associations occur on the sparse trolley (19%, against V2.6's 93%) and at the edges of the pass-by (6%).

## 12. Range error, association accuracy, false-association rate, miscalibration robustness

Range errors are over ON cases, measured against the object footprint's nearest face.

| Set | Method | ON | False association (OFF + GP-OFF) | Precision of LiDAR associations | Abstain | Abs. range error median / p95 |
|---|---|---|---|---|---|---|
| eval | V2.6 | 0.514 | 0.477 | 0.519 | 0.009 | 0.225 / 0.628 m |
| eval | **V3** | **0.806** | **0.072** | **0.938** | 0.042 | 0.223 / 0.489 m |
| eval, GT boxes | V2.6 / V3 | 0.466 / 0.777 | 0.525 / 0.059 | 0.470 / 0.944 | | |
| controlled | V2.6 | 0.761 | 0.239 | 0.761 | 0 | 0.129 / 0.346 m |
| controlled | **V3** | **0.893** | **0.036** | **0.972** | 0.010 | 0.127 / 0.342 m |
| dev (tuning set) | V2.6 / V3 | 0.545 / 0.838 | 0.449 / 0.059 | 0.548 / 0.952 | | |
| eval, **0.79° pitch offset** | V2.6 / V3 | 0.510 / 0.765 | 0.480 / **0.133** | 0.515 / 0.865 | | 0.234 / 0.633 · 0.240 / 0.840 m |
| controlled, 0.79° pitch offset | V2.6 / V3 | 0.753 / 0.871 | 0.247 / 0.113 | | | |

Visual-only objects (signs, markers), `eval`:
- V2.6 returned a false range for **435/468**.
- V3 correctly marked **468/468** as visual-only.

**Calibration sensitivity.** The ground-contact prior depends on camera pitch. With the 0.79° offset, V3's false-association rate rises from 0.072 to 0.133, and to 0.297 at 4+ m, though that is still below V2.6's 0.48 and 0.64. Camera extrinsic calibration therefore matters for V3.

## 13. Runtime latency and resource usage (RTX 3050 Laptop 4 GB, 16-thread CPU)

| Measure | Value | Source |
|---|---|---|
| YOLO in the V3 detection node (per frame) | mean 10.8–11.4 ms, p95 12.2–13.1 ms | runtime `summary` |
| V3 fusion node, whole frame (projection, association, tracking/TTC) | mean 1.3–2.7 ms, p95 2.2–4.9 ms | runtime `summary` |
| V3 association, offline, per detection | 0.42 ms mean (eval), 1.27 ms (controlled, large boxes), p95 ≤ 3.1 ms; scan projection 0.22 ms | `fusion_eval.json` |
| V2.6 association, offline, per detection | 0.17–0.20 ms | `fusion_eval.json` |
| Total perception latency, image → fused objects | ≈ 12–16 ms (detection + fusion means) | |
| Detection throughput | 27 Hz with the camera at 30 Hz (runtime S1 window) | |
| GPU memory | detection node process 144 MiB; GPU total in use 163 MiB of 4,096; utilisation about 18% | `nvidia-smi` under load |
| CPU (per process, % of one core) | detection 48% (RSS 1.5 GB), V3 fusion 29% (89 MB), navigation_perception 17% (83 MB), Gazebo 64% | `ps` under load |

The pipeline is real-time. V3's association is about 2–6× slower than V2.6's per detection, but it adds only milliseconds per frame.

## 14. Safety integration (fusion → semantic obstacles → costmap → Nav2)

- **Chain.** The V3 fusion node is a V3-local copy of the V2.6 fusion node with only the association step replaced. The code diff is limited to the association block, the extra `/v3/spatial_objects` publisher, and the class name. It publishes the same `/vision/semantic_obstacles`, `/fused_objects`, `/vision/ttc` and markers. The unchanged V2.6 `navigation_perception_node` turns them into `/vision/costmap_obstacles` for the Nav2 costmaps.
- **The fusion layer never commands velocity.**
- **Only LiDAR-sourced estimates** enter the tracker, TTC and costmap path. Ground-plane fallbacks and visual-only objects are published only on `/v3/spatial_objects`.
- **Beds, carts and trolleys become spatial obstacles.** S3: 43 costmap clouds (105,252 points); S2: 30 clouds; S4: 90 clouds.
- **No phantom obstacles from wrong associations.** False associations in the runtime scenarios: 0. In S3, **0 of 105,252** evaluated costmap points were more than 1 m from a real object.
- **Missing LiDAR returns.** The sparse trolley was lost in 19% of frames with no crash or exception (the node ran for every scenario).
- **Pre-existing V2.6 downstream issues** (V2.6 code, reproduced with the *unmodified* V2.6 fusion node, **not changed in Phase 3**):
  1. **False TTC on static objects during robot motion.** The V2.6 tracker computes velocity against its *exponentially smoothed* previous state. For a static object, the ramp lag inflates the apparent closing speed to about v·(1 + 0.65/0.35), and ego-motion compensation removes only v. Measured on a **static bed**:

     | Fusion node | Robot toward bed: TTC < 1.8 s (min) | Robot past bed: TTC < 1.8 s (min) |
     |---|---|---|
     | V2.6 (unmodified) | **7** (min 1.4 s) | **19** (min 0.4 s) |
     | V3 | 1 (min 1.7 s) | 4 (min 1.1 s) |

     The V2.6 decision engine yields at TTC < 1.8 s.
  2. **Costmap points persist in base_link.** `navigation_perception_node` keeps points for 2.5 s in the robot frame without motion compensation, so they travel with the robot. During the pass-by (S4), **7.1%** of costmap points (11,159 of 156,408 evaluated) were more than 1 m from any real object; with the trolley moving (S2), 1.1%.

  Both are Phase 7 (dynamic obstacle integration) items. Sources: `runtime_v26_node_S3_robot_toward_bed.json`, `runtime_v26_node_S4_robot_past_bed.json`, `runtime_v3_S*.json`.
- **Nav2 with live perception.** Route spawn → collection point → lab, `V3/docs/evidence/phase3_regression/nav2/`:

  | Perception chain | Route runs | Spawn → collection | Collection → lab |
  |---|---|---|---|
  | **V3** | **4 of 5 succeeded**; 1 run aborted at the ward door (controller "collision ahead") | 26.8–38.2 s | 52.4–55.3 s |
  | V2.6 (V2.5 model + V2.6 fusion) | 3 of 3 succeeded | 38.5–66.8 s | 61.0–67.3 s |

  The abort was not reproduced in the next 4 V3 runs, and this sample is too small to show whether V3 changes abort risk. In the recorded V3 reproduction, the ward bed at 2.4 m was flagged dynamic at the doorway (issue 1).

## 15. Regression (Phase 2 perception, V2.6, world, sensors, Nav2)

| Check | Result | Evidence |
|---|---|---|
| YOLO on the canonical test + subsets A–J | **identical** TP/FP/FN on all 11 sets (e.g. test P 0.958, R 0.9249) | `phase3_regression/yolo/evaluation.json` |
| Hard-negative behaviour | identical (same subsets B–E, I) | same |
| V2.6 source files | unchanged: `git diff --stat` is empty, and V2.6 nodes run only as untouched processes | §19 |
| V3 world launches, camera, LiDAR | camera 21.2 Hz, LiDAR 20.0 Hz, all 10 viewpoints teleport, finite beams as in Phase 1 | `phase3_regression/sensors/` |
| Nav2 | see §14: V3 chain 4/5, V2.6 chain 3/3 | `phase3_regression/nav2/` |

## 16. Before/after summary

| Metric (held-out `eval`, exact calibration) | V2.6 | V3 |
|---|---|---|
| Correct LiDAR association (ON) | 0.514 | **0.806** |
| False association | 0.477 | **0.072** |
| ON beyond 2 m (2–3 / 3–4 / 4+) | 0.479 / 0.280 / 0.319 | **0.863 / 0.756 / 0.637** |
| ON below 1 m | 0.978 | 0.978 (0 false; 1.000 including fallback) |
| Foreground selections (all selections of a nearer wrong surface) | 83 | V3 OFF total 31 |
| Sparse classes (iv_stand / trolley / cart) ON | 0.185 / 0.318 / 0.418 | **0.519 / 0.776 / 0.747** |
| False range on visual-only objects | 435 / 468 | **0 / 468** |
| Dynamic trolley ON (same frames) | 0.070 | **0.808** |

## 17. Known limitations

1. **Sparse objects are still hard.** IV stands abstain in 26% of cases (their LiDAR cross-section is often 0–1 beams), and the trolley is lost in 19% of dynamic frames. V3 abstains rather than guessing.
2. **4+ m:** ON 0.637 with OFF 0.15. The ground-contact prior's σ grows with the square of range, so discrimination weakens.
3. **Person behind bed** (scene 08) is the weakest controlled case: V3 had 4 OFF out of 20.
4. **Calibration sensitivity:** a 0.79° pitch error raises V3's false-association rate from 0.072 to 0.133. A real deployment needs a calibrated camera pitch, or on-line estimation.
5. **Flat-floor assumption** in the ground-contact prior. It is valid in this facility; ramps or steps would bias it.
6. **Simulation only.** The same meshes appear in the corpora and at runtime, and Gazebo sensor noise is idealised.
7. **V2.6 downstream issues,** documented and not fixed (Phase 7): false TTC on static objects during robot motion, and costmap points persisting in base_link.
8. **Nav2 with V3 perception:** one abort in 5 runs (§14); the sample is too small to rule out an effect.
9. The runtime range-error figures in S1 and S4 carry GT-definition offsets (see the notes under §11). The ON criterion is the reliable measure there.

## 18. Success criteria (Phase 3.12)

| Criterion | Verdict | Evidence |
|---|---|---|
| Reproducible baseline | **Met** | §1 |
| Measurable improvement over baseline | **Met** | ON 0.514 → 0.806, false association 0.477 → 0.072 (held out) |
| Improved association beyond 2 m | **Met** | 2–3 m 0.479 → 0.863; 3–4 m 0.280 → 0.756; 4+ m 0.319 → 0.637 |
| Fewer foreground/background errors | **Met** | V2.6: 83 foreground + 72 background wrong selections; V3: 31 OFF in total |
| Better sparse-object handling | **Met, with limits** | iv_stand 0.185 → 0.519 (0.074 false), trolley 0.318 → 0.776, cart 0.418 → 0.747; still limited (§17.1) |
| No major regression below 1 m | **Met** | 0.978 = 0.978, 0 false |
| Stable dynamic-object association | **Met** | 0 false in all scenarios; jitter σ 0.008–0.030 m where GT is well defined; max excess jump ≤ 0.155 m except the S4 FOV-edge case (0.52 m) |
| Real time on the RTX 3050 | **Met** | ≈ 12–16 ms total, 163 MiB GPU |
| Integration with the V3 perception/navigation pipeline | **Partly met** | Semantic obstacles → costmap → Nav2 work with no phantoms from associations. But: 1 Nav2 abort in 5 runs (V2.6 chain 3/3), and false TTC on static objects persists (a pre-existing V2.6 tracker issue, reduced but not removed) |

## 19. Evidence locations

**Corpora** (git-ignored, reproducible): `V3/datasets/fusion_corpus/{eval,dev,controlled}` and `*_pitchoffset`.

**Fusion evaluation:** `V3/docs/evidence/phase3_fusion/`
- `fusion_eval.json`
- `fusion_eval_dev_nominal.json`
- `rows_*.json`, one row per association
- `frozen_algorithm_sha256.txt`

**Debug images:** `V3/docs/evidence/phase3_debug/`, showing the camera image with projected, candidate, rejected and selected returns, plus a top-down view with V3, V2.6 and GT.

**Runtime:** `V3/docs/evidence/phase3_runtime/`
- `runtime_v3_S1…S4.json`
- `runtime_v26_node_S3/S4*.json`
- `runtime_fusion.json` (first run)

**Regression:** `V3/docs/evidence/phase3_regression/{yolo,sensors,nav2}`

**Code:**
- `V3/perception/spatial_fusion.py` (new association)
- `V3/perception/lidar_fusion.py` (V2.6 port, plus `fuse_box_debug`)
- `ros2_ws/src/hospital_logistics/hospital_logistics/{v3_detection_node,v3_spatial_fusion_node}.py`
- `V3/tests/{record_fusion_corpus.py, record_all_fusion_corpora.sh, eval_fusion.py, fusion_debug_viz.py, test_v3_runtime_fusion.py, record_semantic_during_nav.py, nav_perception_trials.sh}`

## 20. Exact commands

```
cd ~/CV_Autonomous_Navigation
# 3.1 baseline (dataset world running: V3/datasets/run_dataset_world.sh)
SP=$(.venv/bin/python -c "import site;print(site.getsitepackages()[0])")
PYTHONPATH=$PYTHONPATH:$SP python3 V3/tests/test_v3_fusion.py --model V3/models_trained/v3_yolov8n_from_v25.pt --frames 300
# corpora (fresh world per set)
LOG_DIR=/tmp V3/tests/record_all_fusion_corpora.sh
# evaluation (V2.6 vs V3)
.venv/bin/python V3/tests/eval_fusion.py --set dev
.venv/bin/python V3/tests/eval_fusion.py --set eval --set controlled --set eval_pitchoffset --set controlled_pitchoffset
# debug images
.venv/bin/python V3/tests/fusion_debug_viz.py --set controlled --scenes 04_bed_ivstand 08_person_behind_bed 09_multi_depth 10_ivstand_distance 12_partial_occlusion 06_trolley_forklift --max 2
# runtime
ros2 launch hospital_logistics v3_world.launch.py headless:=true
ros2 run hospital_logistics v3_detection_node --ros-args -p use_sim_time:=true
ros2 run hospital_logistics v3_spatial_fusion_node --ros-args -p use_sim_time:=true
ros2 run autonomous_robot_perception navigation_perception_node --ros-args -p use_sim_time:=true
python3 V3/tests/test_v3_runtime_fusion.py S1_person_crossing --out-name=runtime_v3_S1_person_crossing.json   # likewise S2-S4
# regression
.venv/bin/python V3/training/evaluate_v3.py --models v3_from_v25=V3/models_trained/v3_yolov8n_from_v25.pt --out V3/docs/evidence/phase3_regression/yolo
python3 V3/tests/test_v3_world_sensors.py
ros2 launch autonomous_robot_navigation localization.launch.py map:=$PWD/V3/maps/hospital_logistics_map.yaml
ros2 launch autonomous_robot_navigation navigation.launch.py
V3/tests/nav_perception_trials.sh v3 3 V3/docs/evidence/phase3_regression/nav2/v3_chain
V3/tests/nav_perception_trials.sh v26 3 V3/docs/evidence/phase3_regression/nav2/v26_chain
```
