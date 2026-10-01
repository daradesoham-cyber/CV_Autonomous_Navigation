# V3 Perception Validation Report (Phase 2)

Date: 2026-09-30 · Branch: `v3.0-dev` (uncommitted working tree)
Primary V3 model: `V3/models_trained/v3_yolov8n_from_v25.pt` (YOLOv8n, 3.01 M parameters, 9 classes, `V3/perception/v3_classes.yaml`)
Full generated tables: `V3/docs/phase2_result_tables.md`. Raw evidence: `V3/docs/evidence/phase2_*`.

## 1. Models and training

| Run | Init | Epochs (best) | Train time | Val mAP50 / mAP50-95 / P / R (best epoch) |
|---|---|---|---|---|
| `v3_yolov8n_from_v25` (primary) | `models/yolov8n_v25.pt`, detection head re-initialised for 9 classes | 60 (60) | 2,744 s | 0.948 / 0.872 / 0.957 / 0.880 |
| `v3_yolov8n_from_coco` (comparison) | `yolov8n.pt` (COCO) | 60 (58) | 2,737 s | 0.948 / 0.874 / 0.942 / 0.902 |

Recipe: `V3/training/train_v3.py`. Same as V2.5 (YOLOv8n, imgsz 640, batch 16, default augmentation, seed 42) except the data, 60 epochs with patience 15, and `deterministic=True`. Val comes from the training distribution. The evaluations below use the held-out canonical test split and the challenge subsets.

## 2. Evaluation protocol (`V3/training/evaluate_v3.py`)

- **Ground truth:** Gazebo instance segmentation, verified per frame (dataset report §4 and §7).
- **Matching:** same class, IoU ≥ 0.5, greedy by confidence.
- **Operating point:** confidence 0.35 for V3 (the V2.6 global default). V2.5 is evaluated at 0.35 and at its deployed per-class thresholds.
- **Not counted as false positives:**
  - predictions on instances the generator dropped as too small or mostly occluded (IoU ≥ 0.5, same class)
  - `person` predictions at least 70% inside a ground-truth bed or wheelchair, because the occupant is part of that mesh
- **V2.5 out-of-taxonomy predictions** (forklift, pallet, box, obstacle, door, charging_station) are always false positives and are listed separately.
- **False-positive attribution:** every false positive is attributed to the scene object it covers (projected 3D bounds of placed props, V2.6 furniture, pillars and walls, or a ground-truth object of another class).
- **Reported per set and model:** AP50 and AP50-95 per class; TP / FP / FN / precision / recall at the operating point; FP per image; example images.
- **ultralytics `val`** is also run for the V3 models: mAP, P, R and the confusion matrix.

## 3. Canonical V3 test split (500 images, 1,504 instances)

| Metric | V2.5 (deployed thresholds) | **V3 (init V2.5)** | V3 (init COCO) |
|---|---|---|---|
| Precision @ operating point | 0.589 | **0.958** | 0.939 |
| Recall @ operating point | 0.294 | **0.925** | 0.916 |
| FP per image | 0.618 (+56 out-of-taxonomy) | **0.122** | 0.180 |
| mAP50 / mAP50-95 (evaluator) | 0.106 / 0.058 | **0.804 / 0.689** | 0.809 / 0.676 |
| ultralytics mAP50 / mAP50-95 / P / R | not comparable (different taxonomy) | 0.821 / 0.710 / 0.833 / 0.801 | 0.796 / 0.683 / 0.853 / 0.755 |

Per class, V3 (init V2.5), at the 0.35 operating point:

| Class | GT | Precision | Recall | AP50 | AP50-95 |
|---|---|---|---|---|---|
| person | 186 | 0.989 | 0.973 | 0.981 | 0.952 |
| hospital_bed | 47 | 0.850 | 0.723 | 0.823 | 0.667 |
| cart | 39 | 0.842 | **0.410** | 0.538 | 0.471 |
| wheelchair | 3 | 1.000 | 0.667 | 0.667 | 0.333 |
| iv_stand | 12 | **0.421** | 0.667 | 0.514 | 0.345 |
| surgical_trolley | 178 | 0.972 | 0.961 | 0.996 | 0.939 |
| directional_sign | 1,003 | 0.965 | 0.941 | 0.969 | 0.843 |
| collection_point_marker | 4 | 1.000 | 0.750 | 0.750 | 0.725 |
| lab_test_point_marker | 32 | 1.000 | 1.000 | 1.000 | 0.921 |

Confusion matrix: `V3/docs/evidence/phase2_eval/ultralytics_val_v3_from_v25/confusion_matrix.png`

## 4. Challenge subsets (40 frames each; precision, recall, FP, FN over all classes)

| Subset | V2.5 deployed P / R / FP / FN | **V3 (init V2.5) P / R / FP / FN** | Key target-class result (V3) |
|---|---|---|---|
| A hospital beds | 0.177 / 0.109 / 51 / 90 | **0.946 / 0.861 / 5 / 14** | bed 43/48, 2 FP |
| B bed vs lab bench | 0.093 / 0.062 / 49 / 75 | **0.973 / 0.887 / 2 / 9** | bed 20/20, 0 FP (V2.5: 13 bed FPs on benches) |
| C bed vs reception desk | 0.033 / 0.056 / 59 / 34 | **0.886 / 0.861 / 4 / 5** | bed 20/21, 0 FP (V2.5: 24 pallet FPs on desks) |
| D chair vs cart | 0.090 / 0.200 / 132 / 52 | **0.902 / 0.846 / 6 / 10** | cart 20/20, 1 FP on a chair (V2.5: 45 `person` + 13 `cart` FPs on chairs) |
| E trolley vs forklift/cart | 0.056 / 0.059 / 51 / 48 | **0.900 / 0.882 / 5 / 6** | trolley 19/20; 1 FP on the warehouse cart, 0 on the forklift |
| F wheelchair | 0.092 / 0.065 / 59 / 86 | **0.916 / 0.826 / 7 / 16** | wheelchair 38/40, 2 FP |
| G IV stand | 0.338 / 0.165 / 51 / 132 | **0.913 / 0.861 / 13 / 22** | IV stand 23/25, 0 FP |
| H person | 0.600 / 0.468 / 34 / 58 | **0.971 / 0.927 / 3 / 8** | person 70/70, 1 FP |
| I empty hallway | 0.148 / 0.111 / 23 / 32 | **0.865 / 0.889 / 5 / 4** | 0.125 FP/image (V2.5 0.575) |
| J cluttered corridor | 0.283 / 0.183 / 66 / 116 | **0.920 / 0.810 / 10 / 27** | |

Subset recall below the target-class figure mostly comes from incidental small or distant `directional_sign` and `person` instances in the same frames.

**Hard-negative false-positive attribution, V3 (init V2.5):**
- **B:** 1 `directional_sign` on a bed, 1 duplicate trolley box. No detection on a bench.
- **C:** 2 duplicate person boxes, 1 sign and 1 lab marker on pillars. No detection on a desk.
- **D:** 1 cart on a chair, 2 person on a wall.
- **E:** 1 trolley on the warehouse cart, 1 on the storage rack. No forklift confusion.
- **I:** 3 `directional_sign` on storage racks, 1 trolley on the assembly bench, 1 wheelchair on a cart.

**Example detections** (V2.5 @0.35 | V2.5 deployed | V3 init V2.5 | V3 init COCO, side by side, ground truth in green): `V3/docs/evidence/phase2_eval/examples_<set>.png` for the test set and each subset.

## 5. Remaining perception failures (canonical test, V3 init V2.5)

| Failure | Evidence | Assessment |
|---|---|---|
| **cart recall 0.41** (TP 16, FN 23) | 19 of 23 missed carts are the lab `InstrumentCart1` seen at about 30 px on a side (distant, beside the metal cabinet); 2 BPCart and 2 BloodPressureMonitor misses | Small, distant, specific-instance weakness. In subset D (carts at 1.2–4.5 m) recall is 1.00. |
| **iv_stand precision 0.42** (12 GT; TP 8, FP 11, FN 4) | 6 of 11 FPs are boxes on the real IV stand with IoU < 0.5 (thin pole: localisation and duplicates); 5 genuine FPs (2 on a wall, 1 on a pillar, 1 on a storage rack, 1 on background) | Mostly localisation of a very thin object. Subset G, which is IV-stand focused, has 0 FPs. |
| hospital_bed recall 0.72 (TP 34, FN 13) | In a separate conf-0.35 re-run, 9 of 11 misses were the TrolleyBed: 6 cut off at the image edge at close range, the rest distant (about 20 px) | Close-range partial views of large objects. |
| duplicate/localisation `directional_sign` | 20 FPs "sign on GT:sign" (IoU < 0.5) plus 11 on background, out of 1,003 signs | Low rate, 0.035 per sign. |
| Rare classes thinly tested on the canonical split | wheelchair 3, IV stand 12, collection marker 4 GT | Subsets F and G carry the evidence for these classes. |
| Generalisation beyond these meshes / to real images | not measured | See the domain-transfer report §6. |

## 6. Inference cost (RTX 3050 Laptop 4 GB)

`V3/perception/benchmark_inference.py`: 300 canonical test images, 20 warm-up, single-image calls as in the ROS node. Gazebo and training were stopped during the run.

| Model | Preprocess / Inference / Postprocess | End-to-end mean / p50 / p95 | FPS | Peak torch allocated / reserved | Process VRAM (nvidia-smi) |
|---|---|---|---|---|---|
| V2.5 | 0.51 / 4.59 / 0.78 ms | 6.93 / 6.89 / 7.64 ms | 144.3 | 35 / 60 MiB | 144 MiB |
| **V3** | 0.49 / 4.55 / 0.79 ms | 6.88 / 6.82 / 7.50 ms | **145.4** | 47 / 70 MiB | 154 MiB† |

The architecture and size are identical, so the cost is identical. This is well within real time on the 30 Hz camera (the V2.6 node uses frame_skip 2).
† Measured in the same process right after V2.5, so it includes the CUDA context. The model itself is the torch-allocated figure.

## 7. Camera-LiDAR fusion

`V3/tests/test_v3_fusion.py`: 300 rendered frames (canonical and randomized), dataset-rig `gpu_lidar` identical to the robot LiDAR, V3 detections at conf 0.35.

- **Method under test:** the V2.6 association, ported unchanged to `V3/perception/lidar_fusion.py`:
  - linear column → bearing mapping
  - camera–LiDAR baseline envelope
  - closest cluster at its 20th percentile
  - position in base_link
- **Ground truth:** the object's collision footprint in the LiDAR plane.
- **Two criteria:**
  - **range OK:** within ±0.30 m of the footprint's nearest face
  - **ON object:** the fused point lies within 0.25 m of the object footprint, meaning the LiDAR cluster belongs to the object

| Boxes | Associations | Not in LiDAR plane | Measurable | Range OK | ON object | Range error of OK cases (mean / median / p95) | Position error of OK cases (mean / p95) |
|---|---|---|---|---|---|---|---|
| V3 detections | 894 | 466 | 428 | 0.367 | **0.512** | 0.182 / 0.194 / 0.285 m | 0.262 / 0.405 m |
| GT boxes (oracle) | 961 | 504 | 457 | 0.337 | 0.468 | 0.180 / 0.191 / 0.281 m | 0.262 / 0.403 m |

**ON object by true range (V3 detections):**

| Range | 0–1 m | 1–2 m | 2–3 m | 3–4 m | 4–5 m | 5–6 m | 6–7 m |
|---|---|---|---|---|---|---|---|
| ON object | 0.98 | 0.70 | 0.48 | 0.26 | 0.30 | 0.42 | 0.29 |

**ON object per class (V3 detections):**

| Class | ON object |
|---|---|
| person | 121/161 |
| hospital_bed | 19/42 |
| cart | 35/81 |
| surgical_trolley | 34/108 |
| wheelchair | 5/10 |
| iv_stand | 5/26 |

`directional_sign`, `lab_test_point_marker` and `collection_point_marker` (466 associations) lie above the 0.235 m LiDAR plane and cannot be measured by the 2D LiDAR. Their range must come from other means, such as known map positions.

**Findings.**
1. **Association works reliably only at close range.** Within 1 m, 98% of associations land on the object, which is the regime that matters for the V2.6 TTC stop logic (critical distance 2.5 m). Beyond 2 m the V2.6 method picks the wrong surface most of the time.
2. **Detection quality is not the limiting factor.** Perfect ground-truth boxes do no better (46.8% vs 51.2%).
3. **The failure mode is structural.** The V2.6 envelope spans the whole box width plus the baseline correction plus 0.03 rad, and takes the *closest* cluster in it. For sparse objects (trolleys, carts, beds, IV stands) the beams pass between legs and wheels, so the closest cluster is often a nearer occluder, a wall edge or a different object in the same angular window. Of the 267 range failures, 146 are too far (beams passed through or beside the object) and 121 too near. Of the 205 off-object associations, 122 hit something nearer (an occluder or a neighbouring object in the envelope) and 83 something farther (background seen through or beside the object).
4. **Fused distances are real LiDAR measurements, but association is unreliable beyond about 1.5 m.** The dashboard must show which object a range belongs to and its association confidence, not an unqualified distance.
5. **Consequence for later phases** (not changed in Phase 2, V2.6 untouched): an improved V3 association should be evaluated with this same harness against this baseline. Candidates:
   - a narrower central-column window
   - per-class sparse-object handling
   - a cluster chosen by the box's vertical extent and expected size rather than simply the closest

Examples (green = ON object / range OK, red = wrong surface, yellow = not in LiDAR plane): `V3/docs/evidence/phase2_fusion/fusion_examples.png`

## 8. Collection point / laboratory markers

- `collection_point_marker` and `lab_test_point_marker` are dedicated classes, trained on the V3 plaques placed on random walls from varied angles and distances (train: 134 and 159 instances).
- Canonical test: collection marker 3/4 at precision 1.00; lab marker 32/32 at precision 1.00. Both markers' AP50-95 is above 0.72.
- They provide visual **semantic confirmation** of the two logistics locations. Navigation truth stays with the map coordinates, as the brief requires.
- **Limitation:** the collection plaque appears only 4 times in the canonical test split, so its recall figure is weak evidence.
- The markers are above the LiDAR plane, so their range is not LiDAR-measurable.

## 9. Reproduction

```
cd ~/CV_Autonomous_Navigation
LOG_DIR=/tmp V3/datasets/generate_all.sh                          # dataset (about 2 h on the RTX 3050)
.venv/bin/python V3/datasets/validate_dataset.py                  # label verification + statistics
.venv/bin/python V3/training/train_v3.py --name v3_yolov8n_from_v25
.venv/bin/python V3/training/train_v3.py --name v3_yolov8n_from_coco --init coco
.venv/bin/python V3/training/evaluate_v3.py --models v25=models/yolov8n_v25.pt v25_deployed=models/yolov8n_v25.pt \
    v3_from_v25=V3/models_trained/v3_yolov8n_from_v25.pt v3_from_coco=V3/models_trained/v3_yolov8n_from_coco.pt
.venv/bin/python V3/perception/benchmark_inference.py --models v25=models/yolov8n_v25.pt v3=V3/models_trained/v3_yolov8n_from_v25.pt
V3/datasets/run_dataset_world.sh &    # then, with ROS sourced and the venv site-packages on PYTHONPATH:
python3 V3/tests/test_v3_fusion.py --model V3/models_trained/v3_yolov8n_from_v25.pt --frames 300
python3 V3/training/make_phase2_tables.py
```
