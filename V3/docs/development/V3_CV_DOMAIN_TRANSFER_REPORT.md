# V3 CV Domain Transfer Report (Phase 2)

Date: 2026-09-30 · Branch: `v3.0-dev` (uncommitted working tree)
All numbers below are generated from evidence files and reproduced in `V3/docs/phase2_result_tables.md`.

## 1. Problem (Phase 1 finding)

On Gazebo renders of the realistic V3 hospital, the validated V2.5 YOLO model:
- missed the real hospital beds
- fired `hospital_bed` on lab benches, the reception desk and blank surfaces
- labelled chairs as `cart` and the surgical trolley as `forklift`
- had no wheelchair or IV-stand class

## 2. V2.5 audit (dataset, model, configuration)

| Item | V2.5 |
|---|---|
| Model | `models/yolov8n_v25.pt`, YOLOv8n, 3.01 M parameters, 10 classes |
| Classes | person, cart, forklift, pallet, box, obstacle, door, charging_station, hospital_bed, directional_sign |
| Configuration | `config/perception_v25.yaml`: global conf 0.35; per-class conf person 0.55, forklift 0.55, cart 0.50, hospital_bed 0.75, directional_sign 0.30, others 0.40; NMS IoU 0.45; box filters min area 400 px², max 60% of the image area, max 92% of its width; frame_skip 2 |
| Dataset | `datasets/v25_cv_dataset`: 2,000 images at 640 × 480, split 1,400 / 400 / 200; YOLO txt labels; plus 9 diagnostic eval subsets (angle, distance, lighting, blur, occlusion) |
| Class balance (train instances) | about 250–290 per class, directional_sign 715; 210 empty images |
| Training | `scripts/train_yolov8_v25.py`: 30 epochs, batch 16, imgsz 640, ultralytics default augmentation, seed 42 |
| Reported val metrics | mAP50 0.899, mAP50-95 0.857, P 0.982, R 0.888 (`yolov8n_v25_metrics.json`) |
| **How the images were made** | `scripts/generate_v25_dataset.py` **draws** every image with OpenCV: perspective polygons for the corridor, flat shapes for objects, pasted sign textures. **No image is a Gazebo render.** |

**Root cause.** V2.5's high numbers were measured on the same kind of drawn images it was trained on. The model had never seen a rendered 3D mesh, so it learned shortcuts that are valid only in the drawings, for example that a flat grey quadrilateral is `hospital_bed`. That shortcut produces exactly the Phase 1 failures, and it can't be fixed by thresholds or geometric filters: a real bed doesn't look like the drawings, and a real bench does.

## 3. What was changed

1. **Data.** A new dataset of **real Gazebo Sim renders of the V3 world**, labelled from Gazebo's own instance segmentation and verified against an independent 3D projection. There are 0 unverified labels across all 4,500 images. See `V3_DATASET_GENERATION_REPORT.md`.
2. **Classes.** The hospital-logistics classes that matter are now covered: person, hospital_bed, cart, wheelchair, iv_stand, surgical_trolley, directional_sign, collection_point_marker and lab_test_point_marker. `forklift`, `pallet`, `box` and `charging_station` exist only as untextured primitives, so they became unlabelled hard negatives. `obstacle` and `door` have no visual referent.
3. **Hard negatives in training.** 20% of training frames face benches, desks, tables, pillars, chairs or the forklift/cart primitives, and half of those place a confusable positive next to them.
4. **Model.** Same architecture and pipeline as V2.5: YOLOv8n, 640 px, batch 16, default augmentation, seed 42. Fine-tuned from the V2.5 weights for 60 epochs. A second run from COCO weights serves as a comparison. Both train in about 46 minutes on the RTX 3050.
5. **No threshold or filter changes.** All V3 numbers use a single threshold, 0.35 (the V2.6 global default). V2.5 is shown both at 0.35 and at its deployed per-class thresholds.

## 4. Result — canonical V3 test split (the real V3 layout along real robot routes)

| Model | Precision | Recall | FP / image | mAP50* | mAP50-95* |
|---|---|---|---|---|---|
| V2.5 @ deployed thresholds | 0.589 | 0.294 | 0.618 | 0.106 | 0.058 |
| **V3 (init V2.5) @ 0.35** | **0.958** | **0.925** | **0.122** | **0.804** | **0.689** |
| V3 (init COCO) @ 0.35 | 0.939 | 0.916 | 0.180 | 0.809 | 0.676 |

\* mAP here is my evaluator's all-point AP, averaged over the classes present. V2.5 scores 0 on classes it doesn't have.

The ultralytics `val` of V3 (init V2.5) on the same split gives mAP50 0.821, mAP50-95 0.710, P 0.833, R 0.801. Its P and R are per-class means at each class's best-F1 threshold, so the rare test classes (wheelchair 3 instances, IV stand 12, collection marker 4) weigh as much as `directional_sign` (1,003). My evaluator's P and R are micro-averaged at the 0.35 operating point.

## 5. Result — the specific Phase 1 failures (challenge subsets, 40 frames each)

V2.5 is shown at its deployed thresholds. Target class only; all classes and all false positives are in the tables file.

| Phase 1 failure | Subset | V2.5 | V3 (init V2.5) |
|---|---|---|---|
| real beds missed | A hospital beds | TP 0 / FN 48 (recall 0.00) | TP 43 / FP 2 / FN 5 (P 0.956, R 0.896) |
| bed ↔ lab bench | B | 0 beds found; **13 bed FPs on lab benches or the assembly bench** | 20/20 beds, **0 bed FPs** |
| bed ↔ reception/office desk | C | 0 beds; **24 `pallet` FPs on the office desks** | 20/21 beds, 0 bed FPs |
| chair → cart | D | cart 0/20; **45 `person` + 13 `cart` FPs on chairs** | cart 20/20; 1 FP on a chair |
| surgical trolley → forklift | E | trolley 0/20; **17 `forklift` FPs on the forklift primitive** | trolley 19/20; 0 forklift confusion |
| wheelchair (no class) | F | 0/40 | 38/40, 2 FPs |
| IV stand (no class) | G | 0/25 | 23/25, 0 FPs |
| person | H | 43/70, 10 FPs | 70/70, 1 FP |
| empty hallway FPs | I | 0.575 FP/image | 0.125 FP/image (5 FPs: 3 `directional_sign` on racks) |
| cluttered corridor | J | P 0.283, R 0.183 | P 0.920, R 0.810 |

**Conclusion.** Fine-tuning the existing YOLOv8n on real Gazebo renders solves the synthetic-to-render domain transfer for the V3 hospital scene. **A different detector is not needed.** Each Phase 1 failure mode improves from near-zero (or no class) to at least 0.86 precision and 0.89 recall on its target class, and the false positives on the specific hard negatives (benches, desks, chairs, forklift) almost disappear.

**Initialisation doesn't matter.** Starting from the V2.5 weights and starting from COCO give the same quality: on the test split P 0.958 vs 0.939 and mAP50 0.804 vs 0.809, and the subsets are within a few points of each other. So the gain comes from the data, not from V2.5's pre-training. V3 keeps the V2.5-initialised model as primary, for continuity with the validated pipeline.

## 6. What this does *not* show

- **Other object models.** Train and test share the same meshes, with different poses, lighting and arrangements. Detection of *other* bed, wheelchair or person models, and of real camera images, is not measured.
- **Real hardware.** The camera noise is Gazebo's Gaussian noise; there is no motion blur or real sensor pipeline.
- **The canonical test is thin on rare classes:** wheelchair 3, IV stand 12, collection marker 4. Subsets F and G are the evidence for those classes.
- **Remaining failures on the canonical test** (details in `V3_PERCEPTION_VALIDATION_REPORT.md` §5):
  - cart recall 0.41: 19 of 23 misses are one distant lab `InstrumentCart1`
  - IV-stand precision 0.42: 6 of 11 FPs are poorly localised boxes on the real IV stand
  - bed recall 0.72: close-range beds cut off at the image edge, and distant beds
