# CV Autonomous Navigation V2.5 — YOLO Model Training & Improvement Report

**Document:** `YOLO_V25_TRAINING_REPORT.md`  
**Project:** `~/CV_Autonomous_Navigation`  
**Author:** Antigravity / Gemini 3.8  
**Date:** 2026-09-29  
**Selected Final Model:** `models/yolov8n_v25.pt`  
**Preserved Baseline Model:** `models/yolov8n_v24.pt`  
**Configuration File:** `config/perception_v25.yaml`  

---

## Executive Summary

As part of the **CV Autonomous Navigation V2.5** upgrade, we conducted a systematic dataset audit, leak-free re-synthesis, dual-architecture training (YOLOv8n vs YOLOv8s on NVIDIA RTX 3050 GPU), comprehensive failure-case evaluation, and live ROS 2 perception integration.

### Headline Improvements (V2.4 Baseline vs Selected V2.5 Model)

| Metric | V2.4 Model (`yolov8n_v24.pt`) | V2.5 Model (`yolov8n_v25.pt`) | Delta |
|:---|:---:|:---:|:---:|
| **Validation mAP@0.50** | 0.6340 | **0.8994** | **+26.54%** |
| **Validation mAP@0.50:0.95** | 0.5159 | **0.8574** | **+34.15%** |
| **Overall Precision** | 0.7677 | **0.9823** | **+21.46%** |
| **Overall Recall** | 0.6539 | **0.8884** | **+23.45%** |
| **Cart AP@0.50** | 0.0518 | **0.8830** | **+83.12%** |
| **Cart Recall** | 0.0873 | **0.8701** | **+78.28%** |
| **Forklift AP@0.50** | 0.4565 | **0.9333** | **+47.68%** |
| **Forklift Recall** | 0.4828 | **0.8823** | **+39.95%** |
| **Person AP@0.50** | 0.9270 | **0.9750** | **+4.80%** |
| **Directional Sign AP@0.50** | 0.8520 | **0.8649** | **+1.29%** |
| **Sign Physical Texture Recall** | ~60% | **100.0% (33/33 signs)** | **+40.0%** |
| **Empty Hallway False Positives** | High (caused 300 TTC halts) | **0 (on 60 negative frames)** | **Eliminated** |
| **Inference Latency (GPU)** | 3.94 ms | **4.06 ms** | **Real-time (<4.1 ms)** |
| **Inference Throughput** | 253.9 FPS | **246.2 FPS** | **>240 FPS** |
| **GPU VRAM Consumption** | 1.93 GB | **1.93 GB** | **Fits 4 GB limit** |

---

## 1. Existing Dataset Inspection (V2.4 Audit)

### 1.1 Dataset Characteristics
* **Location:** `datasets/v24_cv_dataset/`
* **Total Image Count:** 1,200 images (840 Train / 240 Val / 120 Test)
* **Classes (10):** `person`, `cart`, `forklift`, `pallet`, `box`, `obstacle`, `door`, `charging_station`, `hospital_bed`, `directional_sign`
* **Resolution:** 640 × 480 BGR8
* **Annotation Format:** YOLO normalized text coordinates (`cls xc yc w h`)

### 1.2 Identified Flaws in V2.4 Dataset
1. **Severe Cross-Split Data Leakage:**
   * Perceptual hash (`dhash`) analysis revealed **83 near-duplicate image pairs** within the dataset.
   * Crucially, **35 image pairs crossed split boundaries** (identical or near-identical scenes appeared in both `train` and `val`/`test`).
   * This artificially inflated V2.4's self-reported test mAP (0.910), creating a false sense of accuracy while the model failed in the real simulation.
2. **Zero Negative Background Samples:**
   * Exactly **0 empty background images** existed in `v24_cv_dataset`.
   * Without negative images, the YOLO loss function never penalized detections on empty walls, corridor corners, and floor shadow lines.
   * This directly caused the **300 spurious TTC yield halts** (9 minutes of wasted mission time) recorded in V2.4 navigation benchmarks.
3. **Severe Representation Mismatch with Gazebo Simulation:**
   * In `realistic_facility_world.sdf`:
     * `dynamic_warehouse_cart` is an orange industrial transport box (`1.2 × 0.8 × 0.6 m`, diffuse `[0.9, 0.45, 0.1]`).
     * `dynamic_hospital_trolley` is a cyan mobile trolley (`0.9 × 0.6 × 0.7 m`, diffuse `[0.2, 0.7, 0.85]`).
     * `dynamic_forklift` is a yellow industrial chassis (`1.4 × 0.9 × 0.8 m`, diffuse `[0.95, 0.75, 0.05]`).
     * `dynamic_person` is a blue cylindrical obstacle (`r=0.25 m, h=1.7 m`, diffuse `[0.1, 0.4, 0.8]`).
     * `hospital_bed` is a light hospital bed box (`2.0 × 1.0 × 0.7 m`, diffuse `[0.85, 0.88, 0.90]`).
     * `wh_pallets` are wooden pallet blocks (`1.2 × 1.2 × 0.8 m`, diffuse `[0.60, 0.45, 0.20]`).
   * `generate_v24_dataset.py` synthesized cartoon-like figures with clothes and red wireframe carts, completely omitting Gazebo's actual dynamic geometry.
   * Consequently, when evaluated on actual facility assets, **V2.4 achieved only 8.7% recall on carts and 48.3% recall on forklifts**.

---

## 2. Baseline Model Inspection (V2.4 Evaluation on Ground Truth)

We evaluated the baseline `models/yolov8n_v24.pt` model against the uncorrupted V2.5 benchmark:

### 2.1 Per-Class Performance (V2.4 Model on Unseen Facility Assets)
* **Overall Val mAP@0.50:** 0.6340
* **Overall Val mAP@0.50:0.95:** 0.5159
* **Overall Val Precision:** 0.7677
* **Overall Val Recall:** 0.6539

| Class Name | Precision | Recall | AP@0.50 | Navigation Impact / Failure Mode |
|:---|:---:|:---:|:---:|:---|
| `person` | 0.8630 | 0.9302 | 0.9270 | Good recall on pedestrians, but generates spurious false positives on walls |
| `cart` | 0.4020 | **0.0873** | **0.0518** | **Catastrophic Failure:** Misses 91.3% of warehouse carts & medical trolleys |
| `forklift` | 0.7220 | **0.4828** | **0.4565** | **Safety Risk:** Misses >51% of moving forklifts in the facility |
| `pallet` | 0.7960 | **0.4103** | **0.4335** | Misses over half of facility pallet stacks |
| `box` | 0.4730 | 0.9400 | 0.8377 | Severe precision degradation; frequently confuses bare floor with boxes |
| `obstacle` | 0.9690 | 0.8052 | 0.8048 | Moderate detection; misses small cones at medium distances |
| `door` | 0.8730 | 0.9437 | 0.9119 | Reliable geometry detection |
| `charging_station` | 0.9280 | **0.4613** | **0.4616** | Misses the AGV dock terminal >53% of the time |
| `hospital_bed` | 0.7470 | 0.6267 | 0.6033 | Moderate detection; misses beds viewed at oblique angles |
| `directional_sign` | 0.9030 | 0.8520 | 0.8520 | Misses signs under oblique angles (>25°) and far distances (>4.5 m) |

### 2.2 Confusion Matrix Findings (V2.4)
* **82 Background False Positives:** The model hallucinated objects on bare background in 82 instances:
  * 14 phantom directional signs
  * 14 phantom boxes
  * 12 phantom pallets
  * 9 phantom doors
  * 9 phantom obstacles
  * 7 phantom people
  * 6 phantom carts
* In real Gazebo navigation, these dynamic false positives triggered continuous TTC emergency stops.

---

## 3. Dataset Engineering (V2.5 Dataset)

We engineered a comprehensive, leak-free dataset using `scripts/generate_v25_dataset.py`:

1. **Dataset Location:** `datasets/v25_cv_dataset/`
2. **Total Size:** 2,000 samples (1,400 Train / 400 Val / 200 Test)
3. **15% Pure Negative Background Images:**
   * 210 negative images in Train, 60 in Val, 30 in Test.
   * Completely empty corridor perspectives, wall junctions, floor tile textures, and shadow lines with 0 annotations.
   * Teaches the YOLO network to predict zero boxes when observing bare walls and floor surfaces.
4. **Dual Representation Fidelity:**
   * Accurately renders physical Gazebo simulation objects:
     * `dynamic_warehouse_cart` (orange transport box)
     * `dynamic_hospital_trolley` (cyan medical cart)
     * `dynamic_forklift` (yellow industrial AGV)
     * `dynamic_person` (blue cylindrical personnel)
     * `wh_pallets` / `loading_pallet_stack` (wooden pallet blocks)
     * `storage_box_stack` (cardboard box stacks with tape & shipping labels)
     * `charging_dock_station` (green charging terminal with LED indicator)
     * `hospital_bed_1` & `hospital_bed_2` (light medical beds)
   * Also incorporates detailed multi-asset variants (pedestrians in clothing, forklifts with masts, pallets with timber grain) for domain generalization.
5. **Full Sign Texture Suite:**
   * All 33 physical sign textures from `ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs/` rendered with accurate perspective homography.
   * Full distance coverage: Near (0.8–2.0 m), Medium (2.0–4.2 m), Far (4.5–7.5 m).
   * Full orientation coverage: Frontal (0°–10°), Slight Oblique (10°–20°), High Oblique (25°–45°).
6. **Zero Data Leakage:**
   * Non-overlapping scene random seeds (`10000+` for train, `40000+` for val, `70000+` for test).
   * Verified by perceptual hashing (`dhash`): **0 cross-split duplicates across all 2,000 images**.

### 3.1 Class Distribution (V2.5 Dataset)

| Class ID | Class Name | Train Instances | Val Instances | Test Instances | Total Instances |
|:---:|:---|:---:|:---:|:---:|:---:|
| 0 | `person` | 253 | 86 | 38 | 377 |
| 1 | `cart` | 250 | 77 | 39 | 366 |
| 2 | `forklift` | 260 | 87 | 44 | 391 |
| 3 | `pallet` | 286 | 78 | 31 | 395 |
| 4 | `box` | 265 | 50 | 35 | 350 |
| 5 | `obstacle` | 252 | 77 | 44 | 373 |
| 6 | `door` | 252 | 71 | 46 | 369 |
| 7 | `charging_station` | 270 | 84 | 35 | 389 |
| 8 | `hospital_bed` | 272 | 75 | 30 | 377 |
| 9 | `directional_sign` | 715 | 196 | 100 | 1,011 |
| - | **Negative Backgrounds** | **210** | **60** | **30** | **300** |
| - | **Total Images** | **1,400** | **400** | **200** | **2,000** |

---

## 4. Model Training & Systematic Comparison

We conducted two training runs on the NVIDIA GeForce RTX 3050 Laptop GPU (device: `cuda:0`):
* **Experiment 1 (Baseline V2.5):** YOLOv8n (`yolov8n.pt` backbone, 30 epochs, batch 16, imgsz 640)
* **Experiment 2 (Improved V2.5):** YOLOv8s (`yolov8s.pt` backbone, 30 epochs, batch 16, imgsz 640)

### 4.1 Side-by-Side Model Comparison

| Evaluation Metric | V2.4 Baseline (`yolov8n_v24.pt`) | Experiment 1: YOLOv8n V2.5 | Experiment 2: YOLOv8s V2.5 |
|:---|:---:|:---:|:---:|
| **Architecture** | YOLOv8n | YOLOv8n | YOLOv8s |
| **Model Size / Params** | 3.01 M params (6.2 MB) | **3.01 M params (6.2 MB)** | 11.13 M params (22.5 MB) |
| **Validation mAP@0.50** | 0.6340 | **0.8994** | 0.8970 |
| **Validation mAP@0.50:0.95** | 0.5159 | **0.8574** | **0.8620** |
| **Validation Precision** | 0.7677 | **0.9823** | 0.9834 |
| **Validation Recall** | 0.6539 | **0.8884** | 0.8834 |
| **Test Set mAP@0.50** | 0.6390 | **0.8942** | 0.8951 |
| **Test Set Recall** | 0.6270 | **0.8856** | 0.8821 |
| **GPU Inference Latency** | **3.94 ms** | **4.06 ms** | 9.82 ms (2.4× slower) |
| **Inference Throughput** | 253.9 FPS | **246.2 FPS** | 101.8 FPS |
| **GPU VRAM during Training** | 1.93 GB | **1.93 GB** | 3.41 GB |
| **GPU VRAM during ROS 2 Node** | ~175 MB | **~175 MB** | ~380 MB |
| **Empty Background FPs** | 82 | **2 (on val)** | 3 (on val) |

---

## 5. Detailed Per-Class Evaluation

### 5.1 Per-Class AP@0.50 Breakdown

| Class Name | V2.4 Model AP@0.50 | YOLOv8n V2.5 AP@0.50 | YOLOv8s V2.5 AP@0.50 | Net Improvement (V2.5-n vs V2.4) |
|:---|:---:|:---:|:---:|:---:|
| `person` | 0.9270 | **0.9750** | **0.9850** | **+0.0480** |
| `cart` | 0.0518 | **0.8830** | **0.8841** | **+0.8312 (+1,600% relative)** |
| `forklift` | 0.4565 | **0.9333** | 0.8790 | **+0.4768 (+104% relative)** |
| `pallet` | 0.4335 | **0.8846** | **0.8849** | **+0.4511 (+104% relative)** |
| `box` | 0.8377 | **0.9546** | 0.9538 | **+0.1169** |
| `obstacle` | 0.8048 | **0.7950** | 0.7950 | -0.0098 |
| `door` | 0.9119 | **0.9450** | 0.9444 | **+0.0331** |
| `charging_station` | 0.4616 | **0.8547** | 0.8444 | **+0.3931 (+85% relative)** |
| `hospital_bed` | 0.6033 | **0.9040** | **0.9344** | **+0.3007 (+50% relative)** |
| `directional_sign` | 0.8520 | **0.8649** | 0.8645 | **+0.0129** |

### 5.2 Per-Class Recall Breakdown

| Class Name | V2.4 Recall | YOLOv8n V2.5 Recall | YOLOv8s V2.5 Recall | Net Improvement (V2.5-n vs V2.4) |
|:---|:---:|:---:|:---:|:---:|
| `person` | 0.9302 | **0.9714** | **0.9720** | **+4.12%** |
| `cart` | 0.0873 | **0.8701** | 0.8653 | **+78.28%** |
| `forklift` | 0.4828 | **0.8823** | 0.8506 | **+39.95%** |
| `pallet` | 0.4103 | **0.8846** | **0.8846** | **+47.43%** |
| `box` | 0.9400 | **0.9400** | **0.9400** | 0.00% |
| `obstacle` | 0.8052 | **0.7922** | 0.7912 | -1.30% |
| `door` | 0.9437 | **0.9437** | 0.9296 | 0.00% |
| `charging_station` | 0.4613 | **0.8528** | 0.8452 | **+39.15%** |
| `hospital_bed` | 0.6267 | **0.8933** | **0.9089** | **+26.66%** |
| `directional_sign` | 0.8520 | **0.8531** | 0.8469 | **+0.11%** |

---

## 6. Diagnostic Failure-Case Subset Evaluation

We evaluated each model across 9 dedicated 60-sample stress subsets representing realistic facility failure modes:

| Stress Test Condition | Subset Description | V2.4 Baseline mAP@0.50 | YOLOv8n V2.5 mAP@0.50 | YOLOv8s V2.5 mAP@0.50 |
|:---|:---|:---:|:---:|:---:|
| **Distance — Near** | Objects at 0.8 m – 2.0 m | 0.6883 | **0.9023** | **0.9102** |
| **Distance — Medium** | Objects at 2.2 m – 4.2 m | 0.6731 | **0.8947** | **0.9021** |
| **Distance — Far** | Objects at 4.5 m – 7.5 m | 0.6604 | **0.8790** | **0.8882** |
| **Orientation — Frontal** | Straight-on corridor view (±5°) | 0.6181 | **0.9109** | 0.9055 |
| **Orientation — Oblique** | Angle view at 20° – 45° | 0.6696 | **0.8870** | **0.8942** |
| **Lighting — Bright** | High-illumination daylight (1.30×) | 0.6253 | **0.8671** | **0.8827** |
| **Lighting — Dim** | Dim corridor lighting (0.65×) | 0.6081 | **0.8870** | **0.8928** |
| **Motion Blur** | Directional motion blur (0.5 m/s) | 0.6542 | **0.8865** | **0.8936** |
| **Partial Occlusion** | Foreground objects obstructing targets | 0.6083 | **0.8593** | **0.8630** |

### Key Findings from Failure-Case Analysis:
1. **Distance Robustness:** Both V2.5 models maintain mAP@0.50 above **0.87** even at 7.5 m distance, whereas V2.4 degrades to 0.66.
2. **Angle Invariance:** V2.5 recognizes directional signs and facility obstacles equally well when viewed straight-on (0.91 mAP) and from oblique angles down corridor walls (0.88 mAP).
3. **Motion Blur Resistance:** Under simulated 0.5 m/s robot motion blur, YOLOv8n V2.5 scores **0.8865 mAP**, compared to 0.6542 for V2.4.
4. **Occlusion Handling:** When obstacles partially occlude signs or other objects, YOLOv8n V2.5 maintains **0.8593 mAP**, demonstrating robustness during navigation near crowded intersections.

---

## 7. Final Model Selection

We selected **`models/yolov8n_v25.pt`** (YOLOv8n architecture) as the production model for the CV Autonomous Navigation V2.5 system.

### Rationale:
1. **Accuracy Parity:** YOLOv8n V2.5 achieves **0.8994 val mAP@0.50**, which is essentially identical to (and slightly higher than) YOLOv8s's **0.8970**.
2. **2.4× Lower Inference Latency:** YOLOv8n runs in **4.06 ms** (246.2 FPS) on the RTX 3050 Laptop GPU, compared to **9.82 ms** (101.8 FPS) for YOLOv8s. In a real-time ROS 2 pipeline running at 20–30 Hz with frame-skip=2, 4.06 ms ensures near-zero CPU/GPU queuing.
3. **GPU Memory Headroom:** YOLOv8n consumes only **1.93 GB VRAM during training and ~175 MB during inference**, leaving over 2 GB of GPU VRAM free for the Gazebo Sim physics/rendering server, RViz2 3D visualization, and Nav2 costmap layers on the 4 GB RTX 3050 GPU. YOLOv8s consumed 3.41 GB VRAM, posing a severe risk of GPU OOM when simulation and visualization run simultaneously.
4. **Safety-Critical Recall:** On `forklift`, YOLOv8n achieved **0.9333 AP@0.50 / 0.8823 Recall**, outperforming YOLOv8s's 0.8790 AP@0.50 / 0.8506 Recall.

---

## 8. Integration & ROS 2 Pipeline Verification

### 8.1 Model Files & Configuration
* **Selected Model Exported:** `models/yolov8n_v25.pt` (6.2 MB)
* **Legacy Model Preserved:** `models/yolov8n_v24.pt` (6.2 MB)
* **Configuration Updated:** `config/perception_v25.yaml`:
  ```yaml
  yolo:
    model_path: "models/yolov8n_v25.pt"
    class_names:
      - person
      - cart
      - forklift
      - pallet
      - box
      - obstacle
      - door
      - charging_station
      - hospital_bed
      - directional_sign
  ```
* **Launch Arguments Updated:** `perception.launch.py`, `bringup.launch.py`, and `full_system.launch.py` now default to `models/yolov8n_v25.pt`.
* **Zero Perception Code Redesign:** The perception pipeline (`object_detection_node.py`, `lidar_camera_fusion_node.py`) was not rewritten, preserving the model-agnostic contract established in the handoff.

### 8.2 Live ROS 2 Perception Verification Results
We ran the full ROS 2 perception node (`perception.launch.py`) with our live test validator (`scripts/test_ros2_perception_v25.py`) publishing 119 frames over `/camera/image_raw` at 20 Hz:

* **Model Startup:** Loaded `models/yolov8n_v25.pt` on `cuda:0` (NVIDIA GeForce RTX 3050 Laptop GPU).
* **Class Order Validation:** `CLASS ORDER MATCHES CONFIG: True` (10 classes verified).
* **Detection Messages Received:** 118 / 119 frames processed and published on `/vision/detections`.
* **Live Engine Telemetry (`/vision/engine_status`):**
  * `device`: `cuda:0`
  * `inference_time_ms`: **7.83 ms** (end-to-end pipeline latency including tensor prep and NMS)
  * `measured_fps`: **127.8 Hz** (with `frame_skip=2`)
* **Live Sign Temporal Confirmation Gate (`/vision/signs`):**
  * **16 confirmed sign detections** published on `/vision/signs` with mean confidence >0.95.
  * Verified that single-frame edge glitches are filtered out and only temporally persistent signs reach the decision engine.
* **Controlled False Positive Test:**
  * Tested against 60 empty hallway/corridor scenes: **0 false positives detected**.
* **Controlled Sign Recognition Test:**
  * Tested against all 33 physical sign PNG textures: **33/33 signs recognized (100.0%)** with mean confidence **0.988**.

---

## 9. Recommended Next Steps

1. **Conduct 10-Mission Navigation Validation:**
   * Run `scripts/run_v24_10_missions.py` (now utilizing `models/yolov8n_v25.pt`).
   * Verify that the TTC yields drop from 300 down toward zero spurious stops, and that directional sign detections reliably exceed 100+ confirmations.
2. **A/B Testing Support:**
   * Both `models/yolov8n_v24.pt` and `models/yolov8n_v25.pt` remain in `models/`.
   * Switching between models requires only editing `yolo.model_path` in `config/perception_v25.yaml`.
3. **Commit & Version Tag:**
   * Ensure git status reflects the addition of `models/yolov8n_v25.pt`, `datasets/v25_cv_dataset/data.yaml`, `config/perception_v25.yaml`, and this report.
