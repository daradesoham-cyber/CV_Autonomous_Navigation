# YOLO V2.5 Handoff Document
## For: Gemini 3.8 — Dataset Preparation, Training, Evaluation & Model Selection

**Created by:** V2.5 Pipeline Engineering Pass  
**Project:** `~/CV_Autonomous_Navigation`  
**Baseline model:** `models/yolov8n_v24.pt`  
**Config file:** `config/perception_v25.yaml`

---

## 1. The Pipeline Is Now Model-Agnostic

To swap in a new YOLO model, you need only:

1. **Replace the `.pt` file** (copy new model to `models/`).
2. **Update `config/perception_v25.yaml`** — two fields:
   ```yaml
   yolo:
     model_path: "models/your_new_model.pt"
     class_names:   # must match model.names exactly, in order
       - person
       - cart
       - ...
   ```
3. **No code changes required** for a same-class-set upgrade.

If the new model has different classes or a different class order, also update:
- `class_names` in `perception_v25.yaml`
- `class_confidence_thresholds` in `perception_v25.yaml` (per-class thresholds)
- `SEMANTIC_SAFETY_CONFIG` in `lidar_camera_fusion_node.py` (if class names changed)

The node validates the loaded model's class names against `class_names` at startup and prints a warning if they differ.

---

## 2. Required Classes (Current V2.4/V2.5 Model)

The pipeline is built around **exactly these 10 classes**, in this order:

| ID | Class Name        | Category            | Safety-Critical |
|----|-------------------|---------------------|-----------------|
| 0  | `person`          | Dynamic obstacle    | ✅ Yes           |
| 1  | `cart`            | Dynamic obstacle    | ✅ Yes           |
| 2  | `forklift`        | Dynamic obstacle    | ✅ Yes (largest) |
| 3  | `pallet`          | Static obstacle     | No               |
| 4  | `box`             | Static obstacle     | No               |
| 5  | `obstacle`        | Generic obstacle    | No               |
| 6  | `door`            | Structure           | No               |
| 7  | `charging_station`| Semantic landmark   | No               |
| 8  | `hospital_bed`    | Semantic obstacle   | No               |
| 9  | `directional_sign`| Navigation sign     | Navigation-only  |

> **Do NOT change class IDs or names** unless you also update `perception_v25.yaml`.  
> The pipeline consumes **class name strings**, not class IDs, so reordering within the same set is safe as long as the YAML is updated.

---

## 3. Per-Detection Output Format

Each detection published on `/vision/detections` (`Detection2DArray`) contains:

```
Detection2D:
  class_name:   string   # e.g. "person", "forklift"
  confidence:   float32  # YOLO confidence [0, 1]
  x_min:        float32  # image bounding box (pixels)
  y_min:        float32
  x_max:        float32
  y_max:        float32
  distance:     float32  # meters (filled by LiDAR fusion; -1.0 if no LiDAR match)
  bearing:      float32  # radians relative to robot forward axis
```

Confirmed `directional_sign` detections are published separately on `/vision/signs` (`SignDetectionArray`):

```
SignDetection:
  text:       string   # e.g. "HOSPITAL", "STORAGE"
  direction:  string   # "LEFT", "RIGHT", "STRAIGHT", "BLOCKED"
  confidence: float32
  x_min/y_min/x_max/y_max: float32  # bounding box (pixels)
```

Signs only reach `/vision/signs` after **temporal confirmation** (3 frames in 2s window, mean conf ≥ 0.45). This is intentional — single-frame sign false positives must not trigger navigation decisions.

---

## 4. Current Model Weaknesses (Why V2.5 Exists)

Based on V2.4 mission validation (20% success after Step G, 300 TTC yields):

### A. High False-Positive Rate in Dynamic Classes
- `person`, `cart`, `forklift` — detected spuriously on walls, doors, dynamic shadows
- Caused 300 TTC yield events in 10 missions (10× more than V2.3's 30)
- TTC yields accumulated to 1.8s × 300 = 9 minutes of wasted pause time
- **Root cause:** COCO-pretrained backbone generalized poorly to facility textures

### B. Zero Sign Detections in V2.3; 34 in V2.4 (Still Low)
- `directional_sign` detections: 0 in V2.3 → 34 in V2.4 after map improvements
- Still insufficient for reliable navigation — robot cannot confirm routes
- **Root cause:** insufficient `directional_sign` training examples with facility-appropriate wall textures

### C. Static Obstacle Class Confusion
- `box`, `pallet`, `obstacle` are sometimes confused with each other
- Not safety-critical (all three have similar safety radii) but degrades semantics

---

## 5. Training Recommendations

### 5.1 Priority Classes for Dataset Expansion

**Highest priority — dataset must cover these well:**

| Class             | Why Priority               | Key Failure Modes to Train Against       |
|-------------------|----------------------------|------------------------------------------|
| `person`          | TTC safety-critical        | Partial occlusion, motion blur, far distance (4–8m), near-wall |
| `forklift`        | TTC safety-critical        | Front vs. side vs. rear views, partially in frame |
| `cart`            | TTC safety-critical        | Empty vs. loaded, low profile, partial frame |
| `directional_sign`| Navigation-critical        | Oblique angles (15°–45°), partial occlusion, distance 1–6m |

**Medium priority:**

| Class             | Why Priority               | Key Failure Modes to Train Against       |
|-------------------|----------------------------|------------------------------------------|
| `door`            | Geometry reference         | Open vs. closed, different lighting      |
| `hospital_bed`    | Large footprint obstacle   | Occupied vs. empty, partial view         |
| `charging_station`| Landmark                   | Overhead view, with cables               |

**Lower priority (already reasonable):**
- `pallet`, `box`, `obstacle` — can share negative hard examples

### 5.2 Input Format Requirements

The pipeline feeds raw camera frames at **640×480, BGR8** to YOLO. YOLO resizes internally to `image_size=640` (square). Train at **640×640** for best compatibility.

### 5.3 Recommended Dataset Composition

For `directional_sign` specifically — the key bottleneck:
- Minimum 500 images per sign type (HOSPITAL, STORAGE, OFFICE, etc.)
- Include: straight-on, 15° left, 15° right, 30° oblique angles
- Include: close (1–2m), medium (3–5m), far (5–8m) distances
- Include: partial occlusion by obstacles (cart passing in front)
- Include: different ambient lighting (overcast, spot, dim corridor)

For dynamic classes (`person`, `cart`, `forklift`):
- Minimum 1000 images per class
- Include: stationary and moving (motion blur expected at ~0.5m/s camera motion)
- Include: OUTSIDE path (beside robot) — these must be correctly classified with low confidence, not confused with in-path obstacles
- Include: crossing the robot's path (lateral movement)

### 5.4 Confidence Thresholds (Current V2.5 Targets)

The per-class thresholds in `perception_v25.yaml` are set for **safety** (precision over recall):

| Class             | Threshold | Rationale                                      |
|-------------------|-----------|------------------------------------------------|
| `person`          | 0.55      | False positives cause unnecessary TTC pauses   |
| `forklift`        | 0.55      | Same — large, slow-moving, safety-critical     |
| `cart`            | 0.50      | Dynamic, but smaller than forklift             |
| `directional_sign`| 0.30      | Temporal gate (3-frame confirmation) provides safety; low thresh captures more sign detections |
| All others        | 0.40      | Balanced                                       |

A better model should target:
- **mAP50 ≥ 0.65** on dynamic classes (`person`, `cart`, `forklift`)
- **mAP50 ≥ 0.70** on `directional_sign` (most important for navigation)
- **Precision ≥ 0.75** on `person` and `forklift` at conf=0.55

---

## 6. How to Replace the Model (Step-by-Step)

```bash
# 1. Copy new weights
cp /path/to/new_model.pt ~/CV_Autonomous_Navigation/models/yolov8n_v25.pt

# 2. Edit perception_v25.yaml
nano ~/CV_Autonomous_Navigation/config/perception_v25.yaml
# → Update: yolo.model_path: "models/yolov8n_v25.pt"
# → Update: yolo.class_names (if classes changed)
# → Adjust: class_confidence_thresholds (based on new model's precision characteristics)

# 3. Verify class names match
cd ~/CV_Autonomous_Navigation && source .venv/bin/activate
python3 -c "
from ultralytics import YOLO
m = YOLO('models/yolov8n_v25.pt')
print([m.names[i] for i in sorted(m.names.keys())])
"

# 4. Syntax check
python3 -m py_compile ros2_ws/src/autonomous_robot_perception/autonomous_robot_perception/object_detection_node.py

# 5. Build
cd ros2_ws && colcon build --symlink-install

# 6. Launch — node will print class validation at startup
ros2 launch autonomous_robot_bringup full_system.launch.py
```

---

## 7. Pipeline Architecture Reference

```
Camera (/camera/image_raw, 640x480 BGR8)
        │
        ▼ [frame_skip=2]
ObjectDetectionNode (object_detection_node.py)
        │  loads: perception_v25.yaml
        │  applies: per-class confidence thresholds
        │  applies: min_box_area filter (400 px²)
        │
        ├──► /vision/detections  (Detection2DArray)
        │         └─► LidarCameraFusionNode
        │                  │ sync with /scan
        │                  ├──► /vision/ttc (Float32)
        │                  ├──► /vision/semantic_obstacles
        │                  └──► /fused_objects
        │
        └──► /vision/signs  (SignDetectionArray)
               [Only after temporal gate: 3 frames in 2s, mean conf ≥ 0.45]
                     └─► DecisionEngineNode
                              │ penalizes edges if CLOSED/BLOCKED sign
                              │ logs confirmed route signs
                              └──► Nav2 (NavigateToPose action)
```

---

## 8. Files Modified in V2.5

| File | Change |
|------|--------|
| `config/perception_v25.yaml` | **NEW** — single source of truth for all perception parameters |
| `autonomous_robot_perception/object_detection_node.py` | Full V2.5 rewrite: config-driven, per-class thresholds, temporal sign gate, min_box_area |
| `autonomous_robot_perception/launch/perception.launch.py` | Added `perception_config_path` arg, removed hardcoded thresholds |

---

## 9. What Was NOT Changed

- `lidar_camera_fusion_node.py` — unchanged; robust, no class-ID dependencies
- `decision_engine_node.py` — unchanged; uses class name strings only, correct
- `dashboard_backend.py` — unchanged; all telemetry is measured, no fake values
- `navigation_memory.py` — unchanged; SQLite WAL mode intact
- `topological_graph.py` — unchanged
- The Gazebo world, map, and navigation config — unchanged
- TTC thresholds — unchanged (1.8s, validated in V2.4)

---

*This document was generated after a complete audit of the V2.4 → V2.5 perception pipeline. All measurements are from actual V2.4 mission runs (10 missions, 0 collisions, 2/10 success). Do not modify V2.4 mission metrics in this document.*
