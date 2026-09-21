# Semantic Sign Perception & Guidance

## 1. Overview

The robot visually detects, interprets, and logs high-contrast directional signs located at corridor junctions. These signs provide high-level semantic hints (e.g., `HOSPITAL ->`, `WAREHOUSE ^`, `STORAGE <-`) that help confirm topological decisions or guide exploration.

---

## 2. Sign Textures & Specifications

15 high-contrast signboards are rendered at $640 \times 320$ resolution in `ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs/`:

| Filename | Semantic Text | Direction | Primary Color (BGR) |
| :--- | :--- | :--- | :--- |
| `hospital_right.png` | HOSPITAL | RIGHT | `(180, 50, 20)` (Royal Blue/Indigo) |
| `hospital_straight.png` | HOSPITAL | STRAIGHT | `(180, 50, 20)` |
| `warehouse_straight.png`| WAREHOUSE | STRAIGHT | `(20, 100, 200)` (Warm Gold/Orange) |
| `warehouse_right.png` | WAREHOUSE | RIGHT | `(20, 100, 200)` |
| `office_straight.png` | OFFICE | STRAIGHT | `(30, 30, 30)` (Charcoal) |
| `office_right.png` | OFFICE | RIGHT | `(30, 30, 30)` |
| `office_left.png` | OFFICE | LEFT | `(30, 30, 30)` |
| `lab_left.png` | LAB | LEFT | `(150, 40, 140)` (Purple) |
| `lab_straight.png` | LAB | STRAIGHT | `(150, 40, 140)` |
| `storage_left.png` | STORAGE | LEFT | `(40, 120, 180)` (Amber) |
| `storage_right.png` | STORAGE | RIGHT | `(40, 120, 180)` |
| `cafeteria_right.png` | CAFETERIA | RIGHT | `(40, 140, 40)` (Green) |
| `cafeteria_left.png` | CAFETERIA | LEFT | `(40, 140, 40)` |
| `exit_straight.png` | EXIT | STRAIGHT | `(30, 140, 30)` (Bright Green) |
| `emergency_exit.png` | EMERGENCY EXIT | STRAIGHT | `(20, 20, 200)` (Red) |

---

## 3. Computer Vision Detection Pipeline

`sign_detection_node.py` processes `/camera/image_raw` at >100 FPS through the following stages:

1. **Candidate Region Extraction**:
   - Multiple intensity thresholds ($160, 185$) detect bright signboard backgrounds against darker corridor walls.
   - Contour filtering selects rectangular shapes with aspect ratio between $1.1$ and $3.2$, with bounding area $> 300\text{ px}$.
   - Non-Maximum Suppression (NMS) removes overlapping bounding boxes.

2. **Template Matching & Classification**:
   - Each candidate ROI is extracted and normalized to $160 \times 80$ grayscale.
   - Normalized cross-correlation (`cv2.TM_CCOEFF_NORMED`) is evaluated against all 15 precomputed sign thumbnails.
   - Detections with peak score $\ge 0.45$ are classified.

3. **Message Publishing**:
   - Publishes `SignDetectionArray` on `/vision/signs`.
   - Publishes annotated image with green bounding boxes, semantic labels, and confidence tags on `/vision/annotated_signs`.

---

## 4. Directional Interpretation & Decision Making

When a sign is observed at node $u$ with label $L$ and direction $D$:
1. The detection is recorded in SQLite: `memory.record_sign(node_id, text, direction, conf)`.
2. If the active mission destination matches $L$:
   - The Decision Engine verifies that the outgoing edge from $u$ in the planned path aligns with the indicated direction (e.g. `RIGHT` at $u$ matches the branch with heading $+90^\circ$ relative to the robot's approach direction).
   - If the robot was in exploration mode without a prior map, the sign directly resolves which corridor branch to take.
