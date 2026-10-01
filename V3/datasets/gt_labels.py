"""
Ground-truth extraction shared by the V3 dataset generator, calibrator and validator.

Source of truth: Gazebo's instance-segmentation camera on the dataset rig (pixel = instance id,
label), rendered in the same frame as the RGB image. For each labelled instance:
  visible_px   = number of pixels of that instance
  xyxy         = tight box around those visible pixels (identical to Gazebo's boundingbox_camera
                 '2d' box, which is kept as a cross-check)
  fill         = visible_px / box area
  vis_est      = fill / reference_fill(model)   (reference = median unoccluded fill measured by
                 calibrate_visibility.py; planar signs/markers use 0.9)
Label rule: keep if visible_px >= MIN_VISIBLE_PX, both box sides >= MIN_BOX_PX and
vis_est >= MIN_VIS_EST. Otherwise the instance is recorded as dropped (small / occluded) and not
written to the YOLO label file. The rule exists because a tight box around the scattered visible
slivers of a mostly-hidden object spans its occluder (see V3_DATASET_GENERATION_REPORT.md).
Instance -> scene object assignment (needed for per-model reference fill) uses the projection of the
object's known 3D visual bounds through the rig pose and pinhole intrinsics (best IoU, same class).
"""
import json
import math
import os

import numpy as np

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_META = json.load(open(os.path.join(V3_ROOT, "datasets", "dataset_world_objects.json")))
OBJ = {o["name"]: o for o in WORLD_META["objects"]}
CALIB_PATH = os.path.join(V3_ROOT, "datasets", "visibility_calibration.json")
W, H, HFOV = 640, 480, 1.15
F = (W / 2) / math.tan(HFOV / 2)
NAMES = ["person", "hospital_bed", "cart", "wheelchair", "iv_stand", "surgical_trolley",
         "directional_sign", "collection_point_marker", "lab_test_point_marker"]
PLANAR_CLASSES = {6, 7, 8}
PLANAR_REF_FILL = 0.9
MIN_VISIBLE_PX = 40
MIN_BOX_PX = 6
MIN_VIS_EST = 0.35
FIXED_LABELLED = {n: o["canonical_pose"] for n, o in OBJ.items() if o["class_id"] is not None and n.startswith("sign_")}
VISUAL_MODEL = {"dynamic_person": "Scrubs", "dynamic_hospital_trolley": "SurgicalTrolley"}


def model_key(name):
    return OBJ[name].get("model") or VISUAL_MODEL.get(name, name)


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def project_object(name, pose, rig):
    """Projected image box of the object's 3D visual AABB -> ((x0, y0, x1, y1), partial) or (None, False)."""
    mn, mx = OBJ[name]["visual_aabb"]
    corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
    world = corners @ rot(*pose[3:6]).T + np.array(pose[:3])
    cam = (world - np.array(rig[:3])) @ rot(*rig[3:6])
    fwd, right, down = cam[:, 0], -cam[:, 1], -cam[:, 2]
    front = fwd > 0.08
    if not front.any():
        return None, False
    u = W / 2 + F * right[front] / fwd[front]
    v = H / 2 + F * down[front] / fwd[front]
    box = [u.min(), v.min(), u.max(), v.max()]
    partial = not front.all()
    if partial:
        box = [min(box[0], 0), min(box[1], 0), max(box[2], W), max(box[3], H)]
    return tuple(box), partial


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def extract_instances(seg):
    # Gazebo instance ids are only unique within a label, so an instance is the (id, label) pair.
    inst_id = seg[..., 0].astype(np.int32) + 256 * seg[..., 1].astype(np.int32)
    label = seg[..., 2].astype(np.int32)
    key = inst_id * 256 + label
    out = []
    for k in np.unique(key):
        iid, lab = int(k // 256), int(k % 256)
        if lab == 0:
            continue
        mask = key == k
        ys, xs = np.nonzero(mask)
        x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1
        px = int(mask.sum())
        out.append({"instance": int(iid), "class_id": lab - 1, "visible_px": px,
                    "xyxy": [x0, y0, x1, y1], "fill": round(px / max(1, (x1 - x0) * (y1 - y0)), 4)})
    return out


def assign_objects(instances, placed, rig):
    """Attach the most likely scene object (same class, best IoU of projected 3D bounds)."""
    cands = dict(FIXED_LABELLED)
    cands.update(placed)
    proj = {}
    for n, pose in cands.items():
        if n in OBJ and OBJ[n]["class_id"] is not None and "visual_aabb" in OBJ[n]:
            proj[n] = project_object(n, pose, rig)
    for inst in instances:
        best, best_iou = None, 0.0
        for n, (box, _) in proj.items():
            if box is None or OBJ[n]["class_id"] != inst["class_id"]:
                continue
            v = iou(inst["xyxy"], box)
            if v > best_iou:
                best, best_iou = n, v
        inst["object"] = best
        inst["assign_iou"] = round(best_iou, 3)
    return instances


def load_calibration():
    return json.load(open(CALIB_PATH))["reference_fill"] if os.path.exists(CALIB_PATH) else None


def decide(inst, calib):
    x0, y0, x1, y1 = inst["xyxy"]
    if inst["visible_px"] < MIN_VISIBLE_PX or x1 - x0 < MIN_BOX_PX or y1 - y0 < MIN_BOX_PX:
        inst["status"] = "dropped_small"
        return inst
    if inst["class_id"] in PLANAR_CLASSES:
        ref = PLANAR_REF_FILL
    else:
        key = model_key(inst["object"]) if inst.get("object") else None
        ref = calib.get(key) if key else None
        if ref is None:  # unassigned: use the smallest reference of the class (most permissive)
            refs = [v for k, v in calib.items() if any(model_key(n) == k and OBJ[n]["class_id"] == inst["class_id"] for n in OBJ)]
            ref = min(refs) if refs else PLANAR_REF_FILL
    inst["ref_fill"] = ref
    inst["vis_est"] = round(inst["fill"] / ref, 3)
    inst["status"] = "kept" if inst["vis_est"] >= MIN_VIS_EST else "dropped_occluded"
    return inst


def verify_boxes(boxes, placed, rig, tol_px=8.0):
    """Independent label check: each box must lie inside the projected 3D bounds of a placed object of
    the same class (or of a fixed labelled sign). Returns a status per box: VERIFIED / PARTIAL /
    UNVERIFIED."""
    cands = dict(FIXED_LABELLED)
    cands.update(placed)
    out = []
    for b in boxes:
        x0, y0, x1, y1 = b["xyxy"]
        status = "UNVERIFIED"
        for n, pose in cands.items():
            if n not in OBJ or OBJ[n]["class_id"] != b["class_id"] or "visual_aabb" not in OBJ[n]:
                continue
            proj, partial = project_object(n, pose, rig)
            if proj is None:
                continue
            if x0 >= proj[0] - tol_px and y0 >= proj[1] - tol_px and x1 <= proj[2] + tol_px and y1 <= proj[3] + tol_px:
                status = "PARTIAL" if partial else "VERIFIED"
                if not partial:
                    break
        out.append(status)
    return out
