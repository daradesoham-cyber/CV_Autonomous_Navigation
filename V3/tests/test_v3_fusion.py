#!/usr/bin/env python3
"""
V3 camera-LiDAR fusion validation against Gazebo ground truth (dataset world with rig LiDAR).

For each rendered frame (canonical V3 layout and randomized scenes):
  1. detections: V3 YOLO model on the RGB frame (conf >= --conf), matched to Gazebo instance GT
     (same class, IoU >= 0.5) to know which scene object each detection is
  2. fusion: V2.6 association (V3/perception/lidar_fusion.py) on the synchronized /v3ds/scan
  3. truth: range from the LiDAR origin to the object's collision footprint (rotated rectangle),
     i.e. its nearest surface in the LiDAR plane; position error in base_link
The same fusion is also run on the GT boxes ("oracle boxes") to separate detection errors from
fusion errors. Objects whose visual geometry does not cross the LiDAR plane (z = 0.235 m) are
reported as not measurable, not as failures.
Outcome per association: OK (|range error| <= 0.30 m vs. the footprint's nearest face),
WRONG_SURFACE (a cluster was found but its range differs by > 0.30 m), NO_CLUSTER (no cluster).
Because the 2D LiDAR passes between legs/wheels of sparse objects (Phase 1), a second, association
criterion is reported: ON_OBJECT = the fused point lies within 0.25 m of the object's footprint
polygon (the LiDAR cluster belongs to the object), otherwise OFF_OBJECT (background / occluder).
Evidence: V3/docs/evidence/phase2_fusion/fusion_validation.json + example images.

Usage (dataset world running): python3 test_v3_fusion.py --model V3/models_trained/X.pt
"""
import argparse
import json
import math
import os
import random
import statistics
import sys
import time
from collections import Counter, defaultdict

import cv2
import numpy as np

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, os.path.join(V3_ROOT, "datasets"))
sys.path.insert(0, os.path.join(V3_ROOT, "perception"))
import generate_dataset as g  # noqa: E402
import gt_labels as gt  # noqa: E402
from lidar_fusion import LIDAR_X_IN_BASE, fuse_box  # noqa: E402

LIDAR_Z = 0.235
OUT = os.path.join(V3_ROOT, "docs", "evidence", "phase2_fusion")
OK_TOL = 0.30


def footprint_corners(name, pose):
    (ax, ay), (bx, by) = g.OBJ[name]["footprint"]
    c, s = math.cos(pose[5]), math.sin(pose[5])
    return [(pose[0] + c * px - s * py, pose[1] + s * px + c * py) for px, py in ((ax, ay), (bx, ay), (bx, by), (ax, by))]


def dist_point_polygon(px, py, poly):
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > py) != (y1 > py) and px < (x1 - x0) * (py - y0) / (y1 - y0) + x0:
            inside = not inside
    if inside:
        return 0.0, (px, py)
    best = (1e9, None)
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x1 - x0, y1 - y0
        t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
        qx, qy = x0 + t * dx, y0 + t * dy
        d = math.hypot(px - qx, py - qy)
        if d < best[0]:
            best = (d, (qx, qy))
    return best


def in_lidar_plane(name):
    mn, mx = g.OBJ[name]["visual_aabb"]
    z0 = g.OBJ[name]["canonical_pose"][2] if name.startswith(("dynamic_", "v3_sign", "sign_")) else 0.0
    return z0 + mn[2] <= LIDAR_Z <= z0 + mx[2]


def world_to_base(rx, ry, yaw, wx, wy):
    dx, dy = wx - rx, wy - ry
    return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)


def evaluate_assoc(box, name, pose, robot, scan):
    rx, ry, yaw = robot
    lx, ly = rx + LIDAR_X_IN_BASE * math.cos(yaw), ry + LIDAR_X_IN_BASE * math.sin(yaw)
    f = fuse_box(box[0], box[2], scan.ranges, scan.angle_min, scan.angle_step, scan.range_min, scan.range_max)
    rec = {"object": name, "model": g.OBJ[name].get("model", name), "class": gt.NAMES[g.OBJ[name]["class_id"]],
           "box": [round(v, 1) for v in box], "fused_range_m": round(f["distance"], 3), "n_valid_beams": f["n_valid"]}
    if not in_lidar_plane(name) or name not in g.OBJ or not g.OBJ[name].get("footprint"):
        rec["outcome"] = "NOT_IN_LIDAR_PLANE"
        return rec
    true_d, near = dist_point_polygon(lx, ly, footprint_corners(name, pose))
    rec["true_range_m"] = round(true_d, 3)
    if f["distance"] <= 0:
        rec["outcome"] = "NO_CLUSTER"
        return rec
    err = f["distance"] - true_d
    tb = world_to_base(rx, ry, yaw, *near)
    rec["range_error_m"] = round(err, 3)
    rec["fused_base_xy"] = [round(v, 3) for v in f["base_xy"]]
    rec["true_nearest_base_xy"] = [round(v, 3) for v in tb]
    rec["position_error_m"] = round(math.hypot(f["base_xy"][0] - tb[0], f["base_xy"][1] - tb[1]), 3)
    rec["outcome"] = "OK" if abs(err) <= OK_TOL else "WRONG_SURFACE"
    fx = lx + f["distance"] * math.cos(yaw + f["bearing"])
    fy = ly + f["distance"] * math.sin(yaw + f["bearing"])
    d_obj, _ = dist_point_polygon(fx, fy, footprint_corners(name, pose))
    rec["fused_point_to_object_m"] = round(d_obj, 3)
    rec["association"] = "ON_OBJECT" if d_obj <= 0.25 else "OFF_OBJECT"
    return rec


def summarize(recs):
    out = {"n": len(recs), "outcomes": dict(Counter(r["outcome"] for r in recs))}
    meas = [r for r in recs if r["outcome"] != "NOT_IN_LIDAR_PLANE"]
    ok = [r for r in meas if r["outcome"] == "OK"]
    out["measurable"] = len(meas)
    out["ok_rate_of_measurable"] = round(len(ok) / len(meas), 3) if meas else None
    if ok:
        errs = [abs(r["range_error_m"]) for r in ok]
        pos = [r["position_error_m"] for r in ok]
        out["ok_abs_range_error_m"] = {"mean": round(statistics.mean(errs), 3), "median": round(statistics.median(errs), 3),
                                       "p95": round(sorted(errs)[int(0.95 * (len(errs) - 1))], 3)}
        out["ok_position_error_m"] = {"mean": round(statistics.mean(pos), 3), "p95": round(sorted(pos)[int(0.95 * (len(pos) - 1))], 3)}
    assoc = [r for r in meas if "association" in r]
    out["association"] = dict(Counter(r["association"] for r in assoc))
    out["on_object_rate_of_measurable"] = round(sum(r["association"] == "ON_OBJECT" for r in assoc) / len(meas), 3) if meas else None
    on = [r for r in assoc if r["association"] == "ON_OBJECT"]
    if on:
        e = [r["range_error_m"] for r in on]
        out["on_object_range_error_m"] = {"mean": round(statistics.mean(e), 3), "median": round(statistics.median(e), 3)}
    by_cls = defaultdict(list)
    for r in recs:
        by_cls[r["class"]].append(r)
    out["per_class"] = {c: dict(Counter(r["outcome"] for r in v)) for c, v in by_cls.items()}
    out["per_class_association"] = {c: dict(Counter(r.get("association", r["outcome"]) for r in v)) for c, v in by_cls.items()}
    bins = defaultdict(list)
    for r in meas:
        if "true_range_m" in r:
            bins[f"{int(min(r['true_range_m'], 6.99))}-{int(min(r['true_range_m'], 6.99)) + 1}m"].append(r)
    out["ok_rate_by_true_range"] = {k: round(sum(x["outcome"] == "OK" for x in v) / len(v), 3) for k, v in sorted(bins.items())}
    out["on_object_rate_by_true_range"] = {k: round(sum(x.get("association") == "ON_OBJECT" for x in v) / len(v), 3)
                                           for k, v in sorted(bins.items())}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--seed", type=int, default=505)
    a = ap.parse_args()
    from ultralytics import YOLO
    model = YOLO(a.model if os.path.isabs(a.model) else os.path.join(PROJECT_ROOT, a.model))
    os.makedirs(OUT, exist_ok=True)
    calib = gt.load_calibration()
    gz = g.GzScene(with_scan=True)
    t0 = time.time()
    while gz.sim_time < 1.0 and time.time() - t0 < 60:
        time.sleep(0.2)
    rng = random.Random(a.seed)
    det_recs, oracle_recs, unmatched_dets = [], [], 0
    examples = []
    for i in range(a.frames):
        rng.seed(f"{a.seed}-{i}")
        sc, robot = g.scene_canonical(rng, {}, 10 + i) if i % 2 == 0 else g.scene_random(rng, {})
        light = g.lighting(rng, "default")
        rgb, boxes, seg, stamp, rig = g.render(gz, sc, robot, light, random.Random(0))
        scan = gz.last_scan
        placed = {n: p for n, p in sc.poses.items() if p[2] != g.PARK_Z}
        inst = [gt.decide(x, calib) for x in gt.assign_objects(gt.extract_instances(seg), placed, rig)]
        kept = [x for x in inst if x["status"] == "kept" and x.get("object")]
        for x in kept:
            pose = placed.get(x["object"], gt.FIXED_LABELLED.get(x["object"]))
            oracle_recs.append(evaluate_assoc(x["xyxy"], x["object"], pose, robot, scan))
        res = model.predict(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), conf=a.conf, imgsz=640, device=0, verbose=False)[0]
        frame_recs = []
        for c, s, b in zip(res.boxes.cls.tolist(), res.boxes.conf.tolist(), res.boxes.xyxy.tolist()):
            name = model.names[int(c)]
            match = max(((gt.iou(b, x["xyxy"]), x) for x in kept if gt.NAMES[x["class_id"]] == name),
                        default=(0, None), key=lambda t: t[0])
            if match[0] < 0.5:
                unmatched_dets += 1
                continue
            x = match[1]
            pose = placed.get(x["object"], gt.FIXED_LABELLED.get(x["object"]))
            r = evaluate_assoc(b, x["object"], pose, robot, scan)
            r["conf"] = round(s, 3)
            det_recs.append(r)
            frame_recs.append(r)
        if frame_recs and len(examples) < 12 and any(r["outcome"] != "NOT_IN_LIDAR_PLANE" for r in frame_recs):
            im = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for r in frame_recs:
                x0, y0, x1, y1 = map(int, r["box"])
                col = (0, 200, 0) if r["outcome"] == "OK" else (0, 0, 255) if r["outcome"] != "NOT_IN_LIDAR_PLANE" else (200, 200, 0)
                cv2.rectangle(im, (x0, y0), (x1, y1), col, 2)
                txt = f"{r['class']} fused {r['fused_range_m']:.2f}m"
                if "true_range_m" in r:
                    txt += f" / true {r['true_range_m']:.2f}m"
                cv2.putText(im, txt, (x0 + 2, max(14, y0 - 4)), 0, 0.45, col, 2)
            examples.append(cv2.resize(im, (480, 360)))
        if i % 50 == 0:
            print(f"{i}/{a.frames} det_assoc={len(det_recs)} oracle={len(oracle_recs)} {time.time() - t0:.0f}s", flush=True)
    while len(examples) % 3:
        examples.append(np.zeros((360, 480, 3), np.uint8))
    if examples:
        cv2.imwrite(os.path.join(OUT, "fusion_examples.png"),
                    np.vstack([np.hstack(examples[k:k + 3]) for k in range(0, len(examples), 3)]))
    report = {"model": a.model, "frames": a.frames, "conf": a.conf, "ok_tolerance_m": OK_TOL,
              "detections_not_matched_to_gt": unmatched_dets,
              "detected_objects": summarize(det_recs), "oracle_gt_boxes": summarize(oracle_recs),
              "records_detected": det_recs}
    json.dump(report, open(os.path.join(OUT, "fusion_validation.json"), "w"), indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != "records_detected"}, indent=2), flush=True)
    gz.close()
    os._exit(0)


if __name__ == "__main__":
    main()
