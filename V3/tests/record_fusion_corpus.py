#!/usr/bin/env python3
"""
Record a camera-LiDAR fusion corpus from the V3 dataset world (Gazebo renders + rig gpu_lidar).

Each frame stores what a runtime fusion algorithm would see (RGB image, raw LaserScan, V3 YOLO
detections) plus Gazebo ground truth used ONLY for evaluation (instance boxes with object identity,
poses of every placed object, rig/robot pose). Fusion algorithms are then compared offline on
identical data (V3/tests/eval_fusion.py).

Sets:
  eval       seed 505: exactly the Phase 2 baseline frames (same scene calls as test_v3_fusion.py)
  dev        seed 606: same distribution, disjoint; used to design/tune the new association
  controlled 12 controlled scene types x 5 distances x 2 layouts in the warehouse main aisle
Camera pitch: exactly the robot calibration (-0.05 rad) unless --pitch-offset is given. (The first
corpora recorded for Phase 3, like the Phase 2 fusion test, used a constant +0.0138 rad offset by
accident; they are kept as the calibration-offset robustness sets *_pitchoffset.)
Output: V3/datasets/fusion_corpus/<set>/{frames.jsonl, images/*.jpg}

Usage (dataset world running; ROS sourced; venv site-packages on PYTHONPATH):
  python3 record_fusion_corpus.py --set eval
"""
import argparse
import json
import math
import os
import random
import sys
import time

import cv2

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, os.path.join(V3_ROOT, "datasets"))
import generate_dataset as g  # noqa: E402
import gt_labels as gt  # noqa: E402

OUT_ROOT = os.path.join(V3_ROOT, "datasets", "fusion_corpus")
DISTANCES = [1.5, 2.5, 3.5, 4.5, 5.5]
AISLE_X, CAM_Y0 = 8.3, 1.2  # warehouse main aisle (x 6.8-9.8, y 1-11.5), robot faces +y


def pool(sc, model, k=0):
    names = [n for n in g.MOVABLE if g.OBJ[n].get("model") == model and sc.poses[n][2] == g.PARK_Z]
    return names[k] if k < len(names) else None


def put(sc, name, x, y, yaw):
    base_z = g.OBJ[name]["canonical_pose"][2] if name.startswith("dynamic_") else 0.0
    sc.poses[name] = (x, y, base_z, 0.0, 0.0, yaw)


CONTROLLED = {
    # scene -> list of (model or dynamic name, dx lateral, dd extra depth, yaw)
    "01_bed_alone": [("MalePatientBed", 0.0, 0.0, 0.0)],
    "02_bed_trolley": [("TrolleyBed", 0.3, 0.0, 0.0), ("SurgicalTrolley", -0.9, -0.3, 1.57)],
    "03_bed_wheelchair": [("MalePatientBed", 0.4, 0.2, 0.0), ("PatientWheelChair", -0.8, -0.4, 2.5)],
    "04_bed_ivstand": [("TrolleyBed", 0.2, 0.0, 0.0), ("IVStand", -0.7, -0.2, 0.0)],
    "05_cart_chair": [("InstrumentCart1", 0.4, 0.0, 0.3), ("Chair", -0.4, -0.1, 1.2)],
    "06_trolley_forklift": [("SurgicalTrolley", -0.5, -0.3, 0.2), ("dynamic_forklift", 0.6, 0.5, 1.57)],
    "07_person_near_bed": [("MalePatientBed", 0.5, 0.2, 0.0), ("Scrubs", -0.6, -0.2, -1.57)],
    "08_person_behind_bed": [("TrolleyBed", 0.0, 0.0, 0.0), ("FemaleVisitor", 0.3, 1.4, -1.57)],
    "09_multi_depth": [("Scrubs", -0.6, -0.8, -1.57), ("BPCart", 0.5, 0.2, 0.0), ("TrolleyBed", -0.2, 1.8, 0.0)],
    "10_ivstand_distance": [("IVStand", 0.0, 0.0, 0.0)],
    "11_wheelchair_distance": [("PatientWheelChair", 0.0, 0.0, 1.0)],
    "12_partial_occlusion": [("SurgicalTrolley", 0.1, -0.9, 1.57), ("MalePatientBed", 0.0, 0.3, 0.0)],
}


def controlled_scene(rng, key, i):
    """Scene `key` at distance DISTANCES[i % 5], layout variant i // 5 (mirrored lateral offsets)."""
    sc = g.Scene(rng, canonical=False)
    d = DISTANCES[i % len(DISTANCES)]
    mirror = -1.0 if (i // len(DISTANCES)) % 2 else 1.0
    used = {}
    for model, dx, dd, yaw in CONTROLLED[key]:
        if model.startswith("dynamic_"):
            name = model
        else:
            k = used.get(model, 0)
            name = pool(sc, model, k)
            used[model] = k + 1
        put(sc, name, AISLE_X + mirror * dx, CAM_Y0 + d + dd, yaw + (0.0 if mirror > 0 else math.pi))
    return sc, (AISLE_X, CAM_Y0 - g.CAM_FWD, math.pi / 2)


class FixedPitch:
    """rng stand-in for generate_dataset.render(): its only draw is the camera pitch jitter
    uniform(-0.02, 0.02); this returns a fixed, explicit offset instead."""

    def __init__(self, offset):
        self.offset = offset

    def uniform(self, a, b):
        return self.offset


def scan_to_dict(scan):
    return {"angle_min": scan.angle_min, "angle_step": scan.angle_step, "range_min": scan.range_min,
            "range_max": scan.range_max, "ranges": [round(float(r), 4) if math.isfinite(r) else None for r in scan.ranges]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", required=True, choices=["eval", "dev", "controlled"])
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--conf", type=float, default=0.25, help="detections stored at >= this confidence")
    ap.add_argument("--pitch-offset", type=float, default=0.0,
                    help="camera pitch offset from the robot calibration (rad); 0 = exact robot camera pitch")
    ap.add_argument("--name", default=None, help="output set name (default: --set)")
    ap.add_argument("--model", default=os.path.join(V3_ROOT, "models_trained", "v3_yolov8n_from_v25.pt"))
    a = ap.parse_args()
    from ultralytics import YOLO
    model = YOLO(a.model)
    calib = gt.load_calibration()
    out = os.path.join(OUT_ROOT, a.name or a.set)
    os.makedirs(os.path.join(out, "images"), exist_ok=True)
    gz = g.GzScene(with_scan=True)
    t0 = time.time()
    while gz.sim_time < 1.0 and time.time() - t0 < 60:
        time.sleep(0.2)
    seed = {"eval": 505, "dev": 606, "controlled": 707}[a.set]
    if a.set == "controlled":
        jobs = [(k, i) for k in CONTROLLED for i in range(2 * len(DISTANCES))]
    else:
        jobs = [(a.set, i) for i in range(a.frames)]
    with open(os.path.join(out, "frames.jsonl"), "w") as fo:
        for n, (key, i) in enumerate(jobs):
            rng = random.Random()
            if a.set == "controlled":
                rng.seed(f"{seed}-{key}-{i}")
                sc, robot = controlled_scene(rng, key, i)
            else:  # identical scene construction to V3/tests/test_v3_fusion.py (Phase 2 baseline)
                rng.seed(f"{seed}-{i}")
                sc, robot = g.scene_canonical(rng, {}, 10 + i) if i % 2 == 0 else g.scene_random(rng, {})
            light = g.lighting(rng, "default")
            r = g.render(gz, sc, robot, light, FixedPitch(a.pitch_offset), calib)
            if r is None:
                print("skip unverifiable frame", key, i, flush=True)
                continue
            rgb, boxes, seg, stamp, rig = r
            scan = gz.last_scan
            placed = {k: [round(v, 4) for v in p] for k, p in sc.poses.items() if p[2] != g.PARK_Z}
            inst = [gt.decide(x, calib) for x in gt.assign_objects(gt.extract_instances(seg), placed, rig)]
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            t_det = time.perf_counter()
            res = model.predict(bgr, conf=a.conf, imgsz=640, device=0, verbose=False)[0]
            det_ms = (time.perf_counter() - t_det) * 1000
            img_name = f"{key}_{i:04d}.jpg"
            cv2.imwrite(os.path.join(out, "images", img_name), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
            rec = {"set": a.set, "scene": key, "index": i, "sim_stamp": stamp, "image": f"images/{img_name}",
                   "robot_pose": [round(v, 4) for v in robot], "rig_pose": [round(v, 4) for v in rig],
                   "scan": scan_to_dict(scan), "scan_stamp": scan.header.stamp.sec + scan.header.stamp.nsec * 1e-9,
                   "yolo_ms": round(det_ms, 2),
                   "detections": [{"class": model.names[int(c)], "conf": round(float(s), 4), "xyxy": [round(float(v), 1) for v in b]}
                                  for c, s, b in zip(res.boxes.cls.tolist(), res.boxes.conf.tolist(), res.boxes.xyxy.tolist())],
                   "gt": [{"class": gt.NAMES[x["class_id"]], "xyxy": x["xyxy"], "object": x.get("object"),
                           "status": x["status"], "visible_px": x["visible_px"]} for x in inst],
                   "placed": placed}
            fo.write(json.dumps(rec) + "\n")
            if n % 25 == 0:
                print(f"{a.set} {n}/{len(jobs)} {time.time() - t0:.0f}s", flush=True)
    print("DONE", a.set, len(jobs), f"{time.time() - t0:.0f}s", flush=True)
    gz.close()
    os._exit(0)


if __name__ == "__main__":
    main()
