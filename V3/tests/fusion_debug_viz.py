#!/usr/bin/env python3
"""
V3 fusion debug visualization from recorded corpus frames (real Gazebo renders + real LiDAR scans).

For each selected frame:
  left  - camera image: YOLO boxes (class, confidence), ALL LiDAR returns projected through the
          calibration (grey), candidate returns inside the box (yellow), accepted V3 hypothesis
          (green), returns rejected by the vertical-consistency test (red), V3 and V2.6 range text
  right - top-down (robot frame, x forward up): camera FOV, LiDAR returns, V3 fused positions
          (green), V2.6 fused positions (red), ground-truth object footprints (blue, evaluation only)
Output: V3/docs/evidence/phase3_debug/<set>_<scene>_<index>.png
Usage: python3 fusion_debug_viz.py --set controlled --scenes 04_bed_ivstand 12_partial_occlusion --max 4
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(V3_ROOT, "perception"))
sys.path.insert(0, os.path.join(V3_ROOT, "tests"))
import lidar_fusion as old  # noqa: E402
import spatial_fusion as new  # noqa: E402
from eval_fusion import CORPUS, footprint, gt  # noqa: E402

OUT = os.path.join(V3_ROOT, "docs", "evidence", "phase3_debug")
TOP = 480  # px
SCALE = 55.0  # px per metre in the top-down view


def to_top(x, y):
    return int(TOP / 2 - y * SCALE), int(TOP - 20 - x * SCALE)


def world_to_base(robot, wx, wy):
    rx, ry, yaw = robot
    dx, dy = wx - rx, wy - ry
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


def draw(rec, set_name):
    img = cv2.imread(os.path.join(CORPUS, set_name, rec["image"]))
    s = rec["scan"]
    sp = new.ScanPoints.from_scan(s["ranges"], s["angle_min"], s["angle_step"], s["range_min"], s["range_max"])
    top = np.full((TOP, TOP, 3), 255, np.uint8)
    # camera FOV wedge
    for sgn in (-1, 1):
        a = sgn * 0.575
        cv2.line(top, to_top(0.22, 0), to_top(0.22 + 8 * math.cos(a), 8 * math.sin(a)), (200, 200, 200), 1)
    # all LiDAR returns (top) and their projections (image)
    for ang, r in zip(sp.angles, sp.ranges):
        cv2.circle(top, to_top(0.10 + r * math.cos(ang), r * math.sin(ang)), 1, (120, 120, 120), -1)
    for u, v in zip(sp.u, sp.v):
        if 0 <= u < 640 and 0 <= v < 480:
            cv2.circle(img, (int(u), int(v)), 1, (150, 150, 150), -1)
    # ground truth footprints (evaluation only)
    for n, p in rec["placed"].items():
        if n in gt.OBJ and gt.OBJ[n].get("footprint") and gt.OBJ[n]["class_id"] is not None:
            pts = [to_top(*world_to_base(rec["robot_pose"], x, y)) for x, y in footprint(n, p)]
            cv2.polylines(top, [np.array(pts, np.int32)], True, (220, 120, 0), 2)
    cv2.circle(top, to_top(0, 0), 6, (0, 0, 0), -1)
    ranges = [math.inf if r is None else r for r in s["ranges"]]
    y_txt = 16
    for d in rec["detections"]:
        if d["conf"] < 0.35:
            continue
        box = d["xyxy"]
        x0, y0, x1, y1 = map(int, box)
        a = new.associate(box, d["class"], sp, with_debug=True)
        o = old.fuse_box(box[0], box[2], ranges, s["angle_min"], s["angle_step"], s["range_min"], s["range_max"])
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 0), 2)
        for i in a.get("_cand", []):
            cv2.circle(img, (int(sp.u[i]), int(sp.v[i])), 3, (0, 220, 255), -1)
        for i in a.get("_rejected_vertical", []):
            if 0 <= sp.v[i] < 480:
                cv2.circle(img, (int(sp.u[i]), int(sp.v[i])), 3, (0, 0, 255), -1)
        for i in a.get("_selected", []):
            cv2.circle(img, (int(sp.u[i]), int(sp.v[i])), 4, (0, 200, 0), -1)
            cv2.circle(top, to_top(0.10 + sp.ranges[i] * math.cos(sp.angles[i]), sp.ranges[i] * math.sin(sp.angles[i])), 3, (0, 200, 0), -1)
        v3txt = (f"{a['range']:.2f}m ({a['range_source']}, c={a['association_confidence']:.2f})"
                 if a["range"] is not None else a["range_source"])
        v26txt = f"{o['distance']:.2f}m" if o["distance"] > 0 else "none"
        cv2.putText(img, f"{d['class']} {d['conf']:.2f}", (x0 + 2, max(14, y0 + 14)), 0, 0.45, (255, 0, 0), 2)
        cv2.putText(img, f"V3 {v3txt} | V2.6 {v26txt}", (x0 + 2, min(476, y1 - 4)), 0, 0.4, (0, 120, 0), 1)
        if a["x"] is not None:
            cv2.drawMarker(top, to_top(a["x"], a["y"]), (0, 170, 0), cv2.MARKER_TILTED_CROSS, 14, 3)
            cv2.putText(top, f"{d['class']} ({a['x']:.2f},{a['y']:.2f})", (5, y_txt), 0, 0.4, (0, 140, 0), 1)
            y_txt += 14
        if o["distance"] > 0:
            cv2.drawMarker(top, to_top(*o["base_xy"]), (0, 0, 220), cv2.MARKER_CROSS, 12, 2)
    cv2.putText(top, "V3 x | V2.6 + | GT footprint", (5, TOP - 5), 0, 0.4, (0, 0, 0), 1)
    return np.hstack([img, top])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", required=True)
    ap.add_argument("--scenes", nargs="*")
    ap.add_argument("--frames", nargs="*", type=int)
    ap.add_argument("--max", type=int, default=3)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    recs = [json.loads(l) for l in open(os.path.join(CORPUS, a.set, "frames.jsonl"))]
    done = {}
    for r in recs:
        if a.scenes and r["scene"] not in a.scenes:
            continue
        if a.frames is not None and r["index"] not in a.frames:
            continue
        if done.get(r["scene"], 0) >= a.max:
            continue
        done[r["scene"]] = done.get(r["scene"], 0) + 1
        path = os.path.join(OUT, f"{a.set}_{r['scene']}_{r['index']:04d}.png")
        cv2.imwrite(path, draw(r, a.set))
        print(path)


if __name__ == "__main__":
    main()
