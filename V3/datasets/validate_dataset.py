#!/usr/bin/env python3
"""
V3 dataset validation.

1. Label/scene consistency (independent of the boundingbox_camera): for every GT box, project the
   3D visual AABB (8 corners) of every placed object of the same class through the rig pose and the
   pinhole intrinsics (640x480, hfov 1.15). A GT box is VERIFIED when it lies inside such a
   projection (tolerance TOL_PX). Objects with corners behind the camera plane are projected with
   the visible corners and the image border ('PARTIAL' when that is the only match).
   Any UNVERIFIED box is a labelling error and fails the check.
2. Statistics: images per split, instances per class, empty (negative) images, box-size
   distribution, modes, dropped tiny boxes.
3. Audit sheets: random frames with GT boxes (red) and dropped instances (yellow)
   (docs/evidence/phase2_dataset_audit_*.png).

Usage: python3 validate_dataset.py [--root V3/datasets/v3_hospital_cv]
"""
import argparse
import glob
import json
import math
import os
import random
import sys
from collections import Counter, defaultdict

import cv2
import numpy as np

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gt_labels import NAMES, verify_boxes  # noqa: E402

TOL_PX = 8.0


def verify_frame(meta):
    return verify_boxes(meta["boxes"], meta["placed_objects"], meta["rig_pose"], TOL_PX)


def load_split(root, split):
    path = os.path.join(root, "meta", f"{split}.jsonl")
    return [json.loads(line) for line in open(path)] if os.path.exists(path) else []


def audit_sheet(root, metas, out_png, n=15, seed=0):
    rng = random.Random(seed)
    pick = rng.sample(metas, min(n, len(metas)))
    tiles = []
    for m in pick:
        im = cv2.imread(os.path.join(root, m["image"]))
        for b in m["boxes"]:
            x0, y0, x1, y1 = map(int, b["xyxy"])
            cv2.rectangle(im, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2.putText(im, NAMES[b["class_id"]], (x0 + 2, max(12, y0 + 14)), 0, 0.5, (0, 0, 255), 2)
        for b in m.get("dropped_instances", []):  # not labelled (too small / estimated occluded)
            x0, y0, x1, y1 = map(int, b["xyxy"])
            cv2.rectangle(im, (x0, y0), (x1, y1), (0, 220, 255), 1)
            cv2.putText(im, f"drop {NAMES[b['class_id']]}", (x0 + 2, min(478, y1 - 3)), 0, 0.4, (0, 220, 255), 1)
        cv2.putText(im, m.get("mode", ""), (5, 470), 0, 0.6, (255, 0, 0), 2)
        tiles.append(cv2.resize(im, (320, 240)))
    while len(tiles) % 5:
        tiles.append(np.zeros((240, 320, 3), np.uint8))
    rows = [np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)]
    cv2.imwrite(out_png, np.vstack(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(V3_ROOT, "datasets", "v3_hospital_cv"))
    ap.add_argument("--evidence", default=os.path.join(V3_ROOT, "docs", "evidence"))
    a = ap.parse_args()
    report = {"root": a.root, "splits": {}, "subsets": {}}
    fail = 0
    groups = [("splits", s, a.root) for s in ("train", "val", "test")]
    groups += [("subsets", os.path.basename(d), d) for d in sorted(glob.glob(os.path.join(a.root, "subsets", "*")))]
    for kind, name, root in groups:
        metas = load_split(root, "test" if kind == "subsets" else name)
        if not metas:
            continue
        inst, sizes, status = Counter(), defaultdict(list), Counter()
        for m in metas:
            st = verify_frame(m)
            status.update(st)
            for b, s in zip(m["boxes"], st):
                inst[NAMES[b["class_id"]]] += 1
                x0, y0, x1, y1 = b["xyxy"]
                sizes[NAMES[b["class_id"]]].append(math.sqrt((x1 - x0) * (y1 - y0)))
                if s == "UNVERIFIED":
                    print(f"UNVERIFIED {name} {m['image']} {NAMES[b['class_id']]} {b['xyxy']}")
        fail += status["UNVERIFIED"]
        report[kind][name] = {
            "images": len(metas),
            "empty_images": sum(1 for m in metas if not m["boxes"]),
            "instances": dict(inst),
            "modes": dict(Counter(m.get("mode") for m in metas)),
            "dropped_instances": dict(Counter(d["status"] for m in metas for d in m.get("dropped_instances", []))),
            "bbox_camera_mismatches": sum(m.get("bbox_cam_mismatch", 0) for m in metas),
            "target_class_present": dict(Counter(str(m.get("target_class_present")) for m in metas)),
            "box_sqrt_area_px_median": {k: round(float(np.median(v)), 1) for k, v in sizes.items()},
            "label_verification": dict(status),
        }
        os.makedirs(a.evidence, exist_ok=True)
        if kind == "splits":
            audit_sheet(root, metas, os.path.join(a.evidence, f"phase2_dataset_audit_{name}.png"))
        else:
            audit_sheet(root, metas, os.path.join(a.evidence, f"phase2_subset_audit_{name}.png"), n=10)
        print(f"{name:28s} imgs={len(metas):5d} empty={report[kind][name]['empty_images']:4d} "
              f"verify={dict(status)} inst={dict(inst)}")
    out = os.path.join(a.root, "dataset_validation.json")
    json.dump(report, open(out, "w"), indent=2)
    print("report:", out, "| UNVERIFIED boxes:", fail)
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
