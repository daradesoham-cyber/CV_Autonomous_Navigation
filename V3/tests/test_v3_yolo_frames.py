#!/usr/bin/env python3
"""
V3 Phase 1 preliminary CV observation (NOT an accuracy measurement).

Runs the validated V2.5 YOLO model (config/perception_v25.yaml -> models/yolov8n_v25.pt) on the
camera frames captured by test_v3_world_sensors.py, using the same global confidence threshold as
the V2.6 detection node. Records which classes fire on which viewpoint and saves annotated frames.
No ground-truth labels exist for these frames yet, so no precision/recall is claimed.
Evidence: V3/docs/evidence/phase1_yolo/*.png and phase1_yolo_observation.json
"""
import glob
import json
import os
import sys

import yaml
from ultralytics import YOLO

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
FRAMES = os.path.join(V3_ROOT, "docs", "evidence", "phase1_sensors")
OUT_DIR = os.path.join(V3_ROOT, "docs", "evidence", "phase1_yolo")


def main():
    cfg = yaml.safe_load(open(os.path.join(PROJECT_ROOT, "config", "perception_v25.yaml")))["yolo"]
    conf = float(cfg.get("global_confidence_threshold", 0.35))
    model = YOLO(os.path.join(PROJECT_ROOT, cfg["model_path"]))
    os.makedirs(OUT_DIR, exist_ok=True)
    rows = []
    for path in sorted(glob.glob(os.path.join(FRAMES, "*.png"))):
        r = model.predict(path, conf=conf, device=cfg.get("device", "cuda:0"), verbose=False)[0]
        dets = [{"class": r.names[int(b.cls)], "conf": round(float(b.conf), 3),
                 "xyxy": [round(float(v), 1) for v in b.xyxy[0]]} for b in r.boxes]
        r.save(filename=os.path.join(OUT_DIR, os.path.basename(path)))
        rows.append({"frame": os.path.basename(path), "detections": dets})
        print(f"{os.path.basename(path):30s} {[(d['class'], d['conf']) for d in dets]}")
    json.dump({"model": cfg["model_path"], "classes": model.names, "conf_threshold": conf, "frames": rows},
              open(os.path.join(V3_ROOT, "docs", "evidence", "phase1_yolo_observation.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
