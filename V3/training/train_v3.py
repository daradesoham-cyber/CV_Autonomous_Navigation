#!/usr/bin/env python3
"""
V3 YOLOv8 fine-tuning on the Gazebo-rendered V3 hospital dataset.

Baseline pipeline = V2.5 (scripts/train_yolov8_v25.py): ultralytics YOLOv8n, imgsz 640, batch 16,
default ultralytics augmentation, seed 42, device 0. Differences, all explicit here:
  - data: V3/datasets/v3_hospital_cv (9 classes, V3/perception/v3_classes.yaml)
  - init: the validated V2.5 weights models/yolov8n_v25.pt (default) or COCO yolov8n.pt
          (--init coco, ablation); the detection head is re-initialised for 9 classes
  - epochs 60 with patience 15 (V2.5 used 30 fixed epochs)
Output: V3/training/runs/<name>/ (ultralytics run) and V3/models_trained/<name>.pt (best weights)

Usage: python3 train_v3.py --name v3_yolov8n_from_v25
       python3 train_v3.py --name v3_yolov8n_from_coco --init coco
"""
import argparse
import json
import os
import shutil
import time

import yaml
from ultralytics import YOLO

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
DATA_ROOT = os.path.join(V3_ROOT, "datasets", "v3_hospital_cv")
DATA_YAML = os.path.join(DATA_ROOT, "data.yaml")
OUT_MODELS = os.path.join(V3_ROOT, "models_trained")
INITS = {"v25": os.path.join(PROJECT_ROOT, "models", "yolov8n_v25.pt"),
         "coco": os.path.join(PROJECT_ROOT, "yolov8n.pt")}


def write_data_yaml():
    classes = yaml.safe_load(open(os.path.join(V3_ROOT, "perception", "v3_classes.yaml")))["classes"]
    data = {"path": DATA_ROOT, "train": "images/train", "val": "images/val", "test": "images/test",
            "nc": len(classes), "names": {c["id"]: c["name"] for c in classes}}
    with open(DATA_YAML, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--init", choices=list(INITS), default="v25")
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--patience", type=int, default=15)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()

    data = write_data_yaml()
    model = YOLO(INITS[a.init])
    t0 = time.time()
    model.train(data=DATA_YAML, epochs=a.epochs, patience=a.patience, batch=a.batch, imgsz=a.imgsz,
                device=0, project=os.path.join(V3_ROOT, "training", "runs"), name=a.name, exist_ok=True,
                workers=4, optimizer="auto", seed=42, deterministic=True, plots=True, verbose=True)
    dur = time.time() - t0
    run_dir = os.path.join(V3_ROOT, "training", "runs", a.name)
    best = os.path.join(run_dir, "weights", "best.pt")
    os.makedirs(OUT_MODELS, exist_ok=True)
    dst = os.path.join(OUT_MODELS, f"{a.name}.pt")
    shutil.copy2(best, dst)
    summary = {"name": a.name, "init": INITS[a.init], "data": data, "epochs_requested": a.epochs,
               "patience": a.patience, "batch": a.batch, "imgsz": a.imgsz, "train_seconds": round(dur, 1),
               "best_weights": dst}
    json.dump(summary, open(os.path.join(run_dir, "v3_train_summary.json"), "w"), indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
