#!/usr/bin/env python3
"""
CV Autonomous Navigation V2.5 — YOLO Model Training & Systematic Evaluation Script.
Supports training and validating candidate architectures (YOLOv8n and YOLOv8s)
on the balanced V2.5 dataset on NVIDIA RTX 3050 GPU (device=0).
Generates detailed metrics, confusion matrix analysis, failure-case breakdown,
and comparison against V2.4 baseline.
"""

import os
import sys
import time
import shutil
import json
import argparse
import torch
import numpy as np
from ultralytics import YOLO

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
DATA_YAML = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset/data.yaml")
MODELS_DIR = os.path.join(PROJECT_ROOT, "models")
V24_MODEL_PATH = os.path.join(MODELS_DIR, "yolov8n_v24.pt")

CLASSES = [
    'person', 'cart', 'forklift', 'pallet', 'box',
    'obstacle', 'door', 'charging_station', 'hospital_bed', 'directional_sign'
]


def train_model(arch="yolov8n", epochs=30, batch=16, imgsz=640, run_name=None):
    if run_name is None:
        run_name = f"{arch}_v25"

    print("=" * 70)
    print(f"TRAINING {arch.upper()} (RUN: {run_name})")
    print(f"Epochs: {epochs} | Batch: {batch} | ImgSz: {imgsz} | Device: cuda:0")
    print("=" * 70)

    # Base weights
    base_weights = f"{arch}.pt"
    print(f"Loading pretrained backbone: {base_weights}")
    model = YOLO(base_weights)

    t0 = time.time()
    results = model.train(
        data=DATA_YAML,
        epochs=epochs,
        batch=batch,
        imgsz=imgsz,
        device=0,
        project=os.path.join(PROJECT_ROOT, "runs/train"),
        name=run_name,
        exist_ok=True,
        workers=2,
        optimizer="auto",
        verbose=True,
        seed=42,
        plots=True
    )
    train_duration = time.time() - t0
    print(f"Training completed in {train_duration:.2f} seconds.")

    best_weights = os.path.join(PROJECT_ROOT, f"runs/train/{run_name}/weights/best.pt")
    if not os.path.exists(best_weights):
        best_weights = os.path.join(PROJECT_ROOT, f"runs/train/{run_name}/weights/last.pt")
    print(f"Best weights: {best_weights}")
    return best_weights, train_duration


def evaluate_model_full(model_path, data_yaml=DATA_YAML, split="val", imgsz=640):
    """Evaluates model and returns overall metrics, per-class metrics, confusion matrix, and speed."""
    model = YOLO(model_path)
    res = model.val(data=data_yaml, split=split, device=0, imgsz=imgsz, conf=0.25, plots=True)

    cm = res.confusion_matrix.matrix
    names = list(model.names.values())
    p = res.box.p
    r = res.box.r
    ap50 = res.box.ap50
    ap = res.box.ap

    per_class = {}
    for i, name in enumerate(names):
        pi = float(p[i]) if i < len(p) else 0.0
        ri = float(r[i]) if i < len(r) else 0.0
        a50 = float(ap50[i]) if i < len(ap50) else 0.0
        ai = float(ap[i]) if i < len(ap) else 0.0
        per_class[name] = {
            'precision': pi,
            'recall': ri,
            'ap50': a50,
            'ap50_95': ai
        }

    # Background false positives: background is index 10 (last row of cm)
    # cm[10, j] is ground-truth background predicted as class j
    bg_fps = {}
    total_bg_fps = 0
    if cm.shape[0] > len(names):
        bg_row = cm[len(names), :]
        for j, name in enumerate(names):
            cnt = int(bg_row[j])
            bg_fps[name] = cnt
            total_bg_fps += cnt

    speed_info = res.speed  # preprocess, inference, loss, postprocess (ms)

    metrics = {
        'split': split,
        'mAP50': float(res.box.map50),
        'mAP50_95': float(res.box.map),
        'precision': float(res.box.mp),
        'recall': float(res.box.mr),
        'per_class': per_class,
        'bg_false_positives': bg_fps,
        'total_bg_false_positives': total_bg_fps,
        'speed_ms': speed_info,
        'inference_fps': 1000.0 / max(0.1, speed_info.get('inference', 5.0))
    }
    return metrics, cm


def evaluate_diagnostic_subsets(model_path):
    """Evaluates model against diagnostic failure-case subsets."""
    model = YOLO(model_path)
    subsets_dir = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset/eval_subsets")
    subset_names = sorted(os.listdir(subsets_dir))

    subset_results = {}
    print("\n" + "=" * 60)
    print("DIAGNOSTIC FAILURE-CASE SUBSET EVALUATION:")
    print("=" * 60)
    for sname in subset_names:
        s_yaml = os.path.join(subsets_dir, sname, "data.yaml")
        if not os.path.exists(s_yaml):
            continue
        try:
            res = model.val(data=s_yaml, split="val", device=0, imgsz=640, conf=0.25, plots=False)
            subset_results[sname] = {
                'mAP50': float(res.box.map50),
                'mAP50_95': float(res.box.map),
                'precision': float(res.box.mp),
                'recall': float(res.box.mr),
            }
            print(f"Subset [{sname:<18}]: mAP50={res.box.map50:.4f}, Prec={res.box.mp:.4f}, Rec={res.box.mr:.4f}")
        except Exception as e:
            print(f"Subset [{sname}]: Error: {e}")
    return subset_results


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--arch', type=str, default='yolov8n', choices=['yolov8n', 'yolov8s'])
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch', type=int, default=16)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--name', type=str, default=None)
    args = parser.parse_args()

    best_w, train_time = train_model(
        arch=args.arch,
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        run_name=args.name
    )

    val_metrics, cm = evaluate_model_full(best_w, split="val")
    test_metrics, _ = evaluate_model_full(best_w, split="test")
    subsets = evaluate_diagnostic_subsets(best_w)

    print("\nVAL METRICS SUMMARY:")
    print(f"  mAP50:      {val_metrics['mAP50']:.4f}")
    print(f"  mAP50-95:   {val_metrics['mAP50_95']:.4f}")
    print(f"  Precision:  {val_metrics['precision']:.4f}")
    print(f"  Recall:     {val_metrics['recall']:.4f}")
    print(f"  Inference:  {val_metrics['speed_ms']['inference']:.2f} ms ({val_metrics['inference_fps']:.1f} FPS)")
    print(f"  Background False Positives: {val_metrics['total_bg_false_positives']}")

    run_id = args.name if args.name else f"{args.arch}_v25"
    summary_path = os.path.join(PROJECT_ROOT, f"{run_id}_metrics.json")
    record = {
        'model_name': run_id,
        'arch': args.arch,
        'epochs': args.epochs,
        'batch_size': args.batch,
        'imgsz': args.imgsz,
        'train_time_sec': train_time,
        'best_weights': best_w,
        'val_metrics': val_metrics,
        'test_metrics': test_metrics,
        'subsets': subsets
    }
    with open(summary_path, 'w') as f:
        json.dump(record, f, indent=2)
    print(f"\nFull evaluation metrics saved to: {summary_path}")
