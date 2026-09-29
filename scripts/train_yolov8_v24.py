#!/usr/bin/env python3
"""
CV Autonomous Navigation V2.4 — YOLOv8n Retraining & Evaluation Script.
Trains YOLOv8n on the newly generated current-facility dataset (10 classes)
using NVIDIA RTX 3050 GPU (device=0), performs validation, evaluates on unseen test set,
and exports the best model to models/yolov8n_v24.pt.
"""

import os
import sys
import shutil
import json
import torch
from ultralytics import YOLO

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
DATA_YAML = os.path.join(PROJECT_ROOT, "datasets/v24_cv_dataset/data.yaml")
MODELS_DIR = os.path.join(PROJECT_ROOT, "models")
V24_MODEL_PATH = os.path.join(MODELS_DIR, "yolov8n_v24.pt")
METRICS_JSON_PATH = os.path.join(PROJECT_ROOT, "v24_training_metrics.json")


def main():
    print("================================================================")
    print("CV Autonomous Navigation V2.4 — Retraining YOLOv8n on RTX 3050")
    print("================================================================")

    # 1. Verify CUDA
    cuda_avail = torch.cuda.is_available()
    print(f"CUDA Available: {cuda_avail}")
    if not cuda_avail:
        print("ERROR: CUDA device not detected! Exiting.")
        sys.exit(1)
    
    device_name = torch.cuda.get_device_name(0)
    print(f"GPU Device: {device_name}")
    print(f"Dataset config: {DATA_YAML}")

    # 2. Initialize YOLOv8n
    # Using yolov8n.pt pretrained backbone
    base_model = "yolov8n.pt"
    print(f"Loading base architecture: {base_model}")
    model = YOLO(base_model)

    # 3. Train on GPU
    epochs = 25
    batch_size = 16
    imgsz = 640
    print(f"Starting training: epochs={epochs}, batch={batch_size}, imgsz={imgsz}, device=0...")

    results = model.train(
        data=DATA_YAML,
        epochs=epochs,
        batch=batch_size,
        imgsz=imgsz,
        device=0,
        project=os.path.join(PROJECT_ROOT, "runs/train"),
        name="yolov8n_v24",
        exist_ok=True,
        workers=2,
        optimizer="auto",
        verbose=True,
        seed=42
    )

    # 4. Locate best model weights
    best_weights = os.path.join(PROJECT_ROOT, "runs/train/yolov8n_v24/weights/best.pt")
    if not os.path.exists(best_weights):
        best_weights = os.path.join(PROJECT_ROOT, "runs/train/yolov8n_v24/weights/last.pt")
    print(f"\nTraining completed! Best weights located at: {best_weights}")

    # Copy to models/yolov8n_v24.pt
    shutil.copyfile(best_weights, V24_MODEL_PATH)
    print(f"[OK] Exported V2.4 model to: {V24_MODEL_PATH}")

    # Also update models/yolov8n.pt so downstream nodes default to this V2.4 model
    # (old model is safely preserved in models/yolov8n_old.pt)
    shutil.copyfile(best_weights, os.path.join(MODELS_DIR, "yolov8n.pt"))
    print(f"[OK] Updated primary model models/yolov8n.pt (old preserved in yolov8n_old.pt)")

    # 5. Evaluate on Validation Set
    print("\n--- Evaluating Model on Validation Set ---")
    val_model = YOLO(V24_MODEL_PATH)
    val_results = val_model.val(data=DATA_YAML, split="val", device=0, imgsz=imgsz)

    # 6. Evaluate on Unseen Test Set
    print("\n--- Evaluating Model on Unseen Test Set ---")
    test_results = val_model.val(data=DATA_YAML, split="test", device=0, imgsz=imgsz)

    # Collect Metrics
    metrics_summary = {
        "model_name": "yolov8n_v24",
        "device": device_name,
        "epochs": epochs,
        "batch_size": batch_size,
        "imgsz": imgsz,
        "val_metrics": {
            "mAP50": float(val_results.box.map50),
            "mAP50_95": float(val_results.box.map),
            "precision": float(val_results.box.mp),
            "recall": float(val_results.box.mr),
        },
        "test_metrics": {
            "mAP50": float(test_results.box.map50),
            "mAP50_95": float(test_results.box.map),
            "precision": float(test_results.box.mp),
            "recall": float(test_results.box.mr),
        },
        "per_class_test": {}
    }

    # Per-class metrics
    class_names = val_model.names
    for idx, name in class_names.items():
        if idx < len(test_results.box.maps):
            metrics_summary["per_class_test"][name] = {
                "mAP50": float(test_results.box.maps[idx])
            }

    with open(METRICS_JSON_PATH, "w") as f:
        json.dump(metrics_summary, f, indent=2)
    print(f"\nSaved metrics summary to: {METRICS_JSON_PATH}")

    print("\n================================================================")
    print("V2.4 MODEL PERFORMANCE SUMMARY:")
    print(f"Device: {device_name}")
    print(f"Val mAP@50:      {metrics_summary['val_metrics']['mAP50']:.4f}")
    print(f"Val mAP@50-95:   {metrics_summary['val_metrics']['mAP50_95']:.4f}")
    print(f"Val Precision:   {metrics_summary['val_metrics']['precision']:.4f}")
    print(f"Val Recall:      {metrics_summary['val_metrics']['recall']:.4f}")
    print("----------------------------------------------------------------")
    print(f"Test mAP@50:     {metrics_summary['test_metrics']['mAP50']:.4f}")
    print(f"Test mAP@50-95:  {metrics_summary['test_metrics']['mAP50_95']:.4f}")
    print(f"Test Precision:  {metrics_summary['test_metrics']['precision']:.4f}")
    print(f"Test Recall:     {metrics_summary['test_metrics']['recall']:.4f}")
    print("================================================================")


if __name__ == '__main__':
    main()
