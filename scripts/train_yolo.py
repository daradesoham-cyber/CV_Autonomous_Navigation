#!/usr/bin/env python3
"""
Train Custom YOLOv8n on Gazebo Simulation Dataset using NVIDIA RTX 3050 CUDA.
"""
import os
import sys
import torch
from ultralytics import YOLO

def main():
    print("=" * 70)
    print("STARTING CUSTOM YOLOV8N FINE-TUNING ON GAZEBO SIMULATION DATASET")
    print("=" * 70)

    # 1. Device check
    if not torch.cuda.is_available():
        print("[ERROR] CUDA is not available. Training requires GPU.")
        sys.exit(1)

    device_name = torch.cuda.get_device_name(0)
    total_mem = torch.cuda.get_device_properties(0).total_memory / (1024 ** 2)
    print(f"[GPU] Using device: cuda:0 ({device_name})")
    print(f"[GPU] Total VRAM: {total_mem:.1f} MB")

    data_yaml = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_cv/data.yaml"
    base_model = "/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt"
    project_dir = "/home/soham-darade/CV_Autonomous_Navigation/models"
    run_name = "custom_yolov8n"

    if not os.path.exists(data_yaml):
        print(f"[ERROR] Dataset configuration not found: {data_yaml}")
        sys.exit(1)
    if not os.path.exists(base_model):
        print(f"[ERROR] Pretrained weights not found: {base_model}")
        sys.exit(1)

    print(f"[INFO] Base weights: {base_model}")
    print(f"[INFO] Dataset config: {data_yaml}")
    print(f"[INFO] Target project directory: {project_dir}/{run_name}")

    # 2. Load model
    model = YOLO(base_model)

    # 3. Train model
    print("\n[TRAINING] Commencing fine-tuning for 15 epochs...")
    results = model.train(
        data=data_yaml,
        epochs=15,
        imgsz=640,
        batch=16,
        device=0,
        project=project_dir,
        name=run_name,
        exist_ok=True,
        workers=2,
        optimizer='AdamW',
        lr0=0.001,
        verbose=True
    )

    best_weights = os.path.join(project_dir, run_name, "weights", "best.pt")
    if os.path.exists(best_weights):
        print("\n" + "=" * 70)
        print(f"[SUCCESS] Custom YOLOv8n fine-tuning complete!")
        print(f"[SUCCESS] Best weights saved at: {best_weights}")
        print("=" * 70)
    else:
        print(f"[WARNING] Expected weights at {best_weights} but file not found.")

if __name__ == '__main__':
    main()
