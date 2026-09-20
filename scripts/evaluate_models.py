#!/usr/bin/env python3
"""
Evaluate and Compare Pretrained YOLOv8n vs Custom Fine-Tuned YOLOv8n on Gazebo Test Split.
Measures Precision, Recall, mAP50, mAP50-95, Inference Latency, FPS, and GPU VRAM usage.
Saves comparison table to results/model_comparison.csv.
"""
import os
import sys
import time
import csv
import torch
import numpy as np
from ultralytics import YOLO

def benchmark_inference(model, sample_img_path, num_warmup=20, num_runs=100):
    """Accurately measure GPU inference latency, FPS, and peak VRAM."""
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()

    # Warmup
    for _ in range(num_warmup):
        _ = model.predict(sample_img_path, device=0, verbose=False)

    latencies = []
    starter = torch.cuda.Event(enable_timing=True)
    ender = torch.cuda.Event(enable_timing=True)

    for _ in range(num_runs):
        starter.record()
        _ = model.predict(sample_img_path, device=0, verbose=False)
        ender.record()
        torch.cuda.synchronize()
        latencies.append(starter.elapsed_time(ender))

    avg_latency = float(np.mean(latencies))
    fps = 1000.0 / avg_latency if avg_latency > 0 else 0.0
    peak_vram = torch.cuda.max_memory_allocated(0) / (1024 ** 2)

    return avg_latency, fps, peak_vram

def main():
    print("=" * 75)
    print("OBJECT DETECTION MODEL EVALUATION & COMPARISON")
    print("=" * 75)

    if not torch.cuda.is_available():
        print("[ERROR] CUDA is required for GPU benchmarking.")
        sys.exit(1)

    device_name = torch.cuda.get_device_name(0)
    print(f"[HARDWARE] GPU: {device_name} (cuda:0)")

    data_yaml = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_cv/data.yaml"
    pretrained_weights = "/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt"
    custom_weights = "/home/soham-darade/CV_Autonomous_Navigation/models/custom_yolov8n/weights/best.pt"
    results_csv = "/home/soham-darade/CV_Autonomous_Navigation/results/model_comparison.csv"
    os.makedirs(os.path.dirname(results_csv), exist_ok=True)

    test_img_dir = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_cv/images/test"
    test_images = [os.path.join(test_img_dir, f) for f in os.listdir(test_img_dir) if f.endswith(('.jpg', '.png'))]
    if not test_images:
        print("[ERROR] No test images found in test split!")
        sys.exit(1)
    sample_img = test_images[0]

    models_to_eval = [
        ("Pretrained YOLOv8n (COCO)", pretrained_weights),
        ("Custom YOLOv8n (Gazebo Fine-Tuned)", custom_weights)
    ]

    records = []

    for label, weights_path in models_to_eval:
        print(f"\n--- Evaluating: {label} ---")
        if not os.path.exists(weights_path):
            print(f"[ERROR] Weights file not found: {weights_path}")
            continue

        model = YOLO(weights_path)

        # 1. Run validation on test split
        print(f"Running validation on test split ({data_yaml})...")
        try:
            val_results = model.val(data=data_yaml, split='test', device=0, verbose=False)
            precision = float(val_results.results_dict.get('metrics/precision(B)', 0.0))
            recall = float(val_results.results_dict.get('metrics/recall(B)', 0.0))
            map50 = float(val_results.results_dict.get('metrics/mAP50(B)', 0.0))
            map50_95 = float(val_results.results_dict.get('metrics/mAP50-95(B)', 0.0))
        except Exception as e:
            print(f"[WARN] Validation on custom dataset failed (expected if class count mismatch for COCO): {e}")
            precision, recall, map50, map50_95 = 0.0, 0.0, 0.0, 0.0

        # 2. Benchmark inference latency & FPS
        print(f"Benchmarking inference latency and FPS on {device_name}...")
        latency_ms, fps, vram_mb = benchmark_inference(model, sample_img)

        print(f"Results for {label}:")
        print(f"  Precision:   {precision:.4f}")
        print(f"  Recall:      {recall:.4f}")
        print(f"  mAP@0.50:    {map50:.4f}")
        print(f"  mAP@0.50-0.95:{map50_95:.4f}")
        print(f"  Latency:     {latency_ms:.2f} ms")
        print(f"  Throughput:  {fps:.1f} FPS")
        print(f"  Peak VRAM:   {vram_mb:.1f} MB")

        records.append({
            'Model': label,
            'Weights': os.path.basename(weights_path),
            'Precision': f"{precision:.4f}",
            'Recall': f"{recall:.4f}",
            'mAP50': f"{map50:.4f}",
            'mAP50-95': f"{map50_95:.4f}",
            'Latency_ms': f"{latency_ms:.2f}",
            'FPS': f"{fps:.1f}",
            'Peak_VRAM_MB': f"{vram_mb:.1f}"
        })

    # Save to CSV
    with open(results_csv, 'w', newline='') as f:
        fieldnames = ['Model', 'Weights', 'Precision', 'Recall', 'mAP50', 'mAP50-95', 'Latency_ms', 'FPS', 'Peak_VRAM_MB']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in records:
            writer.writerow(r)

    print("\n" + "=" * 75)
    print(f"Comparison saved to: {results_csv}")
    print("=" * 75)

if __name__ == '__main__':
    main()
