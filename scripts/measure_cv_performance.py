#!/usr/bin/env python3
import os
import time
import csv
import subprocess
import psutil
import torch
from ultralytics import YOLO
import numpy as np

def get_gpu_metrics():
    try:
        cmd = "nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader,nounits"
        out = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()
        gpu_util, mem_used, mem_total = [float(x.strip()) for x in out.split(',')]
        return gpu_util, mem_used, mem_total
    except Exception as e:
        print(f"Error querying nvidia-smi: {e}")
        return 0.0, 0.0, 0.0

def main():
    print("=" * 70)
    print("LIVE CV PIPELINE PERFORMANCE & GPU MEASUREMENT")
    print("=" * 70)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"Compute Device: {device}")
    if torch.cuda.is_available():
        print(f"GPU Model: {torch.cuda.get_device_name(0)}")

    model_path = "/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt"
    print(f"Loading YOLOv8n from {model_path}...")
    model = YOLO(model_path)
    model.to(device)

    # Verify model parameters on CUDA
    param_device = next(model.model.parameters()).device
    assert str(param_device) == 'cuda:0', f"Expected cuda:0, got {param_device}"
    print(f"Model parameters verified on: {param_device}")

    # Simulated camera frame stream at 640x480 (typical Gazebo camera output)
    num_frames = 120
    test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Warmup
    print("Warming up pipeline with 15 frames...")
    for _ in range(15):
        _ = model(test_img, device=device, verbose=False)
    if torch.cuda.is_available():
        torch.cuda.synchronize()

    print(f"Profiling {num_frames} frames under active GPU execution...")
    latencies = []
    gpu_utils = []
    gpu_mems = []
    cpu_utils = []

    # Initial CPU measurement
    psutil.cpu_percent(interval=None)

    t_start = time.perf_counter()
    for i in range(num_frames):
        t0 = time.perf_counter()
        _ = model(test_img, device=device, verbose=False)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        dt = (time.perf_counter() - t0) * 1000.0
        latencies.append(dt)

        if i % 20 == 0:
            g_util, g_mem, _ = get_gpu_metrics()
            c_util = psutil.cpu_percent(interval=None)
            gpu_utils.append(g_util)
            gpu_mems.append(g_mem)
            cpu_utils.append(c_util)

    total_time = time.perf_counter() - t_start
    inference_fps = num_frames / total_time
    camera_fps = 30.0  # Gazebo configured camera rate
    avg_latency = float(np.mean(latencies))
    p95_latency = float(np.percentile(latencies, 95))
    avg_gpu_util = float(np.mean(gpu_utils)) if gpu_utils else 0.0
    avg_gpu_mem = float(np.mean(gpu_mems)) if gpu_mems else 0.0
    avg_cpu_util = float(np.mean(cpu_utils)) if cpu_utils else 0.0

    print("\n" + "=" * 70)
    print("MEASURED PERFORMANCE METRICS")
    print("=" * 70)
    print(f"Camera Stream Target FPS:  {camera_fps:.1f} FPS")
    print(f"YOLO Inference FPS:        {inference_fps:.1f} FPS")
    print(f"Average YOLO Latency:      {avg_latency:.2f} ms")
    print(f"95th Percentile Latency:   {p95_latency:.2f} ms")
    print(f"GPU VRAM Usage:            {avg_gpu_mem:.1f} MiB")
    print(f"GPU Utilization:           {avg_gpu_util:.1f}%")
    print(f"CPU Utilization:           {avg_cpu_util:.1f}%")
    print("=" * 70)

    # Save to results/cv_performance.csv
    csv_path = "/home/soham-darade/CV_Autonomous_Navigation/results/cv_performance.csv"
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "metric", "value", "unit"
        ])
        writer.writerow(["camera_fps", f"{camera_fps:.1f}", "FPS"])
        writer.writerow(["yolo_inference_fps", f"{inference_fps:.1f}", "FPS"])
        writer.writerow(["yolo_avg_latency_ms", f"{avg_latency:.2f}", "ms"])
        writer.writerow(["yolo_p95_latency_ms", f"{p95_latency:.2f}", "ms"])
        writer.writerow(["gpu_vram_used_mib", f"{avg_gpu_mem:.1f}", "MiB"])
        writer.writerow(["gpu_utilization_pct", f"{avg_gpu_util:.1f}", "%"])
        writer.writerow(["cpu_utilization_pct", f"{avg_cpu_util:.1f}", "%"])

    print(f"Performance metrics successfully saved to {csv_path}")

if __name__ == '__main__':
    main()
