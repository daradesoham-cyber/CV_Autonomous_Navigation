#!/usr/bin/env python3
import time
import torch
from ultralytics import YOLO
import numpy as np

def main():
    print("=" * 60)
    print("NVIDIA RTX 3050 INFERENCE & BENCHMARK SUITE")
    print("=" * 60)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"Device: {device}")
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        vram_total = torch.cuda.get_device_properties(0).total_memory / (1024**2)
        print(f"GPU Model: {gpu_name}")
        print(f"Total VRAM: {vram_total:.1f} MB")
        print(f"CUDA Compute Capability: {torch.cuda.get_device_capability(0)}")

    model_path = "/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt"
    print(f"\nLoading model: {model_path}...")
    model = YOLO(model_path)
    model.to(device)

    # Warmup
    print("Warming up GPU with 20 dummy frames (640x480)...")
    dummy = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    for _ in range(20):
        _ = model(dummy, device=device, verbose=False)

    if torch.cuda.is_available():
        torch.cuda.synchronize()

    # Benchmark run
    num_frames = 100
    print(f"Running benchmark over {num_frames} frames...")
    latencies = []

    t_start = time.perf_counter()
    for _ in range(num_frames):
        t0 = time.perf_counter()
        _ = model(dummy, device=device, verbose=False)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        latencies.append((time.perf_counter() - t0) * 1000.0)

    total_time = time.perf_counter() - t_start
    fps = num_frames / total_time
    avg_latency = np.mean(latencies)
    p95_latency = np.percentile(latencies, 95)
    p99_latency = np.percentile(latencies, 99)

    vram_used = torch.cuda.memory_allocated(0) / (1024**2) if torch.cuda.is_available() else 0.0

    print("\n" + "=" * 60)
    print("BENCHMARK RESULTS")
    print("=" * 60)
    print(f"Throughput:       {fps:.1f} FPS")
    print(f"Average Latency:  {avg_latency:.2f} ms")
    print(f"95th Percentile:  {p95_latency:.2f} ms")
    print(f"99th Percentile:  {p99_latency:.2f} ms")
    print(f"VRAM Allocated:   {vram_used:.1f} MB")
    print(f"Real-Time Target: {'PASSED (>30 FPS)' if fps >= 30 else 'FAILED'}")
    print("=" * 60)

if __name__ == '__main__':
    main()
