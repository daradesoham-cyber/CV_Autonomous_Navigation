#!/usr/bin/env python3
"""
V3 inference benchmark on the RTX 3050 (4 GB): per-image latency (ultralytics preprocess /
inference / postprocess), end-to-end wall-clock FPS for single-image calls (how the ROS node calls
the model), and GPU memory (torch peak allocated/reserved + process VRAM from nvidia-smi).

Usage: python3 benchmark_inference.py --models v25=models/yolov8n_v25.pt v3=V3/models_trained/X.pt
Images: V3 canonical test split (real Gazebo renders, 640x480). Warm-up frames are excluded.
Note: run with Gazebo stopped to measure the model alone; the value is also recorded with the
caller-supplied --note describing the GPU load at the time.
"""
import argparse
import glob
import json
import os
import statistics
import subprocess
import time

import torch
from ultralytics import YOLO

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
IMAGES = sorted(glob.glob(os.path.join(V3_ROOT, "datasets", "v3_hospital_cv", "images", "test", "*.jpg")))
OUT = os.path.join(V3_ROOT, "docs", "evidence", "phase2_inference_benchmark.json")


def process_vram_mib():
    try:
        out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True).stdout
        for line in out.strip().splitlines():
            pid, mem = [x.strip() for x in line.split(",")]
            if int(pid) == os.getpid():
                return int(mem)
    except Exception:
        pass
    return None


def bench(path, n, warmup):
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    model = YOLO(path)
    imgs = IMAGES[:n + warmup]
    for p in imgs[:warmup]:
        model.predict(p, imgsz=640, device=0, conf=0.25, verbose=False)
    pre, inf, post, wall = [], [], [], []
    for p in imgs[warmup:]:
        torch.cuda.synchronize()
        t = time.perf_counter()
        r = model.predict(p, imgsz=640, device=0, conf=0.25, verbose=False)[0]
        torch.cuda.synchronize()
        wall.append((time.perf_counter() - t) * 1000)
        pre.append(r.speed["preprocess"])
        inf.append(r.speed["inference"])
        post.append(r.speed["postprocess"])
    q = lambda v, p: sorted(v)[int(p * (len(v) - 1))]
    return {"weights": path, "images": len(wall), "warmup": warmup,
            "latency_ms": {"preprocess_mean": round(statistics.mean(pre), 2), "inference_mean": round(statistics.mean(inf), 2),
                           "postprocess_mean": round(statistics.mean(post), 2), "end_to_end_mean": round(statistics.mean(wall), 2),
                           "end_to_end_p50": round(q(wall, 0.5), 2), "end_to_end_p95": round(q(wall, 0.95), 2)},
            "fps_end_to_end": round(1000 / statistics.mean(wall), 1),
            "torch_peak_allocated_mib": round(torch.cuda.max_memory_allocated() / 2 ** 20, 1),
            "torch_peak_reserved_mib": round(torch.cuda.max_memory_reserved() / 2 ** 20, 1),
            "process_vram_mib_nvidia_smi": process_vram_mib(),
            "params_M": round(sum(p.numel() for p in model.model.parameters()) / 1e6, 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", required=True)
    ap.add_argument("--n", type=int, default=300)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--note", default="")
    a = ap.parse_args()
    res = {"gpu": torch.cuda.get_device_name(0), "torch": torch.__version__, "note": a.note, "results": {}}
    for spec in a.models:
        tag, path = spec.split("=", 1)
        path = path if os.path.isabs(path) else os.path.join(PROJECT_ROOT, path)
        res["results"][tag] = bench(path, a.n, a.warmup)
        print(tag, json.dumps(res["results"][tag]), flush=True)
    json.dump(res, open(OUT, "w"), indent=2)
    print("written", OUT)


if __name__ == "__main__":
    main()
