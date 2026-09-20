# GPU Setup & Optimization Guide

## Hardware
* **GPU**: NVIDIA GeForce RTX 3050 Laptop GPU (GA107M, 4096 MiB VRAM)
* **Driver Version**: 595.84 (Proprietary NVIDIA Linux Driver)
* **CUDA Version**: 13.2 (Driver API) / 13.0 (PyTorch CUDA Runtime)

## Important Principles
1. **VS Code is an editor**: It runs on the CPU and does not consume GPU VRAM.
2. **PyTorch Tensor Allocation**: Data must be transferred to the GPU via `device="cuda:0"`.
3. **Memory Optimization**: With 4 GB VRAM, batch sizes for real-time inference should remain at 1, and models should be lightweight (YOLOv8n) to leave ample headroom for graphical rendering and simulation.
