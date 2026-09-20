# Experimental Results & Performance Verification

## Test 1: Real GPU Benchmark (100 Frames)
* **Throughput**: 178.0 FPS
* **Average Latency**: 5.62 ms
* **95th Percentile**: 7.24 ms
* **VRAM Footprint**: 36.2 MB
* **Result**: PASSED (Exceeds 30 FPS real-time robotics requirement by 5.9x).

## Test 2: Sensor Fusion Distance Estimation
* **Ground Truth Distance**: 2.100 m
* **Estimated Distance**: 2.085 m
* **Absolute Error**: 0.015 m (1.5 cm error)
* **Bearing**: 1.95 degrees
* **Result**: PASSED.

## Test 3: Simulation Launch & Topics
* Gazebo Sim 10.5.0 launched headlessly and verified.
* Sensor bridges confirmed: `/camera/image_raw`, `/camera/camera_info`, `/scan`, `/odom`, `/tf`, `/clock`, `/cmd_vel`.
