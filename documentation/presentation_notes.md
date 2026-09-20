# University Project Presentation Notes

## Key Highlights
1. **Target Environment**: ROS 2 Lyrical on Ubuntu 26.04 LTS (Resolute Raccoon) with native Gazebo Sim 10.5.0.
2. **Main Innovation**: Seamless combination of 2D LiDAR range data with real-time deep learning semantic detections from an RGB camera.
3. **Hardware Acceleration**: Zero-copy CUDA inference on NVIDIA RTX 3050 achieving 178 FPS with only 36 MB VRAM.
4. **Autonomous Navigation**: Nav2 integration with tuned Regulated Pure Pursuit controller and dynamic obstacle avoidance.
5. **Complex World**: 25m x 25m environment with 3 distinct operational zones (Zone A: Plaza, Zone B: Warehouse, Zone C: Office/Hospital).
