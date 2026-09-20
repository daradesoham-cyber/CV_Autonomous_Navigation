# Installation & Setup Guide

## 1. Prerequisites
Ensure Ubuntu 26.04 LTS has the official ROS 2 Lyrical repositories configured.

Install required ROS and system development packages:
```bash
sudo apt update
sudo apt install -y \
  ros-lyrical-ros-gz \
  ros-lyrical-ros-gz-bridge \
  ros-lyrical-ros-gz-sim \
  ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-amcl \
  ros-lyrical-nav2-map-server \
  ros-lyrical-nav2-lifecycle-manager \
  ros-lyrical-nav2-planner \
  ros-lyrical-nav2-controller \
  ros-lyrical-nav2-costmap-2d \
  ros-lyrical-cv-bridge \
  ros-lyrical-image-transport
```

## 2. Python AI Virtual Environment (RTX 3050 CUDA)
The project utilizes a dedicated virtual environment with PyTorch 2.14.0+cu130 and Ultralytics YOLO:
```bash
# Virtual environment is located at:
~/CV_Autonomous_Navigation/.venv

# Verify GPU availability:
~/CV_Autonomous_Navigation/.venv/bin/python -c "import torch; print('CUDA:', torch.cuda.is_available(), 'GPU:', torch.cuda.get_device_name(0))"
```

## 3. Building the Workspace
```bash
cd ~/CV_Autonomous_Navigation/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
```

## 4. Sourcing the Workspace
```bash
source ~/CV_Autonomous_Navigation/ros2_ws/install/setup.bash
```
