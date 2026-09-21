#!/usr/bin/env bash
set -e

# Setup ROS 2 Lyrical & Workspace environment
source /opt/ros/lyrical/setup.bash
source /home/soham-darade/CV_Autonomous_Navigation/ros2_ws/install/setup.bash

# Ensure Python AI venv is in PATH
export PATH="/home/soham-darade/CV_Autonomous_Navigation/.venv/bin:$PATH"

# Clean up any stale simulation / ROS 2 processes to ensure exactly one stack runs
cleanup_stale_processes() {
    pkill -f "gz sim" > /dev/null 2>&1 || true
    pkill -f "parameter_bridge" > /dev/null 2>&1 || true
    pkill -f "rviz2" > /dev/null 2>&1 || true
    pkill -f "nav2_" > /dev/null 2>&1 || true
    pkill -f "autonomous_robot_" > /dev/null 2>&1 || true
    pkill -f "navigation_" > /dev/null 2>&1 || true
    sleep 1
}

trap cleanup_stale_processes EXIT INT TERM
cleanup_stale_processes

WORLD=${1:-complex_world}
SLAM=${2:-false}

echo "=========================================================="
echo "LAUNCHING AUTONOMOUS CV NAVIGATION SYSTEM"
echo "World: $WORLD"
echo "SLAM Mapping Mode: $SLAM"
echo "Perception on GPU: NVIDIA RTX 3050"
echo "=========================================================="

ros2 launch autonomous_robot_bringup bringup.launch.py \
    world:=$WORLD \
    slam:=$SLAM \
    navigation:=true \
    perception:=true \
    rviz:=true \
    use_sim_time:=true
