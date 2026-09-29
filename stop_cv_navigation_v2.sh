#!/usr/bin/env bash
# ==============================================================================
# CV Autonomous Navigation V2.1 — Master System Teardown
# Cleanly terminates all Gazebo, ROS 2 nodes, and Dashboard processes.
# ==============================================================================

echo "================================================================="
echo "   STOPPING CV AUTONOMOUS NAVIGATION V2.1 PROCESSES              "
echo "================================================================="

echo "Terminating ROS 2 navigation nodes..."
pkill -9 -f "decision_engine_node" 2>/dev/null || true
pkill -9 -f "dashboard_backend" 2>/dev/null || true
pkill -9 -f "object_detection_node" 2>/dev/null || true
pkill -9 -f "lidar_camera_fusion_node" 2>/dev/null || true
pkill -9 -f "dynamic_obstacles_node" 2>/dev/null || true
pkill -9 -f "navigation_visualizer_node" 2>/dev/null || true
pkill -9 -f "camera_node" 2>/dev/null || true
pkill -9 -f "sign_detection_node" 2>/dev/null || true

echo "Terminating Nav2 stack..."
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "behavior_server" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true

echo "Terminating Bridges & Simulators..."
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby.*gz" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "full_system.launch.py" 2>/dev/null || true

echo "[OK] All CV Autonomous Navigation processes stopped cleanly."
echo "================================================================="
