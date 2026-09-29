#!/usr/bin/env python3
"""
Full System Launch File for Autonomous Navigation V2.
Brings up:
- Realistic Gazebo Sim Warehouse Environment (realistic_facility_world.sdf)
- Dynamic Obstacles Patrol Trajectories (cart, trolley, forklift, person)
- ros_gz_bridge (LaserScan, Cameras, Odometry, cmd_vel)
- Nav2 Navigation Stack & AMCL Localization
- Perception Pipeline (YOLOv8 + LiDAR-Camera Fusion + Sign Recognition)
- Decision Engine V2 (Multi-criteria Route Cost & Candidate Selection)
- Topological Visualizer & RViz2
- Dedicated Web Navigation Dashboard (http://127.0.0.1:5050)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('autonomous_robot_bringup')
    bringup_launch_path = os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='realistic')
    rviz = LaunchConfiguration('rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'slam': 'false',
            'navigation': 'true',
            'perception': 'true',
            'model_path': '/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n_v24.pt',
            'rviz': rviz,
            'headless': headless
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('world', default_value='realistic', description='World environment (realistic or complex)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2 visualization'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless without GUI'),
        bringup_launch
    ])
