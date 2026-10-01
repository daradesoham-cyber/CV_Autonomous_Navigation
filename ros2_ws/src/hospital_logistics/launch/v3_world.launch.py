"""
V3 Hospital Logistics world launch.

Same robot, spawn procedure and ros_gz_bridge config as V2.6 (autonomous_robot_gazebo/launch/gazebo.launch.py),
but loads the NEW hospital V3/worlds/v3_hospital_world.sdf (AWS RoboMaker Hospital World floor plan, built by
V3/scripts/build_aws_hospital_world.py) and adds V3/hospital/models + V3/models to GZ_SIM_RESOURCE_PATH.
Spawn: main entrance lobby (V3/scripts/v3_hospital_layout.py SPAWN).
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_V3_ROOT = os.environ.get('V3_ROOT', os.path.expanduser('~/CV_Autonomous_Navigation/V3'))
WORLD_NAME = 'realistic_facility_world'  # kept from V2.6 so the bridge clock topic matches


def launch_setup(context, *args, **kwargs):
    v3_root = LaunchConfiguration('v3_root').perform(context)
    world_path = os.path.join(v3_root, 'worlds', 'v3_hospital_world.sdf')
    if not os.path.isfile(world_path):
        raise RuntimeError(f'V3 world not found: {world_path} (run V3/scripts/build_aws_hospital_world.py)')

    pkg_gazebo = get_package_share_directory('autonomous_robot_gazebo')
    pkg_desc = get_package_share_directory('autonomous_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'

    resource_path = os.pathsep.join(
        p for p in [os.path.join(v3_root, 'hospital', 'models'), os.path.join(v3_root, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"{'-s ' if headless else ''}-r -v 1 {world_path}"}.items())

    robot_description = xacro.process_file(os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')).toxml()
    urdf_path = '/tmp/autonomous_robot_v3.urdf'
    with open(urdf_path, 'w') as f:
        f.write(robot_description)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', WORLD_NAME, '-name', 'autonomous_robot', '-file', urdf_path,
                            '-x', LaunchConfiguration('x_pose').perform(context),
                            '-y', LaunchConfiguration('y_pose').perform(context),
                            '-z', LaunchConfiguration('z_pose').perform(context),
                            '-Y', LaunchConfiguration('yaw_pose').perform(context)])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')}])

    return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path), gz_sim, rsp, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('v3_root', default_value=DEFAULT_V3_ROOT),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        # NEW hospital spawn: main entrance lobby facing south (v3_hospital_layout.SPAWN)
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='13.0'),
        DeclareLaunchArgument('z_pose', default_value='0.1'),
        DeclareLaunchArgument('yaw_pose', default_value='-1.5708'),
        OpaqueFunction(function=launch_setup),
    ])
