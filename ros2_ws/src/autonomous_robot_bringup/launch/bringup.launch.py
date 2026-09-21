import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_bringup = get_package_share_directory('autonomous_robot_bringup')
    pkg_gazebo = get_package_share_directory('autonomous_robot_gazebo')
    pkg_nav = get_package_share_directory('autonomous_robot_navigation')
    pkg_perception = get_package_share_directory('autonomous_robot_perception')

    world_str = context.perform_substitution(LaunchConfiguration('world')).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    navigation = LaunchConfiguration('navigation')
    perception = LaunchConfiguration('perception')
    rviz = LaunchConfiguration('rviz')
    model_path = LaunchConfiguration('model_path')
    use_custom_controller = LaunchConfiguration('use_custom_controller')
    headless = LaunchConfiguration('headless')

    # Resolve world and map
    if world_str in ['realistic', 'realistic_facility', 'realistic_facility_world']:
        resolved_world = 'realistic'
        resolved_map = 'realistic_facility_map.yaml'
        init_x = '0.0'
        init_y = '-11.0'
        init_yaw = '1.57'
    else:
        resolved_world = 'complex'
        resolved_map = 'complex_map.yaml'
        init_x = '0.0'
        init_y = '-8.0'
        init_yaw = '1.57'

    # 1. Gazebo Simulation & Robot Spawning
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': resolved_world,
            'use_sim_time': use_sim_time,
            'headless': headless
        }.items()
    )

    # 2. SLAM Toolbox (optional mapping)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(slam)
    )

    # 3. Localization & Map Server
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': resolved_map,
            'initial_pose_x': init_x,
            'initial_pose_y': init_y,
            'initial_pose_yaw': init_yaw
        }.items(),
        condition=IfCondition(navigation)
    )

    # 4. Nav2 Navigation Stack (Planner, Controller, Behaviors, BT Navigator, Decision Engine, Visualizer)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'initial_x': init_x,
            'initial_y': init_y,
            'initial_yaw': init_yaw
        }.items(),
        condition=IfCondition(navigation)
    )

    # 5. Perception Pipeline (Custom YOLOv8 on GPU + LiDAR-Camera Association + Signs)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model_path': model_path,
            'use_custom_controller': use_custom_controller
        }.items(),
        condition=IfCondition(perception)
    )

    # 6. RViz2 Visualization
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'cv_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return [
        gazebo_launch,
        slam_launch,
        localization_launch,
        navigation_launch,
        perception_launch,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='realistic',
            description='World to load: realistic (realistic_facility_world) or complex (complex_world)'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('navigation', default_value='true'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/soham-darade/CV_Autonomous_Navigation/models/custom_yolov8n/weights/best.pt',
            description='Path to custom YOLOv8 model weights'
        ),
        DeclareLaunchArgument(
            'use_custom_controller',
            default_value='false',
            description='Enable experimental custom navigation controller (disabled by default)'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo Sim in headless mode (server only, no GUI)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
