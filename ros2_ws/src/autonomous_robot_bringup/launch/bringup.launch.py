import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('autonomous_robot_bringup')
    pkg_gazebo = get_package_share_directory('autonomous_robot_gazebo')
    pkg_nav = get_package_share_directory('autonomous_robot_navigation')
    pkg_perception = get_package_share_directory('autonomous_robot_perception')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='complex_world')
    slam = LaunchConfiguration('slam', default='false')
    navigation = LaunchConfiguration('navigation', default='true')
    perception = LaunchConfiguration('perception', default='true')
    rviz = LaunchConfiguration('rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_world = DeclareLaunchArgument('world', default_value='complex_world')
    declare_slam = DeclareLaunchArgument('slam', default_value='false')
    declare_navigation = DeclareLaunchArgument('navigation', default_value='true')
    declare_perception = DeclareLaunchArgument('perception', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    # 1. Gazebo Simulation & Robot Spawning
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time
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
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(navigation)
    )

    # 4. Perception Pipeline (YOLOv8 on GPU + LiDAR-Camera Association)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')
        ),
        condition=IfCondition(perception)
    )

    # 5. RViz2 Visualization
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'cv_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_slam,
        declare_navigation,
        declare_perception,
        declare_rviz,
        gazebo_launch,
        slam_launch,
        localization_launch,
        perception_launch,
        rviz_node
    ])
