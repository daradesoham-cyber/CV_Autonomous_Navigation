import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('autonomous_robot_navigation')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_arg = LaunchConfiguration('map', default='realistic_facility_map.yaml')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='realistic_facility_map.yaml',
        description='Map YAML file in maps/ directory or absolute path'
    )
    declare_initial_x = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    declare_initial_y = DeclareLaunchArgument('initial_pose_y', default_value='-11.0')
    declare_initial_yaw = DeclareLaunchArgument('initial_pose_yaw', default_value='1.57')

    initial_pose_x = LaunchConfiguration('initial_pose_x', default='0.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='-11.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='1.57')

    # Resolve map path
    map_file = PathJoinSubstitution([pkg_nav, 'maps', map_arg])

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params,
            {
                'use_sim_time': use_sim_time,
                'initial_pose.x': initial_pose_x,
                'initial_pose.y': initial_pose_y,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': initial_pose_yaw,
            }
        ]
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 10.0
        }]
    )

    initial_pose_publisher_node = Node(
        package='autonomous_robot_navigation',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        map_server_node,
        amcl_node,
        lifecycle_mgr,
        initial_pose_publisher_node
    ])
