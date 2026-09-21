import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('autonomous_robot_navigation')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically activate lifecycle nodes'
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_mgr_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='-11.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='1.57')

    declare_initial_x = DeclareLaunchArgument('initial_x', default_value='0.0')
    declare_initial_y = DeclareLaunchArgument('initial_y', default_value='-11.0')
    declare_initial_yaw = DeclareLaunchArgument('initial_yaw', default_value='1.57')

    decision_engine_node = Node(
        package='autonomous_robot_navigation',
        executable='decision_engine_node',
        name='decision_engine_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'db_path': os.path.join(pkg_nav, 'config', 'navigation_memory.db'),
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw
        }]
    )

    navigation_visualizer_node = Node(
        package='autonomous_robot_navigation',
        executable='navigation_visualizer_node',
        name='navigation_visualizer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'db_path': os.path.join(pkg_nav, 'config', 'navigation_memory.db')
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_mgr_nav,
        decision_engine_node,
        navigation_visualizer_node
    ])
