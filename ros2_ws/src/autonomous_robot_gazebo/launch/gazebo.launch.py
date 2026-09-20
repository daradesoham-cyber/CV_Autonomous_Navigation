import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('autonomous_robot_gazebo')
    pkg_desc = get_package_share_directory('autonomous_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='complex_world',
        description='World to load: complex_world or maze_world'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    world_name = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_path = PathJoinSubstitution([
        pkg_gazebo, 'worlds', [world_name, '.sdf']
    ])

    # Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -v 1 ', world_path]}.items()
    )

    # Robot State Publisher
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'autonomous_robot',
            '-topic', 'robot_description',
            '-x', '-10.0',
            '-y', '-10.0',
            '-z', '0.1'
        ]
    )

    # ROS-GZ Bridge Node
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gz_sim,
        rsp_node,
        spawn_robot,
        bridge_node
    ])
