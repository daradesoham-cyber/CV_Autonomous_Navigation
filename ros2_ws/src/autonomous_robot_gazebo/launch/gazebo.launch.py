import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('autonomous_robot_gazebo')
    pkg_desc = get_package_share_directory('autonomous_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_str = context.perform_substitution(LaunchConfiguration('world')).strip()
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve world name and default spawn coordinates
    if world_str in ['realistic', 'realistic_facility', 'realistic_facility_world']:
        actual_world_file = 'realistic_facility_world.sdf'
        default_x = '0.0'
        default_y = '-11.0'
        default_z = '0.1'
        default_yaw = '1.57'
    else:
        actual_world_file = 'complex_world.sdf'
        default_x = '0.0'
        default_y = '-8.0'
        default_z = '0.1'
        default_yaw = '1.57'

    x_arg = context.perform_substitution(LaunchConfiguration('x_pose')).strip()
    y_arg = context.perform_substitution(LaunchConfiguration('y_pose')).strip()
    z_arg = context.perform_substitution(LaunchConfiguration('z_pose')).strip()
    yaw_arg = context.perform_substitution(LaunchConfiguration('yaw_pose')).strip()

    spawn_x = x_arg if x_arg else default_x
    spawn_y = y_arg if y_arg else default_y
    spawn_z = z_arg if z_arg else default_z
    spawn_yaw = yaw_arg if yaw_arg else default_yaw

    world_path = os.path.join(pkg_gazebo, 'worlds', actual_world_file)
    actual_world_name = 'realistic_facility_world' if world_str in ['realistic', 'realistic_facility', 'realistic_facility_world'] else 'complex_world'

    headless_str = context.perform_substitution(LaunchConfiguration('headless')).strip().lower()
    gz_extra = "-s " if headless_str == 'true' else ""

    # Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"{gz_extra}-r -v 1 {world_path}"}.items()
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

    # Save URDF to temporary file for reliable entity spawning in Gazebo Sim
    urdf_path = '/tmp/autonomous_robot.urdf'
    with open(urdf_path, 'w') as f:
        f.write(robot_description)

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', actual_world_name,
            '-name', 'autonomous_robot',
            '-file', urdf_path,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
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

    return [gz_sim, rsp_node, spawn_robot, bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='realistic',
            description='World to load: realistic (realistic_facility_world) or complex (complex_world)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='',
            description='Robot spawn X coordinate (auto-configured based on world if empty)'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='',
            description='Robot spawn Y coordinate (auto-configured based on world if empty)'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.1',
            description='Robot spawn Z coordinate'
        ),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='',
            description='Robot spawn Yaw angle (auto-configured based on world if empty)'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo Sim in headless mode (server only, no GUI)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
