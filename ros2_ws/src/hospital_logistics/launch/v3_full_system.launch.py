"""
V3 Hospital Logistics - complete system launch.

  1. V3 world (Gazebo Sim, robot, ros_gz_bridge)             hospital_logistics/v3_world.launch.py
  2. Localization (V2.6 nodes) on the NEW map with V3/config/v3_nav2_params.yaml
  3. Nav2 servers - same nodes as V2.6 navigation.launch.py, V3 copy of its parameters (V3/config/v3_nav2_params.yaml)
  4. V2.6 decision engine (topological routing, signs, replans, TTC yield) on the V3 memory DB, run via the
     V3 entry point v3_decision_engine (adds the missing ReplanReason.OBSTACLE_BLOCKED; engine code unchanged)
  5. Traffic (OFF unless traffic:=true or enabled at runtime): one V3 traffic node per moving model of the NEW
     hospital (staff, visitors, trolley; v3_hospital_layout.TRAFFIC), on corridors / lobby crossed by the routes
  6. V3 perception: detection (+ signs), spatial fusion, costmap obstacles
  7. V3 mission manager, V3 web UI server
V2.6 navigation.launch.py is not used because it hard-codes the V2.6 memory DB and starts the V2.6
dashboard; nothing in V2.6 is modified.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

V3_ROOT = os.environ.get('V3_ROOT', os.path.expanduser('~/CV_Autonomous_Navigation/V3'))
sys.path.insert(0, os.path.join(V3_ROOT, 'scripts'))
import v3_hospital_layout as LAYOUT  # noqa: E402  (NEW hospital: spawn, traffic lines)


def generate_launch_description():
    pkg_v3 = get_package_share_directory('hospital_logistics')
    pkg_nav = get_package_share_directory('autonomous_robot_navigation')
    nav2_params = os.path.join(V3_ROOT, 'config', 'v3_nav2_params.yaml')  # V3 copy (new hospital spawn, no AMCL random recovery)
    st = {'use_sim_time': True}
    headless = LaunchConfiguration('headless')
    traffic = LaunchConfiguration('traffic')
    web_port = LaunchConfiguration('web_port')

    sx, sy, syaw = LAYOUT.SPAWN
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_v3, 'launch', 'v3_world.launch.py')),
        launch_arguments={'headless': headless, 'v3_root': V3_ROOT, 'x_pose': str(sx), 'y_pose': str(sy),
                          'yaw_pose': str(syaw)}.items())
    # Localization: same nodes as V2.6 localization.launch.py, but with the V3 parameter file and the NEW map
    localization = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': os.path.join(V3_ROOT, 'maps', 'v3_hospital_map.yaml'), 'use_sim_time': True}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[nav2_params, {'use_sim_time': True, 'initial_pose.x': sx, 'initial_pose.y': sy,
                                       'initial_pose.z': 0.0, 'initial_pose.yaw': syaw}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl'],
                                           'bond_timeout': 10.0}]),
        Node(package='autonomous_robot_navigation', executable='initial_pose_publisher', name='initial_pose_publisher',
             output='screen', parameters=[{'use_sim_time': True, 'initial_pose_x': sx, 'initial_pose_y': sy,
                                           'initial_pose_yaw': syaw}]),
    ]

    nav2 = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[nav2_params, st], remappings=[('cmd_vel', '/cmd_vel')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[nav2_params, st]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[nav2_params, st], remappings=[('cmd_vel', '/cmd_vel')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[nav2_params, st]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'use_sim_time': True, 'autostart': True,
                                           'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]),
    ]
    decision = Node(package='hospital_logistics', executable='v3_decision_engine', name='decision_engine_node',
                    output='screen', parameters=[{'use_sim_time': True,
                                                  'db_path': os.path.join(V3_ROOT, 'data', 'v3_hospital_navigation_memory.db'),
                                                  'initial_x': sx, 'initial_y': sy, 'initial_yaw': syaw}])
    on = PythonExpression(["'", traffic, "' == 'true'"])
    v3_traffic = [Node(package='hospital_logistics', executable='v3_traffic_node', name=f'v3_traffic_{name}', output='screen',
                       parameters=[{'use_sim_time': True, 'enabled': on, 'model': name, 'line': line, 'park': park,
                                    'speed_mps': speed, 'yield_radius_m': 1.6}])
                  for name, _vis, _kind, line, park, speed in LAYOUT.TRAFFIC]
    v3 = [
        *v3_traffic,
        Node(package='hospital_logistics', executable='v3_detection_node', name='v3_detection_node', output='screen', parameters=[st]),
        Node(package='hospital_logistics', executable='v3_spatial_fusion_node', name='v3_spatial_fusion_node', output='screen', parameters=[st]),
        Node(package='hospital_logistics', executable='v3_costmap_obstacle_node', name='v3_costmap_obstacle_node', output='screen', parameters=[st]),
        Node(package='hospital_logistics', executable='v3_mission_manager', name='v3_mission_manager', output='screen', parameters=[st]),
        Node(package='hospital_logistics', executable='v3_web_server', name='v3_web_server', output='screen',
             parameters=[{'use_sim_time': True, 'port': web_port}]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('traffic', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        SetEnvironmentVariable('V3_ROOT', V3_ROOT),
        # Staggered start (measured failure without it: lifecycle change_state responses timed out while
        # ~25 processes incl. Gazebo and YOLO started at once, so Nav2 never became active).
        world,
        TimerAction(period=6.0, actions=[*localization, *nav2, decision]),
        TimerAction(period=14.0, actions=v3),
    ])
