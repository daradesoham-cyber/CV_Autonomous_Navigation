from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# V2.5 Perception Launch
# Changes from V2.4:
#   - All model/class/threshold/camera parameters now loaded from config/perception_v25.yaml
#   - perception_config_path argument added — swap model by editing YAML + pointing here
#   - Per-class confidence thresholds (higher for safety-critical classes)
#   - Temporal sign confirmation gate prevents single-frame false positives
#   - camera_node DISABLED (unchanged from V2.4)
#   - sign_detection_node DISABLED (unchanged from V2.4)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration(
        'model_path',
        default='/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n_v25.pt'
    )
    perception_config = LaunchConfiguration(  # noqa: F841 — declared for CLI availability
        'perception_config_path',
        default='/home/soham-darade/CV_Autonomous_Navigation/config/perception_v25.yaml'
    )
    use_custom_controller = LaunchConfiguration('use_custom_controller', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n_v25.pt',
        description=(
            'Path to YOLOv8 model weights (.pt). '
            'To swap model: update this + class_names in perception_v25.yaml'
        )
    )
    declare_perception_config = DeclareLaunchArgument(
        'perception_config_path',
        default_value='/home/soham-darade/CV_Autonomous_Navigation/config/perception_v25.yaml',
        description='Path to V2.5 perception YAML config (model, classes, thresholds, camera intrinsics)'
    )
    declare_use_custom_controller = DeclareLaunchArgument(
        'use_custom_controller',
        default_value='false',
        description='Enable experimental custom pure-pursuit navigation controller (disabled by default)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model_path,
        declare_perception_config,
        declare_use_custom_controller,

        # V2.4/V2.5: camera_node DISABLED (dummy frame counter, ~16.7% CPU, no useful output)
        # Node(
        #     package='autonomous_robot_perception',
        #     executable='camera_node',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),

        # V2.5: Object detection node — loads all config from perception_v25.yaml
        # model_path overrides YAML value for quick CLI swap during experiments.
        # Per-class confidence thresholds + temporal sign confirmation gate active.
        Node(
            package='autonomous_robot_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'publish_annotated_image': True,
                'use_sim_time': use_sim_time
                # confidence_threshold, frame_skip, device, iou_threshold
                # all loaded from perception_v25.yaml by the node itself
            }]
        ),

        # LiDAR-Camera Fusion: semantic obstacle detection, tracking, TTC
        # V2.5: camera_hfov_rad matches perception_v25.yaml camera.hfov_rad
        Node(
            package='autonomous_robot_perception',
            executable='lidar_camera_fusion_node',
            name='lidar_camera_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_hfov_rad': 1.15,   # V2.4/V2.5 FOV (matches perception_v25.yaml)
            }]
        ),

        # Navigation perception: converts semantic obstacles to Nav2 costmap cloud
        Node(
            package='autonomous_robot_perception',
            executable='navigation_perception_node',
            name='navigation_perception_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # V2.4/V2.5: sign_detection_node DISABLED — CPU template matching replaced by
        # YOLO directional_sign + semantic_map.yaml lookup in object_detection_node.py
        # Kept in file for reference / fallback if needed.
        # Node(
        #     package='autonomous_robot_perception',
        #     executable='sign_detection_node',
        #     name='sign_detection_node',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'confidence_threshold': 0.45,
        #         'publish_annotated_image': True
        #     }]
        # ),

        # Experimental custom controller - disabled by default
        Node(
            package='autonomous_robot_perception',
            executable='navigation_controller_node',
            name='navigation_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_custom_controller)
        )
    ])
