from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# V2.4 Perception Launch
# Changes from V2.3:
#   - Default model_path points to yolov8n_v24.pt (V2.4 10-class model)
#   - camera_node DISABLED: it was a dummy frame-counter consuming ~16.7% CPU with no useful output
#   - sign_detection_node DISABLED: replaced by YOLO directional_sign class + semantic lookup
#   - object_detection_node now publishes /vision/signs directly
#   - frame_skip=2 parameter reduces YOLO inference rate from camera Hz to ~15Hz effective

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration(
        'model_path',
        default='/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n_v24.pt'
    )
    use_custom_controller = LaunchConfiguration('use_custom_controller', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n_v24.pt',
        description='Path to YOLOv8 model weights (V2.4: yolov8n_v24.pt, 10 classes)'
    )
    declare_use_custom_controller = DeclareLaunchArgument(
        'use_custom_controller',
        default_value='false',
        description='Enable experimental custom pure-pursuit navigation controller (disabled by default)'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model_path,
        declare_use_custom_controller,

        # V2.4: camera_node DISABLED (dummy frame counter, ~16.7% CPU, no useful output)
        # Node(
        #     package='autonomous_robot_perception',
        #     executable='camera_node',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),

        # V2.4: Object detection node — runs yolov8n_v24.pt on GPU, frame_skip=2
        # Also publishes /vision/signs via YOLO directional_sign class + semantic_map.yaml lookup
        Node(
            package='autonomous_robot_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': 0.35,
                'device': 'cuda:0',
                'publish_annotated_image': True,
                'frame_skip': 2,
                'use_sim_time': use_sim_time
            }]
        ),

        # LiDAR-Camera Fusion: semantic obstacle detection, tracking, TTC
        Node(
            package='autonomous_robot_perception',
            executable='lidar_camera_fusion_node',
            name='lidar_camera_fusion_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_hfov_rad': 1.15,   # V2.4: updated FOV
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

        # V2.4: sign_detection_node DISABLED — CPU template matching replaced by
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
