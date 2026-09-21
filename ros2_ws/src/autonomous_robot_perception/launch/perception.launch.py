from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_path = LaunchConfiguration(
        'model_path',
        default='/home/soham-darade/CV_Autonomous_Navigation/models/custom_yolov8n/weights/best.pt'
    )
    use_custom_controller = LaunchConfiguration('use_custom_controller', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/soham-darade/CV_Autonomous_Navigation/models/custom_yolov8n/weights/best.pt',
        description='Path to custom YOLOv8 model weights'
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
        Node(
            package='autonomous_robot_perception',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
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
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='autonomous_robot_perception',
            executable='lidar_camera_fusion_node',
            name='lidar_camera_fusion_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='autonomous_robot_perception',
            executable='navigation_perception_node',
            name='navigation_perception_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        Node(
            package='autonomous_robot_perception',
            executable='sign_detection_node',
            name='sign_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'confidence_threshold': 0.45,
                'publish_annotated_image': True
            }]
        ),
        # Experimental custom controller - disabled by default to avoid competing with Nav2
        Node(
            package='autonomous_robot_perception',
            executable='navigation_controller_node',
            name='navigation_controller_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_custom_controller)
        )
    ])
