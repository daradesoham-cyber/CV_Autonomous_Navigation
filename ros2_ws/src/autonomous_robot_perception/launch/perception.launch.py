from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_robot_perception',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='autonomous_robot_perception',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'model_path': '/home/soham-darade/CV_Autonomous_Navigation/models/yolov8n.pt',
                'confidence_threshold': 0.35,
                'device': 'cuda:0',
                'publish_annotated_image': True
            }]
        ),
        Node(
            package='autonomous_robot_perception',
            executable='lidar_camera_fusion_node',
            name='lidar_camera_fusion_node',
            output='screen'
        ),
        Node(
            package='autonomous_robot_perception',
            executable='navigation_perception_node',
            name='navigation_perception_node',
            output='screen'
        ),
        Node(
            package='autonomous_robot_perception',
            executable='navigation_controller_node',
            name='navigation_controller_node',
            output='screen'
        )
    ])
