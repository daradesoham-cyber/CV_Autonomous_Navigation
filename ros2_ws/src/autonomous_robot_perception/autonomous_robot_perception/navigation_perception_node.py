#!/usr/bin/env python3
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import rclpy
from rclpy.node import Node
from autonomous_robot_interfaces.msg import ObstacleWarning, Detection2DArray
from std_msgs.msg import String

class NavigationPerceptionNode(Node):
    def __init__(self):
        super().__init__('navigation_perception_node')

        self.declare_parameter('safety_stop_distance_m', 0.8)
        self.declare_parameter('slowdown_distance_m', 1.8)

        self.safety_dist = self.get_parameter('safety_stop_distance_m').get_parameter_value().double_value
        self.slowdown_dist = self.get_parameter('slowdown_distance_m').get_parameter_value().double_value

        self.sub_obstacles = self.create_subscription(ObstacleWarning, '/vision/obstacles', self.obstacle_callback, 10)
        self.pub_nav_status = self.create_publisher(String, '/navigation/perception_status', 10)

        self.get_logger().info('NavigationPerceptionNode ready. Safety monitoring active.')

    def obstacle_callback(self, msg: ObstacleWarning):
        status_msg = String()
        if msg.distance < self.safety_dist:
            status_msg.data = f'CRITICAL_OBSTACLE: {msg.obstacle_type} at {msg.distance:.2f}m. Clearance required.'
            self.get_logger().warn(status_msg.data, throttle_duration_sec=1.0)
        elif msg.distance < self.slowdown_dist:
            status_msg.data = f'PROXIMITY_ALERT: {msg.obstacle_type} at {msg.distance:.2f}m. Slowing down.'
            self.get_logger().info(status_msg.data, throttle_duration_sec=1.0)
        else:
            status_msg.data = f'TRACKING: {msg.obstacle_type} at {msg.distance:.2f}m.'

        self.pub_nav_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
