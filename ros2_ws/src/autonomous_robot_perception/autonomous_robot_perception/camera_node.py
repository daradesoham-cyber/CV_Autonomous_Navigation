#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import time

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('info_topic').get_parameter_value().string_value

        self.sub_image = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self.info_callback, 10)

        self.frame_count = 0
        self.last_log_time = time.time()
        self.get_logger().info(f'CameraNode initialized. Subscribed to {image_topic} and {info_topic}')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        now = time.time()
        if now - self.last_log_time >= 5.0:
            fps = self.frame_count / (now - self.last_log_time)
            self.get_logger().info(f'Camera feed active: {msg.width}x{msg.height}, rate: {fps:.1f} FPS, encoding: {msg.encoding}')
            self.frame_count = 0
            self.last_log_time = now

    def info_callback(self, msg: CameraInfo):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
