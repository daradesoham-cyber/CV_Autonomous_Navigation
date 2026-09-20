#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from autonomous_robot_interfaces.msg import Detection2DArray, ObstacleWarning

class LidarCameraFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion_node')

        self.declare_parameter('camera_hfov_rad', 1.089)  # approx 62.4 deg
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('critical_distance_m', 2.0)

        self.hfov = self.get_parameter('camera_hfov_rad').get_parameter_value().double_value
        self.img_w = self.get_parameter('image_width').get_parameter_value().double_value
        self.crit_dist = self.get_parameter('critical_distance_m').get_parameter_value().double_value

        self.latest_scan = None

        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_detections = self.create_subscription(Detection2DArray, '/vision/detections', self.detections_callback, 10)

        # Publishers
        self.pub_objects = self.create_publisher(Detection2DArray, '/vision/objects', 10)
        self.pub_obstacles = self.create_publisher(ObstacleWarning, '/vision/obstacles', 10)

        self.get_logger().info('LidarCameraFusionNode initialized: angular association pipeline active.')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_scan is None:
            return

        scan = self.latest_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = np.array(scan.ranges)

        updated_detections = Detection2DArray()
        updated_detections.header = msg.header

        for det in msg.detections:
            # Center of bounding box horizontally in pixels
            x_center = (det.x_min + det.x_max) / 2.0
            box_width = det.x_max - det.x_min

            # Angular projection (0 is straight ahead, positive is left in standard ROS optical/robot frame)
            # Pixel 0 is left edge (+hfov/2), Pixel W is right edge (-hfov/2)
            bearing = (0.5 - (x_center / self.img_w)) * self.hfov
            angular_span = (box_width / self.img_w) * self.hfov

            # Calculate LiDAR index range corresponding to the bounding box angular slice
            min_angle = bearing - (angular_span / 2.0)
            max_angle = bearing + (angular_span / 2.0)

            idx_start = int((min_angle - angle_min) / angle_inc)
            idx_end = int((max_angle - angle_min) / angle_inc)

            if idx_start > idx_end:
                idx_start, idx_end = idx_end, idx_start

            idx_start = max(0, min(idx_start, len(ranges) - 1))
            idx_end = max(0, min(idx_end, len(ranges) - 1))

            slice_ranges = ranges[idx_start:idx_end + 1]
            valid_ranges = slice_ranges[(slice_ranges >= scan.range_min) & (slice_ranges <= scan.range_max) & np.isfinite(slice_ranges)]

            if len(valid_ranges) > 0:
                # Use 20th percentile to get front surface distance robust to background points
                distance = float(np.percentile(valid_ranges, 20))
            else:
                distance = -1.0

            det.distance = distance
            det.bearing = float(bearing)
            updated_detections.detections.append(det)

            # Publish obstacle warning if close
            if distance > 0 and distance <= self.crit_dist:
                warn = ObstacleWarning()
                warn.header = msg.header
                warn.obstacle_type = det.class_name
                warn.distance = distance
                warn.bearing = float(bearing)
                warn.requires_clearance = det.class_name in ['person', 'chair', 'dining table']
                self.pub_obstacles.publish(warn)

                self.get_logger().info(
                    f'Object: {det.class_name.upper()} | Conf: {det.confidence:.2f} | Dist: {distance:.2f}m | Bearing: {math.degrees(bearing):.1f}°',
                    throttle_duration_sec=1.0
                )

        self.pub_objects.publish(updated_detections)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
