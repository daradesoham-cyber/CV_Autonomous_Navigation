#!/usr/bin/env python3
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from autonomous_robot_interfaces.msg import (
    Detection2DArray,
    ObstacleWarning,
    SemanticObstacle,
    SemanticObstacleArray
)

# Configurable semantic safety parameters
SEMANTIC_SAFETY_CONFIG = {
    'person': {'radius': 0.90, 'is_dynamic': True},
    'cart': {'radius': 0.75, 'is_dynamic': True},
    'chair': {'radius': 0.55, 'is_dynamic': False},
    'dining table': {'radius': 0.65, 'is_dynamic': False},
    'table': {'radius': 0.65, 'is_dynamic': False},
    'box': {'radius': 0.45, 'is_dynamic': False},
    'suitcase': {'radius': 0.45, 'is_dynamic': False},
    'traffic cone': {'radius': 0.50, 'is_dynamic': False},
    'cone': {'radius': 0.50, 'is_dynamic': False},
    'shelf': {'radius': 0.60, 'is_dynamic': False},
    'pallet': {'radius': 0.55, 'is_dynamic': False},
    'hospital_bed': {'radius': 0.85, 'is_dynamic': False},
    'bed': {'radius': 0.85, 'is_dynamic': False},
}
DEFAULT_CONFIG = {'radius': 0.50, 'is_dynamic': False}

class LidarCameraFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion_node')

        self.declare_parameter('camera_hfov_rad', 1.089)  # approx 62.4 deg
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('critical_distance_m', 2.5)

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
        self.pub_semantic = self.create_publisher(SemanticObstacleArray, '/vision/semantic_obstacles', 10)

        self.get_logger().info('LidarCameraFusionNode initialized: angular association and semantic safety pipeline active.')

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

        semantic_array = SemanticObstacleArray()
        semantic_array.header = msg.header

        for det in msg.detections:
            # Center of bounding box horizontally in pixels
            x_center = (det.x_min + det.x_max) / 2.0
            box_width = det.x_max - det.x_min

            # Angular projection (0 is straight ahead, positive is left in standard ROS optical/robot frame)
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

            # Retrieve semantic classification parameters
            cls_name = det.class_name.lower()
            cfg = SEMANTIC_SAFETY_CONFIG.get(cls_name, DEFAULT_CONFIG)
            safety_radius = cfg['radius']
            is_dynamic = cfg['is_dynamic']

            if distance > 0.1 and distance <= self.crit_dist:
                # Semantic Obstacle Message
                sem_obs = SemanticObstacle()
                sem_obs.header = msg.header
                sem_obs.class_name = det.class_name
                sem_obs.confidence = det.confidence
                sem_obs.distance = distance
                sem_obs.bearing = float(bearing)
                sem_obs.safety_radius = safety_radius
                sem_obs.is_dynamic = is_dynamic
                semantic_array.obstacles.append(sem_obs)

                # Obstacle Warning Message
                warn = ObstacleWarning()
                warn.header = msg.header
                warn.obstacle_type = det.class_name
                warn.distance = distance
                warn.bearing = float(bearing)
                warn.requires_clearance = is_dynamic
                self.pub_obstacles.publish(warn)

                self.get_logger().info(
                    f'Semantic Obstacle: {det.class_name.upper()} | Conf: {det.confidence:.2f} | Dist: {distance:.2f}m | '
                    f'Bearing: {math.degrees(bearing):.1f}° | Safety Radius: {safety_radius:.2f}m | Dynamic: {is_dynamic}',
                    throttle_duration_sec=1.0
                )

        self.pub_objects.publish(updated_detections)
        self.pub_semantic.publish(semantic_array)

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
