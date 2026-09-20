#!/usr/bin/env python3
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header, String
from autonomous_robot_interfaces.msg import SemanticObstacleArray, SemanticObstacle

class NavigationPerceptionNode(Node):
    def __init__(self):
        super().__init__('navigation_perception_node')

        # Configurable Nav2 integration parameters
        self.declare_parameter('min_detection_distance_m', 0.25)
        self.declare_parameter('max_detection_distance_m', 6.0)
        self.declare_parameter('min_confidence_thresh', 0.35)
        self.declare_parameter('obstacle_persistence_sec', 2.5)
        self.declare_parameter('costmap_points_per_ring', 24)
        self.declare_parameter('base_frame_id', 'base_link')

        self.min_dist = self.get_parameter('min_detection_distance_m').get_parameter_value().double_value
        self.max_dist = self.get_parameter('max_detection_distance_m').get_parameter_value().double_value
        self.min_conf = self.get_parameter('min_confidence_thresh').get_parameter_value().double_value
        self.persistence = self.get_parameter('obstacle_persistence_sec').get_parameter_value().double_value
        self.pts_per_ring = self.get_parameter('costmap_points_per_ring').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # Subscribers
        self.sub_semantic = self.create_subscription(
            SemanticObstacleArray,
            '/vision/semantic_obstacles',
            self.semantic_callback,
            10
        )

        # Publishers
        self.pub_costmap_obstacles = self.create_publisher(PointCloud2, '/vision/costmap_obstacles', 10)
        self.pub_nav_status = self.create_publisher(String, '/navigation/perception_status', 10)

        # Active obstacle cache for decay management: list of (timestamp, points_list)
        self.active_obstacles = []

        # Periodic timer for decay and costmap publishing (10 Hz)
        self.create_timer(0.1, self.timer_publish_costmap)

        self.get_logger().info(
            f'NavigationPerceptionNode active. Feeding semantic obstacles to Nav2 on /vision/costmap_obstacles '
            f'(Persistence: {self.persistence}s, Frame: {self.frame_id})'
        )

    def semantic_callback(self, msg: SemanticObstacleArray):
        now = time.time()
        new_points = []
        status_alerts = []

        for obs in msg.obstacles:
            if obs.confidence < self.min_conf:
                continue
            if obs.distance < self.min_dist or obs.distance > self.max_dist:
                continue

            # Obstacle center in base_link coordinates (x forward, y left)
            cx = obs.distance * math.cos(obs.bearing)
            cy = obs.distance * math.sin(obs.bearing)
            r = obs.safety_radius

            # Generate circular obstacle ring + center points
            angles = np.linspace(0, 2 * math.pi, self.pts_per_ring, endpoint=False)
            for a in angles:
                px = cx + r * math.cos(a)
                py = cy + r * math.sin(a)
                new_points.append([float(px), float(py), 0.20])
                # Add inner ring for dense lethal marking
                new_points.append([float(cx + 0.5 * r * math.cos(a)), float(cy + 0.5 * r * math.sin(a)), 0.20])
            new_points.append([float(cx), float(cy), 0.20])

            status_alerts.append(
                f"{'DYNAMIC' if obs.is_dynamic else 'STATIC'} {obs.class_name.upper()} at {obs.distance:.2f}m (R={r:.2f}m)"
            )

        if new_points:
            self.active_obstacles.append((now, new_points))

        if status_alerts:
            status_msg = String()
            status_msg.data = f"SEMANTIC_OBSTACLE: {', '.join(status_alerts)}"
            self.pub_nav_status.publish(status_msg)

    def timer_publish_costmap(self):
        now = time.time()
        # Prune expired obstacles
        self.active_obstacles = [
            (t, pts) for (t, pts) in self.active_obstacles
            if now - t <= self.persistence
        ]

        all_points = []
        for _, pts in self.active_obstacles:
            all_points.extend(pts)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        if all_points:
            cloud = pc2.create_cloud_xyz32(header, all_points)
        else:
            # Publish empty cloud to clear costmap when no obstacles present
            cloud = pc2.create_cloud_xyz32(header, [])

        self.pub_costmap_obstacles.publish(cloud)

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
