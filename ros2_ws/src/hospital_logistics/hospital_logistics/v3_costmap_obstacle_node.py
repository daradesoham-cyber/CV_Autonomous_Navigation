#!/usr/bin/env python3
"""
V3 costmap obstacle node = V3-local copy of the V2.6 navigation_perception_node
(ros2_ws/src/autonomous_robot_perception/.../navigation_perception_node.py, unchanged in V2.6).

Same inputs/outputs and the same obstacle geometry (ring of radius safety_radius around a centre at
distance + radius along the bearing, inner ring, centre point, 0.30 m robot-footprint exclusion,
2.5 s persistence, 10 Hz publishing on /vision/costmap_obstacles).

V3 Phase 7 fix (measured in Phase 3): V2.6 stored the points in base_link and re-published them
unchanged for 2.5 s, so while the robot moved the remembered obstacles moved WITH the robot (7.1 % of
points > 1 m from any real object during a pass-by). V3 stores each obstacle's points in the odom
frame using the robot odom pose at the obstacle message time, and on every publish re-projects all
stored points into base_link with the CURRENT odom pose. Remembered obstacles therefore stay fixed in
the world, and the cloud origin stays at the robot (Nav2 obstacle_range / raytracing are measured from
the cloud frame origin).
"""
import math
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String

from autonomous_robot_interfaces.msg import SemanticObstacleArray


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class V3CostmapObstacleNode(Node):
    def __init__(self):
        super().__init__('v3_costmap_obstacle_node')
        self.declare_parameter('min_detection_distance_m', 0.25)
        self.declare_parameter('max_detection_distance_m', 6.0)
        self.declare_parameter('min_confidence_thresh', 0.35)
        self.declare_parameter('obstacle_persistence_sec', 2.5)
        self.declare_parameter('costmap_points_per_ring', 24)
        self.declare_parameter('base_frame_id', 'base_link')
        self.min_dist = self.get_parameter('min_detection_distance_m').value
        self.max_dist = self.get_parameter('max_detection_distance_m').value
        self.min_conf = self.get_parameter('min_confidence_thresh').value
        self.persistence = self.get_parameter('obstacle_persistence_sec').value
        self.pts_per_ring = int(self.get_parameter('costmap_points_per_ring').value)
        self.frame_id = self.get_parameter('base_frame_id').value

        self.odom_hist = deque(maxlen=400)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.create_subscription(SemanticObstacleArray, '/vision/semantic_obstacles', self.semantic_callback, 10)
        self.pub_costmap_obstacles = self.create_publisher(PointCloud2, '/vision/costmap_obstacles', 10)
        self.pub_nav_status = self.create_publisher(String, '/navigation/perception_status', 10)
        self.active_obstacles = []  # (timestamp_wall_ros, [(x_odom, y_odom, z), ...])
        self.create_timer(0.1, self.timer_publish_costmap)
        self.get_logger().info(
            f'V3CostmapObstacleNode active: world-fixed (odom) persistence {self.persistence}s, '
            f'published in {self.frame_id}')

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        self.odom_hist.append((t, p.position.x, p.position.y, _yaw(p.orientation)))

    def pose_at(self, t=None):
        if not self.odom_hist:
            return None
        if t is None:
            return self.odom_hist[-1][1:]
        best = min(self.odom_hist, key=lambda e: abs(e[0] - t))
        return best[1:] if abs(best[0] - t) <= 0.15 else None

    def semantic_callback(self, msg: SemanticObstacleArray):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = self.pose_at(st)
        if pose is None:
            return  # no odom pose at the obstacle time: do not guess its world position
        rx, ry, ryaw = pose
        c, s = math.cos(ryaw), math.sin(ryaw)
        now = self.get_clock().now().nanoseconds / 1e9
        new_points, status_alerts = [], []
        for obs in msg.obstacles:
            if obs.confidence < self.min_conf or obs.distance < self.min_dist or obs.distance > self.max_dist:
                continue
            r = obs.safety_radius
            center_dist = obs.distance + r
            cx = center_dist * math.cos(obs.bearing)
            cy = center_dist * math.sin(obs.bearing)
            base_pts = []
            for k in range(self.pts_per_ring):
                a = 2 * math.pi * k / self.pts_per_ring
                px, py = cx + r * math.cos(a), cy + r * math.sin(a)
                if math.hypot(px, py) >= 0.30:
                    base_pts.append((px, py))
                ipx, ipy = cx + 0.5 * r * math.cos(a), cy + 0.5 * r * math.sin(a)
                if math.hypot(ipx, ipy) >= 0.30:
                    base_pts.append((ipx, ipy))
            if math.hypot(cx, cy) >= 0.30:
                base_pts.append((cx, cy))
            for px, py in base_pts:  # base_link at obstacle time -> odom
                new_points.append((rx + c * px - s * py, ry + s * px + c * py, 0.20))
            status_alerts.append(
                f"{'DYNAMIC' if obs.is_dynamic else 'STATIC'} {obs.class_name.upper()} at {obs.distance:.2f}m (R={r:.2f}m)")
        if new_points:
            self.active_obstacles.append((now, new_points))
        if status_alerts:
            self.pub_nav_status.publish(String(data=f"SEMANTIC_OBSTACLE: {', '.join(status_alerts)}"))

    def timer_publish_costmap(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.active_obstacles = [(t, pts) for (t, pts) in self.active_obstacles if now - t <= self.persistence]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        pose = self.pose_at()
        out = []
        if pose is not None:
            rx, ry, ryaw = pose
            c, s = math.cos(ryaw), math.sin(ryaw)
            for _, pts in self.active_obstacles:  # odom -> CURRENT base_link
                for x, y, z in pts:
                    dx, dy = x - rx, y - ry
                    bx, by = c * dx + s * dy, -s * dx + c * dy
                    if math.hypot(bx, by) >= 0.30:
                        out.append([float(bx), float(by), float(z)])
        self.pub_costmap_obstacles.publish(pc2.create_cloud_xyz32(header, out))


def main(args=None):
    rclpy.init(args=args)
    node = V3CostmapObstacleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
