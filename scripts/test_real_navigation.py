#!/usr/bin/env python3
"""
Real End-to-End Autonomous Navigation Test Script.
Monitors the actual Gazebo robot executing Nav2 NavigateToPose.
Logs real metric results: path length, clearance, YOLO detections, and travel time.

Usage:
  python3 scripts/test_real_navigation.py --location goal_a
  python3 scripts/test_real_navigation.py --location goal_b
  python3 scripts/test_real_navigation.py --x 10.0 --y -10.0 --yaw 0.0
"""
import os
import sys
import time
import math
import argparse
import yaml
import numpy as np

# Ensure ROS 2 and venv paths are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from autonomous_robot_interfaces.msg import Detection2DArray, SemanticObstacleArray
import tf2_ros

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "navigation_locations.yaml")

def load_locations():
    if not os.path.exists(CONFIG_PATH):
        return {}
    with open(CONFIG_PATH, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('locations', {})

def euler_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

class RealNavigationMonitor(Node):
    def __init__(self, target_goal):
        super().__init__(
            'real_navigation_monitor',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.target_goal = target_goal  # (gx, gy, gyaw)
        self.start_pose = None
        self.last_pose = None
        self.path_length = 0.0
        self.min_lidar_clearance = float('inf')
        self.total_yolo_detections = 0
        self.detected_classes = set()
        self.semantic_obstacles_count = 0
        self.replan_count = 0
        self.recovery_count = 0
        self.cmd_vel_count = 0
        self.costmap_updates_count = 0

        self.start_time = None
        self.end_time = None
        self.goal_done = False
        self.success = False

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions to monitor live simulation
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_detections = self.create_subscription(Detection2DArray, '/vision/detections', self.detections_callback, 10)
        self.sub_semantic = self.create_subscription(SemanticObstacleArray, '/vision/semantic_obstacles', self.semantic_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_plan = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)

        # Periodic pose tracking timer
        self.create_timer(0.1, self.track_robot_pose)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)]
        if len(valid) > 0:
            min_r = float(np.min(valid))
            if min_r < self.min_lidar_clearance:
                self.min_lidar_clearance = min_r

    def detections_callback(self, msg: Detection2DArray):
        self.total_yolo_detections += len(msg.detections)
        for d in msg.detections:
            self.detected_classes.add(d.class_name)

    def semantic_callback(self, msg: SemanticObstacleArray):
        self.semantic_obstacles_count += len(msg.obstacles)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_count += 1

    def plan_callback(self, msg: Path):
        self.replan_count += 1

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap_updates_count += 1

    def track_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            cur_pose = (x, y)

            if self.start_pose is None:
                self.start_pose = cur_pose
                self.last_pose = cur_pose
            else:
                step = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
                if step > 0.01:
                    self.path_length += step
                    self.last_pose = cur_pose
        except Exception:
            pass

    def run_test(self):
        self.get_logger().info('Connecting to Nav2 Action Server (/navigate_to_pose)...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 Action Server /navigate_to_pose is NOT available!')
            return False

        # Wait briefly for clock sync and robot pose
        start_wait = time.time()
        while rclpy.ok() and (self.start_pose is None) and (time.time() - start_wait < 5.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        gx, gy, gyaw = self.target_goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(gyaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending NavigateToPose goal: target=({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f} rad)'
        )

        self.start_time = time.time()
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

        while rclpy.ok() and not self.goal_done:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.end_time = time.time()
        return self.success

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal was REJECTED!')
            self.goal_done = True
            self.success = False
            return

        self.get_logger().info('NavigateToPose goal ACCEPTED. Navigating...')
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {fb.distance_remaining:.2f}m | ETA: {fb.estimated_time_remaining.sec}s | "
            f"Clearance: {self.min_lidar_clearance:.2f}m | YOLO dets: {self.total_yolo_detections}",
            throttle_duration_sec=2.0
        )

    def get_result_callback(self, future):
        result = future.result()
        self.goal_done = True
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('NavigateToPose SUCCEEDED!')
            self.success = True
        else:
            self.get_logger().warning(f'NavigateToPose finished with status: {result.status}')
            self.success = False

    def print_report(self):
        duration = (self.end_time - self.start_time) if (self.end_time and self.start_time) else 0.0
        sx, sy = self.start_pose if self.start_pose else (0.0, 0.0)
        gx, gy, gyaw = self.target_goal

        print("\n" + "=" * 70)
        print("       REAL GAZEBO END-TO-END NAVIGATION TEST REPORT")
        print("=" * 70)
        print(f"Result                  : {'SUCCESS [PASS]' if self.success else 'FAILURE [FAIL]'}")
        print(f"Start Position          : ({sx:.2f}, {sy:.2f})")
        print(f"Goal Position           : ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f} rad)")
        print(f"Navigation Time         : {duration:.2f} s")
        print(f"Actual Path Length      : {self.path_length:.2f} m")
        print(f"Min LiDAR Clearance     : {self.min_lidar_clearance:.2f} m")
        print(f"Total YOLO Detections   : {self.total_yolo_detections}")
        print(f"Classes Detected        : {list(self.detected_classes) if self.detected_classes else 'None'}")
        print(f"Semantic Obstacles Seen : {self.semantic_obstacles_count}")
        print(f"Cmd_Vel Messages        : {self.cmd_vel_count}")
        print(f"Costmap Updates         : {self.costmap_updates_count}")
        print(f"Global Replans          : {self.replan_count}")
        print(f"Recovery Behaviors      : {self.recovery_count}")
        print("=" * 70 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Run real Gazebo end-to-end navigation test')
    parser.add_argument('--location', '-l', type=str, help='Predefined location name (e.g. goal_a, goal_b)')
    parser.add_argument('--x', type=float, help='Target X coordinate')
    parser.add_argument('--y', type=float, help='Target Y coordinate')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target Yaw in radians')
    args = parser.parse_args()

    locations = load_locations()
    target_x, target_y, target_yaw = None, None, 0.0

    if args.location:
        if args.location not in locations:
            print(f"[ERROR] Unknown location '{args.location}'. Available: {list(locations.keys())}")
            sys.exit(1)
        loc = locations[args.location]
        target_x, target_y = loc['x'], loc['y']
        target_yaw = loc.get('yaw', 0.0)
    elif args.x is not None and args.y is not None:
        target_x, target_y, target_yaw = args.x, args.y, args.yaw
    else:
        print("[ERROR] Must specify --location <name> or both --x and --y.")
        sys.exit(1)

    rclpy.init()
    monitor = RealNavigationMonitor((target_x, target_y, target_yaw))
    success = monitor.run_test()
    monitor.print_report()
    monitor.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
