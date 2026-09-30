#!/usr/bin/env python3
"""
Set Initial Robot Start Pose for AMCL / Nav2 Localization.
Usage:
  python3 scripts/set_start.py --location start_a
  python3 scripts/set_start.py --location start_b
  python3 scripts/set_start.py --list
  python3 scripts/set_start.py --x -10.0 --y -10.0 --yaw 0.0
"""
import os
import sys
import time
import math
import argparse
import yaml

# Ensure ROS 2 Python environment is accessible
if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "navigation_locations.yaml")

def load_locations():
    if not os.path.exists(CONFIG_PATH):
        print(f"[ERROR] Locations config not found at: {CONFIG_PATH}")
        return {}
    with open(CONFIG_PATH, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('locations', {})

def euler_to_quaternion(yaw):
    """Convert yaw angle (radians) to ROS quaternion (x, y, z, w)."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__(
            'set_start_pose_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_pose(self, x, y, yaw, num_repeats=5):
        # Allow clock to sync if using simulation time
        start_wait = time.time()
        while self.get_clock().now().nanoseconds == 0 and (time.time() - start_wait) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Standard AMCL initial covariance matrix (6x6 row-major)
        # Covariance: x=0.25, y=0.25, yaw=0.068 rad^2 (~15 deg)
        cov = [0.0] * 36
        cov[0] = 0.25       # x
        cov[7] = 0.25       # y
        cov[35] = 0.0685    # yaw (rotation around z)
        msg.pose.covariance = cov

        self.get_logger().info(
            f'Publishing Initial Pose to /initialpose: [x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)]'
        )

        for _ in range(num_repeats):
            rclpy.spin_once(self, timeout_sec=0.1)
            sim_now = self.get_clock().now()
            msg.header.stamp = (sim_now - rclpy.time.Duration(seconds=0.05)).to_msg()
            self.pub.publish(msg)
            time.sleep(0.1)

        self.get_logger().info('Initial pose successfully published to AMCL.')

def main():
    parser = argparse.ArgumentParser(description='Publish initial pose for AMCL localization')
    parser.add_argument('--location', '-l', type=str, help='Predefined location name (e.g. start_a, start_b)')
    parser.add_argument('--list', action='store_true', help='List all available predefined locations')
    parser.add_argument('--x', type=float, help='Custom X coordinate')
    parser.add_argument('--y', type=float, help='Custom Y coordinate')
    parser.add_argument('--yaw', type=float, default=0.0, help='Custom Yaw in radians (default: 0.0)')
    args = parser.parse_args()

    locations = load_locations()

    if args.list:
        print("Available locations:")
        for name, data in locations.items():
            desc = data.get('description', '')
            print(f"  - {name:18} [x={data['x']:6.1f}, y={data['y']:6.1f}, yaw={data.get('yaw', 0.0):5.2f}] : {desc}")
        sys.exit(0)

    target_x = None
    target_y = None
    target_yaw = 0.0

    if args.location:
        if args.location not in locations:
            print(f"[ERROR] Unknown location '{args.location}'. Run with --list to see options.")
            sys.exit(1)
        loc = locations[args.location]
        target_x = loc['x']
        target_y = loc['y']
        target_yaw = loc.get('yaw', 0.0)
        print(f"Selected predefined start pose: '{args.location}' -> {loc.get('description', '')}")
    elif args.x is not None and args.y is not None:
        target_x = args.x
        target_y = args.y
        target_yaw = args.yaw
    else:
        print("[ERROR] Must specify either --location <name> or both --x and --y. Run with --help.")
        sys.exit(1)

    rclpy.init()
    node = InitialPosePublisher()
    node.publish_pose(target_x, target_y, target_yaw)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
