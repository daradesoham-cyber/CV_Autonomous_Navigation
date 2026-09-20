#!/usr/bin/env python3
"""
Send Navigation Goal to Nav2 via ROS 2 Lyrical Action Interface.
Usage:
  python3 scripts/send_goal.py --location goal_a
  python3 scripts/send_goal.py --location goal_b
  python3 scripts/send_goal.py --list
  python3 scripts/send_goal.py --x 5.0 --y -2.0 --yaw 0.0
"""
import os
import sys
import math
import argparse
import yaml

# Ensure ROS 2 Python environment is accessible
if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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

class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('send_goal_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_done = False
        self.success = False

    def send_goal(self, x, y, yaw, wait=True):
        self.get_logger().info(f'Connecting to Nav2 action server (/navigate_to_pose)...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server /navigate_to_pose not available! Ensure Nav2 is running.'
            )
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending Nav2 Goal: [x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)]'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        if wait:
            while rclpy.ok() and not self.goal_done:
                rclpy.spin_once(self, timeout_sec=0.1)
            return self.success
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal was REJECTED by planner!')
            self.goal_done = True
            self.success = False
            return

        self.get_logger().info('Nav2 Goal ACCEPTED. Robot is navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist = feedback.distance_remaining
        time_rem = feedback.estimated_time_remaining.sec
        self.get_logger().info(
            f'Navigating... Distance remaining: {dist:.2f} m | Estimated time: {time_rem} s',
            throttle_duration_sec=2.0
        )

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        self.goal_done = True

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('SUCCESS: Goal reached successfully!')
            self.success = True
        else:
            self.get_logger().warn(f'Navigation finished with status: {status}')
            self.success = False

def main():
    parser = argparse.ArgumentParser(description='Send a navigation goal to Nav2')
    parser.add_argument('--location', '-l', type=str, help='Predefined location name (e.g. goal_a, goal_b)')
    parser.add_argument('--list', action='store_true', help='List all available predefined locations')
    parser.add_argument('--x', type=float, help='Custom X coordinate')
    parser.add_argument('--y', type=float, help='Custom Y coordinate')
    parser.add_argument('--yaw', type=float, default=0.0, help='Custom Yaw in radians (default: 0.0)')
    parser.add_argument('--no-wait', action='store_true', help='Send goal and exit immediately without waiting')
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
        print(f"Selected predefined location: '{args.location}' -> {loc.get('description', '')}")
    elif args.x is not None and args.y is not None:
        target_x = args.x
        target_y = args.y
        target_yaw = args.yaw
    else:
        print("[ERROR] Must specify either --location <name> or both --x and --y. Run with --help.")
        sys.exit(1)

    rclpy.init()
    node = Nav2GoalSender()
    success = node.send_goal(target_x, target_y, target_yaw, wait=not args.no_wait)
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
