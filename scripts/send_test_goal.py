#!/usr/bin/env python3
"""
CLI Tool to Send Test Goals to Nav2 in ROS 2 Lyrical.
Supports direct action client (/navigate_to_pose) and RViz simulation topic (/goal_pose).

Usage:
  python3 scripts/send_test_goal.py --x 0.0 --y -8.0 --yaw 1.57
  python3 scripts/send_test_goal.py --location reception
  python3 scripts/send_test_goal.py --location hospital
  python3 scripts/send_test_goal.py --topic --x 0.0 --y -8.0 --yaw 1.57
  python3 scripts/send_test_goal.py --list
"""

import os
import sys
import time
import math
import argparse
import yaml

# Ensure ROS 2 Python environment is accessible
ros_site = '/opt/ros/lyrical/lib/python3.14/site-packages'
if ros_site not in sys.path:
    sys.path.insert(0, ros_site)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "navigation_locations.yaml")


def load_locations():
    if not os.path.exists(CONFIG_PATH):
        return {}
    with open(CONFIG_PATH, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('locations', {})


def euler_to_quaternion(yaw):
    """Convert yaw angle (radians) to ROS quaternion (x, y, z, w)."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class Nav2GoalTester(Node):
    def __init__(self):
        super().__init__(
            'send_test_goal_client',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._topic_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_done = False
        self.success = False

    def wait_for_sim_clock(self, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            if time.time() - start > timeout_sec:
                self.get_logger().warn("Simulation clock not received yet. Proceeding with system time.")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_goal_action(self, x, y, yaw, timeout_sec=60.0):
        self.get_logger().info('Connecting to Nav2 action server (/navigate_to_pose)...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server /navigate_to_pose not available! Check if Nav2 bt_navigator is running.'
            )
            return False

        self.wait_for_sim_clock()

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
            f'Dispatching Nav2 Goal: [x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad ({math.degrees(yaw):.1f}°)]'
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        start_time = time.time()
        while rclpy.ok() and not self.goal_done:
            if time.time() - start_time > timeout_sec:
                self.get_logger().error(f"Navigation timed out after {timeout_sec}s!")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)

        return self.success

    def send_goal_topic(self, x, y, yaw):
        """Simulate RViz 2D Goal Pose tool by publishing PoseStamped to /goal_pose."""
        self.wait_for_sim_clock()

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.get_logger().info(
            f'Publishing to /goal_pose (RViz emulation): [x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad]'
        )
        # Publish multiple times to ensure reception
        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._topic_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info('Goal pose successfully published to /goal_pose.')
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal was REJECTED by bt_navigator!')
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
            f'[FEEDBACK] Distance remaining: {dist:.2f} m | Est. time: {time_rem} s',
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
            self.get_logger().warn(f'Navigation terminated with status code: {status}')
            self.success = False


def main():
    parser = argparse.ArgumentParser(description='Send navigation goal to Nav2')
    parser.add_argument('--location', '-l', type=str, help='Predefined location name (e.g. reception, hospital, goal_a)')
    parser.add_argument('--list', action='store_true', help='List all available predefined locations')
    parser.add_argument('--x', type=float, help='Target X coordinate in map frame')
    parser.add_argument('--y', type=float, help='Target Y coordinate in map frame')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target Yaw in radians (default: 0.0)')
    parser.add_argument('--topic', action='store_true', help='Publish to /goal_pose topic (RViz emulation) instead of action')
    parser.add_argument('--timeout', type=float, default=60.0, help='Timeout in seconds (default: 60.0)')
    args = parser.parse_args()

    locations = load_locations()

    if args.list:
        print("Available Predefined Locations:")
        for name, data in sorted(locations.items()):
            desc = data.get('description', '')
            print(f"  {name:18} [x={data['x']:6.1f}, y={data['y']:6.1f}, yaw={data.get('yaw', 0.0):5.2f}] : {desc}")
        sys.exit(0)

    target_x = None
    target_y = None
    target_yaw = 0.0

    if args.location:
        loc_key = args.location.lower()
        if loc_key not in locations:
            print(f"[ERROR] Unknown location '{args.location}'. Available options:")
            for k in locations:
                print(f"  - {k}")
            sys.exit(1)
        loc = locations[loc_key]
        target_x = loc['x']
        target_y = loc['y']
        target_yaw = loc.get('yaw', 0.0)
        print(f"Selected Location: '{loc_key}' -> {loc.get('description', '')}")
    elif args.x is not None and args.y is not None:
        target_x = args.x
        target_y = args.y
        target_yaw = args.yaw
    else:
        print("[ERROR] Must specify either --location <name> or both --x and --y. Use --help for usage.")
        sys.exit(1)

    rclpy.init()
    node = Nav2GoalTester()

    if args.topic:
        success = node.send_goal_topic(target_x, target_y, target_yaw)
    else:
        success = node.send_goal_action(target_x, target_y, target_yaw, timeout_sec=args.timeout)

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
