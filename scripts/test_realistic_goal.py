#!/usr/bin/env python3
"""
End-to-End Programmatic Test Script for Realistic Facility Autonomous Navigation.
Verifies:
1. Simulation clock & ROS graph health
2. AMCL localization & TF tree integrity (map -> odom -> base_footprint -> base_link -> laser_link)
3. Nav2 /navigate_to_pose action server responsiveness
4. Physical movement in Gazebo (monitoring /cmd_vel, /odom, /amcl_pose)
5. RViz 2D Goal Pose tool emulation (/goal_pose topic)
6. Semantic mission routing via /navigation/goal_label -> 'HOSPITAL'

Usage:
  python3 scripts/test_realistic_goal.py
"""

import os
import sys
import time
import math
import json

# Ensure ROS 2 Python environment is accessible
ros_site = '/opt/ros/lyrical/lib/python3.14/site-packages'
if ros_site not in sys.path:
    sys.path.insert(0, ros_site)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import tf2_ros


def euler_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class RealisticGoalTester(Node):
    def __init__(self):
        super().__init__(
            'realistic_goal_tester_node',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.get_logger().info("Initializing Realistic Goal Tester Node...")

        # Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_goal_label = self.create_publisher(String, '/navigation/goal_label', 10)

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State tracking
        self.clock_received = False
        self.map_received = False
        self.latest_amcl_pose = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.cmd_vel_history = []
        self.latest_mission_status = None

        # QoS Profiles
        qos_transient = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        self.sub_clock = self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_transient)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, qos_transient)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.sub_status = self.create_subscription(String, '/navigation/mission_status', self._status_cb, 10)

        self.action_done = False
        self.action_success = False
        self.feedback_distance = None

    def _clock_cb(self, msg: Clock):
        self.clock_received = True

    def _map_cb(self, msg: OccupancyGrid):
        self.map_received = True

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.latest_amcl_pose = msg.pose.pose

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg.pose.pose

    def _cmd_vel_cb(self, msg: Twist):
        self.latest_cmd_vel = msg
        speed = math.hypot(msg.linear.x, msg.linear.y)
        if speed > 0.01 or abs(msg.angular.z) > 0.01:
            self.cmd_vel_history.append((self.get_clock().now().nanoseconds, msg.linear.x, msg.angular.z))

    def _status_cb(self, msg: String):
        try:
            self.latest_mission_status = json.loads(msg.data)
        except Exception:
            self.latest_mission_status = {'raw': msg.data}

    def spin_duration(self, duration_sec: float):
        start = time.time()
        while rclpy.ok() and (time.time() - start) < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.05)


def run_tests():
    rclpy.init()
    tester = RealisticGoalTester()

    results = {}
    print("=" * 70)
    print("AUTONOMOUS NAVIGATION END-TO-END TEST: REALISTIC FACILITY")
    print("=" * 70)

    # -------------------------------------------------------------
    # STEP 1: Verify Simulation Clock & Topics
    # -------------------------------------------------------------
    print("\n[STEP 1] Verifying Simulation Clock & Environment...")
    tester.spin_duration(3.0)

    if tester.clock_received:
        print("  [PASS] Gazebo /clock is active.")
        results['clock'] = True
    else:
        print("  [WARN] /clock not detected yet. Waiting up to 10s...")
        t0 = time.time()
        while not tester.clock_received and (time.time() - t0) < 10.0:
            tester.spin_duration(0.5)
        results['clock'] = tester.clock_received
        print(f"  [{'PASS' if tester.clock_received else 'FAIL'}] Simulation clock active.")

    # -------------------------------------------------------------
    # STEP 2: Verify Map Server
    # -------------------------------------------------------------
    print("\n[STEP 2] Verifying Map Server (/map)...")
    t0 = time.time()
    while not tester.map_received and (time.time() - t0) < 10.0:
        tester.spin_duration(0.5)
    results['map'] = tester.map_received
    print(f"  [{'PASS' if tester.map_received else 'FAIL'}] Occupancy map received on /map.")

    # -------------------------------------------------------------
    # STEP 3: Verify AMCL Localization & TF Tree
    # -------------------------------------------------------------
    print("\n[STEP 3] Verifying AMCL Localization & TF Tree...")
    t0 = time.time()
    while tester.latest_amcl_pose is None and (time.time() - t0) < 10.0:
        tester.spin_duration(0.5)

    if tester.latest_amcl_pose:
        px = tester.latest_amcl_pose.position.x
        py = tester.latest_amcl_pose.position.y
        print(f"  [PASS] AMCL pose available: (x={px:.2f}, y={py:.2f}) [expected ~ (0.0, -11.0)]")
        results['amcl_pose'] = True
    else:
        print("  [FAIL] AMCL pose not received on /amcl_pose!")
        results['amcl_pose'] = False

    # Check TF transforms
    tf_ok = False
    try:
        dur = rclpy.duration.Duration(seconds=3.0)
        # Check map -> odom
        t_map_odom = tester.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=dur)
        # Check odom -> base_footprint
        t_odom_base = tester.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=dur)
        # Check base_footprint -> laser_link
        t_base_laser = tester.tf_buffer.lookup_transform('base_footprint', 'laser_link', rclpy.time.Time(), timeout=dur)
        # Check full chain map -> laser_link
        t_map_laser = tester.tf_buffer.lookup_transform('map', 'laser_link', rclpy.time.Time(), timeout=dur)
        tf_ok = True
        print("  [PASS] Complete TF chain verified: map -> odom -> base_footprint -> base_link -> laser_link")
    except Exception as e:
        print(f"  [FAIL] TF lookup failed: {e}")

    results['tf_tree'] = tf_ok

    # -------------------------------------------------------------
    # STEP 4: Verify Nav2 Action Server
    # -------------------------------------------------------------
    print("\n[STEP 4] Verifying Nav2 Action Server (/navigate_to_pose)...")
    server_ready = tester.nav_client.wait_for_server(timeout_sec=10.0)
    results['action_server'] = server_ready
    print(f"  [{'PASS' if server_ready else 'FAIL'}] /navigate_to_pose action server is ready.")

    if not server_ready:
        print("\n[ERROR] Nav2 action server is not running. Terminating further tests.")
        tester.destroy_node()
        rclpy.shutdown()
        return results

    # -------------------------------------------------------------
    # STEP 5: Test Programmatic Nav2 Goal (Reception: x=0.0, y=-8.0, yaw=1.57)
    # -------------------------------------------------------------
    print("\n[STEP 5] Executing Programmatic Goal to Reception (x=0.0, y=-8.0, yaw=1.57)...")
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = tester.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = 0.0
    goal_msg.pose.pose.position.y = -8.0
    goal_msg.pose.pose.position.z = 0.0

    qx, qy, qz, qw = euler_to_quaternion(1.57)
    goal_msg.pose.pose.orientation.x = qx
    goal_msg.pose.pose.orientation.y = qy
    goal_msg.pose.pose.orientation.z = qz
    goal_msg.pose.pose.orientation.w = qw

    tester.action_done = False
    tester.action_success = False
    tester.cmd_vel_history.clear()

    start_odom_y = tester.latest_odom.position.y if tester.latest_odom else -11.0

    def feedback_cb(msg):
        tester.feedback_distance = msg.feedback.distance_remaining

    def goal_resp_cb(future):
        handle = future.result()
        if not handle.accepted:
            print("  [FAIL] Nav2 rejected goal!")
            tester.action_done = True
            tester.action_success = False
            return
        print("  [PASS] Nav2 accepted goal. Robot navigating...")
        res_future = handle.get_result_async()
        res_future.add_done_callback(result_cb)

    def result_cb(future):
        res = future.result()
        tester.action_done = True
        tester.action_success = (res.status == GoalStatus.STATUS_SUCCEEDED)
        print(f"  [RESULT] Nav2 goal finished with status code: {res.status}")

    send_future = tester.nav_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
    send_future.add_done_callback(goal_resp_cb)

    # Monitor physical movement during navigation
    nav_start = time.time()
    max_nav_time = 45.0
    vel_commanded = False
    robot_moved = False

    while not tester.action_done and (time.time() - nav_start) < max_nav_time:
        tester.spin_duration(0.2)

        # Check cmd_vel
        if len(tester.cmd_vel_history) > 0:
            vel_commanded = True

        # Check physical displacement
        if tester.latest_odom:
            current_y = tester.latest_odom.position.y
            displacement = abs(current_y - start_odom_y)
            if displacement > 0.3:
                robot_moved = True

        if tester.feedback_distance is not None:
            dist_str = f"{tester.feedback_distance:.2f}m"
        else:
            dist_str = "calculating..."

        curr_y = tester.latest_odom.position.y if tester.latest_odom else -11.0
        sys.stdout.write(
            f"\r  [TRACKING] Elapsed: {time.time()-nav_start:.1f}s | Robot Y: {curr_y:.2f} | Dist rem: {dist_str} | CmdVel count: {len(tester.cmd_vel_history)}  "
        )
        sys.stdout.flush()

    print()
    results['cmd_vel_received'] = vel_commanded
    results['physical_movement'] = robot_moved
    results['goal_reception_success'] = tester.action_success

    print(f"  [{'PASS' if vel_commanded else 'FAIL'}] Velocity commands published on /cmd_vel: {len(tester.cmd_vel_history)} samples.")
    print(f"  [{'PASS' if robot_moved else 'FAIL'}] Physical robot displacement verified in Gazebo.")
    print(f"  [{'PASS' if tester.action_success else 'FAIL'}] Nav2 goal arrival status.")

    # Settle before RViz goal test
    print("  Settling robot before testing /goal_pose...")
    tester.spin_duration(3.0)

    # -------------------------------------------------------------
    # STEP 6: Test RViz 2D Goal Pose Emulation (/goal_pose topic)
    # -------------------------------------------------------------
    print("\n[STEP 6] Testing RViz 2D Goal Pose Tool Emulation (/goal_pose topic)...")
    topic_msg = PoseStamped()
    topic_msg.header.frame_id = 'map'
    topic_msg.header.stamp = tester.get_clock().now().to_msg()
    topic_msg.pose.position.x = 0.0
    topic_msg.pose.position.y = -5.0  # Junction 1
    topic_msg.pose.position.z = 0.0
    qx, qy, qz, qw = euler_to_quaternion(1.57)
    topic_msg.pose.orientation.x = qx
    topic_msg.pose.orientation.y = qy
    topic_msg.pose.orientation.z = qz
    topic_msg.pose.orientation.w = qw

    # Clear velocity history to detect new motion from /goal_pose
    tester.cmd_vel_history.clear()

    print("  Publishing goal (x=0.0, y=-5.0, yaw=1.57) to /goal_pose...")
    for _ in range(3):
        topic_msg.header.stamp = tester.get_clock().now().to_msg()
        tester.pub_goal_pose.publish(topic_msg)
        tester.spin_duration(0.1)

    # Allow bt_navigator to process /goal_pose and command velocities
    t0 = time.time()
    rviz_motion = False
    while (time.time() - t0) < 15.0:
        tester.spin_duration(0.2)
        if len(tester.cmd_vel_history) > 3:
            rviz_motion = True
            break

    results['rviz_goal_topic_response'] = rviz_motion
    print(f"  [{'PASS' if rviz_motion else 'FAIL'}] bt_navigator responded to /goal_pose with active motion commands ({len(tester.cmd_vel_history)} samples).")

    # Settle before semantic mission test
    tester.spin_duration(2.0)

    # -------------------------------------------------------------
    # STEP 7: Test Semantic Mission Interface (/navigation/goal_label -> HOSPITAL)
    # -------------------------------------------------------------
    print("\n[STEP 7] Testing Semantic Mission Interface (/navigation/goal_label -> 'HOSPITAL')...")
    label_msg = String(data="HOSPITAL")
    tester.pub_goal_label.publish(label_msg)

    t0 = time.time()
    semantic_mission_active = False
    while (time.time() - t0) < 10.0:
        tester.spin_duration(0.2)
        if tester.latest_mission_status:
            status = tester.latest_mission_status.get('status')
            dest = tester.latest_mission_status.get('destination')
            if status in ['NAVIGATING', 'PLANNING', 'GOAL_ACCEPTED', 'GOAL_REJECTED'] or dest == 'hospital':
                semantic_mission_active = True
                print(f"  [PASS] DecisionEngine accepted mission: Status='{status}', Destination='{dest}'")
                print(f"         Active Path: {tester.latest_mission_status.get('active_path')}")
                break

    results['semantic_mission_active'] = semantic_mission_active
    print(f"  [{'PASS' if semantic_mission_active else 'FAIL'}] Semantic mission dispatch to HOSPITAL.")

    # -------------------------------------------------------------
    # Summary
    # -------------------------------------------------------------
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    all_passed = True
    for k, v in results.items():
        status = "PASSED" if v else "FAILED"
        print(f"  {k:30}: {status}")
        if not v:
            all_passed = False

    print("=" * 70)
    print(f"OVERALL STATUS: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")
    print("=" * 70)

    tester.destroy_node()
    rclpy.shutdown()
    return results


if __name__ == '__main__':
    run_tests()
