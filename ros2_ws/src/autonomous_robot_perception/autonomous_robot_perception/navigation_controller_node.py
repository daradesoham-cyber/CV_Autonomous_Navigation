#!/usr/bin/env python3
"""
Autonomous Navigation Controller Node for ROS 2 Lyrical & Nav2 Integration.
Provides:
1. RViz 2D Goal Pose subscription (/goal_pose)
2. Nav2 Action Server (/navigate_to_pose)
3. Collision-free path generation and replanning (/plan, /local_plan)
4. Pure pursuit trajectory tracking with semantic obstacle avoidance (/cmd_vel)
"""
import sys
import os
import math
import time
import numpy as np

# Ensure ROS and Virtualenv site-packages are available
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from autonomous_robot_interfaces.msg import SemanticObstacleArray
import tf2_ros

def euler_from_quaternion(q):
    """Convert quaternion (x, y, z, w) to yaw angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw):
    """Convert yaw angle to quaternion (x, y, z, w)."""
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

class NavigationControllerNode(Node):
    def __init__(self):
        super().__init__('navigation_controller_node')

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('xy_goal_tolerance', 0.25)
        self.declare_parameter('yaw_goal_tolerance', 0.15)
        self.declare_parameter('lookahead_dist', 0.6)
        self.declare_parameter('robot_radius', 0.25)

        self.max_v = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.xy_tol = self.get_parameter('xy_goal_tolerance').get_parameter_value().double_value
        self.yaw_tol = self.get_parameter('yaw_goal_tolerance').get_parameter_value().double_value
        self.lookahead = self.get_parameter('lookahead_dist').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_global_plan = self.create_publisher(Path, '/plan', 10)
        self.pub_local_plan = self.create_publisher(Path, '/local_plan', 10)

        # Subscribers
        self.sub_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10,
            callback_group=self.cb_group
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10,
            callback_group=self.cb_group
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10,
            callback_group=self.cb_group
        )
        self.sub_semantic = self.create_subscription(
            SemanticObstacleArray, '/vision/semantic_obstacles', self.semantic_callback, 10,
            callback_group=self.cb_group
        )
        self.sub_estop = self.create_subscription(
            Twist, '/cmd_vel_stop', self.estop_callback, 10,
            callback_group=self.cb_group
        )

        # Action Server for Nav2 NavigateToPose
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_action_goal,
            goal_callback=self.action_goal_callback,
            cancel_callback=self.action_cancel_callback,
            callback_group=self.cb_group
        )

        # State Variables
        self.odom_pose = None
        self.latest_scan = None
        self.active_semantic_obstacles = []

        # Navigation State
        self.nav_active = False
        self.target_goal = None    # (x, y, yaw)
        self.current_path = []     # List of np.array([x, y])
        self.path_idx = 0
        self.active_goal_handle = None

        # Control Loop Timer (20 Hz)
        self.create_timer(0.05, self.control_loop, callback_group=self.cb_group)

        self.get_logger().info('NavigationControllerNode active: /goal_pose and /navigate_to_pose ready.')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q)
        self.odom_pose = (p.x, p.y, yaw)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def semantic_callback(self, msg: SemanticObstacleArray):
        self.active_semantic_obstacles = msg.obstacles

    def estop_callback(self, msg: Twist):
        self.get_logger().warning('EMERGENCY STOP RECEIVED! Halting all robot motion.')
        self.stop_robot()
        self.nav_active = False
        self.target_goal = None
        if self.active_goal_handle:
            try:
                self.active_goal_handle.abort()
            except Exception:
                pass

    def get_robot_pose(self):
        """Retrieve latest robot pose in map frame via TF, with odom fallback."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            r = t.transform.rotation
            yaw = euler_from_quaternion(r)
            return p.x, p.y, yaw
        except Exception:
            if self.odom_pose is not None:
                # Odom frame offset: spawn at (-10.0, -10.0) in map
                return self.odom_pose[0] - 10.0, self.odom_pose[1] - 10.0, self.odom_pose[2]
            return None

    def plan_path(self, start, goal):
        """Generate smooth waypoints from start to goal avoiding known obstacles."""
        start_pt = np.array([start[0], start[1]])
        goal_pt = np.array([goal[0], goal[1]])

        vec = goal_pt - start_pt
        dist = np.linalg.norm(vec)
        if dist < 0.1:
            return [goal_pt]

        dir_u = vec / dist
        perp_u = np.array([-dir_u[1], dir_u[0]])

        # Check if active semantic obstacles intersect path
        detours = []
        for obs in self.active_semantic_obstacles:
            obs_dist = obs.distance
            obs_bearing = obs.bearing
            obs_map_x = start[0] + obs_dist * math.cos(start[2] + obs_bearing)
            obs_map_y = start[1] + obs_dist * math.sin(start[2] + obs_bearing)
            obs_p = np.array([obs_map_x, obs_map_y])

            to_obs = obs_p - start_pt
            proj = np.dot(to_obs, dir_u)
            if 0 < proj < dist:
                lat = np.linalg.norm(to_obs - proj * dir_u)
                margin = obs.safety_radius + self.robot_radius
                if lat < margin:
                    shift = margin + 0.35
                    detour_wp = obs_p + perp_u * shift
                    detours.append((proj, detour_wp))

        if not detours:
            num_pts = max(10, int(dist / 0.2))
            pts = np.linspace(start_pt, goal_pt, num_pts)
            return [np.array([p[0], p[1]]) for p in pts]

        detours.sort(key=lambda x: x[0])
        path = []
        cur = start_pt
        for _, wp in detours:
            for p in np.linspace(cur, wp, 10)[1:]:
                path.append(np.array([p[0], p[1]]))
            cur = wp
        for p in np.linspace(cur, goal_pt, 10)[1:]:
            path.append(np.array([p[0], p[1]]))
        return path

    def publish_path_msgs(self, waypoints):
        """Publish global plan to /plan."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wp in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = 0.05
            msg.poses.append(ps)
        self.pub_global_plan.publish(msg)

    def goal_pose_callback(self, msg: PoseStamped):
        """Handle goal sent from RViz 2D Goal Pose tool."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gyaw = euler_from_quaternion(msg.pose.orientation)

        self.get_logger().info(f'Received RViz Goal: [x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f} rad]')
        self.start_navigation((gx, gy, gyaw))

    def action_goal_callback(self, goal_request):
        self.get_logger().info('Received Nav2 Action Goal request.')
        return GoalResponse.ACCEPT

    def action_cancel_callback(self, goal_handle):
        self.get_logger().info('Received Nav2 Action Cancel request.')
        self.stop_robot()
        self.nav_active = False
        return CancelResponse.ACCEPT

    def execute_action_goal(self, goal_handle):
        """Execute goal sent via Nav2 Action Client."""
        self.active_goal_handle = goal_handle
        pose = goal_handle.request.pose
        gx = pose.pose.position.x
        gy = pose.pose.position.y
        gyaw = euler_from_quaternion(pose.pose.orientation)

        self.get_logger().info(f'Executing Action Goal: [x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f}]')
        self.start_navigation((gx, gy, gyaw))

        feedback = NavigateToPose.Feedback()

        # Monitor navigation while control_loop executes concurrently
        while rclpy.ok() and self.nav_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                self.get_logger().info('Action Goal Canceled.')
                result = NavigateToPose.Result()
                return result

            robot_pose = self.get_robot_pose()
            if robot_pose:
                d_rem = math.hypot(gx - robot_pose[0], gy - robot_pose[1])
                feedback.distance_remaining = float(d_rem)
                feedback.estimated_time_remaining.sec = int(d_rem / max(0.1, self.max_v))
                goal_handle.publish_feedback(feedback)

            time.sleep(0.2)

        result = NavigateToPose.Result()
        if not self.nav_active and self.target_goal is None:
            goal_handle.succeed()
            self.get_logger().info('Action Goal Succeeded!')
        else:
            goal_handle.abort()
            self.get_logger().warning('Action Goal Aborted.')
        return result

    def start_navigation(self, goal):
        self.target_goal = goal
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warning('Waiting for robot localization...')
            # Retry after brief wait
            time.sleep(0.2)
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                return

        self.current_path = self.plan_path(robot_pose, goal)
        self.path_idx = 0
        self.nav_active = True
        self.publish_path_msgs(self.current_path)
        self.get_logger().info(f'Planned path with {len(self.current_path)} waypoints. Commencing navigation.')

    def stop_robot(self):
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)

    def control_loop(self):
        if not self.nav_active or self.target_goal is None:
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        rx, ry, ryaw = robot_pose
        gx, gy, gyaw = self.target_goal

        dist_to_goal = math.hypot(gx - rx, gy - ry)

        # 1. Goal Reached Check
        if dist_to_goal < self.xy_tol:
            # Align yaw with final orientation
            yaw_diff = (gyaw - ryaw + math.pi) % (2 * math.pi) - math.pi
            if abs(yaw_diff) > self.yaw_tol:
                cmd = Twist()
                cmd.angular.z = float(np.clip(yaw_diff * 1.5, -self.max_w, self.max_w))
                self.pub_cmd_vel.publish(cmd)
                return
            else:
                # Fully arrived
                self.stop_robot()
                self.nav_active = False
                self.target_goal = None
                self.get_logger().info('Destination reached successfully!')
                return

        # 2. Recheck active obstacles for replanning
        if self.active_semantic_obstacles:
            needs_replan = False
            for obs in self.active_semantic_obstacles:
                if obs.distance < (obs.safety_radius + self.robot_radius + 0.3):
                    needs_replan = True
                    break
            if needs_replan:
                self.current_path = self.plan_path((rx, ry, ryaw), (gx, gy, gyaw))
                self.path_idx = 0
                self.publish_path_msgs(self.current_path)

        # 3. Find lookahead point along path
        target_wp = None
        for i in range(self.path_idx, len(self.current_path)):
            wp = self.current_path[i]
            d = math.hypot(wp[0] - rx, wp[1] - ry)
            if d >= self.lookahead:
                target_wp = wp
                self.path_idx = i
                break

        if target_wp is None:
            target_wp = self.current_path[-1]

        # 4. Pure Pursuit Steering
        target_angle = math.atan2(target_wp[1] - ry, target_wp[0] - rx)
        heading_err = (target_angle - ryaw + math.pi) % (2 * math.pi) - math.pi

        # 5. Compute Velocity Commands
        cmd = Twist()
        speed_scale = max(0.15, math.cos(heading_err))
        dist_scale = min(1.0, dist_to_goal / 1.5)
        cmd.linear.x = float(self.max_v * speed_scale * dist_scale)
        cmd.angular.z = float(np.clip(heading_err * 2.0, -self.max_w, self.max_w))

        self.pub_cmd_vel.publish(cmd)

        self.get_logger().info(
            f'Robot at ({rx:.2f}, {ry:.2f}) -> Target ({gx:.2f}, {gy:.2f}) | Dist: {dist_to_goal:.2f}m | v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s',
            throttle_duration_sec=1.5
        )

        # Publish local plan for RViz
        loc_path = Path()
        loc_path.header.frame_id = 'map'
        loc_path.header.stamp = self.get_clock().now().to_msg()
        for p in [robot_pose[:2], target_wp]:
            ps = PoseStamped()
            ps.header = loc_path.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = 0.05
            loc_path.poses.append(ps)
        self.pub_local_plan.publish(loc_path)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
