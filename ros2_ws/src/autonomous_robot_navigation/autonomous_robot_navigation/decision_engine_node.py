#!/usr/bin/env python3
"""
Decision Engine Node for Semantic Memory-Based Autonomous Navigation.
Manages topological navigation, semantic sign interpretation, dynamic obstacle replanning,
and coordinates with Nav2 (via /navigate_to_pose) without bypassing Nav2 local/global controllers.
"""

import math
import json
import time
import os
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

# Ensure autonomous_robot_navigation package is in path
nav_pkg_dir = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation'
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from autonomous_robot_interfaces.msg import SignDetectionArray, SemanticObstacleArray
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


class DecisionEngineNode(Node):
    def __init__(self):
        super().__init__('decision_engine_node')

        self.declare_parameter(
            'db_path',
            '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db'
        )
        self.declare_parameter('node_arrival_distance', 0.85)
        self.declare_parameter('obstacle_corridor_distance', 2.5)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', -11.0)
        self.declare_parameter('initial_yaw', 1.57)

        db_path = self.get_parameter('db_path').get_parameter_value().string_value
        self.arrival_dist = self.get_parameter('node_arrival_distance').get_parameter_value().double_value
        self.obs_corridor_dist = self.get_parameter('obstacle_corridor_distance').get_parameter_value().double_value

        self.memory = NavigationMemory(db_path)
        self.graph: TopologicalGraph = self.memory.load_graph()

        # Robot state
        self.robot_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.robot_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.robot_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value
        nearest = self.graph.find_nearest_node(self.robot_x, self.robot_y)
        self.current_node_id: str = nearest if nearest else "start"
        self.current_goal_node_id: str = ""
        self.active_path: list = []
        self.current_target_index: int = 0
        self.mission_status: str = "IDLE"

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._navigating_to_node = None

        # QoS for sensor data
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.sub_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )

        self.sub_signs = self.create_subscription(
            SignDetectionArray,
            '/vision/signs',
            self._signs_callback,
            qos_best_effort
        )

        self.sub_obstacles = self.create_subscription(
            SemanticObstacleArray,
            '/vision/semantic_obstacles',
            self._obstacles_callback,
            qos_best_effort
        )

        self.sub_goal_label = self.create_subscription(
            String,
            '/navigation/goal_label',
            self._goal_label_callback,
            10
        )

        # Publications
        self.pub_status = self.create_publisher(
            String,
            '/navigation/mission_status',
            10
        )

        self.pub_active_path = self.create_publisher(
            String,
            '/navigation/active_path',
            10
        )

        # Status loop timer (2 Hz)
        self.timer = self.create_timer(0.5, self._status_loop)

        self.get_logger().info(
            f"DecisionEngineNode initialized with {len(self.graph.nodes)} topological nodes and {len(self.graph.edges)} edges."
        )

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Find nearest topological node
        nearest = self.graph.find_nearest_node(self.robot_x, self.robot_y)
        if nearest:
            dist = math.hypot(self.graph.nodes[nearest].x - self.robot_x, self.graph.nodes[nearest].y - self.robot_y)
            if dist < self.arrival_dist:
                if self.current_node_id != nearest:
                    self.get_logger().info(f"Entered topological node: {nearest} ({self.graph.nodes[nearest].name})")
                    self.current_node_id = nearest

    def _signs_callback(self, msg: SignDetectionArray):
        """Process detected semantic signs and integrate into navigation memory."""
        for sign in msg.signs:
            if sign.confidence < 0.50:
                continue

            # Record sign in memory associated with nearest node
            nearest = self.current_node_id
            self.memory.record_sign(nearest, sign.text, sign.direction, float(sign.confidence))
            self.get_logger().info(
                f"[DECISION SIGN] Observed '{sign.text}' -> '{sign.direction}' at node '{nearest}' (conf: {sign.confidence:.2f})"
            )

            # If current mission target matches sign text, use sign for directional routing
            if self.current_goal_node_id and self.mission_status == "NAVIGATING":
                target_node = self.graph.nodes.get(self.current_goal_node_id)
                if target_node and target_node.semantic_label.upper() == sign.text.upper():
                    self.get_logger().info(
                        f"[DECISION] Sign confirmed route to destination '{sign.text}' via '{sign.direction}'!"
                    )

    def _obstacles_callback(self, msg: SemanticObstacleArray):
        """Detect corridor blockages and trigger dynamic replanning."""
        if self.mission_status != "NAVIGATING" or not self._navigating_to_node:
            return

        # Check if an obstacle is directly in the path toward current target node
        for obs in msg.obstacles:
            # Check obstacle distance
            d = math.hypot(obs.x, obs.y) if hasattr(obs, 'x') and hasattr(obs, 'y') else obs.distance
            if d < self.obs_corridor_dist:
                cls_name = getattr(obs, 'class_name', getattr(obs, 'label', 'obstacle'))
                self.get_logger().warning(
                    f"[CORRIDOR BLOCKED] Detected '{cls_name}' at distance {d:.2f}m blocking edge "
                    f"'{self.current_node_id}' -> '{self._navigating_to_node}'!"
                )
                self._handle_corridor_blocked(self.current_node_id, self._navigating_to_node, cls_name)
                break

    def _handle_corridor_blocked(self, u: str, v: str, obstacle_class: str):
        """Cancel current waypoint navigation, mark edge blocked in memory, and replan."""
        self.get_logger().warning(f"Cancelling Nav2 goal and replanning around blocked edge ({u} -> {v})...")

        # Cancel active Nav2 goal
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        # Mark edge blocked in graph and persistent memory
        self.graph.mark_edge_blocked(u, v, blocked=True)
        self.memory.record_obstacle(u, v, obstacle_class, self.robot_x, self.robot_y)

        # Replan from current node to current destination
        if self.current_goal_node_id:
            new_path = self.graph.get_shortest_path(self.current_node_id, self.current_goal_node_id)
            if new_path and len(new_path) > 1:
                self.get_logger().info(f"Replanned alternate route: {' -> '.join(new_path)}")
                self.active_path = new_path
                self.current_target_index = 1
                self._dispatch_next_waypoint()
            else:
                self.get_logger().error(f"No alternative route found to destination '{self.current_goal_node_id}'!")
                self.mission_status = "ROUTE_UNAVAILABLE"

    def _goal_label_callback(self, msg: String):
        """Accept goal semantic label (e.g. 'HOSPITAL', 'WAREHOUSE', 'EXIT') and plan mission."""
        label = msg.data.strip().upper()
        self.get_logger().info(f"Received mission goal request: '{label}'")

        if label == "EXPLORE":
            self._start_exploration_mission()
            return

        # Lookup destination node by semantic label
        dest_id = self.graph.find_node_by_semantic_label(label)
        if not dest_id:
            # Check if it matches a node id directly
            if label.lower() in self.graph.nodes:
                dest_id = label.lower()
            else:
                self.get_logger().error(f"Unknown destination label: '{label}'")
                self.mission_status = "INVALID_GOAL"
                return

        self.current_goal_node_id = dest_id
        # Plan shortest path from current location
        start_id = self.current_node_id
        path = self.graph.get_shortest_path(start_id, dest_id)

        if not path:
            self.get_logger().error(f"No path found from '{start_id}' to '{dest_id}'!")
            self.mission_status = "PATH_NOT_FOUND"
            return

        self.active_path = path
        self.current_target_index = 1 if len(path) > 1 else 0
        self.mission_status = "NAVIGATING"
        self.get_logger().info(f"Planned mission path to {dest_id}: {' -> '.join(path)}")

        self._dispatch_next_waypoint()

    def _start_exploration_mission(self):
        """Autonomous exploration mode: visit unvisited nodes."""
        unvisited = [nid for nid, node in self.graph.nodes.items()
                     if node.node_type != 'dead_end' and nid != self.current_node_id]
        if not unvisited:
            self.get_logger().info("All exploration targets visited!")
            self.mission_status = "EXPLORATION_COMPLETE"
            return

        target = unvisited[0]
        self._goal_label_callback(String(data=target))

    def _dispatch_next_waypoint(self):
        """Send next topological node as goal to Nav2 via NavigateToPose action."""
        if self.current_target_index >= len(self.active_path):
            self.get_logger().info(f"[MISSION COMPLETE] Arrived at destination: {self.current_goal_node_id}!")
            self.mission_status = "MISSION_COMPLETED"
            self._navigating_to_node = None
            return

        next_nid = self.active_path[self.current_target_index]
        target_node = self.graph.nodes[next_nid]
        self._navigating_to_node = next_nid

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 /navigate_to_pose action server not available!")
            self.mission_status = "NAV2_UNAVAILABLE"
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_node.x
        goal_msg.pose.pose.position.y = target_node.y
        goal_msg.pose.pose.position.z = 0.0

        # Orientation quaternion
        half_theta = target_node.theta / 2.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(half_theta)
        goal_msg.pose.pose.orientation.w = math.cos(half_theta)

        self.get_logger().info(
            f"Dispatching waypoint {self.current_target_index + 1}/{len(self.active_path)}: "
            f"'{next_nid}' ({target_node.x:.2f}, {target_node.y:.2f}) to Nav2..."
        )

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected waypoint goal!")
            self.mission_status = "GOAL_REJECTED"
            return

        self._goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._goal_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        # Optional: log distance remaining
        pass

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Successfully reached waypoint '{self._navigating_to_node}'!")
            # Record successful traversal in memory
            prev_node = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
            self.memory.record_traversal(prev_node, self._navigating_to_node, success=True)

            self.current_node_id = self._navigating_to_node
            self.current_target_index += 1
            self._dispatch_next_waypoint()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning(f"Nav2 aborted navigation to '{self._navigating_to_node}'!")
            # Mark edge blocked and replan
            prev_node = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
            self._handle_corridor_blocked(prev_node, self._navigating_to_node, "Nav2Aborted")
        else:
            self.get_logger().info(f"Nav2 navigation finished with status: {status}")

    def _status_loop(self):
        """Periodic status update published to /navigation/mission_status."""
        status_info = {
            'status': self.mission_status,
            'current_node': self.current_node_id,
            'target_node': self._navigating_to_node,
            'destination': self.current_goal_node_id,
            'active_path': self.active_path,
            'robot_pose': {'x': round(self.robot_x, 2), 'y': round(self.robot_y, 2), 'yaw': round(self.robot_yaw, 2)}
        }
        self.pub_status.publish(String(data=json.dumps(status_info)))

        if self.active_path:
            self.pub_active_path.publish(String(data=','.join(self.active_path)))


def main(args=None):
    rclpy.init(args=args)
    node = DecisionEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
