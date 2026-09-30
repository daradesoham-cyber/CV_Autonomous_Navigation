#!/usr/bin/env python3
"""
Autonomous Navigation Decision Engine Node V2.2 & V2.3.
Features:
- Full Navigation State Machine (IDLE, PLANNING, NAVIGATING, OBSTACLE_DETECTED,
  REPLANNING, DEAD_END, BACKTRACKING, RECOVERY, GOAL_REACHED, FAILED)
- Multi-criteria Route Cost Planning with Configurable YAML Weights
- Candidate Route Alternatives Generation (K-shortest paths)
- Persistent SQLite Journey & Bayesian Segment Reliability Tracking
- Explicit Oscillation Detector (Angular & Linear reversal gating)
- AMCL Teleport Jump Filtering for accurate real distance tracking
- Categorized Replan Reasons Enum (DYNAMIC_OBSTACLE, LOW_CLEARANCE, DEAD_END, etc.)
- Progress Rate & Path Tracking Deviation Monitoring
- Smart Dynamic Obstacle Handling (Yielding vs Replanning)
- Dead-End Intelligence with Coordinates Storage & Safe Backtracking
- Robust Recovery Behaviors (Oscillation, Stalled, Critical Clearance, Nav2 Abort)
- Comprehensive Real-Time Diagnostics & Telemetry Publishing
"""

import os
import sys
import math
import json
import time
import uuid
import yaml
from collections import deque
from typing import Tuple, List, Dict, Optional, Any
import numpy as np

# Ensure Python virtual environment and local package paths are available
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation'
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from autonomous_robot_interfaces.msg import SignDetectionArray, SemanticObstacleArray
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


class ReplanReason:
    """Standardized Replan Reason Enum (V2.2 Section 6)."""
    DYNAMIC_OBSTACLE = "DYNAMIC_OBSTACLE"
    LOW_CLEARANCE = "LOW_CLEARANCE"
    DEAD_END = "DEAD_END"
    NO_PROGRESS = "NO_PROGRESS"
    NAV2_ABORT = "NAV2_ABORT"
    GOAL_INVALID = "GOAL_INVALID"
    ROUTE_BLOCKED = "ROUTE_BLOCKED"
    RECOVERY = "RECOVERY"
    UNKNOWN = "UNKNOWN"


class OscillationDetector:
    """
    Explicit oscillation detector tracking heading/velocity reversals and displacement (V2.2 Section 5).
    Detects LEFT <-> RIGHT or FORWARD <-> BACKWARD oscillations without progress.
    """
    def __init__(self, time_window: float = 5.0, min_reversals: int = 6, max_displacement: float = 0.25):
        self.time_window = time_window
        self.min_reversals = min_reversals
        self.max_displacement = max_displacement
        self.history = deque(maxlen=40)  # (time, x, y, yaw, lin_vel, ang_vel)

    def reset(self):
        self.history.clear()

    def update(self, t: float, x: float, y: float, yaw: float, lin_vel: float, ang_vel: float) -> Tuple[bool, int, int, str]:
        self.history.append((t, x, y, yaw, lin_vel, ang_vel))
        
        # Purge entries older than time_window
        cutoff = t - self.time_window
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

        if len(self.history) < 8:
            return False, 0, 0, ""

        duration = self.history[-1][0] - self.history[0][0]
        if duration < 3.0:
            return False, 0, 0, ""

        # Measure net displacement
        x0, y0 = self.history[0][1], self.history[0][2]
        x1, y1 = self.history[-1][1], self.history[-1][2]
        displacement = math.hypot(x1 - x0, y1 - y0)

        # Count angular velocity reversals (LEFT <-> RIGHT)
        ang_reversals = 0
        last_ang_sign = 0
        for entry in self.history:
            w = entry[5]
            if abs(w) > 0.25:
                sign = 1 if w > 0 else -1
                if last_ang_sign != 0 and sign != last_ang_sign:
                    ang_reversals += 1
                last_ang_sign = sign

        # Count linear velocity reversals (FORWARD <-> BACKWARD)
        lin_reversals = 0
        last_lin_sign = 0
        for entry in self.history:
            v = entry[4]
            if abs(v) > 0.08:
                sign = 1 if v > 0 else -1
                if last_lin_sign != 0 and sign != last_lin_sign:
                    lin_reversals += 1
                last_lin_sign = sign

        if displacement < self.max_displacement:
            if ang_reversals >= self.min_reversals:
                msg = f"Angular oscillation detected: {ang_reversals} reversals in {duration:.1f}s (disp: {displacement:.2f}m)"
                return True, ang_reversals, lin_reversals, msg
            if lin_reversals >= 4:
                msg = f"Linear oscillation detected: {lin_reversals} reversals in {duration:.1f}s (disp: {displacement:.2f}m)"
                return True, ang_reversals, lin_reversals, msg

        return False, ang_reversals, lin_reversals, ""


class DecisionEngineNode(Node):
    """
    Advanced Decision Engine for Autonomous Navigation V2.2 & V2.3.
    """
    def __init__(self):
        super().__init__('decision_engine_node')

        # Parameters
        self.declare_parameter(
            'db_path',
            '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db'
        )
        self.declare_parameter(
            'config_path',
            '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_v2.yaml'
        )
        self.declare_parameter('node_arrival_distance', 0.85)
        self.declare_parameter('obstacle_corridor_distance', 2.0)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', -11.0)
        self.declare_parameter('initial_yaw', 1.57)

        self.db_path = self.get_parameter('db_path').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        self.arrival_dist = self.get_parameter('node_arrival_distance').get_parameter_value().double_value
        self.obs_corridor_dist = self.get_parameter('obstacle_corridor_distance').get_parameter_value().double_value

        # Load YAML configuration for route weights & recovery
        self._load_v2_config()

        # Navigation memory & topological graph
        self.memory = NavigationMemory(self.db_path)
        # V2.6 Phase 1: Clear any stale transient blockages from prior sessions on startup
        unblocked = self.memory.reset_transient_blockages()
        if unblocked > 0:
            self.get_logger().info(f"[NAVIGATION MEMORY] Reset {unblocked} transient blocked edges on startup.")
        self.graph: TopologicalGraph = self.memory.load_graph()
        self.graph.reset_blockages()

        # Robot kinematic state
        self.robot_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.robot_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.robot_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # LiDAR clearances
        self.min_clearance = 10.0
        self.front_clearance = 10.0
        self.left_clearance = 10.0
        self.right_clearance = 10.0
        self.rear_clearance = 10.0

        # Navigation State Machine
        # States: IDLE, PLANNING, NAVIGATING, OBSTACLE_DETECTED, REPLANNING, DEAD_END, BACKTRACKING, RECOVERY, GOAL_REACHED, FAILED
        self.state: str = "IDLE"
        self.mission_status: str = "IDLE"
        self.is_paused: bool = False

        nearest = self.graph.find_nearest_node(self.robot_x, self.robot_y)
        self.current_node_id: str = nearest if nearest else "start"
        self.start_node_id: str = self.current_node_id
        self.current_goal_node_id: str = ""
        self.active_path: list = []
        self.alternative_routes: list = []
        self.current_target_index: int = 0
        self.selected_route_meta: dict = {}

        # Journey analytics and metrics tracking (V2.2 Section 4, 13, 14)
        self.journey_uuid: str = ""
        self.journey_start_time: float = 0.0
        self.planned_distance: float = 0.0
        self.actual_distance: float = 0.0
        self.journey_distance: float = 0.0
        self.replanning_detour_distance: float = 0.0
        self.recovery_distance: float = 0.0
        self.backtracking_distance: float = 0.0
        self.last_pose_sampled = (self.robot_x, self.robot_y)
        self.last_pose_time = time.time()
        self.path_coords: list = []
        self.obstacles_encountered_count: int = 0
        self.replans_count: int = 0
        self.recovery_events_count: int = 0
        self.dead_ends_count: int = 0
        self.journey_min_clearance: float = 10.0

        # Oscillation Detector & Replan Reason Tracking
        self.oscillation_detector = OscillationDetector()
        self.oscillation_events_count: int = 0
        self.last_oscillation_recovery_time: float = 0.0
        self.replan_reasons_list: list = []
        self.last_replan_reason: str = "NONE"

        # Path tracking deviation
        self.current_path_deviation: float = 0.0
        self.max_path_deviation: float = 0.0
        self.avg_path_deviation: float = 0.0
        self.path_deviation_samples: list = []
        self._last_deviation_warn_time: float = 0.0

        # Progress tracking
        self.progress_rate: float = 0.0
        self.prev_dist_to_goal: float = 0.0
        self.last_progress_check_time: float = time.time()
        self.last_progress_time: float = time.time()
        self.last_progress_pose = (self.robot_x, self.robot_y)

        # Dynamic obstacle pause/yield handling (V2.6 Phase 1)
        self.dynamic_obstacle_pause_until: float = 0.0
        self.ttc_yield_active: bool = False

        # Recovery tracking
        self.recovery_in_progress: bool = False
        self.recovery_start_time: float = 0.0
        self.recovery_attempts: int = 0
        self.last_replan_time: float = 0.0

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._navigating_to_node = None

        # QoS Profiles
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
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initialpose_callback,
            10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_best_effort
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
        self.sub_ttc = self.create_subscription(
            Float32,
            '/vision/ttc',
            self._ttc_callback,
            qos_best_effort
        )
        self.latest_ttc = -1.0
        self.last_ttc_time = 0.0
        self.ttc_events_count = 0

        self.sub_goal_label = self.create_subscription(
            String,
            '/navigation/goal_label',
            self._goal_label_callback,
            10
        )
        self.sub_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_pose_callback,
            10
        )
        self.sub_control = self.create_subscription(
            String,
            '/navigation/control',
            self._control_callback,
            10
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/navigation/mission_status', 10)
        self.pub_state = self.create_publisher(String, '/navigation/current_state', 10)
        self.pub_active_path = self.create_publisher(String, '/navigation/active_path', 10)
        self.pub_alt_paths = self.create_publisher(String, '/navigation/alternative_paths', 10)
        self.pub_events = self.create_publisher(String, '/navigation/events', 10)

        # Main monitoring loop timer (4 Hz)
        self.timer = self.create_timer(0.25, self._monitoring_loop)

        self._emit_event("DecisionEngineNode V2.2 ready with Oscillation Detector & Teleport Filtering.", "INFO")
        self.get_logger().info(
            f"DecisionEngineNode V2.2 ready: {len(self.graph.nodes)} nodes, {len(self.graph.edges)} edges."
        )

    def _load_v2_config(self):
        """Loads route cost weights and recovery parameters from YAML config."""
        self.route_weights = {
            'distance_weight': 1.0,
            'obstacle_weight': 3.5,
            'history_failure_weight': 5.0,
            'congestion_weight': 2.0,
            'turn_penalty_weight': 0.8,
            'narrow_corridor_penalty': 2.5,
            'travel_time_weight': 0.5,
            'dead_end_penalty': 10000.0,
            'blocked_edge_penalty': 100000.0
        }
        self.recovery_params = {
            'max_recovery_attempts': 3,
            'reverse_duration_sec': 2.5,
            'stuck_timeout': 5.0,
            'critical_clearance': 0.28,
            'min_clearance_recovery_threshold': 0.22
        }

        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                    if 'route_cost_weights' in cfg:
                        self.route_weights.update(cfg['route_cost_weights'])
                    if 'recovery' in cfg:
                        self.recovery_params.update(cfg['recovery'])
                    if 'navigation' in cfg:
                        nav = cfg['navigation']
                        self.recovery_params['stuck_timeout'] = nav.get('stuck_timeout', 5.0)
                        self.recovery_params['critical_clearance'] = nav.get('critical_distance', 0.28)
                        self.obs_corridor_dist = nav.get('replanning_distance', 2.0)
                self.get_logger().info(f"Loaded V2 configuration from {self.config_path}")
            except Exception as e:
                self.get_logger().warning(f"Could not parse {self.config_path}: {e}")

    def _set_state(self, new_state: str, log_message: str = ""):
        """Transitions state machine and notifies listeners."""
        if self.state != new_state:
            old_state = self.state
            self.state = new_state
            self.mission_status = new_state
            self.pub_state.publish(String(data=new_state))
            if new_state in ["IDLE", "GOAL_REACHED", "FAILED"]:
                self.ttc_yield_active = False
                self.dynamic_obstacle_pause_until = 0.0
            msg = log_message or f"State changed from {old_state} to {new_state}"
            self._emit_event(msg, "INFO")
            self.get_logger().info(f"[STATE] {old_state} -> {new_state}: {msg}")

    def _emit_event(self, message: str, level: str = "INFO"):
        """Publishes timestamped event message for live dashboard log."""
        timestamp_str = time.strftime("%H:%M:%S", time.localtime())
        payload = {
            'timestamp': timestamp_str,
            'message': message,
            'level': level,
            'state': self.state
        }
        self.pub_events.publish(String(data=json.dumps(payload)))

    def _odom_callback(self, msg: Odometry):
        self.linear_velocity = float(msg.twist.twist.linear.x)
        self.angular_velocity = float(msg.twist.twist.angular.z)

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle manual or benchmark initial pose reset."""
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        new_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_x = new_x
        self.robot_y = new_y
        self.robot_yaw = new_yaw
        self.last_pose_sampled = (new_x, new_y)
        self.last_pose_time = time.time()
        self.get_logger().info(f"Updated robot pose from /initialpose to ({new_x:.2f}, {new_y:.2f}, yaw={new_yaw:.2f})")

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """AMCL pose update with particle teleport jump filtering (Section 4 & 5)."""
        now = time.time()
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        new_yaw = math.atan2(siny_cosp, cosy_cosp)

        dt = max(0.01, now - self.last_pose_time)
        self.last_pose_time = now
        d = math.hypot(new_x - self.last_pose_sampled[0], new_y - self.last_pose_sampled[1])

        # Filter AMCL kidnapping jumps:
        # If speed > 1.8 m/s or single step jump > 1.0 m, it's an AMCL particle jump / teleportation!
        is_jump = (d > 1.0) or ((d / dt) > 1.8)
        if is_jump:
            self.get_logger().warning(f"Filtered AMCL pose jump of {d:.2f}m ({d/dt:.1f} m/s) to prevent distance corruption.")
            self.last_pose_sampled = (new_x, new_y)
        else:
            if d >= 0.15:
                self.journey_distance += d
                self.actual_distance += d
                if self.state == "BACKTRACKING":
                    self.backtracking_distance += d
                elif self.state == "RECOVERY":
                    self.recovery_distance += d
                self.last_pose_sampled = (new_x, new_y)
                self.path_coords.append([round(new_x, 2), round(new_y, 2)])

        self.robot_x = new_x
        self.robot_y = new_y
        self.robot_yaw = new_yaw

        # Update path tracking deviation
        self._update_path_deviation()

        # Update oscillation detector
        if self.state == "NAVIGATING" and not self.recovery_in_progress:
            is_osc, h_chg, v_chg, osc_msg = self.oscillation_detector.update(
                now, self.robot_x, self.robot_y, self.robot_yaw,
                self.linear_velocity, self.angular_velocity
            )
            if is_osc and (now - self.last_oscillation_recovery_time) > 8.0:
                self.last_oscillation_recovery_time = now
                self._handle_oscillation_detected(h_chg, v_chg, osc_msg)

        # Progress check for stuck detection
        dist_from_last = math.hypot(self.robot_x - self.last_progress_pose[0], self.robot_y - self.last_progress_pose[1])
        if dist_from_last > 0.15:
            self.last_progress_time = now
            self.last_progress_pose = (self.robot_x, self.robot_y)

        # Identify nearest topological node
        nearest = self.graph.find_nearest_node(self.robot_x, self.robot_y)
        if nearest:
            dist = math.hypot(self.graph.nodes[nearest].x - self.robot_x, self.graph.nodes[nearest].y - self.robot_y)
            if dist < self.arrival_dist and self.current_node_id != nearest:
                node_name = self.graph.nodes[nearest].name
                self.get_logger().info(f"Entered topological node: '{nearest}' ({node_name})")
                self.current_node_id = nearest

    def _update_path_deviation(self):
        """Calculates current, max, and running average perpendicular distance from active route segment (Section 8)."""
        if not self.active_path or self.current_target_index >= len(self.active_path):
            return

        curr_nid = self.active_path[self.current_target_index]
        prev_nid = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
        
        curr_node = self.graph.nodes.get(curr_nid)
        prev_node = self.graph.nodes.get(prev_nid)
        if not curr_node or not prev_node:
            return

        x1, y1 = prev_node.x, prev_node.y
        x2, y2 = curr_node.x, curr_node.y
        px, py = self.robot_x, self.robot_y

        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-4:
            dev = math.hypot(px - x1, py - y1)
        else:
            t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dev = math.hypot(px - proj_x, py - proj_y)

        self.current_path_deviation = round(dev, 2)
        if dev > self.max_path_deviation:
            self.max_path_deviation = round(dev, 2)
        
        self.path_deviation_samples.append(dev)
        if len(self.path_deviation_samples) > 200:
            self.path_deviation_samples.pop(0)
        self.avg_path_deviation = round(float(np.mean(self.path_deviation_samples)), 2)

        if dev > 1.2 and (time.time() - self._last_deviation_warn_time) > 5.0:
            self._last_deviation_warn_time = time.time()
            self._emit_event(f"PATH_DEVIATION: Robot is {dev:.2f}m off planned corridor centerline ({prev_nid} -> {curr_nid})", "WARN")

    def _handle_oscillation_detected(self, heading_changes: int, velocity_reversals: int, msg: str):
        """Oscillation Detection System (Section 5): stops, logs to SQLite, triggers recovery, replans, and continues."""
        self.oscillation_events_count += 1
        self._emit_event(f"OSCILLATION DETECTED! {msg}. Halting and executing pivot recovery.", "WARN")
        self._set_state("RECOVERY", "Oscillation recovery")

        # 1. Stop
        self._stop_robot()
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        # 2. Store oscillation event in SQLite
        self.memory.record_oscillation_event(
            journey_uuid=self.journey_uuid or "standalone",
            x=self.robot_x,
            y=self.robot_y,
            heading_changes=heading_changes,
            velocity_reversals=velocity_reversals,
            time_window=5.0,
            message=msg
        )

        # 3. Trigger recovery: rotate 45 degrees to break alignment trap
        pivot_cmd = Twist()
        pivot_cmd.angular.z = 0.40
        t0 = time.time()
        while (time.time() - t0) < 1.4 and rclpy.ok():
            self.pub_cmd_vel.publish(pivot_cmd)
            time.sleep(0.05)
        self._stop_robot()

        self.oscillation_detector.reset()

        # 4. Recalculate route
        self._execute_replan(ReplanReason.RECOVERY, "Oscillation recovery")

    def _scan_callback(self, msg: LaserScan):
        """Analyzes LiDAR sectors for clearances and dead-end detection."""
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)

        if not np.any(valid_mask):
            return

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        self.min_clearance = float(np.min(valid_ranges))
        if self.min_clearance < self.journey_min_clearance:
            self.journey_min_clearance = self.min_clearance

        # Sector masks: Front (-25° to +25°), Left (+25° to +90°), Right (-90° to -25°), Rear (|angle| >= 135°)
        front_mask = np.abs(valid_angles) < 0.44
        left_mask = (valid_angles >= 0.44) & (valid_angles <= 1.57)
        right_mask = (valid_angles <= -0.44) & (valid_angles >= -1.57)
        rear_mask = np.abs(valid_angles) >= 2.35

        self.front_clearance = float(np.min(valid_ranges[front_mask])) if np.any(front_mask) else 10.0
        self.left_clearance = float(np.min(valid_ranges[left_mask])) if np.any(left_mask) else 10.0
        self.right_clearance = float(np.min(valid_ranges[right_mask])) if np.any(right_mask) else 10.0
        self.rear_clearance = float(np.min(valid_ranges[rear_mask])) if np.any(rear_mask) else 10.0

        # Check for critical clearance ahead during navigation
        if self.state == "NAVIGATING" and not self.recovery_in_progress:
            if self.front_clearance < self.recovery_params['min_clearance_recovery_threshold'] or self.min_clearance < 0.16:
                self._emit_event(f"LiDAR clearance dangerously low (front: {self.front_clearance:.2f}m, min: {self.min_clearance:.2f}m)! Triggering recovery.", "WARN")
                self._trigger_recovery(ReplanReason.LOW_CLEARANCE)

    def _signs_callback(self, msg: SignDetectionArray):
        """
        V2.4/V2.6: Processes semantic signs from YOLO directional_sign class.
        - Records sign observation in topological memory
        - Validates semantic consistency with active destination and topological route
        - Penalizes topological edges if sign indicates CLOSED or BLOCKED
        - Disregards contradictory or irrelevant signs to prevent route hijacking
        - Logs validated sign-driven navigation events
        """
        for sign in msg.signs:
            if sign.confidence < 0.50:
                continue
            nearest = self.current_node_id or "unknown"
            text = sign.text.strip().upper()
            direction = sign.direction.strip().upper()

            try:
                self.memory.record_sign(nearest, text, direction, float(sign.confidence))
            except Exception as e:
                self.get_logger().warning(f"Could not record sign to memory: {e}")

            dest_name = text.replace("CLOSED", "").replace("BLOCKED", "").strip().lower().replace(" ", "_")

            # --- Dynamic sign semantics → navigation edge decision ---
            # 1. "STORAGE CLOSED" or "X CLOSED" or direction "BLOCKED" → penalize the edge to that destination
            if "CLOSED" in text or direction == "BLOCKED":
                if dest_name:
                    # Find destination node id in graph and penalize incoming/outgoing edges
                    target_node = None
                    for node_id in self.graph.nodes:
                        if dest_name in node_id.lower() or node_id.lower() in dest_name:
                            target_node = node_id
                            break
                    if target_node:
                        # Penalize all edges into this node with factor 10x
                        try:
                            for (u, v), edge in self.graph.edges.items():
                                if v == target_node:
                                    edge.cost *= 10.0
                                    edge.obstacle_density += 2.0
                            self.get_logger().warning(
                                f"[SIGN] '{text}' → penalized edges to node '{target_node}' ×10"
                            )
                            self._emit_event(
                                f"Sign '{text}': Penalized route to '{target_node}' (edges ×10)",
                                "WARN"
                            )
                            # If the closed node affects our active route, trigger replanning
                            if self.active_path and any(target_node == n for n in self.active_path[self.current_target_index:]):
                                self._emit_event(
                                    f"Active path traverses closed node '{target_node}'! Triggering dynamic replan.",
                                    "WARN"
                                )
                                self._trigger_recovery(ReplanReason.OBSTACLE_BLOCKED)
                        except Exception as e:
                            self.get_logger().debug(f"Sign edge penalty failed: {e}")

            # 2. Destination & Route Consistency Gating for Standard Directional Signs
            elif self.current_goal_node_id:
                current_goal = str(self.current_goal_node_id).lower()
                target_waypoint = str(self._navigating_to_node).lower() if self._navigating_to_node else ""
                remaining_path = [str(n).lower() for n in self.active_path[self.current_target_index:]] if self.active_path else []

                # Is this sign relevant to our active destination or along our topological path?
                is_goal_sign = bool(dest_name and (dest_name in current_goal or current_goal in dest_name))
                is_path_sign = bool(dest_name and (any(dest_name in rn or rn in dest_name for rn in remaining_path)))

                if is_goal_sign or is_path_sign:
                    self._emit_event(
                        f"Sign CONFIRMS route: '{text}' -> '{direction}' toward target [{self._navigating_to_node}] (goal: [{self.current_goal_node_id}])",
                        "INFO"
                    )
                else:
                    # Inconsistent or irrelevant sign for the current destination:
                    # Preserving validated global route — DO NOT alter path or confuse decision engine.
                    self.get_logger().info(
                        f"[SIGN_GATING] Sign '{text}' -> '{direction}' observed near [{nearest}], but destination '{dest_name}' does not match current goal [{self.current_goal_node_id}]. Preserving validated route."
                    )
                    self._emit_event(
                        f"Sign observed: '{text}' -> '{direction}' (destination '{dest_name}' not on active route toward [{self.current_goal_node_id}]) — preserving global route",
                        "INFO"
                    )
            else:
                self._emit_event(
                    f"Sign detected: '{text}' -> '{direction}' (conf={sign.confidence:.2f}) near [{nearest}]",
                    "INFO"
                )

    def _ttc_callback(self, msg: Float32):
        """
        V2.4: Processes Time-To-Collision (TTC) from Camera-LiDAR fusion node.
        TTC is used strictly as a proactive safety signal supplementing Nav2 and LiDAR logic.
        - Valid low TTC (< 1.8s) -> early proactive pause/yield to dynamic obstacle
        - Stale TTC (> 1.5s old) -> ignored
        - Invalid TTC (<= 0 or infinite) -> ignored
        """
        now = time.time()
        ttc = float(msg.data)
        self.latest_ttc = ttc
        self.last_ttc_time = now

        if self.state not in ["NAVIGATING", "PLANNING"] or not self._navigating_to_node:
            return

        # Threshold: 1.8 seconds (Nav2 fast-stop is 2.0s; stopping at 1.8s prevents abrupt costmap lockup)
        if 0.0 < ttc < 1.8:
            if not self.ttc_yield_active or now >= self.dynamic_obstacle_pause_until:
                # Transition: NORMAL -> TTC YIELD
                self.ttc_yield_active = True
                self.ttc_events_count += 1
                self.dynamic_obstacle_pause_until = now + 1.8
                self._stop_robot()
                self.get_logger().warning(
                    f"[TTC SAFETY] Approaching dynamic obstacle collision in {ttc:.2f}s! Initiating safety yield (1.8s)."
                )
                self._emit_event(
                    f"TTC ALERT: Collision risk in {ttc:.2f}s! Pausing to yield to dynamic obstacle.",
                    "WARN"
                )
            else:
                # Transition: TTC YIELD -> TTC remains unsafe -> continue safe waiting
                self.dynamic_obstacle_pause_until = max(self.dynamic_obstacle_pause_until, now + 1.8)
                self._stop_robot()

    def _obstacles_callback(self, msg: SemanticObstacleArray):
        """Processes dynamic obstacles with motion classification and smart replanning (Section 9)."""
        if self.state not in ["NAVIGATING", "PLANNING"] or not self._navigating_to_node:
            return

        now = time.time()
        if (now - self.last_replan_time) < 4.0:
            return

        for obs in msg.obstacles:
            d = math.hypot(obs.x, obs.y) if (obs.x != 0.0 or obs.y != 0.0) else obs.distance
            dir_str = getattr(obs, 'direction', '')
            cls_name = obs.class_name.lower()

            # Parse motion classification
            motion_state = "UNKNOWN"
            if "MOVING_AWAY" in dir_str:
                motion_state = "MOVING_AWAY"
            elif "MOVING_TOWARDS" in dir_str:
                motion_state = "MOVING_TOWARDS"
            elif "CROSSING" in dir_str:
                motion_state = "CROSSING"
            elif "OUTSIDE_PATH" in dir_str:
                motion_state = "OUTSIDE_PATH"
            elif "STATIC" in dir_str:
                motion_state = "STATIC"

            # 1. OBJECT OUTSIDE PATH: ignore
            if motion_state == "OUTSIDE_PATH" or abs(obs.y) > 0.85:
                continue

            # 2. OBJECT MOVING AWAY: ignore (clearing corridor)
            if motion_state == "MOVING_AWAY":
                continue

            # 3. Dynamic or stationary obstacle in forward corridor:
            if d < self.obs_corridor_dist and abs(obs.y) < 0.50:
                # If moving towards robot or crossing, check if it's dynamic
                if obs.is_dynamic or motion_state in ["MOVING_TOWARDS", "CROSSING"]:
                    # Wait briefly before blacklisting the whole corridor edge
                    if self.dynamic_obstacle_pause_until == 0.0:
                        self.dynamic_obstacle_pause_until = now + 2.0
                        self._stop_robot()
                        self._emit_event(f"Dynamic obstacle '{cls_name.upper()}' approaching path ({d:.2f}m, {motion_state}). Pausing to yield.", "INFO")
                        return
                    elif now < self.dynamic_obstacle_pause_until:
                        # Still yielding
                        self._stop_robot()
                        return
                    else:
                        # Still blocking after wait
                        self.dynamic_obstacle_pause_until = 0.0
                
                # Path genuinely blocked -> initiate replanning
                self.last_replan_time = now
                self.obstacles_encountered_count += 1
                self._emit_event(
                    f"Obstacle '{cls_name.upper()}' blocking corridor ({d:.2f}m, {motion_state})! Initiating replan.",
                    "WARN"
                )
                self._set_state("OBSTACLE_DETECTED", f"Blocked by {cls_name.upper()} at {d:.2f}m")
                self._handle_corridor_blocked(self.current_node_id, self._navigating_to_node, cls_name)
                break

    def _handle_corridor_blocked(self, u: str, v: str, obstacle_class: str):
        """Cancels active waypoint navigation, marks edge blocked, and recalculates alternative route."""
        # Stop robot safely
        self._stop_robot()
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        # Update graph and persistent SQLite database
        self.graph.mark_edge_blocked(u, v, blocked=True)
        self.memory.record_obstacle(u, v, obstacle_class, self.robot_x, self.robot_y)
        self.memory.update_segment_metrics(
            u, v, success=False,
            clearance=self.min_clearance,
            obstacle_encountered=True,
            replan_triggered=True
        )

        self._execute_replan(ReplanReason.DYNAMIC_OBSTACLE, f"Blocked edge ({u} -> {v}) by {obstacle_class}")

    def _execute_replan(self, reason: str, details: str = ""):
        """Executes centralized replanning with reason classification and detour metrics (Section 6)."""
        self.replans_count += 1
        self.last_replan_reason = reason
        self.replan_reasons_list.append(reason)
        self._set_state("REPLANNING", f"Replan triggered ({reason}): {details}")

        prev_node = self.current_node_id
        target_node = self._navigating_to_node or (self.active_path[self.current_target_index] if self.active_path and self.current_target_index < len(self.active_path) else "unknown")
        
        # Log to SQLite replan_events table
        self.memory.record_replan_event(
            journey_uuid=self.journey_uuid or "standalone",
            x=self.robot_x,
            y=self.robot_y,
            from_node=prev_node,
            to_node=target_node,
            replan_reason=reason,
            trigger_details=details
        )

        # Compute K-shortest candidate alternative routes
        routes = self.graph.find_alternative_routes(
            self.current_node_id,
            self.current_goal_node_id,
            k=3,
            weights=self.route_weights
        )
        self.alternative_routes = routes
        self._publish_alternative_routes(routes)

        if routes:
            best = routes[0]
            old_plan_dist = self.planned_distance
            new_plan_dist = best['distance']
            if new_plan_dist > old_plan_dist:
                self.replanning_detour_distance += (new_plan_dist - old_plan_dist)

            self.active_path = best['path']
            self.selected_route_meta = best
            self.current_target_index = 1 if len(self.active_path) > 1 else 0
            self._emit_event(
                f"Selected alternative route '{best['route_id']}' (Cost: {best['total_cost']:.1f}, Dist: {best['distance']:.1f}m, Reason: {reason})",
                "INFO"
            )
            self._set_state("NAVIGATING", f"Engaging alternative route: {' -> '.join(self.active_path)}")
            self._dispatch_next_waypoint()
        else:
            self._emit_event(
                f"No alternative detour for edge ({prev_node} -> {target_node}). Re-evaluating with local avoidance.",
                "WARN"
            )
            self.graph.mark_edge_blocked(prev_node, target_node, blocked=False)
            self._set_state("NAVIGATING", f"Proceeding via local avoidance along {prev_node} -> {target_node}")
            self._dispatch_next_waypoint()

    def _goal_label_callback(self, msg: String):
        """Handles goal request by semantic label (e.g., 'ROOM A', 'STORAGE', 'HOSPITAL')."""
        label = msg.data.strip().upper()
        self._emit_event(f"Mission goal requested: '{label}'", "INFO")

        if label == "EXPLORE":
            self._start_exploration_mission()
            return

        dest_id = self.graph.find_node_by_semantic_label(label)
        if not dest_id:
            if label.lower() in self.graph.nodes:
                dest_id = label.lower()
            else:
                self.get_logger().error(f"Unknown destination label: '{label}'")
                self._emit_event(f"Unknown destination label: '{label}'", "ERROR")
                self._set_state("FAILED", f"Unknown destination: {label}")
                return

        self._start_mission_to_destination(dest_id)

    def _goal_pose_callback(self, msg: PoseStamped):
        """Handles RViz 2D Goal Pose click by snapping to nearest topological node."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        nearest = self.graph.find_nearest_node(gx, gy)
        if not nearest:
            self.get_logger().error("No topological node near clicked goal pose!")
            return
        dest_name = self.graph.nodes[nearest].name
        self._emit_event(f"RViz 2D Goal Pose received at ({gx:.1f}, {gy:.1f}) -> Snapped to '{nearest}' ({dest_name})", "INFO")
        self._start_mission_to_destination(nearest)

    def _control_callback(self, msg: String):
        """Dashboard controls: 'PAUSE', 'RESUME', 'ABORT'."""
        cmd = msg.data.strip().upper()
        if cmd == "PAUSE":
            self.is_paused = True
            self._stop_robot()
            self._emit_event("Mission PAUSED by user.", "WARN")
            self._set_state("PAUSED", "Mission paused")
        elif cmd == "RESUME":
            if self.is_paused:
                self.is_paused = False
                self._emit_event("Mission RESUMED by user.", "INFO")
                self._set_state("NAVIGATING", "Resuming mission")
                self._dispatch_next_waypoint()
        elif cmd == "ABORT":
            self.is_paused = False
            self._stop_robot()
            if self._goal_handle:
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
            self._emit_event("Mission ABORTED by user.", "WARN")
            self._record_journey_completion(success=False, failure_reason="INTERRUPTED")
            self._set_state("IDLE", "Mission interrupted by user")

    def _start_mission_to_destination(self, dest_id: str):
        """Initializes state, evaluates candidate routes, and dispatches first waypoint."""
        # V2.6 Phase 1: Clear transient runtime blockages before planning new mission.
        # Permanent topology and learned segment metrics are preserved.
        unblocked = self.memory.reset_transient_blockages()
        self.graph = self.memory.load_graph()
        self.graph.reset_blockages()
        if unblocked > 0:
            self.get_logger().info(
                f"[NAVIGATION MEMORY] Cleared {unblocked} transient edge blockages for new mission to '{dest_id}'."
            )
        self.ttc_yield_active = False
        self.dynamic_obstacle_pause_until = 0.0
        nearest = self.graph.find_nearest_node(self.robot_x, self.robot_y)
        if nearest:
            self.current_node_id = nearest
        self.start_node_id = self.current_node_id
        self.current_goal_node_id = dest_id
        self.journey_uuid = str(uuid.uuid4())[:8]
        self.journey_start_time = time.time()
        self.planned_distance = 0.0
        self.actual_distance = 0.0
        self.journey_distance = 0.0
        self.replanning_detour_distance = 0.0
        self.recovery_distance = 0.0
        self.backtracking_distance = 0.0
        self.last_pose_sampled = (self.robot_x, self.robot_y)
        self.last_progress_time = time.time()
        self.last_progress_pose = (self.robot_x, self.robot_y)
        self.path_coords = [[round(self.robot_x, 2), round(self.robot_y, 2)]]
        self.obstacles_encountered_count = 0
        self.replans_count = 0
        self.replan_reasons_list = []
        self.last_replan_reason = "NONE"
        self.oscillation_events_count = 0
        self.recovery_events_count = 0
        self.dead_ends_count = 0
        self.path_deviation_samples = []
        self.current_path_deviation = 0.0
        self.max_path_deviation = 0.0
        self.avg_path_deviation = 0.0
        self.journey_min_clearance = self.min_clearance
        self.is_paused = False
        self.oscillation_detector.reset()

        self._set_state("PLANNING", f"Evaluating route options to '{dest_id}'")

        # Compute K-shortest candidate alternative routes
        routes = self.graph.find_alternative_routes(
            self.start_node_id,
            dest_id,
            k=3,
            weights=self.route_weights
        )
        self.alternative_routes = routes
        self._publish_alternative_routes(routes)

        if not routes:
            self._emit_event(f"No navigable route found from '{self.start_node_id}' to '{dest_id}'!", "ERROR")
            self._set_state("FAILED", "Path not found")
            return

        best_route = routes[0]
        self.active_path = best_route['path']
        self.selected_route_meta = best_route
        self.planned_distance = best_route['distance']
        self.current_target_index = 1 if len(self.active_path) > 1 else 0

        self._emit_event(
            f"Selected {best_route['route_id']} with total cost {best_route['total_cost']:.1f} "
            f"({best_route['distance']:.1f}m, {len(routes)} alternatives evaluated).",
            "INFO"
        )
        self._set_state("NAVIGATING", f"Traveling via {' -> '.join(self.active_path)}")
        self._dispatch_next_waypoint()

    def _start_exploration_mission(self):
        """Autonomous exploration mode: visit unvisited nodes."""
        unvisited = [nid for nid, node in self.graph.nodes.items()
                     if node.node_type not in ['dead_end', 'start'] and nid != self.current_node_id]
        if not unvisited:
            self._emit_event("All exploration targets visited!", "INFO")
            self._set_state("IDLE", "Exploration complete")
            return
        target = unvisited[0]
        self._start_mission_to_destination(target)

    def _dispatch_next_waypoint(self, seamless: bool = False):
        """Dispatches next topological node to Nav2 via NavigateToPose action."""
        if self.is_paused:
            return

        if self.current_target_index >= len(self.active_path):
            self._emit_event(f"Arrived at final destination '{self.current_goal_node_id}'!", "SUCCESS")
            self._record_journey_completion(success=True)
            self._set_state("GOAL_REACHED", f"Arrived at {self.current_goal_node_id}")
            self._navigating_to_node = None
            return

        next_nid = self.active_path[self.current_target_index]
        target_node = self.graph.nodes.get(next_nid)
        if not target_node:
            self.get_logger().error(f"Node '{next_nid}' not found in topological graph!")
            return

        # Check for dead-end condition before entering
        if target_node.node_type == 'dead_end':
            self._handle_dead_end_detected(next_nid)
            return

        self._navigating_to_node = next_nid

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 /navigate_to_pose action server unavailable!")
            self._set_state("FAILED", "Nav2 action server unavailable")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_node.x
        goal_msg.pose.pose.position.y = target_node.y
        goal_msg.pose.pose.position.z = 0.0

        half_theta = target_node.theta / 2.0
        goal_msg.pose.pose.orientation.z = math.sin(half_theta)
        goal_msg.pose.pose.orientation.w = math.cos(half_theta)

        if not seamless and self._goal_handle:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
            time.sleep(0.10)

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda f, nid=next_nid, idx=self.current_target_index: self._goal_response_callback(f, nid, idx)
        )

    def _goal_response_callback(self, future, target_node_id: str, target_index: int):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f"Nav2 temporarily busy for goal '{target_node_id}'. Retrying in 0.25s...")
            time.sleep(0.25)
            if self.state == "NAVIGATING" and self.current_target_index == target_index:
                self._dispatch_next_waypoint()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, gh=goal_handle, nid=target_node_id, idx=target_index: self._goal_result_callback(f, gh, nid, idx)
        )

    def _goal_result_callback(self, future, goal_handle, target_node_id: str, target_index: int):
        # Ignore stale callbacks from previously cancelled or superseded goals
        if goal_handle != self._goal_handle:
            return
        if self.state != "NAVIGATING":
            return
        if self.current_target_index != target_index:
            return
        if not target_node_id:
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            prev_node = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
            self.memory.record_traversal(prev_node, target_node_id, success=True)
            self.memory.update_segment_metrics(
                prev_node, target_node_id,
                success=True,
                clearance=self.min_clearance
            )
            self.current_node_id = target_node_id
            self.current_target_index += 1
            self.last_progress_time = time.time()
            self._dispatch_next_waypoint()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning(f"Nav2 aborted navigation toward '{target_node_id}'!")
            self._trigger_recovery(ReplanReason.NAV2_ABORT)

    def _handle_dead_end_detected(self, dead_end_node_id: str):
        """Dead-End Intelligence (Section 7): Stop safely, store in SQLite, backtrack, and replan."""
        self.dead_ends_count += 1
        node = self.graph.nodes[dead_end_node_id]
        self._emit_event(f"DEAD-END DETECTED at '{dead_end_node_id}' ({node.x:.2f}, {node.y:.2f})!", "WARN")
        self._set_state("DEAD_END", f"Dead end encountered: {dead_end_node_id}")

        # 1. Stop safely
        self._stop_robot()
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        # 2 & 3. Record dead end in SQLite memory
        self.memory.record_dead_end(dead_end_node_id, node.x, node.y)

        # 4. Mark segment heavily penalized / undesirable
        prev_node = self.current_node_id
        self.graph.mark_edge_dead_end(prev_node, dead_end_node_id, dead_end=True)

        # 5. Backtrack safely
        self._set_state("BACKTRACKING", f"Safely reversing away from {dead_end_node_id}")
        self._execute_backtrack()

        # 6 & 7. Recalculate and select alternative route
        self._execute_replan(ReplanReason.DEAD_END, f"Backtracked from dead end {dead_end_node_id}")

    def _execute_backtrack(self):
        """Sends brief reverse cmd_vel commands to back out of a tight spot with rear LiDAR safety check."""
        if self.rear_clearance < 0.35:
            self._emit_event(
                f"[SAFETY] Reverse aborted: rear obstacle detected at {self.rear_clearance:.2f}m. Executing safe pivot.",
                "WARN"
            )
            pivot_cmd = Twist()
            pivot_cmd.angular.z = 0.35
            t0 = time.time()
            while (time.time() - t0) < 1.2 and rclpy.ok():
                self.pub_cmd_vel.publish(pivot_cmd)
                time.sleep(0.05)
            self._stop_robot()
            return

        t_start = time.time()
        rev_cmd = Twist()
        rev_cmd.linear.x = -0.18
        while (time.time() - t_start) < 2.0 and rclpy.ok():
            if self.rear_clearance < 0.30:
                self._emit_event(f"[SAFETY] Reverse halted early: rear clearance reached {self.rear_clearance:.2f}m", "WARN")
                break
            self.pub_cmd_vel.publish(rev_cmd)
            time.sleep(0.05)
        self._stop_robot()

    def _trigger_recovery(self, reason: str):
        """Robust Recovery System (Section 8): Detects stuck/abort conditions, stops, reverses, and replans."""
        if self.recovery_in_progress:
            return

        self.recovery_in_progress = True
        self.recovery_events_count += 1
        self.recovery_attempts += 1
        self._emit_event(f"RECOVERY TRIGGERED ({reason})! Attempt {self.recovery_attempts}.", "WARN")
        self._set_state("RECOVERY", f"Executing recovery: {reason}")

        # Stop robot
        self._stop_robot()
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        # Log recovery event to SQLite
        self.memory.record_recovery_event(
            journey_uuid=self.journey_uuid or "standalone",
            x=self.robot_x,
            y=self.robot_y,
            recovery_type=reason,
            actions_taken="STOP -> REVERSE 1.5s -> REORIENT -> REPLAN",
            success=True,
            message=f"Clearance: {self.min_clearance:.2f}m"
        )

        # Backtrack/reverse safely
        self._execute_backtrack()

        # Replan route
        self.recovery_in_progress = False
        self.last_progress_time = time.time()
        self._execute_replan(reason, f"Recovery attempt {self.recovery_attempts}")

    def _stop_robot(self):
        """Immediately commands zero velocity."""
        stop_cmd = Twist()
        self.pub_cmd_vel.publish(stop_cmd)

    def _record_journey_completion(self, success: bool, failure_reason: str = ""):
        """Stores comprehensive journey record into SQLite (Section 5, 13, 14)."""
        if not self.journey_uuid:
            return

        travel_time = max(1.0, time.time() - self.journey_start_time)
        avg_speed = round(self.actual_distance / travel_time, 2)
        route_name = self.selected_route_meta.get('route_id', 'Direct')

        path_eff = (self.planned_distance / self.actual_distance * 100.0) if self.actual_distance > 0.05 else 100.0
        path_eff = min(100.0, max(0.0, path_eff))

        try:
            self.memory.record_journey(
                journey_id=self.journey_uuid,
                start_node=self.start_node_id,
                goal_node=self.current_goal_node_id,
                path_coords=self.path_coords,
                distance_travelled=round(self.actual_distance, 2),
                travel_time=round(travel_time, 1),
                average_speed=avg_speed,
                min_lidar_clearance=round(self.journey_min_clearance, 2),
                obstacles_encountered=self.obstacles_encountered_count,
                replans_count=self.replans_count,
                recovery_events_count=self.recovery_events_count,
                dead_ends_count=self.dead_ends_count,
                route_selected=route_name,
                route_alternatives_considered=json.dumps(self.alternative_routes),
                success=success,
                failure_reason=failure_reason,
                planned_distance=round(self.planned_distance, 2),
                actual_distance=round(self.actual_distance, 2),
                path_efficiency=round(path_eff, 1),
                oscillation_events_count=self.oscillation_events_count,
                backtracking_distance=round(self.backtracking_distance, 2),
                recovery_distance=round(self.recovery_distance, 2),
                replan_reasons=",".join(self.replan_reasons_list),
                average_path_deviation=round(self.avg_path_deviation, 2),
                arrival_x=round(self.robot_x, 2),
                arrival_y=round(self.robot_y, 2)
            )
            self._emit_event(
                f"Saved Journey #{self.journey_uuid} to SQLite memory (Plan: {self.planned_distance:.1f}m, "
                f"Actual: {self.actual_distance:.1f}m, Eff: {path_eff:.1f}%, Time: {travel_time:.1f}s, Success: {success}).",
                "SUCCESS" if success else "WARN"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to record journey in SQLite: {e}")

    def _publish_alternative_routes(self, routes: list):
        """Publishes candidate alternative routes for the dashboard."""
        self.pub_alt_paths.publish(String(data=json.dumps(routes)))

    def _monitoring_loop(self):
        """Checks for stuck conditions, updates metrics, and publishes status telemetry (Section 15, 16)."""
        now = time.time()

        # Check for stuck condition when actively navigating
        if self.state == "NAVIGATING" and not self.is_paused and not self.recovery_in_progress:
            # V2.6 Phase 1: Suspend stuck timeout during intentional safety yields (TTC or dynamic obstacle pause)
            if now < self.dynamic_obstacle_pause_until:
                self._stop_robot()
                self.last_progress_time = now
                self.last_progress_pose = (self.robot_x, self.robot_y)
            else:
                # Transition: TTC YIELD -> TTC CLEAR -> NORMAL
                if self.ttc_yield_active:
                    self.ttc_yield_active = False
                    self.dynamic_obstacle_pause_until = 0.0
                    self.last_progress_time = now
                    self.last_progress_pose = (self.robot_x, self.robot_y)
                    self.get_logger().info("[TTC SAFETY] Dynamic obstacle cleared corridor. Resuming normal navigation.")
                    self._emit_event("TTC CLEAR: Obstacle cleared corridor. Resuming navigation.", "INFO")

                # Genuine stuck detection (mechanical stall, wall collision, etc.)
                if (now - self.last_progress_time) > self.recovery_params['stuck_timeout']:
                    dist_moved = math.hypot(self.robot_x - self.last_progress_pose[0], self.robot_y - self.last_progress_pose[1])
                    if dist_moved < 0.10:
                        self._emit_event(f"Robot stuck near ({self.robot_x:.2f}, {self.robot_y:.2f}) for > {self.recovery_params['stuck_timeout']}s!", "WARN")
                        self._trigger_recovery(ReplanReason.NO_PROGRESS)

        # Check for destination arrival or waypoint arrival threshold
        if self.state == "NAVIGATING" and self.active_path and self.current_target_index < len(self.active_path):
            target_nid = self.active_path[self.current_target_index]
            target_node = self.graph.nodes.get(target_nid)
            is_final_goal = (self.current_target_index == len(self.active_path) - 1)
            if target_node:
                dist_to_target = math.hypot(target_node.x - self.robot_x, target_node.y - self.robot_y)
                arrival_dist = 0.50 if is_final_goal else 0.75
                if dist_to_target <= arrival_dist:
                    if is_final_goal:
                        self._stop_robot()
                        if self._goal_handle:
                            self._goal_handle.cancel_goal_async()
                            self._goal_handle = None
                        prev_node = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
                        self.memory.record_traversal(prev_node, target_nid, success=True)
                        self._emit_event(f"Arrived at final destination '{self.current_goal_node_id}'!", "SUCCESS")
                        self._record_journey_completion(success=True)
                        self._set_state("GOAL_REACHED", f"Arrived at {self.current_goal_node_id}")
                        self._navigating_to_node = None
                    else:
                        prev_node = self.active_path[self.current_target_index - 1] if self.current_target_index > 0 else self.current_node_id
                        self.memory.record_traversal(prev_node, target_nid, success=True)
                        self.current_node_id = target_nid
                        self.current_target_index += 1
                        self.last_progress_time = time.time()
                        self._dispatch_next_waypoint(seamless=True)

        # Estimate distance remaining to goal
        dist_remaining = 0.0
        if self.active_path and self.current_target_index < len(self.active_path):
            curr_target = self.graph.nodes.get(self.active_path[self.current_target_index])
            if curr_target:
                dist_remaining += math.hypot(curr_target.x - self.robot_x, curr_target.y - self.robot_y)
            for i in range(self.current_target_index, len(self.active_path) - 1):
                u, v = self.active_path[i], self.active_path[i + 1]
                edge = self.graph.get_edge(u, v)
                if edge:
                    dist_remaining += edge.distance

        eta_sec = round(dist_remaining / 0.45, 1) if dist_remaining > 0 else 0.0

        # Calculate progress rate
        dt_prog = now - self.last_progress_check_time
        if dt_prog >= 0.5:
            if self.prev_dist_to_goal > 0.0:
                self.progress_rate = max(0.0, (self.prev_dist_to_goal - dist_remaining) / dt_prog)
            self.prev_dist_to_goal = dist_remaining
            self.last_progress_check_time = now

        # Calculate Path Efficiency %
        path_eff = (self.planned_distance / self.actual_distance * 100.0) if self.actual_distance > 0.05 else 100.0
        path_eff = min(100.0, max(0.0, path_eff))

        # Calculate Progress %
        progress_pct = 0.0
        if self.planned_distance > 0.0:
            progress_pct = min(100.0, max(0.0, round((1.0 - (dist_remaining / self.planned_distance)) * 100.0, 1)))

        # Build comprehensive real telemetry payload for the dashboard
        status_info = {
            'state': self.state,
            'mission_status': self.mission_status,
            'is_paused': self.is_paused,
            'robot_pose': {
                'x': round(self.robot_x, 2),
                'y': round(self.robot_y, 2),
                'yaw': round(self.robot_yaw, 2)
            },
            'linear_velocity': round(self.linear_velocity, 2),
            'angular_velocity': round(self.angular_velocity, 2),
            'current_node': self.current_node_id,
            'target_node': self._navigating_to_node,
            'destination': self.current_goal_node_id,
            'start_node': self.start_node_id,
            'distance_remaining': round(dist_remaining, 2),
            'estimated_time_sec': eta_sec,
            'planned_distance': round(self.planned_distance, 2),
            'actual_distance': round(self.actual_distance, 2),
            'distance_travelled': round(self.actual_distance, 2),
            'path_efficiency_pct': round(path_eff, 1),
            'progress_pct': progress_pct,
            'progress_rate': round(self.progress_rate, 2),
            'path_deviation': round(self.current_path_deviation, 2),
            'max_path_deviation': round(self.max_path_deviation, 2),
            'avg_path_deviation': round(self.avg_path_deviation, 2),
            'backtracking_distance': round(self.backtracking_distance, 2),
            'recovery_distance': round(self.recovery_distance, 2),
            'replanning_detours': round(self.replanning_detour_distance, 2),
            'oscillation_count': self.oscillation_events_count,
            'journey_duration_sec': round(now - self.journey_start_time, 1) if self.journey_start_time > 0 else 0.0,
            'clearances': {
                'min': round(self.min_clearance, 2),
                'front': round(self.front_clearance, 2),
                'left': round(self.left_clearance, 2),
                'right': round(self.right_clearance, 2),
                'rear': round(self.rear_clearance, 2)
            },
            'route_cost': self.selected_route_meta.get('total_cost', 0.0),
            'obstacle_density': self.selected_route_meta.get('obstacle_density', 0.0),
            'reliability_pct': self.selected_route_meta.get('reliability_pct', 100.0),
            'cost_breakdown': self.selected_route_meta.get('cost_breakdown', {}),
            'replans_count': self.replans_count,
            'last_replan_reason': self.last_replan_reason,
            'replan_reasons': self.replan_reasons_list,
            'recovery_count': self.recovery_events_count,
            'dead_ends_count': self.dead_ends_count,
            'ttc': round(self.latest_ttc, 2) if (now - self.last_ttc_time) <= 1.5 else -1.0,
            'ttc_events_count': self.ttc_events_count,
            'active_path': self.active_path,
            'alternatives_count': len(self.alternative_routes)
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
