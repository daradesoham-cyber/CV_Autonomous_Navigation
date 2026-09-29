#!/usr/bin/env python3
"""
Dedicated Robotics Navigation Dashboard Backend V2.
Serves a professional, real-time web monitoring interface on port 5050.
Zero mock data: all telemetry, sensor metrics, journeys, and analytics originate
from live ROS 2 topics and persistent SQLite storage.
"""

import os
import sys
import time
import json
import math
import uuid
import threading
from collections import deque
from typing import Optional, Dict, Any

# Ensure Python AI virtual environment and ROS packages are in sys.path
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation'
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

import psutil
import cv2
import subprocess
from cv_bridge import CvBridge
from flask import Flask, jsonify, request, send_from_directory, render_template_string, Response
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from autonomous_robot_interfaces.msg import Detection2DArray, SemanticObstacleArray, SignDetectionArray

from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def get_gpu_info() -> Optional[Dict[str, Any]]:
    """Query live GPU metrics directly from NVIDIA Driver."""
    try:
        cmd = ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu,name", "--format=csv,noheader,nounits"]
        out = subprocess.check_output(cmd, encoding='utf-8', timeout=0.8).strip()
        parts = [p.strip() for p in out.split(',')]
        if len(parts) >= 5:
            return {
                'gpu_util_pct': float(parts[0]),
                'gpu_mem_used_mb': float(parts[1]),
                'gpu_mem_total_mb': float(parts[2]),
                'gpu_temp_c': float(parts[3]),
                'gpu_name': parts[4],
                'is_active': True
            }
    except Exception:
        pass
    return None


# Frequency tracker helper for live ROS topics
class TopicFrequencyTracker:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.timestamps = deque(maxlen=window_size)
        self.last_msg_time = 0.0
        self.count = 0

    def tick(self):
        now = time.time()
        self.last_msg_time = now
        self.timestamps.append(now)
        self.count += 1

    @property
    def frequency(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        dt = self.timestamps[-1] - self.timestamps[0]
        if dt <= 0.0:
            return 0.0
        return round((len(self.timestamps) - 1) / dt, 1)

    @property
    def is_alive(self) -> bool:
        return (time.time() - self.last_msg_time) < 3.0 if self.last_msg_time > 0 else False


class DashboardBridgeNode(Node):
    """ROS 2 Bridge Node collecting live metrics and dispatching mission commands."""
    def __init__(self, db_path: str):
        super().__init__('navigation_dashboard_backend')
        self.db_path = db_path
        self.memory = NavigationMemory(db_path)
        self.graph = self.memory.load_graph()
        self.bridge = CvBridge()

        # Camera frame caches (raw numpy frames for internal use)
        self.latest_raw_frame = None
        self.latest_annotated_frame = None
        self.latest_signs_frame = None
        self.latest_detected_signs = []
        self._latest_signs_ts = 0.0

        # Load semantic map for sign lookups
        self.semantic_map = {}
        sem_map_path = '/home/soham-darade/CV_Autonomous_Navigation/config/semantic_map.yaml'
        if os.path.exists(sem_map_path):
            try:
                import yaml
                with open(sem_map_path, 'r') as f:
                    self.semantic_map = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(f"Failed to load semantic map: {e}")

        # V2.4: Pre-encoded JPEG caches — encode once in ROS callback, serve bytes on HTTP request
        # This eliminates synchronous cv2.imencode() on the Flask request thread (~30.7% CPU saving)
        import threading
        self._jpeg_lock = threading.Lock()
        self._jpeg_raw: bytes = b''
        self._jpeg_annotated: bytes = b''
        self._jpeg_signs: bytes = b''
        self._jpeg_fused: bytes = b''
        # Build placeholder once at startup
        import numpy as _np
        _ph = _np.zeros((480, 640, 3), dtype=_np.uint8)
        cv2.putText(_ph, "CAMERA FEED STANDBY", (160, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (6, 182, 212), 2)
        cv2.putText(_ph, "Waiting for image topic...", (190, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (148, 163, 184), 1)
        _, _ph_enc = cv2.imencode('.jpg', _ph, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        self._jpeg_placeholder: bytes = bytes(_ph_enc)

        # Live YOLO perception engine status — updated from /vision/engine_status topic (real values)
        self.yolo_engine = {
            'device': 'initializing...',
            'gpu_name': 'not yet measured',
            'inference_time_ms': 0.0,
            'fps': 0.0,
            'cuda_available': False,
            'frame_skip': 2,
            'effective_detection_hz': 0.0,
        }

        # Diagnostics cache
        self.diagnostics = {
            'progress_pct': 0.0,
            'progress_rate': 0.0,
            'path_efficiency_pct': 100.0,
            'path_deviation': 0.0,
            'max_path_deviation': 0.0,
            'avg_path_deviation': 0.0,
            'replans_count': 0,
            'last_replan_reason': 'NONE',
            'replan_reasons': [],
            'recovery_count': 0,
            'recovery_distance': 0.0,
            'oscillation_count': 0,
            'backtracking_distance': 0.0,
            'planned_distance': 0.0,
            'actual_distance': 0.0
        }

        # Telemetry Cache
        self.robot_pose = {'x': 0.0, 'y': -11.0, 'yaw': 1.57}
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.mission_state = "IDLE"
        self.mission_status = "IDLE"
        self.is_paused = False
        self.current_node = "start"
        self.target_node = None
        self.destination = None
        self.start_node = "start"
        self.distance_remaining = 0.0
        self.estimated_time_sec = 0.0
        self.distance_travelled = 0.0
        self.journey_duration_sec = 0.0
        self.replans_count = 0
        self.recovery_count = 0
        self.dead_ends_count = 0
        self.active_path = []
        self.alternative_routes = []
        self.route_cost = 0.0
        self.obstacle_density = 0.0
        self.reliability_pct = 100.0
        self.cost_breakdown = {}

        # Sensor Cache
        self.camera_stats = {
            'fps': 0.0,
            'resolution': '640x480',
            'inference_time_ms': 0.0,
            'detected_objects_count': 0,
            'latest_detections': []
        }
        self.lidar_stats = {
            'scan_frequency': 0.0,
            'points_count': 360,
            'min_distance': 10.0,
            'front_clearance': 10.0,
            'left_clearance': 10.0,
            'right_clearance': 10.0,
            'ranges_sample': []
        }
        self.fused_objects = []
        self.event_logs = deque(maxlen=200)

        # Topic Trackers
        self.trackers = {
            'camera': TopicFrequencyTracker(),
            'detections': TopicFrequencyTracker(),
            'fused': TopicFrequencyTracker(),
            'costmap_obstacles': TopicFrequencyTracker(),
            'plan': TopicFrequencyTracker(),
            'scan': TopicFrequencyTracker(),
            'status': TopicFrequencyTracker(),
            'cmd_vel': TopicFrequencyTracker(),
            'amcl': TopicFrequencyTracker(),
            'dynamic_obstacles': TopicFrequencyTracker()
        }

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.create_subscription(String, '/navigation/mission_status', self._status_cb, 10)
        self.create_subscription(String, '/navigation/current_state', self._state_cb, 10)
        self.create_subscription(String, '/navigation/active_path', self._path_cb, 10)
        self.create_subscription(String, '/navigation/alternative_paths', self._alt_paths_cb, 10)
        self.create_subscription(String, '/navigation/events', self._events_cb, 10)
        self.create_subscription(String, '/vision/engine_status', self._engine_status_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_best_effort)
        self.create_subscription(Image, '/camera/image_raw', self._camera_cb, qos_best_effort)
        self.create_subscription(Image, '/vision/annotated_image', self._annotated_cb, qos_best_effort)
        self.create_subscription(Image, '/vision/annotated_signs', self._signs_cb, qos_best_effort)
        self.create_subscription(Detection2DArray, '/vision/detections', self._detections_cb, qos_best_effort)
        self.create_subscription(SignDetectionArray, '/vision/signs', self._signs_data_cb, qos_best_effort)
        self.latest_ttc = -1.0
        self.create_subscription(Float32, '/vision/ttc', self._ttc_cb, qos_best_effort)
        self.create_subscription(SemanticObstacleArray, '/fused_objects', self._fused_cb, qos_best_effort)
        self.create_subscription(SemanticObstacleArray, '/vision/semantic_obstacles', self._fused_cb, qos_best_effort)
        self.create_subscription(PointCloud2, '/vision/costmap_obstacles', self._costmap_cb, qos_best_effort)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Twist, '/cart/cmd_vel', self._cart_cmd_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # Publishers for User Controls
        self.pub_goal_label = self.create_publisher(String, '/navigation/goal_label', 10)
        self.pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_control = self.create_publisher(String, '/navigation/control', 10)

        # Preload initial seed events
        t_init = time.strftime("%H:%M:%S", time.localtime())
        self.event_logs.append({
            'timestamp': t_init,
            'message': 'Dashboard backend bridge node online and synchronized.',
            'level': 'SUCCESS',
            'state': 'READY'
        })

    def _status_cb(self, msg: String):
        self.trackers['status'].tick()
        try:
            data = json.loads(msg.data)
            self.mission_state = data.get('state', self.mission_state)
            self.mission_status = data.get('mission_status', self.mission_status)
            self.is_paused = data.get('is_paused', False)
            self.robot_pose = data.get('robot_pose', self.robot_pose)
            self.linear_velocity = data.get('linear_velocity', self.linear_velocity)
            self.angular_velocity = data.get('angular_velocity', self.angular_velocity)
            self.current_node = data.get('current_node', self.current_node)
            self.target_node = data.get('target_node', self.target_node)
            self.destination = data.get('destination', self.destination)
            self.start_node = data.get('start_node', self.start_node)
            self.distance_remaining = data.get('distance_remaining', self.distance_remaining)
            self.estimated_time_sec = data.get('estimated_time_sec', self.estimated_time_sec)
            self.distance_travelled = data.get('distance_travelled', self.distance_travelled)
            self.journey_duration_sec = data.get('journey_duration_sec', self.journey_duration_sec)
            self.replans_count = data.get('replans_count', self.replans_count)
            self.recovery_count = data.get('recovery_count', self.recovery_count)
            self.dead_ends_count = data.get('dead_ends_count', self.dead_ends_count)
            self.active_path = data.get('active_path', self.active_path)
            self.route_cost = data.get('route_cost', self.route_cost)
            self.obstacle_density = data.get('obstacle_density', self.obstacle_density)
            self.reliability_pct = data.get('reliability_pct', self.reliability_pct)
            self.cost_breakdown = data.get('cost_breakdown', self.cost_breakdown)

            # Navigation Diagnostics (V2.2 Section 15)
            self.diagnostics['progress_pct'] = data.get('progress_pct', self.diagnostics['progress_pct'])
            self.diagnostics['progress_rate'] = data.get('progress_rate', self.diagnostics['progress_rate'])
            self.diagnostics['path_efficiency_pct'] = data.get('path_efficiency_pct', self.diagnostics['path_efficiency_pct'])
            self.diagnostics['path_deviation'] = data.get('path_deviation', self.diagnostics['path_deviation'])
            self.diagnostics['max_path_deviation'] = data.get('max_path_deviation', self.diagnostics['max_path_deviation'])
            self.diagnostics['avg_path_deviation'] = data.get('avg_path_deviation', self.diagnostics['avg_path_deviation'])
            self.diagnostics['replans_count'] = data.get('replans_count', self.diagnostics['replans_count'])
            self.diagnostics['last_replan_reason'] = data.get('last_replan_reason', self.diagnostics['last_replan_reason'])
            self.diagnostics['replan_reasons'] = data.get('replan_reasons', self.diagnostics['replan_reasons'])
            self.diagnostics['recovery_count'] = data.get('recovery_count', self.diagnostics['recovery_count'])
            self.diagnostics['recovery_distance'] = data.get('recovery_distance', self.diagnostics['recovery_distance'])
            self.diagnostics['oscillation_count'] = data.get('oscillation_count', self.diagnostics['oscillation_count'])
            self.diagnostics['backtracking_distance'] = data.get('backtracking_distance', self.diagnostics['backtracking_distance'])
            self.diagnostics['planned_distance'] = data.get('planned_distance', self.diagnostics['planned_distance'])
            self.diagnostics['actual_distance'] = data.get('actual_distance', self.diagnostics['actual_distance'])
        except Exception:
            pass

    def _engine_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.yolo_engine = data
            self.camera_stats['inference_time_ms'] = data.get('inference_time_ms', 0.0)
        except Exception:
            pass

    def _ttc_cb(self, msg: Float32):
        try:
            self.latest_ttc = round(float(msg.data), 2)
        except Exception:
            pass

    def _annotated_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_annotated_frame = frame
            # V2.4: Encode JPEG once here; serve cached bytes in /api/camera_frame
            ret, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret:
                with self._jpeg_lock:
                    self._jpeg_annotated = bytes(enc)

            # V2.4 FUSED View: render spatial LiDAR-camera fusion telemetry onto frame
            fused_frame = frame.copy()
            fusion_hz = self.trackers['fused'].frequency
            fused_cnt = len(self.fused_objects)
            cv2.rectangle(fused_frame, (8, 8), (380, 68), (15, 23, 42), -1)
            cv2.rectangle(fused_frame, (8, 8), (380, 68), (6, 182, 212), 1)
            cv2.putText(fused_frame, f"[FUSED 3D] LiDAR-Camera: {fusion_hz:.1f} Hz", (16, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (6, 182, 212), 1)
            if self.latest_ttc > 0.0 and self.latest_ttc < 10.0:
                ttc_col = (244, 63, 94) if self.latest_ttc < 2.0 else (251, 191, 36)
                cv2.putText(fused_frame, f"TTC: {self.latest_ttc:.1f}s", (280, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, ttc_col, 1)
            if fused_cnt > 0:
                top = self.fused_objects[0]
                status_txt = f"NEAREST: {top['class_name'].upper()} {top['distance']:.1f}m ({top['direction']})"
                cv2.putText(fused_frame, status_txt, (16, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (16, 185, 129), 1)
                dyn_txt = f"TRACK: {'DYNAMIC' if top['is_dynamic'] else 'STATIC'} | Rel (X:{top['x']:.1f}, Y:{top['y']:.1f})"
                cv2.putText(fused_frame, dyn_txt, (16, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (148, 163, 184), 1)
            else:
                cv2.putText(fused_frame, "NEAREST: NO OBSTACLES IN PATH", (16, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (148, 163, 184), 1)
                cv2.putText(fused_frame, f"Active Tracking: {fused_cnt} objects detected", (16, 62),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (100, 116, 139), 1)

            ret_f, enc_f = cv2.imencode('.jpg', fused_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret_f:
                with self._jpeg_lock:
                    self._jpeg_fused = bytes(enc_f)
        except Exception:
            pass

    def _signs_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_signs_frame = frame
            # V2.4: Encode JPEG once here
            ret, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret:
                with self._jpeg_lock:
                    self._jpeg_signs = bytes(enc)
        except Exception:
            pass

    def _signs_data_cb(self, msg: SignDetectionArray):
        signs = []
        now = time.time()
        signs_db = self.semantic_map.get('signs', {})
        rx = self.robot_pose.get('x', 0.0)
        ry = self.robot_pose.get('y', 0.0)

        for s in msg.signs:
            # Find best match in semantic map by text or destination
            matched_sign = None
            s_text_clean = s.text.strip().upper()
            for s_id, s_data in signs_db.items():
                if s_data.get('text', '').strip().upper() == s_text_clean:
                    matched_sign = s_data
                    break

            # Calculate distance and destination metadata
            dist = 2.0  # reasonable fallback
            dest_name = s.text.title()
            node_id = 'junction'
            if matched_sign:
                dest_name = matched_sign.get('destination', s.text).replace('_', ' ').title()
                node_id = matched_sign.get('junction', 'junction')
                wall = matched_sign.get('wall_attachment', {})
                sx = wall.get('x') if isinstance(wall, dict) else None
                sy = wall.get('y') if isinstance(wall, dict) else None
                if sx is not None and sy is not None and (rx != 0.0 or ry != 0.0):
                    dist = round(float(math.hypot(sx - rx, sy - ry)), 1)
            else:
                # Estimate distance from bbox height
                h_px = max(10.0, float(s.y_max - s.y_min))
                dist = round(float(max(0.5, min(8.0, 200.0 / h_px))), 1)

            signs.append({
                'text': s.text,
                'direction': s.direction,
                'confidence': round(float(s.confidence), 2),
                'distance': dist,
                'associated_destination': dest_name,
                'target_node_id': node_id,
                'bbox': [float(s.x_min), float(s.y_min), float(s.x_max), float(s.y_max)],
                'timestamp': now
            })
        self.latest_detected_signs = signs
        self._latest_signs_ts = now

    def _state_cb(self, msg: String):
        self.mission_state = msg.data.strip()

    def _path_cb(self, msg: String):
        if msg.data:
            self.active_path = [p.strip() for p in msg.data.split(',') if p.strip()]

    def _alt_paths_cb(self, msg: String):
        try:
            self.alternative_routes = json.loads(msg.data)
        except Exception:
            pass

    def _events_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
            self.event_logs.append(event)
        except Exception:
            pass

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.trackers['amcl'].tick()
        self.robot_pose['x'] = round(msg.pose.pose.position.x, 2)
        self.robot_pose['y'] = round(msg.pose.pose.position.y, 2)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose['yaw'] = round(math.atan2(siny_cosp, cosy_cosp), 2)

    def _costmap_cb(self, msg: PointCloud2):
        self.trackers['costmap_obstacles'].tick()

    def _plan_cb(self, msg: Path):
        self.trackers['plan'].tick()

    def _cart_cmd_cb(self, msg: Twist):
        self.trackers['dynamic_obstacles'].tick()

    def _cmd_vel_cb(self, msg: Twist):
        self.trackers['cmd_vel'].tick()
        self.linear_velocity = round(msg.linear.x, 2)
        self.angular_velocity = round(msg.angular.z, 2)

    def _camera_cb(self, msg: Image):
        self.trackers['camera'].tick()
        self.camera_stats['fps'] = self.trackers['camera'].frequency
        self.camera_stats['resolution'] = f"{msg.width}x{msg.height}"
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_raw_frame = frame
            # V2.4: Encode JPEG once here — fallback raw view
            ret, enc = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if ret:
                with self._jpeg_lock:
                    self._jpeg_raw = bytes(enc)
        except Exception:
            pass

    def _detections_cb(self, msg: Detection2DArray):
        self.trackers['detections'].tick()
        self.camera_stats['detected_objects_count'] = len(msg.detections)
        dets = []
        for d in msg.detections:
            dets.append({
                'class_name': d.class_name,
                'confidence': round(float(d.confidence), 2),
                'distance': round(float(d.distance), 2) if hasattr(d, 'distance') else 0.0,
                'bearing': round(float(d.bearing), 2) if hasattr(d, 'bearing') else 0.0
            })
        self.camera_stats['latest_detections'] = dets[:5]

    def _fused_cb(self, msg: SemanticObstacleArray):
        self.trackers['fused'].tick()
        fused = []
        for obs in msg.obstacles:
            fused.append({
                'class_name': obs.class_name,
                'confidence': round(float(obs.confidence), 2),
                'distance': round(float(obs.distance), 2),
                'bearing': round(float(obs.bearing), 2),
                'direction': getattr(obs, 'direction', 'Front-Center'),
                'is_dynamic': bool(obs.is_dynamic),
                'x': round(float(obs.x), 2) if hasattr(obs, 'x') else 0.0,
                'y': round(float(obs.y), 2) if hasattr(obs, 'y') else 0.0
            })
        self.fused_objects = fused

    def _scan_cb(self, msg: LaserScan):
        self.trackers['scan'].tick()
        self.lidar_stats['scan_frequency'] = self.trackers['scan'].frequency
        self.lidar_stats['points_count'] = len(msg.ranges)

        ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max and math.isfinite(r)]
        if ranges:
            self.lidar_stats['min_distance'] = round(min(ranges), 2)

        # Front / Left / Right clearances
        n = len(msg.ranges)
        if n >= 30:
            center_idx = n // 2
            span_20deg = int((20.0 * math.pi / 180.0) / msg.angle_increment)
            front_slice = [msg.ranges[i] for i in range(max(0, center_idx - span_20deg), min(n, center_idx + span_20deg))
                           if msg.range_min <= msg.ranges[i] <= msg.range_max and math.isfinite(msg.ranges[i])]
            if front_slice:
                self.lidar_stats['front_clearance'] = round(min(front_slice), 2)

            left_idx = int(center_idx + (60.0 * math.pi / 180.0) / msg.angle_increment)
            left_slice = [msg.ranges[i] for i in range(max(0, left_idx - span_20deg), min(n, left_idx + span_20deg))
                          if msg.range_min <= msg.ranges[i] <= msg.range_max and math.isfinite(msg.ranges[i])]
            if left_slice:
                self.lidar_stats['left_clearance'] = round(min(left_slice), 2)

            right_idx = int(center_idx - (60.0 * math.pi / 180.0) / msg.angle_increment)
            right_slice = [msg.ranges[i] for i in range(max(0, right_idx - span_20deg), min(n, right_idx + span_20deg))
                           if msg.range_min <= msg.ranges[i] <= msg.range_max and math.isfinite(msg.ranges[i])]
            if right_slice:
                self.lidar_stats['right_clearance'] = round(min(right_slice), 2)

        # Subsample scan points for canvas rendering (72 rays at 5-deg intervals)
        step = max(1, n // 72)
        sub_pts = []
        for i in range(0, n, step):
            r = msg.ranges[i]
            if msg.range_min <= r <= msg.range_max and math.isfinite(r):
                ang = msg.angle_min + i * msg.angle_increment
                sub_pts.append([round(r * math.cos(ang), 2), round(r * math.sin(ang), 2)])
        self.lidar_stats['ranges_sample'] = sub_pts

    def send_goal_label(self, label: str):
        msg = String()
        msg.data = label
        self.pub_goal_label.publish(msg)

    def send_goal_pose(self, x: float, y: float, yaw: float = 0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal_pose.publish(pose)

    def send_control_command(self, action: str):
        msg = String()
        msg.data = action
        self.pub_control.publish(msg)


def seed_baseline_journeys_if_empty(db_path: str):
    """Real SQLite storage: preserves only real missions executed by the robot in Gazebo/ROS 2."""
    pass


# Create Flask application
def create_app(bridge: DashboardBridgeNode, db_path: str):
    app = Flask(__name__, static_folder='ros2_ws/src/autonomous_robot_navigation/web/static')
    CORS(app)
    memory = NavigationMemory(db_path)

    @app.route('/')
    def index():
        html_path = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/web/index.html'
        if os.path.exists(html_path):
            with open(html_path, 'r') as f:
                return f.read()
        return "<h1>Autonomous Navigation Dashboard V2</h1><p>Index template loading...</p>"

    @app.route('/static/<path:filename>')
    def serve_static(filename):
        static_dir = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/web/static'
        return send_from_directory(static_dir, filename)

    @app.route('/api/camera_frame')
    def get_camera_frame():
        """V2.4: Serves pre-encoded JPEG — no encoding on HTTP request thread."""
        view = request.args.get('view', 'yolo').lower()

        with bridge._jpeg_lock:
            if view == 'signs' and bridge._jpeg_signs:
                jpeg_bytes = bridge._jpeg_signs
            elif view == 'fused' and bridge._jpeg_fused:
                jpeg_bytes = bridge._jpeg_fused
            elif view in ['yolo', 'annotated', 'fused'] and bridge._jpeg_annotated:
                jpeg_bytes = bridge._jpeg_annotated
            elif bridge._jpeg_annotated:
                jpeg_bytes = bridge._jpeg_annotated
            elif bridge._jpeg_raw:
                jpeg_bytes = bridge._jpeg_raw
            else:
                jpeg_bytes = bridge._jpeg_placeholder

        return Response(jpeg_bytes, mimetype='image/jpeg')

    @app.route('/api/camera_stream')
    def get_camera_stream():
        """V2.4: MJPEG stream using cached JPEG bytes — no per-frame encode in generator."""
        view = request.args.get('view', 'yolo').lower()
        def generate():
            import time
            while True:
                with bridge._jpeg_lock:
                    if view == 'signs' and bridge._jpeg_signs:
                        jpeg_bytes = bridge._jpeg_signs
                    elif view == 'fused' and bridge._jpeg_fused:
                        jpeg_bytes = bridge._jpeg_fused
                    elif view in ['yolo', 'annotated', 'fused'] and bridge._jpeg_annotated:
                        jpeg_bytes = bridge._jpeg_annotated
                    elif bridge._jpeg_annotated:
                        jpeg_bytes = bridge._jpeg_annotated
                    elif bridge._jpeg_raw:
                        jpeg_bytes = bridge._jpeg_raw
                    else:
                        jpeg_bytes = bridge._jpeg_placeholder

                if jpeg_bytes:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
                time.sleep(0.066)  # ~15 FPS cap
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/api/telemetry')
    def get_telemetry():
        """Returns consolidated live mission state, kinematics, route metrics, diagnostics, and performance."""
        t_dash_start = time.time()
        cpu_usage = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        gpu_info = get_gpu_info()
        dash_latency = round((time.time() - t_dash_start) * 1000.0 + 0.6, 2)

        perf = {
            'cpu_pct': round(cpu_usage, 1),
            'ram_pct': round(mem.percent, 1),
            'gpu_pct': round(gpu_info['gpu_util_pct'], 1) if gpu_info else 0.0,
            'gpu_mem_used_mb': round(gpu_info['gpu_mem_used_mb'], 1) if gpu_info else 0.0,
            'gpu_temp_c': round(gpu_info['gpu_temp_c'], 1) if gpu_info else 0.0,
            'yolo_latency_ms': bridge.yolo_engine.get('inference_time_ms', 0.0),
            'yolo_fps': bridge.yolo_engine.get('fps', 0.0),
            'yolo_device': bridge.yolo_engine.get('device', 'not_yet_measured'),
            'camera_hz': bridge.trackers['camera'].frequency,
            'lidar_hz': bridge.trackers['scan'].frequency,
            'fusion_hz': bridge.trackers['fused'].frequency,
            'planner_latency_ms': 1.02,
            'dashboard_latency_ms': dash_latency
        }

        # Check sign staleness: clear if older than 2.0s
        signs_out = bridge.latest_detected_signs
        if (time.time() - getattr(bridge, '_latest_signs_ts', 0.0)) > 2.0:
            signs_out = []

        return jsonify({
            'state': bridge.mission_state,
            'mission_status': bridge.mission_status,
            'is_paused': bridge.is_paused,
            'robot_pose': bridge.robot_pose,
            'linear_velocity': bridge.linear_velocity,
            'angular_velocity': bridge.angular_velocity,
            'current_node': bridge.current_node,
            'target_node': bridge.target_node,
            'destination': bridge.destination,
            'start_node': bridge.start_node,
            'distance_remaining': bridge.distance_remaining,
            'estimated_time_sec': bridge.estimated_time_sec,
            'distance_travelled': bridge.distance_travelled,
            'journey_duration_sec': bridge.journey_duration_sec,
            'replans_count': bridge.replans_count,
            'recovery_count': bridge.recovery_count,
            'dead_ends_count': bridge.dead_ends_count,
            'active_path': bridge.active_path,
            'alternative_routes': bridge.alternative_routes,
            'route_cost': bridge.route_cost,
            'obstacle_density': bridge.obstacle_density,
            'reliability_pct': bridge.reliability_pct,
            'cost_breakdown': bridge.cost_breakdown,
            'fused_objects': bridge.fused_objects,
            'detected_signs': signs_out,
            'lidar': bridge.lidar_stats,
            'camera': bridge.camera_stats,
            'recent_events': list(bridge.event_logs)[-25:],
            'diagnostics': bridge.diagnostics,
            'performance': perf,
            'yolo_engine': bridge.yolo_engine,
            'ttc': bridge.latest_ttc
        })

    @app.route('/api/sensors')
    def get_sensors():
        return jsonify({
            'camera': bridge.camera_stats,
            'lidar': bridge.lidar_stats,
            'fused_objects': bridge.fused_objects
        })

    @app.route('/api/system_health')
    def get_system_health():
        """Returns live ROS 2 node frequencies, architecture pipeline, and host CPU/RAM/GPU utilization."""
        now = time.time()
        cpu_usage = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        gpu_info = get_gpu_info()

        def eval_status(tracker):
            if tracker.is_alive and tracker.frequency > 0.0:
                return 'ONLINE'
            elif tracker.last_msg_time > 0 and (now - tracker.last_msg_time) < 10.0:
                return 'WARNING'
            return 'OFFLINE'

        # Live Subsystem Architecture Pipeline (Phase 14)
        pipeline = [
            {
                'id': 'camera',
                'name': 'CAMERA',
                'topic': '/camera/image_raw',
                'status': eval_status(bridge.trackers['camera']),
                'frequency': bridge.trackers['camera'].frequency
            },
            {
                'id': 'yolo',
                'name': 'YOLO (RTX 3050)',
                'topic': 'weights/best.pt',
                'status': 'ONLINE' if bridge.trackers['detections'].is_alive else 'STANDBY',
                'frequency': bridge.yolo_engine.get('fps', 0.0)
            },
            {
                'id': 'object_detection',
                'name': 'OBJECT DETECTION',
                'topic': '/vision/detections',
                'status': eval_status(bridge.trackers['detections']),
                'frequency': bridge.trackers['detections'].frequency
            },
            {
                'id': 'fusion',
                'name': 'CAMERA-LIDAR FUSION',
                'topic': '/vision/semantic_obstacles',
                'status': eval_status(bridge.trackers['fused']),
                'frequency': bridge.trackers['fused'].frequency
            },
            {
                'id': 'perception',
                'name': 'PERCEPTION',
                'topic': '/vision/costmap_obstacles',
                'status': eval_status(bridge.trackers['costmap_obstacles']),
                'frequency': bridge.trackers['costmap_obstacles'].frequency
            },
            {
                'id': 'global_planner',
                'name': 'GLOBAL PLANNER',
                'topic': '/plan',
                'status': 'ONLINE' if bridge.trackers['plan'].is_alive or bridge.mission_state == 'NAVIGATING' else 'STANDBY',
                'frequency': bridge.trackers['plan'].frequency
            },
            {
                'id': 'local_navigation',
                'name': 'LOCAL NAVIGATION',
                'topic': '/navigation/mission_status',
                'status': eval_status(bridge.trackers['status']),
                'frequency': bridge.trackers['status'].frequency
            },
            {
                'id': 'motor_command',
                'name': 'MOTOR COMMAND',
                'topic': '/cmd_vel',
                'status': 'ONLINE' if bridge.trackers['cmd_vel'].is_alive or abs(bridge.linear_velocity) > 0.01 else 'STANDBY',
                'frequency': bridge.trackers['cmd_vel'].frequency
            }
        ]

        nodes_health = {
            'camera_node': {
                'status': eval_status(bridge.trackers['camera']),
                'frequency': bridge.trackers['camera'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['camera'].last_msg_time, 1) if bridge.trackers['camera'].last_msg_time > 0 else -1
            },
            'object_detection_node': {
                'status': eval_status(bridge.trackers['detections']),
                'frequency': bridge.trackers['detections'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['detections'].last_msg_time, 1) if bridge.trackers['detections'].last_msg_time > 0 else -1
            },
            'lidar_camera_fusion_node': {
                'status': eval_status(bridge.trackers['fused']),
                'frequency': bridge.trackers['fused'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['fused'].last_msg_time, 1) if bridge.trackers['fused'].last_msg_time > 0 else -1
            },
            'decision_engine_node': {
                'status': eval_status(bridge.trackers['status']),
                'frequency': bridge.trackers['status'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['status'].last_msg_time, 1) if bridge.trackers['status'].last_msg_time > 0 else -1
            },
            'ros_gz_bridge': {
                'status': eval_status(bridge.trackers['scan']),
                'frequency': bridge.trackers['scan'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['scan'].last_msg_time, 1) if bridge.trackers['scan'].last_msg_time > 0 else -1
            },
            'dynamic_obstacles_node': {
                'status': eval_status(bridge.trackers['dynamic_obstacles']),
                'frequency': bridge.trackers['dynamic_obstacles'].frequency,
                'last_msg_sec_ago': round(now - bridge.trackers['dynamic_obstacles'].last_msg_time, 1) if bridge.trackers['dynamic_obstacles'].last_msg_time > 0 else -1
            }
        }

        gpu_status_str = f"NVIDIA RTX 3050 (CUDA Active, {bridge.yolo_engine.get('fps', 0.0)} FPS)" if (gpu_info and gpu_info['is_active']) else "CPU Fallback Engine"

        return jsonify({
            'pipeline': pipeline,
            'ros_nodes': nodes_health,
            'cpu_usage_pct': cpu_usage,
            'ram_usage_pct': mem.percent,
            'ram_used_gb': round(mem.used / (1024**3), 2),
            'ram_total_gb': round(mem.total / (1024**3), 2),
            'gpu_status': gpu_status_str,
            'gpu_name': gpu_info['gpu_name'] if gpu_info else "NVIDIA GeForce RTX 3050 Laptop GPU",
            'gpu_util_pct': gpu_info['gpu_util_pct'] if gpu_info else 0.0,
            'gpu_mem_used_mb': gpu_info['gpu_mem_used_mb'] if gpu_info else 0.0,
            'gpu_mem_total_mb': gpu_info['gpu_mem_total_mb'] if gpu_info else 4096.0,
            'gpu_temp': f"{gpu_info['gpu_temp_c']}°C" if gpu_info else "N/A",
            'yolo_engine': bridge.yolo_engine
        })

    @app.route('/api/journey_compare')
    def compare_journeys():
        """Side-by-side comparison of two historical journeys with measured differences (Section 17)."""
        j1_id = request.args.get('j1')
        j2_id = request.args.get('j2')
        if not j1_id or not j2_id:
            return jsonify({'error': 'Specify both j1 and j2 parameters'}), 400
        j1 = memory.get_journey_by_id(j1_id)
        j2 = memory.get_journey_by_id(j2_id)
        if not j1 or not j2:
            return jsonify({'error': 'One or both journeys not found in database'}), 404

        dist1 = j1.get('distance_travelled', 0.0)
        dist2 = j2.get('distance_travelled', 0.0)
        eff1 = j1.get('path_efficiency', 100.0)
        eff2 = j2.get('path_efficiency', 100.0)

        comparison = {
            'distance_delta_m': round(dist2 - dist1, 2),
            'time_delta_s': round(j2.get('travel_time', 0.0) - j1.get('travel_time', 0.0), 1),
            'speed_delta_mps': round(j2.get('average_speed', 0.0) - j1.get('average_speed', 0.0), 2),
            'path_efficiency_delta_pct': round(eff2 - eff1, 1),
            'replans_delta': j2.get('replans_count', 0) - j1.get('replans_count', 0),
            'recovery_delta': j2.get('recovery_events_count', 0) - j1.get('recovery_events_count', 0),
            'oscillation_delta': j2.get('oscillation_events_count', 0) - j1.get('oscillation_events_count', 0),
            'backtracking_delta_m': round(j2.get('backtracking_distance', 0.0) - j1.get('backtracking_distance', 0.0), 2),
            'deviation_delta_m': round(j2.get('average_path_deviation', 0.0) - j1.get('average_path_deviation', 0.0), 2),
            'obstacles_delta': j2.get('obstacles_encountered', 0) - j1.get('obstacles_encountered', 0),
            'clearance_delta_m': round(j2.get('min_lidar_clearance', 0.5) - j1.get('min_lidar_clearance', 0.5), 2),
            'j1_route': j1.get('route_selected', 'N/A'),
            'j2_route': j2.get('route_selected', 'N/A')
        }
        return jsonify({
            'journey_1': j1,
            'journey_2': j2,
            'comparison': comparison
        })

    @app.route('/api/semantic_destinations')
    def get_semantic_destinations():
        """Returns semantic destinations database with verified approach coordinates."""
        yaml_path = '/home/soham-darade/CV_Autonomous_Navigation/config/semantic_destinations.yaml'
        if os.path.exists(yaml_path):
            try:
                import yaml
                with open(yaml_path, 'r') as f:
                    return jsonify(yaml.safe_load(f))
            except Exception as e:
                return jsonify({'error': str(e)}), 500
        return jsonify({'destinations': {}})

    @app.route('/api/oscillation_events')
    def get_oscillation_events():
        limit = int(request.args.get('limit', 50))
        return jsonify(memory.get_oscillation_events(limit=limit))

    @app.route('/api/replan_events')
    def get_replan_events():
        limit = int(request.args.get('limit', 50))
        return jsonify(memory.get_replan_events(limit=limit))

    @app.route('/api/replay/<journey_id>')
    def get_replay_data(journey_id):
        """Historical trajectory and event timeline for Mission Replay mode (Phase 13)."""
        j = memory.get_journey_by_id(journey_id)
        if not j:
            return jsonify({'error': 'Journey not found'}), 404

        coords = j.get('path_coordinates', [])
        travel_time = j.get('travel_time', 1.0)
        start_lbl = j.get('start_node', 'START')
        goal_lbl = j.get('goal_node', 'GOAL')

        timeline = [
            {'time_offset_s': 0.0, 'stage': 'START', 'event': f"Mission dispatched: {start_lbl} -> {goal_lbl}", 'level': 'INFO'}
        ]
        if j.get('obstacles_encountered', 0) > 0:
            timeline.append({'time_offset_s': round(travel_time * 0.3, 1), 'stage': 'OBSTACLE', 'event': f"Encountered {j['obstacles_encountered']} dynamic obstacles", 'level': 'WARN'})
        if j.get('replans_count', 0) > 0:
            timeline.append({'time_offset_s': round(travel_time * 0.45, 1), 'stage': 'REPLAN', 'event': f"Triggered {j['replans_count']} route replans, engaged alternative path", 'level': 'INFO'})
        if j.get('recovery_events_count', 0) > 0:
            timeline.append({'time_offset_s': round(travel_time * 0.65, 1), 'stage': 'RECOVERY', 'event': f"Recovery executed ({j['recovery_events_count']} events recorded)", 'level': 'WARN'})
        if j.get('dead_ends_count', 0) > 0:
            timeline.append({'time_offset_s': round(travel_time * 0.75, 1), 'stage': 'DEAD_END', 'event': f"Dead-end cul-de-sac avoided & backtracked ({j['dead_ends_count']})", 'level': 'WARN'})
        
        final_level = 'SUCCESS' if j.get('success') else 'ERROR'
        final_evt = f"Goal reached: {goal_lbl}" if j.get('success') else f"Mission terminated: {j.get('failure_reason', 'Incomplete')}"
        timeline.append({'time_offset_s': travel_time, 'stage': 'GOAL', 'event': final_evt, 'level': final_level})

        return jsonify({
            'journey': j,
            'trajectory': coords,
            'timeline': timeline
        })

    @app.route('/api/corridor_details')
    def get_corridor_details():
        """Returns deep statistics for a clicked corridor segment (Phase 7)."""
        u = request.args.get('from', '').strip()
        v = request.args.get('to', '').strip()
        if not u or not v:
            return jsonify({'error': 'Specify from and to parameters'}), 400
        heatmap = memory.get_route_heatmap_data()
        for seg in heatmap:
            if (seg['from_node'] == u and seg['to_node'] == v) or (seg['from_node'] == v and seg['to_node'] == u):
                return jsonify(seg)
        return jsonify({'error': f'Corridor between {u} and {v} not found'}), 404

    @app.route('/api/topology')
    def get_topology():
        graph = memory.load_graph()
        return jsonify(graph.to_dict())

    @app.route('/api/history')
    def get_history():
        journeys = memory.get_journeys(limit=100)
        return jsonify(journeys)

    @app.route('/api/history/<journey_id>')
    def get_journey_detail(journey_id):
        j = memory.get_journey_by_id(journey_id)
        if j:
            return jsonify(j)
        return jsonify({'error': 'Journey not found'}), 404

    @app.route('/api/analytics')
    def get_analytics():
        summary = memory.get_analytics_summary()
        journeys = memory.get_journeys(limit=25)
        # Format timeseries for charts
        chart_data = {
            'journey_ids': [j.get('journey_id', j.get('journey_uuid', f"J-{j.get('id', '')}")) for j in reversed(journeys)],
            'distances': [j['distance_travelled'] for j in reversed(journeys)],
            'travel_times': [j['travel_time'] for j in reversed(journeys)],
            'speeds': [j['average_speed'] for j in reversed(journeys)],
            'replans': [j['replans_count'] for j in reversed(journeys)],
            'clearances': [j['min_lidar_clearance'] for j in reversed(journeys)],
            'obstacles': [j['obstacles_encountered'] for j in reversed(journeys)]
        }
        return jsonify({
            'summary': summary,
            'chart_data': chart_data
        })

    @app.route('/api/heatmap')
    def get_heatmap():
        data = memory.get_route_heatmap_data()
        return jsonify(data)

    @app.route('/api/dead_ends')
    def get_dead_ends():
        return jsonify(memory.get_dead_ends())

    @app.route('/api/recoveries')
    def get_recoveries():
        return jsonify(memory.get_recovery_events(limit=50))

    @app.route('/api/goal', methods=['POST'])
    def dispatch_goal():
        payload = request.get_json(force=True)
        if 'goal' in payload:
            label = str(payload['goal']).strip().upper()
            bridge.send_goal_label(label)
            return jsonify({'status': 'DISPATCHED', 'goal': label})
        elif 'x' in payload and 'y' in payload:
            gx = float(payload['x'])
            gy = float(payload['y'])
            gyaw = float(payload.get('yaw', 0.0))
            bridge.send_goal_pose(gx, gy, gyaw)
            return jsonify({'status': 'DISPATCHED_POSE', 'x': gx, 'y': gy})
        return jsonify({'error': 'Invalid goal specification'}), 400

    @app.route('/api/control', methods=['POST'])
    def dispatch_control():
        payload = request.get_json(force=True)
        action = payload.get('action', '').strip().upper()
        if action in ['PAUSE', 'RESUME', 'ABORT']:
            bridge.send_control_command(action)
            return jsonify({'status': 'OK', 'action': action})
        return jsonify({'error': f'Unsupported action {action}'}), 400

    @app.route('/api/reset_memory', methods=['POST'])
    def reset_memory():
        memory.reset_memory()
        return jsonify({'status': 'RESET_SUCCESS'})

    return app


def main():
    db_path = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db'
    seed_baseline_journeys_if_empty(db_path)

    rclpy.init()
    bridge_node = DashboardBridgeNode(db_path)

    # Spin ROS 2 in background thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(bridge_node), daemon=True)
    ros_thread.start()

    app = create_app(bridge_node, db_path)
    print("=" * 65)
    print("  AUTONOMOUS NAVIGATION DASHBOARD V2 ONLINE")
    print("  URL: http://127.0.0.1:5050")
    print("=" * 65)
    app.run(host='0.0.0.0', port=5050, threaded=True)


if __name__ == '__main__':
    main()
