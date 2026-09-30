#!/usr/bin/env python3
"""
Object Detection Node — CV Autonomous Navigation V2.5

Changes from V2.4:
  - Loads model path, class names, per-class confidence thresholds, IoU threshold,
    image_size, device, and frame_skip from config/perception_v25.yaml
    (model is now fully swappable by editing the YAML only)
  - Per-class confidence thresholds: safety-critical classes (person, forklift) use
    higher thresholds; directional_sign uses a lower threshold
  - Temporal confirmation gate for sign detection: a sign must be observed in
    >= N frames within a rolling time window before being published to /vision/signs
    (prevents single-frame false positives from triggering navigation decisions)
  - Minimum bounding-box area filter: rejects tiny spurious edge detections
  - Validates loaded model's class names against config at startup
  - Camera intrinsics (hfov, img_w, img_h, fx) now loaded from YAML, not hardcoded
  - Avoids unnecessary frame copies in annotated image path
  - All telemetry in /vision/engine_status is measured (no fake values)
"""
import os
import sys
import math
import time
import json
import threading
from collections import deque

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

from autonomous_robot_interfaces.msg import Detection2D, Detection2DArray
from autonomous_robot_interfaces.msg import SignDetection, SignDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped

PROJECT_ROOT = '/home/soham-darade/CV_Autonomous_Navigation'
SEMANTIC_MAP_PATH = os.path.join(PROJECT_ROOT, 'config/semantic_map.yaml')
PERCEPTION_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'config/perception_v25.yaml')


def load_perception_config(config_path: str) -> dict:
    """Load and return the V2.5 perception config. Falls back to safe defaults on error."""
    defaults = {
        'yolo': {
            'model_path': 'models/yolov8n_v24.pt',
            'class_names': [
                'person', 'cart', 'forklift', 'pallet', 'box',
                'obstacle', 'door', 'charging_station', 'hospital_bed', 'directional_sign'
            ],
            'device': 'cuda:0',
            'image_size': 640,
            'frame_skip': 2,
            'iou_threshold': 0.45,
            'min_box_area_px2': 400,
            'class_confidence_thresholds': {},
            'global_confidence_threshold': 0.35,
        },
        'camera': {
            'image_width': 640,
            'image_height': 480,
            'hfov_rad': 1.15,
        },
        'sign_detection': {
            'confirmation_frames': 3,
            'confirmation_window_sec': 2.0,
            'min_confirmed_confidence': 0.45,
            'max_bearing_error_rad': 1.2,
            'max_sign_range_m': 8.0,
            'min_sign_range_m': 0.3,
        },
    }
    if not os.path.exists(config_path):
        print(f"[ObjectDetection] WARNING: perception_v25.yaml not found at {config_path}. Using defaults.")
        return defaults
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        # Merge loaded config into defaults (two-level merge)
        for section, vals in cfg.items():
            if section in defaults and isinstance(vals, dict):
                defaults[section].update(vals)
            else:
                defaults[section] = vals
        return defaults
    except Exception as e:
        print(f"[ObjectDetection] WARNING: Could not parse perception_v25.yaml: {e}. Using defaults.")
        return defaults


def build_sign_lookup(yaml_path: str) -> tuple:
    """
    Build a list of sign entries for semantic lookup.
    Each entry: {'id', 'text', 'direction', 'destination', 'texture_file',
                 'x', 'y', 'z', 'confidence_prior'}
    Also build a texture-to-semantic dict for fast lookup by texture filename.
    """
    if not os.path.exists(yaml_path):
        return [], {}
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    signs = []
    tex_to_semantic = {}  # texture_file -> {text, direction, destination}
    for sign_key, sdata in data.get('signs', {}).items():
        wall = sdata.get('wall_attachment', {})
        entry = {
            'id': sdata.get('id', sign_key),
            'text': sdata.get('text', 'SIGN'),
            'direction': sdata.get('direction', 'STRAIGHT'),
            'destination': sdata.get('destination', ''),
            'texture_file': sdata.get('texture_file', ''),
            'x': float(wall.get('x', 0.0)),
            'y': float(wall.get('y', 0.0)),
            'z': float(wall.get('z', 1.10)),
            'yaw': float(wall.get('yaw', 0.0)),
            'approaching_from': sdata.get('approaching_from', []),
            'branch_node': sdata.get('branch_node', ''),
            'confidence_prior': float(sdata.get('confidence_prior', 0.90)),
            'junction': sdata.get('junction', ''),
        }
        signs.append(entry)
        tex_file = entry['texture_file']
        if tex_file and tex_file not in tex_to_semantic:
            tex_to_semantic[tex_file] = {
                'text': entry['text'],
                'direction': entry['direction'],
                'destination': entry['destination'],
            }
    return signs, tex_to_semantic


class SignConfirmationTracker:
    """
    Temporal confirmation gate for sign detections.

    A sign detection is only "confirmed" and forwarded to navigation when it has been
    observed >= confirmation_frames times within a rolling confirmation_window_sec window.

    This prevents single-frame YOLO false positives from triggering navigation decisions.
    Each unique (text, direction) pair is tracked independently.
    """

    def __init__(self, confirmation_frames: int = 3,
                 confirmation_window_sec: float = 2.0,
                 min_confirmed_confidence: float = 0.45):
        self.confirmation_frames = confirmation_frames
        self.confirmation_window_sec = confirmation_window_sec
        self.min_confirmed_confidence = min_confirmed_confidence
        # key: (text, direction) -> deque of (timestamp, confidence)
        self._history: dict = {}

    def observe(self, text: str, direction: str, confidence: float, timestamp: float) -> bool:
        """
        Record a sign observation.
        Returns True if this sign is now "confirmed" (should be published to /vision/signs).
        Returns False if more observations are needed.
        """
        key = (text.upper(), direction.upper())
        if key not in self._history:
            self._history[key] = deque()
        hist = self._history[key]
        hist.append((timestamp, confidence))

        # Prune entries older than the confirmation window
        cutoff = timestamp - self.confirmation_window_sec
        while hist and hist[0][0] < cutoff:
            hist.popleft()

        if len(hist) >= self.confirmation_frames:
            mean_conf = sum(c for _, c in hist) / len(hist)
            return mean_conf >= self.min_confirmed_confidence
        return False

    def purge_stale(self, current_time: float):
        """Remove sign keys that haven't been observed within 2x the window."""
        cutoff = current_time - self.confirmation_window_sec * 2.0
        stale_keys = [k for k, h in self._history.items() if not h or h[-1][0] < cutoff]
        for k in stale_keys:
            del self._history[k]


class ObjectDetectionNode(Node):
    """
    V2.5 Object Detection Node.
    - Loads all model/class/threshold parameters from config/perception_v25.yaml
    - Per-class confidence thresholds (higher for safety-critical classes)
    - Minimum bounding-box area filter (rejects tiny edge-noise detections)
    - Temporal confirmation gate for sign detections (prevents single-frame nav decisions)
    - Validates loaded model's class names against config at startup
    - All published telemetry is measured (no fake values)
    - Publishes /vision/detections, /vision/signs, /vision/annotated_image, /vision/engine_status
    """

    def __init__(self):
        super().__init__('object_detection_node')

        # --- Load V2.5 perception config (YAML — single source of truth) ---
        self.perc_cfg = load_perception_config(PERCEPTION_CONFIG_PATH)
        yolo_cfg = self.perc_cfg.get('yolo', {})
        cam_cfg = self.perc_cfg.get('camera', {})
        sign_cfg = self.perc_cfg.get('sign_detection', {})

        # --- ROS Parameters (allow CLI override of YAML defaults) ---
        raw_model_path = yolo_cfg.get('model_path', 'models/yolov8n_v24.pt')
        # Make absolute if relative
        if not os.path.isabs(raw_model_path):
            raw_model_path = os.path.join(PROJECT_ROOT, raw_model_path)

        self.declare_parameter('model_path', raw_model_path)
        self.declare_parameter('device', yolo_cfg.get('device', 'cuda:0'))
        self.declare_parameter('frame_skip', int(yolo_cfg.get('frame_skip', 2)))
        self.declare_parameter('publish_annotated_image', True)
        # Note: confidence thresholds are per-class from YAML; single threshold still
        # available as a CLI override for the global fallback.
        self.declare_parameter(
            'confidence_threshold',
            float(yolo_cfg.get('global_confidence_threshold', 0.35))
        )

        model_path = os.path.abspath(
            self.get_parameter('model_path').get_parameter_value().string_value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.frame_skip = max(1, self.get_parameter('frame_skip').get_parameter_value().integer_value)
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.global_conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Per-class confidence thresholds from YAML
        self.class_conf_thresholds: dict = yolo_cfg.get('class_confidence_thresholds', {})

        # Other YOLO parameters
        self.iou_threshold: float = float(yolo_cfg.get('iou_threshold', 0.45))
        self.min_box_area: float = float(yolo_cfg.get('min_box_area_px2', 400))
        self.max_box_area_ratio: float = float(yolo_cfg.get('max_box_area_ratio', 0.60))
        self.max_box_width_ratio: float = float(yolo_cfg.get('max_box_width_ratio', 0.92))
        self.expected_class_names: list = yolo_cfg.get('class_names', [])

        # Camera intrinsics from YAML (no longer hardcoded)
        self.img_w = float(cam_cfg.get('image_width', 640))
        self.img_h = float(cam_cfg.get('image_height', 480))
        self.max_box_area = self.max_box_area_ratio * (self.img_w * self.img_h)
        self.max_box_width = self.max_box_width_ratio * self.img_w
        self.hfov = float(cam_cfg.get('hfov_rad', 1.15))
        self.fx = (self.img_w / 2.0) / math.tan(self.hfov / 2.0)

        # Sign temporal confirmation config from YAML
        self.sign_tracker = SignConfirmationTracker(
            confirmation_frames=int(sign_cfg.get('confirmation_frames', 3)),
            confirmation_window_sec=float(sign_cfg.get('confirmation_window_sec', 2.0)),
            min_confirmed_confidence=float(sign_cfg.get('min_confirmed_confidence', 0.45)),
        )
        self.max_bearing_error_rad = float(sign_cfg.get('max_bearing_error_rad', 1.2))
        self.max_sign_range_m = float(sign_cfg.get('max_sign_range_m', 8.0))
        self.min_sign_range_m = float(sign_cfg.get('min_sign_range_m', 0.3))

        # Verify model file
        if not os.path.isfile(model_path):
            self.get_logger().error(f"CRITICAL: Model file not found at '{model_path}'!")
            raise FileNotFoundError(f"YOLO model not found: {model_path}")

        # CUDA fallback
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warning("CUDA requested but unavailable. Falling back to CPU.")
            self.device = 'cpu'

        cuda_active = torch.cuda.is_available() and self.device.startswith('cuda')
        self.gpu_name = torch.cuda.get_device_name(0) if cuda_active else "CPU (Host Processor)"

        # Load YOLO model
        self.model = YOLO(model_path)
        self.model.to(self.device)
        param_device = next(self.model.model.parameters()).device

        # Validate model class names against config
        class_names_list = [self.model.names[i] for i in sorted(self.model.names.keys())]
        loaded_set = set(class_names_list)
        expected_set = set(self.expected_class_names)
        missing = expected_set - loaded_set
        extra = loaded_set - expected_set
        order_match = (class_names_list == self.expected_class_names)

        print(f"\n==================================================")
        print(f"V2.5 Object Detection Node")
        print(f"CONFIG: {PERCEPTION_CONFIG_PATH}")
        print(f"MODEL:  {model_path}")
        print(f"DEVICE: {param_device}  GPU: {self.gpu_name}")
        print(f"CLASSES ({len(class_names_list)}): {class_names_list}")
        print(f"CLASS ORDER MATCHES CONFIG: {order_match}")
        print(f"FRAME_SKIP: every {self.frame_skip} frame(s)")
        print(f"IoU THRESHOLD: {self.iou_threshold}")
        print(f"MIN BOX AREA: {self.min_box_area} px²")
        print(f"GLOBAL CONF: {self.global_conf_thresh}  |  PER-CLASS CONF: {self.class_conf_thresholds}")
        print(f"SIGN CONFIRMATION: {self.sign_tracker.confirmation_frames} frames / {self.sign_tracker.confirmation_window_sec}s window")
        if missing:
            print(f"WARNING: Missing expected classes: {missing}")
        if extra:
            print(f"INFO: Additional classes not in config: {extra}")
        if not order_match and not missing and not extra:
            print(f"WARNING: Class ORDER differs from config! Check perception_v25.yaml class_names list.")
        print(f"==================================================\n")

        self.get_logger().info(f"YOLO V2.5 initialized on {param_device} ({self.gpu_name})")
        self.get_logger().info(f"Classes: {class_names_list}")

        # Sign semantic lookup table from config/semantic_map.yaml
        self.sign_entries, self.tex_to_semantic = build_sign_lookup(SEMANTIC_MAP_PATH)
        self.get_logger().info(
            f"Loaded {len(self.sign_entries)} sign entries from semantic_map.yaml "
            f"({len(self.tex_to_semantic)} unique textures)"
        )

        # State
        self.bridge = CvBridge()
        self.frame_count = 0
        self.latency_history = []
        self.last_engine_pub = 0.0
        self._last_sign_purge = 0.0

        # Cached last result for frame-skip republishing
        self._last_detections: Detection2DArray = Detection2DArray()
        self._last_sign_array: SignDetectionArray = SignDetectionArray()

        # Robot pose tracking for line-of-sight sign matching
        self.robot_x = 0.0
        self.robot_y = -11.0
        self.robot_yaw = 1.57
        self.has_pose = False
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # Publishers
        self.pub_detections = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/vision/annotated_image', 10)
        self.pub_engine_status = self.create_publisher(String, '/vision/engine_status', 10)
        self.pub_signs = self.create_publisher(SignDetectionArray, '/vision/signs', 10)

        # Subscriber — queue depth 1 to always process newest frame
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1)

        self.get_logger().info('ObjectDetectionNode V2.5 ready on /camera/image_raw')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = float(math.atan2(siny_cosp, cosy_cosp))
        self.has_pose = True

    def _get_class_conf_thresh(self, class_name: str) -> float:
        """Return per-class confidence threshold, falling back to global threshold."""
        return float(self.class_conf_thresholds.get(class_name, self.global_conf_thresh))

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        self.frame_count += 1

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Frame-skip: publish cached result on skipped frames
        if self.frame_count % self.frame_skip != 0:
            if self._last_detections.detections:
                self._last_detections.header = msg.header
                self.pub_detections.publish(self._last_detections)
            if self._last_sign_array.signs:
                self._last_sign_array.header = msg.header
                self.pub_signs.publish(self._last_sign_array)
            return

        # Run YOLO inference
        t0 = time.time()
        results = self.model(
            cv_image,
            conf=self.global_conf_thresh,   # Use global as YOLO pre-filter; per-class applied below
            iou=self.iou_threshold,
            device=self.device,
            verbose=False
        )[0]
        t_infer = (time.time() - t0) * 1000.0

        self.latency_history.append(t_infer)
        if len(self.latency_history) > 30:
            self.latency_history.pop(0)
        avg_latency = float(np.mean(self.latency_history))
        measured_fps = round(1000.0 / max(1.0, avg_latency), 1)

        # Publish engine status periodically (all measured values)
        now = time.time()
        if (now - self.last_engine_pub) >= 1.0:
            self.last_engine_pub = now
            status = {
                'device': str(self.device),
                'gpu_name': self.gpu_name,
                'inference_time_ms': round(avg_latency, 2),
                'fps': measured_fps,
                'cuda_available': bool(torch.cuda.is_available()),
                'frame_skip': self.frame_skip,
                'effective_detection_hz': round(measured_fps / self.frame_skip, 1),
                'model_classes': len(self.model.names),
                'iou_threshold': self.iou_threshold,
                'config_version': '2.5',
            }
            self.pub_engine_status.publish(String(data=json.dumps(status)))

        # Periodically purge stale sign tracker entries
        if (now - self._last_sign_purge) > 5.0:
            self.sign_tracker.purge_stale(now)
            self._last_sign_purge = now

        # Build detection array and sign array
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        confirmed_sign_array = SignDetectionArray()
        confirmed_sign_array.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0].item())
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0].item())

            # Per-class confidence filter (applied after YOLO NMS)
            required_conf = self._get_class_conf_thresh(class_name)
            if conf < required_conf:
                continue

            xyxy = box.xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])

            # Bounding-box geometry filter (rejects tiny noise or full-frame background planes)
            box_area = (x_max - x_min) * (y_max - y_min)
            box_width = (x_max - x_min)
            if box_area < self.min_box_area or box_area > self.max_box_area or box_width > self.max_box_width:
                continue

            # Build Detection2D
            det = Detection2D()
            det.class_name = class_name
            det.confidence = conf
            det.x_min = x_min
            det.y_min = y_min
            det.x_max = x_max
            det.y_max = y_max
            det.distance = -1.0  # Populated by LiDAR fusion node
            det.bearing = 0.0
            detection_array.detections.append(det)

            # Sign temporal confirmation gate
            if class_name == 'directional_sign':
                sign_det = self._parse_sign_semantics(
                    x_min, y_min, x_max, y_max, conf, msg.header)
                if sign_det is not None:
                    # Check temporal confirmation before forwarding to navigation
                    is_confirmed = self.sign_tracker.observe(
                        sign_det.text, sign_det.direction, conf, now)
                    if is_confirmed:
                        confirmed_sign_array.signs.append(sign_det)

        # Cache and publish
        self._last_detections = detection_array
        self._last_sign_array = confirmed_sign_array

        self.pub_detections.publish(detection_array)
        # Always publish sign array (even if empty) so downstream subscribers know
        # immediately when signs leave the field of view
        self.pub_signs.publish(confirmed_sign_array)

        # Annotated image for dashboard
        if self.publish_annotated:
            try:
                annotated_frame = results.plot()
                # Overlay confirmed sign text on annotated image
                for sd in confirmed_sign_array.signs:
                    cx = int((sd.x_min + sd.x_max) / 2)
                    cy = int(sd.y_min) - 8
                    label = f"[CONFIRMED] {sd.text} {sd.direction}"
                    cv2.putText(annotated_frame, label, (max(0, cx - 60), max(10, cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='passthrough')
                annotated_msg.encoding = 'bgr8'
                annotated_msg.header = msg.header
                self.pub_annotated.publish(annotated_msg)
            except Exception as e:
                self.get_logger().warning(f'Annotated image publish failed: {e}')

    # ------------------------------------------------------------------
    # Sign semantic parsing
    # ------------------------------------------------------------------
    def _parse_sign_semantics(self, x_min, y_min, x_max, y_max, conf, header) -> SignDetection:
        """
        Given a YOLO bounding box for 'directional_sign', determine semantic content.

        Strategy (no OCR dependency):
        1. Compute the horizontal bearing of the sign's center pixel using camera FOV.
        2. If robot pose is available (AMCL), find the closest sign in semantic_map.yaml
           by computing the expected bearing from the robot to each sign in the map
           and matching the closest within max_bearing_error_rad tolerance.
        3. Fallback to heuristic (left/center/right bearing) if no map match.

        This is correct for simulation because each sign has a known world position
        and the approach direction is fixed by the corridor layout.
        """
        # Bounding box center in image
        u_center = (x_min + x_max) / 2.0
        v_center = (y_min + y_max) / 2.0

        # Horizontal bearing from camera center (positive = left of center)
        bearing_rad = math.atan2((self.img_w / 2.0 - u_center), self.fx)

        # Vertical bearing (positive = above center)
        v_bearing_rad = math.atan2((self.img_h / 2.0 - v_center), self.fx)

        # Approximate sign range via vertical bearing
        # Sign is at z=0.85m, camera at ~0.35m → height difference ≈ 0.50m
        approx_range = 999.0
        if abs(v_bearing_rad) > 0.01:
            approx_range = abs(0.50 / math.tan(v_bearing_rad))
        approx_range = min(approx_range, self.max_sign_range_m)

        # Best match: find sign entry whose expected bearing most closely matches bearing_rad
        best_entry = None
        best_score = self.max_bearing_error_rad  # Only accept matches within this threshold

        for entry in self.sign_entries:
            if self.has_pose:
                dx = entry['x'] - self.robot_x
                dy = entry['y'] - self.robot_y
                dist = math.hypot(dx, dy)
                if dist < self.min_sign_range_m or dist > self.max_sign_range_m:
                    continue

                # Sign front normal vector in world frame
                # Sign box model: front normal is (-sin(yaw), cos(yaw))
                sign_yaw = entry.get('yaw', 0.0)
                normal_x = -math.sin(sign_yaw)
                normal_y = math.cos(sign_yaw)

                # Vector from sign to robot
                vec_s2r_x = self.robot_x - entry['x']
                vec_s2r_y = self.robot_y - entry['y']

                # 1. Sign Face Normal Check: Robot must be located in front of the sign face
                # dot(V_s2r, Normal) / dist must be > 0.15 (within ~81 deg of front normal)
                front_dot = (vec_s2r_x * normal_x + vec_s2r_y * normal_y) / max(0.01, dist)
                if front_dot <= 0.15:
                    # Robot is behind the sign or at an extreme acute angle -> reject rear face
                    continue

                # 2. Heading Check: Robot camera must be facing roughly opposite to the sign normal
                # dot(R_heading, Normal) must be < -0.15 (facing into the sign)
                robot_dir_x = math.cos(self.robot_yaw)
                robot_dir_y = math.sin(self.robot_yaw)
                heading_dot = robot_dir_x * normal_x + robot_dir_y * normal_y
                if heading_dot >= -0.15:
                    # Robot is looking away from the sign normal (viewing rear or parallel) -> reject
                    continue

                # Angle to sign in map frame
                angle_to_sign = math.atan2(dy, dx)
                # Relative angle in robot body frame
                rel_angle = math.atan2(math.sin(angle_to_sign - self.robot_yaw),
                                       math.cos(angle_to_sign - self.robot_yaw))
                # Check if sign is within camera field of view (+ small margin)
                if abs(rel_angle) > (self.hfov / 2.0 + 0.30):
                    continue
                angle_diff = abs(bearing_rad - rel_angle)
                if angle_diff < best_score:
                    best_score = angle_diff
                    best_entry = entry
            else:
                # No pose yet: use heuristic bearing-to-world-position match
                sign_world_x = entry['x']
                sign_world_y = entry['y']
                expected_bearing = math.atan2(-sign_world_x, max(1.0, abs(sign_world_y)))
                angle_diff = abs(bearing_rad - expected_bearing)
                if angle_diff < best_score:
                    best_score = angle_diff
                    best_entry = entry

        if best_entry is None:
            # No confirmed sign matches pose, viewing angle, and bearing.
            # Do NOT hallucinate fallback signs (e.g. "OFFICE STRAIGHT").
            return None

        text = best_entry['text']
        direction = best_entry['direction']
        destination = best_entry['destination']

        sign_det = SignDetection()
        sign_det.header = header
        sign_det.text = text
        sign_det.direction = direction
        sign_det.confidence = conf
        sign_det.x_min = float(x_min)
        sign_det.y_min = float(y_min)
        sign_det.x_max = float(x_max)
        sign_det.y_max = float(y_max)
        return sign_det


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
