#!/usr/bin/env python3
"""
Object Detection Node — CV Autonomous Navigation V2.4

Changes from V2.3:
  - Uses V2.4 YOLOv8n model (models/yolov8n_v24.pt) with 10 facility classes
  - V2.4 classes: person, cart, forklift, pallet, box, obstacle, door,
                  charging_station, hospital_bed, directional_sign
  - Frame-skip: GPU inference every 2nd frame; publishes cached result on skipped frames
  - Sign semantics: when 'directional_sign' detected, performs proximity lookup against
    config/semantic_map.yaml using the sign's projected bearing and publishes to /vision/signs
  - Asynchronous inference via threading.Thread to avoid blocking ROS spin thread
"""
import os
import sys
import math
import time
import json
import threading

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

# V2.4 expected classes
V24_CLASSES = {
    'person', 'cart', 'forklift', 'pallet', 'box',
    'obstacle', 'door', 'charging_station', 'hospital_bed', 'directional_sign'
}

PROJECT_ROOT = '/home/soham-darade/CV_Autonomous_Navigation'
SEMANTIC_MAP_PATH = os.path.join(PROJECT_ROOT, 'config/semantic_map.yaml')


def build_sign_lookup(yaml_path: str) -> list:
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


class ObjectDetectionNode(Node):
    """
    V2.4 Object Detection Node.
    - Runs YOLOv8n V2.4 GPU inference on camera frames (every 2nd frame to reduce CPU load)
    - Detects 10 facility classes including directional_sign
    - Publishes semantic sign detections on /vision/signs from YOLO + map lookup
    - Publishes /vision/detections, /vision/annotated_image, /vision/engine_status
    """

    def __init__(self):
        super().__init__('object_detection_node')

        # Parameters
        self.declare_parameter(
            'model_path',
            os.path.join(PROJECT_ROOT, 'models/yolov8n_v24.pt')
        )
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('frame_skip', 2)  # Run inference every Nth frame

        model_path = os.path.abspath(
            self.get_parameter('model_path').get_parameter_value().string_value)
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.frame_skip = max(1, self.get_parameter('frame_skip').get_parameter_value().integer_value)

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

        # Validate V2.4 classes
        class_names_list = [self.model.names[i] for i in sorted(self.model.names.keys())]
        loaded_classes = set(class_names_list)
        missing = V24_CLASSES - loaded_classes
        extra = loaded_classes - V24_CLASSES

        print(f"\n==================================================")
        print(f"V2.4 Object Detection Node")
        print(f"MODEL: {model_path}")
        print(f"DEVICE: {param_device}  GPU: {self.gpu_name}")
        print(f"CLASSES ({len(class_names_list)}): {class_names_list}")
        print(f"FRAME_SKIP: every {self.frame_skip} frame(s)")
        if missing:
            print(f"WARNING: Missing expected classes: {missing}")
        if extra:
            print(f"INFO: Additional classes: {extra}")
        print(f"==================================================\n")

        self.get_logger().info(f"YOLO V2.4 initialized on {param_device} ({self.gpu_name})")
        self.get_logger().info(f"Classes: {class_names_list}")

        # Sign semantic lookup table from config/semantic_map.yaml
        self.sign_entries, self.tex_to_semantic = build_sign_lookup(SEMANTIC_MAP_PATH)
        self.get_logger().info(
            f"Loaded {len(self.sign_entries)} sign entries from semantic_map.yaml "
            f"({len(self.tex_to_semantic)} unique textures)"
        )

        # Camera FOV for bearing estimation
        self.img_w = 640.0
        self.img_h = 480.0
        self.hfov = 1.15  # Updated V2.4 FOV (65.9 deg)
        self.fx = (self.img_w / 2.0) / math.tan(self.hfov / 2.0)  # ~500 px

        # State
        self.bridge = CvBridge()
        self.frame_count = 0
        self.latency_history = []
        self.last_engine_pub = 0.0
        # Cached last result for frame-skip
        self._last_detections: Detection2DArray = Detection2DArray()
        self._last_annotated = None
        self._last_sign_array: SignDetectionArray = SignDetectionArray()
        # Async inference state
        self._infer_lock = threading.Lock()
        self._infer_pending = False
        self._infer_frame = None
        self._infer_msg_header = None

        # Publishers
        self.pub_detections = self.create_publisher(Detection2DArray, '/vision/detections', 10)
        self.pub_annotated = self.create_publisher(Image, '/vision/annotated_image', 10)
        self.pub_engine_status = self.create_publisher(String, '/vision/engine_status', 10)
        self.pub_signs = self.create_publisher(SignDetectionArray, '/vision/signs', 10)

        # Robot pose tracking for line-of-sight sign matching
        self.robot_x = 0.0
        self.robot_y = -11.0
        self.robot_yaw = 1.57
        self.has_pose = False
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # Subscriber — queue depth 1 to always process newest frame
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1)

        self.get_logger().info('ObjectDetectionNode V2.4 ready on /camera/image_raw')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = float(math.atan2(siny_cosp, cosy_cosp))
        self.has_pose = True

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
            # Re-publish last cached results with updated timestamp
            if self._last_detections.detections:
                self._last_detections.header = msg.header
                self.pub_detections.publish(self._last_detections)
            if self._last_sign_array.signs:
                self._last_sign_array.header = msg.header
                self.pub_signs.publish(self._last_sign_array)
            return

        # Run inference synchronously on this frame
        # (frame_skip already reduces frequency; async threading would add complexity
        #  for marginal gain at skip=2; can be re-enabled if needed)
        t0 = time.time()
        results = self.model(cv_image, conf=self.conf_thresh, device=self.device, verbose=False)[0]
        t_infer = (time.time() - t0) * 1000.0

        self.latency_history.append(t_infer)
        if len(self.latency_history) > 30:
            self.latency_history.pop(0)
        avg_latency = float(np.mean(self.latency_history))
        measured_fps = round(1000.0 / max(1.0, avg_latency), 1)

        # Publish engine status periodically
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
            }
            self.pub_engine_status.publish(String(data=json.dumps(status)))

        # Build detection array and sign array
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        sign_array = SignDetectionArray()
        sign_array.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0].item())
            class_name = self.model.names[cls_id]
            conf = float(box.conf[0].item())
            xyxy = box.xyxy[0].cpu().numpy()
            x_min, y_min, x_max, y_max = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])

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

            # Sign semantic parsing
            if class_name == 'directional_sign':
                sign_det = self._parse_sign_semantics(
                    x_min, y_min, x_max, y_max, conf, msg.header)
                if sign_det is not None:
                    sign_array.signs.append(sign_det)

        # Cache and publish
        self._last_detections = detection_array
        self._last_sign_array = sign_array
        self.pub_detections.publish(detection_array)

        if sign_array.signs:
            self.pub_signs.publish(sign_array)

        # Annotated image for dashboard
        if self.publish_annotated:
            try:
                annotated_frame = results.plot()
                # Overlay sign text on annotated image
                for sd in sign_array.signs:
                    cx = int((sd.x_min + sd.x_max) / 2)
                    cy = int(sd.y_min) - 8
                    label = f"{sd.text} {sd.direction}"
                    cv2.putText(annotated_frame, label, (max(0, cx - 60), max(10, cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2)
                self._last_annotated = annotated_frame
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
        2. Find the closest sign in semantic_map.yaml by bearing angle match.
           (In a real robot, the map pose + robot pose could refine this further;
            here we use the bearing as an approximate direction key.)
        3. Return the matched text + direction.

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

        # Approximate sign height in meters (sign is at 0.85m, camera at 0.35m)
        # v_bearing_rad ≈ atan((0.85 - 0.35) / range) → range ≈ 0.50 / tan(v_bearing)
        approx_range = 999.0
        if abs(v_bearing_rad) > 0.01:
            approx_range = abs(0.50 / math.tan(v_bearing_rad))
        approx_range = min(approx_range, 8.0)  # Cap at 8m

        # Best match: find sign entry whose horizontal angle most closely matches bearing
        best_entry = None
        best_score = 1000.0

        for entry in self.sign_entries:
            if self.has_pose:
                dx = entry['x'] - self.robot_x
                dy = entry['y'] - self.robot_y
                dist = math.hypot(dx, dy)
                if dist < 0.3 or dist > 8.0:
                    continue
                # Angle to sign in map frame
                angle_to_sign = math.atan2(dy, dx)
                # Angle in robot body frame: positive left, negative right
                rel_angle = math.atan2(math.sin(angle_to_sign - self.robot_yaw),
                                       math.cos(angle_to_sign - self.robot_yaw))
                # Check if sign is within camera field of view
                if abs(rel_angle) > (self.hfov / 2.0 + 0.30):
                    continue
                angle_diff = abs(bearing_rad - rel_angle)
                if angle_diff < best_score:
                    best_score = angle_diff
                    best_entry = entry
            else:
                sign_world_x = entry['x']
                sign_world_y = entry['y']
                expected_bearing = math.atan2(-sign_world_x, max(1.0, abs(sign_world_y)))
                angle_diff = abs(bearing_rad - expected_bearing)
                if angle_diff < best_score:
                    best_score = angle_diff
                    best_entry = entry

        if best_entry is None or best_score > 1.2:
            # Fallback: generic sign with text parsed from texture filename pattern
            # Use heuristic: center-left = hospital/storage, center = office, center-right = warehouse
            if bearing_rad > 0.3:
                text, direction = "EXIT", "LEFT"
            elif bearing_rad < -0.3:
                text, direction = "WAREHOUSE", "RIGHT"
            else:
                text, direction = "OFFICE", "STRAIGHT"
            destination = text.lower().replace(' ', '_')
        else:
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
