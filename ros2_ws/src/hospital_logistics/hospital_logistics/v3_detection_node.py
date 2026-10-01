"""
V3 object detection node.

Runs the Phase 2 V3 YOLOv8n model (V3/models_trained/v3_yolov8n_from_v25.pt, 9 classes) on
/camera/image_raw and publishes autonomous_robot_interfaces/Detection2DArray on /vision/detections
(same interface as the V2.6 object_detection_node, so the V2.6 downstream nodes are unchanged).

Differences to V2.6 (deliberate, see V3_PERCEPTION_VALIDATION_REPORT.md): one confidence threshold
(0.35, the V2.6 global default at which V3 was validated), no V2.5 per-class thresholds and no
box-area/width filters. The detection header is the IMAGE header (stamp used for LiDAR sync).
Measured latency (preprocess+inference+postprocess, wall clock) is published on
/v3/perception/detection_timing (std_msgs/String JSON).

Signs: every 'directional_sign' box is resolved to a semantic sign (text / direction) exactly as in
the V2.6 object_detection_node: bearing of the box centre + AMCL pose matched against the known sign
poses of V3/config/v3_hospital_semantic_map.yaml (NEW hospital signs) (front-face, heading, FOV and bearing checks), then the V2.6
SignConfirmationTracker (3 frames / 2 s / mean conf >= 0.45, values from config/perception_v25.yaml).
The V2.6 helpers are imported unchanged; confirmed signs are published on /vision/signs for the
V2.6 decision engine (destination-aware sign gating). No sign is ever invented when no map sign matches.
"""
import json
import os
import sys
import time

VENV_SITE = os.environ.get("V3_VENV_SITE", os.path.expanduser("~/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages"))
if VENV_SITE not in sys.path:
    sys.path.insert(0, VENV_SITE)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from ultralytics import YOLO  # noqa: E402

import math  # noqa: E402

import yaml  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402

from autonomous_robot_interfaces.msg import Detection2D, Detection2DArray, SignDetection, SignDetectionArray  # noqa: E402
from autonomous_robot_perception.object_detection_node import SignConfirmationTracker, build_sign_lookup  # noqa: E402

PROJECT_ROOT = os.path.dirname(os.environ.get("V3_ROOT", os.path.expanduser("~/CV_Autonomous_Navigation/V3")))
SEMANTIC_MAP = os.path.join(os.environ.get("V3_ROOT", os.path.join(PROJECT_ROOT, "V3")), "config", "v3_hospital_semantic_map.yaml")
PERCEPTION_CFG = os.path.join(PROJECT_ROOT, "config", "perception_v25.yaml")
FX = 493.79245758056641
IMG_W, HFOV = 640.0, 1.15

V3_ROOT = os.environ.get("V3_ROOT", os.path.expanduser("~/CV_Autonomous_Navigation/V3"))


class V3DetectionNode(Node):
    def __init__(self):
        super().__init__("v3_detection_node")
        self.declare_parameter("model_path", os.path.join(V3_ROOT, "models_trained", "v3_yolov8n_from_v25.pt"))
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("frame_skip", 1)
        self.declare_parameter("device", "cuda:0")
        self.conf = self.get_parameter("confidence").value
        self.iou = self.get_parameter("iou").value
        self.skip = max(1, int(self.get_parameter("frame_skip").value))
        self.device = self.get_parameter("device").value
        path = self.get_parameter("model_path").value
        self.model = YOLO(path)
        self.model.predict(np.zeros((480, 640, 3), np.uint8), device=self.device, verbose=False)  # warm-up
        self.n = 0
        self.pub = self.create_publisher(Detection2DArray, "/vision/detections", 10)
        # signs (V2.6 semantics)
        sc = yaml.safe_load(open(PERCEPTION_CFG)).get("sign_detection", {})
        self.sign_tracker = SignConfirmationTracker(int(sc.get("confirmation_frames", 3)),
                                                    float(sc.get("confirmation_window_sec", 2.0)),
                                                    float(sc.get("min_confirmed_confidence", 0.45)))
        self.max_bearing_err = float(sc.get("max_bearing_error_rad", 1.2))
        self.max_sign_range = float(sc.get("max_sign_range_m", 8.0))
        self.min_sign_range = float(sc.get("min_sign_range_m", 0.3))
        self.sign_entries, _ = build_sign_lookup(SEMANTIC_MAP)
        self.has_pose, self.rx, self.ry, self.ryaw = False, 0.0, 0.0, 0.0
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_signs = self.create_publisher(SignDetectionArray, "/vision/signs", 10)
        self.pub_timing = self.create_publisher(String, "/v3/perception/detection_timing", 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"V3 detection: {path} classes={list(self.model.names.values())} conf={self.conf}")

    def on_amcl(self, m):
        p = m.pose.pose
        self.rx, self.ry = p.position.x, p.position.y
        q = p.orientation
        self.ryaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.has_pose = True

    def parse_sign(self, box, conf, header):
        """V2.6 _parse_sign_semantics logic (object_detection_node.py), map-pose matching only."""
        if not self.has_pose:
            return None
        x0, y0, x1, y1 = box
        bearing = math.atan2(IMG_W / 2.0 - (x0 + x1) / 2.0, FX)
        best, best_err = None, self.max_bearing_err
        for e in self.sign_entries:
            dx, dy = e["x"] - self.rx, e["y"] - self.ry
            dist = math.hypot(dx, dy)
            if dist < self.min_sign_range or dist > self.max_sign_range:
                continue
            nx, ny = -math.sin(e.get("yaw", 0.0)), math.cos(e.get("yaw", 0.0))
            if ((self.rx - e["x"]) * nx + (self.ry - e["y"]) * ny) / max(0.01, dist) <= 0.15:
                continue  # robot behind the sign face
            if math.cos(self.ryaw) * nx + math.sin(self.ryaw) * ny >= -0.15:
                continue  # camera not facing the sign
            rel = math.atan2(math.sin(math.atan2(dy, dx) - self.ryaw), math.cos(math.atan2(dy, dx) - self.ryaw))
            if abs(rel) > HFOV / 2.0 + 0.30:
                continue
            err = abs(bearing - rel)
            if err < best_err:
                best, best_err = e, err
        if best is None:
            return None
        sd = SignDetection()
        sd.header = header
        sd.text, sd.direction, sd.confidence = best["text"], best["direction"], float(conf)
        sd.x_min, sd.y_min, sd.x_max, sd.y_max = (float(v) for v in box)
        return sd

    def on_image(self, msg):
        self.n += 1
        if self.n % self.skip:
            return
        t0 = time.perf_counter()
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        r = self.model.predict(np.ascontiguousarray(img), conf=self.conf, iou=self.iou, imgsz=640,
                               device=self.device, verbose=False)[0]
        out = Detection2DArray()
        out.header = msg.header
        for c, s, b in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
            d = Detection2D()
            d.class_name = self.model.names[int(c)]
            d.confidence = float(s)
            d.x_min, d.y_min, d.x_max, d.y_max = (float(v) for v in b)
            d.distance, d.bearing = -1.0, 0.0
            out.detections.append(d)
        self.pub.publish(out)
        signs = SignDetectionArray()
        signs.header = msg.header
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for d in out.detections:
            if d.class_name == "directional_sign":
                sd = self.parse_sign((d.x_min, d.y_min, d.x_max, d.y_max), d.confidence, msg.header)
                if sd is not None and self.sign_tracker.observe(sd.text, sd.direction, d.confidence, stamp_s):
                    signs.signs.append(sd)
        self.sign_tracker.purge_stale(stamp_s)
        self.pub_signs.publish(signs)
        ms = (time.perf_counter() - t0) * 1000
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pub_timing.publish(String(data=json.dumps({"stamp": stamp, "total_ms": round(ms, 2), **{k: round(v, 2) for k, v in r.speed.items()},
                                                        "n": len(out.detections)})))


def main(args=None):
    rclpy.init(args=args)
    node = V3DetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
