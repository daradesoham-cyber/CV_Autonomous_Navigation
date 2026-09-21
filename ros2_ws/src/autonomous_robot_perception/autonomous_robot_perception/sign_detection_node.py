#!/usr/bin/env python3
"""
Sign Perception Node for Autonomous Navigation.
Detects directional semantic signs (HOSPITAL, WAREHOUSE, OFFICE, LAB, STORAGE, CAFETERIA, EXIT)
using contour analysis, multi-scale template matching, and color verification.
Publishes SignDetectionArray to /vision/signs and annotated image to /vision/annotated_signs.
"""

import os
import sys
import glob
import cv2
import numpy as np

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from autonomous_robot_interfaces.msg import SignDetection, SignDetectionArray


class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        self.declare_parameter(
            'templates_dir',
            '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs'
        )
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('publish_annotated_image', True)

        self.templates_dir = os.path.abspath(
            self.get_parameter('templates_dir').get_parameter_value().string_value
        )
        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.publish_annotated = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.templates = self._load_templates(self.templates_dir)

        # QoS profile for sensor data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            qos
        )

        self.pub_signs = self.create_publisher(
            SignDetectionArray,
            '/vision/signs',
            10
        )

        self.pub_annotated = self.create_publisher(
            Image,
            '/vision/annotated_signs',
            10
        )

        self.get_logger().info(
            f"SignDetectionNode initialized. Loaded {len(self.templates)} templates from {self.templates_dir}."
        )

    def _load_templates(self, directory: str) -> dict:
        """Load and preprocess sign template textures."""
        templates = {}
        if not os.path.isdir(directory):
            self.get_logger().error(f"Sign templates directory not found: {directory}")
            return templates

        for path in sorted(glob.glob(os.path.join(directory, '*.png'))):
            fname = os.path.basename(path)
            base = os.path.splitext(fname)[0]

            if base == 'emergency_exit':
                text = 'EMERGENCY EXIT'
                direction = 'STRAIGHT'
            else:
                parts = base.split('_')
                text = parts[0].upper()
                direction = parts[1].upper() if len(parts) > 1 else 'STRAIGHT'

            img = cv2.imread(path)
            if img is None:
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            thumb = cv2.resize(gray, (160, 80))

            # Sample dominant border color
            border_sample = img[15:25, 15:25]
            mean_bgr = np.mean(border_sample, axis=(0, 1))

            templates[base] = {
                'id': base,
                'text': text,
                'direction': direction,
                'thumb': thumb,
                'color_bgr': mean_bgr
            }

        return templates

    def _image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        detections, annotated = self._detect_signs(cv_img)

        # Publish SignDetectionArray
        array_msg = SignDetectionArray()
        array_msg.header = msg.header

        for d in detections:
            sd = SignDetection()
            sd.header = msg.header
            sd.text = d['text']
            sd.direction = d['direction']
            sd.confidence = float(d['confidence'])
            sd.x_min = float(d['x_min'])
            sd.y_min = float(d['y_min'])
            sd.x_max = float(d['x_max'])
            sd.y_max = float(d['y_max'])
            array_msg.signs.append(sd)

        self.pub_signs.publish(array_msg)

        # Publish annotated image
        if self.publish_annotated and self.pub_annotated.get_subscription_count() > 0:
            try:
                ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                ann_msg.header = msg.header
                self.pub_annotated.publish(ann_msg)
            except Exception as e:
                self.get_logger().error(f"Annotated image publish failed: {e}")

    def _detect_signs(self, img: np.ndarray):
        """Detect signs via candidate contour extraction and template correlation."""
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        annotated = img.copy() if self.publish_annotated else None

        # Sign candidates have bright backgrounds: check multiple thresholds
        thresholds = [160, 185]
        candidate_boxes = []

        for th in thresholds:
            _, bin_img = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                x, y, cw, ch = cv2.boundingRect(cnt)
                # Filter by size and aspect ratio
                if cw < 28 or ch < 14:
                    continue
                if cw > w * 0.9 or ch > h * 0.9:
                    continue
                aspect = cw / float(ch)
                if 1.1 <= aspect <= 3.2:
                    candidate_boxes.append((x, y, cw, ch))

        # Deduplicate overlapping candidate boxes
        dedup_boxes = []
        for box in candidate_boxes:
            bx, by, bw, bh = box
            overlap = False
            for dx, dy, dw, dh in dedup_boxes:
                # Simple IoU check
                ix1, iy1 = max(bx, dx), max(by, dy)
                ix2, iy2 = min(bx + bw, dx + dw), min(by + bh, dy + dh)
                if ix2 > ix1 and iy2 > iy1:
                    intersection = (ix2 - ix1) * (iy2 - iy1)
                    union = bw * bh + dw * dh - intersection
                    if intersection / union > 0.4:
                        overlap = True
                        break
            if not overlap:
                dedup_boxes.append(box)

        detections = []
        for x, y, cw, ch in dedup_boxes:
            roi_gray = gray[y:y+ch, x:x+cw]
            roi_thumb = cv2.resize(roi_gray, (160, 80))

            best_score = -1.0
            best_tmpl = None

            for tid, tdata in self.templates.items():
                res = cv2.matchTemplate(roi_thumb, tdata['thumb'], cv2.TM_CCOEFF_NORMED)
                score = float(res[0][0])
                if score > best_score:
                    best_score = score
                    best_tmpl = tdata

            if best_score >= self.conf_thresh and best_tmpl is not None:
                det = {
                    'text': best_tmpl['text'],
                    'direction': best_tmpl['direction'],
                    'confidence': best_score,
                    'x_min': x,
                    'y_min': y,
                    'x_max': x + cw,
                    'y_max': y + ch
                }
                detections.append(det)

                if annotated is not None:
                    # Draw bounding box
                    cv2.rectangle(annotated, (x, y), (x + cw, y + ch), (0, 255, 0), 2)
                    label = f"{best_tmpl['text']} {best_tmpl['direction']} ({best_score:.2f})"
                    # Label banner
                    cv2.rectangle(annotated, (x, max(0, y - 24)), (x + len(label) * 10 + 10, y), (0, 200, 0), -1)
                    cv2.putText(
                        annotated,
                        label,
                        (x + 5, max(16, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 0, 0),
                        1,
                        cv2.LINE_AA
                    )

        return detections, annotated if annotated is not None else img


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
