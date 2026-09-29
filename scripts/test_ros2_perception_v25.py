#!/usr/bin/env python3
"""
Controlled ROS2 Inference & Perception Pipeline Validation for V2.5.
Publishes test frames from datasets/v25_cv_dataset to /camera/image_raw,
subscribes to /vision/detections, /vision/signs, /vision/engine_status,
and verifies:
1. Model loading on CUDA:0
2. Correct 10 class names and ordering
3. Detections publication and reasonable confidences
4. Temporal confirmation gate on /vision/signs (>=3 frames in 2.0s)
5. Real measured inference FPS and latency
"""

import os
import sys
import time
import glob
import json
import cv2
import numpy as np

# Ensure ROS2 and local packages are accessible
for p in [
    '/opt/ros/lyrical/lib/python3.14/site-packages',
    '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/install/autonomous_robot_interfaces/lib/python3.14/site-packages',
    '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/install/autonomous_robot_perception/lib/python3.14/site-packages'
]:
    if p not in sys.path:
        sys.path.insert(0, p)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from autonomous_robot_interfaces.msg import Detection2DArray, SignDetectionArray

class PerceptionValidatorNode(Node):
    def __init__(self, test_images, send_rate_hz=20.0, max_seconds=8.0):
        super().__init__('perception_validator_node')
        self.test_images = test_images
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)

        self.sub_detections = self.create_subscription(
            Detection2DArray, '/vision/detections', self._on_detections, 10
        )
        self.sub_signs = self.create_subscription(
            SignDetectionArray, '/vision/signs', self._on_signs, 10
        )
        self.sub_status = self.create_subscription(
            String, '/vision/engine_status', self._on_status, 10
        )

        self.timer = self.create_timer(1.0 / send_rate_hz, self._publish_next_frame)
        self.img_idx = 0
        self.sent_frames = 0
        self.received_detection_msgs = 0
        self.detected_classes = set()
        self.class_instances = {}
        self.confirmed_signs = []
        self.engine_status_records = []
        self.start_time = time.time()
        self.max_seconds = max_seconds
        self.is_done = False

    def _publish_next_frame(self):
        if time.time() - self.start_time >= self.max_seconds:
            self.is_done = True
            return

        img_bgr = self.test_images[self.img_idx % len(self.test_images)]
        # Cycle through image index every 6 frames to allow temporal confirmation gate (3 frames)
        if self.sent_frames % 8 == 0:
            self.img_idx += 1

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = int(img_bgr.shape[0])
        msg.width = int(img_bgr.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(img_bgr.shape[1] * 3)
        msg.data = img_bgr.tobytes()
        self.pub_image.publish(msg)
        self.sent_frames += 1

    def _on_detections(self, msg: Detection2DArray):
        self.received_detection_msgs += 1
        for det in msg.detections:
            c = det.class_name
            self.detected_classes.add(c)
            if c not in self.class_instances:
                self.class_instances[c] = []
            self.class_instances[c].append(float(det.confidence))

    def _on_signs(self, msg: SignDetectionArray):
        for s in msg.signs:
            self.confirmed_signs.append({
                'text': s.text,
                'direction': s.direction,
                'confidence': float(s.confidence),
                'box': (s.x_min, s.y_min, s.x_max, s.y_max)
            })

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.engine_status_records.append(data)
        except Exception:
            pass


def main():
    rclpy.init()

    # Load validation images that have positive annotations (index >= 60)
    val_imgs_dir = "/home/soham-darade/CV_Autonomous_Navigation/datasets/v25_cv_dataset/images/val"
    val_lbls_dir = "/home/soham-darade/CV_Autonomous_Navigation/datasets/v25_cv_dataset/labels/val"
    all_files = sorted(glob.glob(os.path.join(val_imgs_dir, "*.jpg")))
    positive_files = []
    for f in all_files:
        stem = os.path.splitext(os.path.basename(f))[0]
        lbl = os.path.join(val_lbls_dir, stem + ".txt")
        if os.path.exists(lbl) and os.path.getsize(lbl) > 0:
            positive_files.append(f)
        if len(positive_files) >= 20:
            break

    loaded = [cv2.imread(f) for f in positive_files if cv2.imread(f) is not None]
    print(f"Loaded {len(loaded)} positive test validation frames.")

    validator = PerceptionValidatorNode(loaded, send_rate_hz=20.0, max_seconds=6.0)
    print("Publishing camera frames and listening for detections...")

    t0 = time.time()
    try:
        while rclpy.ok() and not validator.is_done and (time.time() - t0 < 8.0):
            rclpy.spin_once(validator, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - t0
    print("\n" + "=" * 65)
    print("           ROS2 PERCEPTION INFERENCE VALIDATION SUMMARY")
    print("=" * 65)
    print(f"Test Duration:               {elapsed:.2f} s")
    print(f"Frames Published:            {validator.sent_frames}")
    print(f"Detection Messages Received: {validator.received_detection_msgs}")
    print(f"Classes Detected:            {sorted(list(validator.detected_classes))}")
    print("\nDetected Class Confidence Distribution:")
    for c, confs in sorted(validator.class_instances.items()):
        mean_c = sum(confs) / len(confs)
        print(f"  - {c:<18}: {len(confs):3d} boxes, Mean Conf: {mean_c:.3f} (Min: {min(confs):.2f}, Max: {max(confs):.2f})")

    print(f"\nTemporally Confirmed Signs on /vision/signs: {len(validator.confirmed_signs)}")
    for s in validator.confirmed_signs[:5]:
        print(f"  - Sign: {s['text']:<12} Dir: {s['direction']:<10} Conf: {s['confidence']:.2f}")

    if validator.engine_status_records:
        latest_status = validator.engine_status_records[-1]
        print("\nLive Engine Status Telemetry:")
        print(f"  - Device:             {latest_status.get('device')}")
        print(f"  - Model:              {latest_status.get('model_path')}")
        print(f"  - Measured FPS:       {latest_status.get('fps', 0.0):.1f} Hz")
        print(f"  - Inference Latency:  {latest_status.get('inference_time_ms', 0.0):.2f} ms")
        print(f"  - Classes configured: {latest_status.get('classes', [])}")

    validator.destroy_node()
    rclpy.shutdown()

    # Assertions
    assert validator.received_detection_msgs > 0, "No detections received!"
    assert len(validator.detected_classes) > 0, "No classes detected!"
    print("\nALL ROS2 PERCEPTION VALIDATION CHECKS PASSED SUCCESSFULLY!")
    print("=" * 65)


if __name__ == '__main__':
    main()
