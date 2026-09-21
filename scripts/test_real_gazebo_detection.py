#!/usr/bin/env python3
"""
Real Gazebo CV Detection Evaluation Script.
Subscribes to live /camera/image_raw and /vision/detections from the running Gazebo simulation.
Evaluates and reports detection metrics over real camera frames.

Usage:
  python3 scripts/test_real_gazebo_detection.py --frames 50
  python3 scripts/test_real_gazebo_detection.py --duration 10.0
"""
import os
import sys
import time
import argparse
from collections import defaultdict

# Ensure ROS 2 and venv paths are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from autonomous_robot_interfaces.msg import Detection2DArray

class RealGazeboDetectionEvaluator(Node):
    def __init__(self, target_frames=50, max_duration=30.0):
        super().__init__('real_gazebo_detection_evaluator')
        self.target_frames = target_frames
        self.max_duration = max_duration
        self.start_time = None

        self.total_frames = 0
        self.frame_detections = []  # List of dicts per frame
        self.class_detection_counts = defaultdict(int)  # Frames in which class appeared
        self.class_total_instances = defaultdict(int)   # Total instances across all frames
        self.class_confidences = defaultdict(list)
        self.class_boxes = defaultdict(list)

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.sub_detections = self.create_subscription(
            Detection2DArray, '/vision/detections', self.detections_callback, 10
        )

        self.latest_detections = None
        self.done = False

        self.get_logger().info(
            f'RealGazeboDetectionEvaluator started: targeting {self.target_frames} frames (max {self.max_duration}s)...'
        )

    def image_callback(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()

    def detections_callback(self, msg: Detection2DArray):
        if self.start_time is None:
            self.start_time = time.time()

        self.total_frames += 1
        current_dets = msg.detections

        # Record classes present in this frame
        classes_in_frame = set()
        for det in current_dets:
            cname = det.class_name.upper()
            classes_in_frame.add(cname)
            self.class_total_instances[cname] += 1
            self.class_confidences[cname].append(det.confidence)
            self.class_boxes[cname].append((det.x_min, det.y_min, det.x_max, det.y_max))

            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            print(
                f"[FRAME {self.total_frames:03d} | t={stamp_sec:.3f}s] "
                f"Class: {cname:12s} | Conf: {det.confidence:.2f} | "
                f"Box: [{det.x_min:5.1f}, {det.y_min:5.1f}, {det.x_max:5.1f}, {det.y_max:5.1f}]"
            )

        for cname in classes_in_frame:
            self.class_detection_counts[cname] += 1

        if self.total_frames >= self.target_frames or (time.time() - self.start_time) >= self.max_duration:
            self.done = True

    def print_summary(self):
        elapsed = time.time() - (self.start_time if self.start_time else time.time())
        fps = self.total_frames / max(0.001, elapsed)

        print("\n" + "=" * 70)
        print(f"       REAL GAZEBO CV DETECTION EVALUATION REPORT")
        print("=" * 70)
        print(f"Total Frames Evaluated : {self.total_frames}")
        print(f"Elapsed Time           : {elapsed:.2f} s ({fps:.1f} FPS)")
        print(f"Subscribed Topics      : /camera/image_raw, /vision/detections")
        print("-" * 70)
        print(f"{'CLASS':<15} | {'FRAMES DETECTED':<18} | {'RATE':<8} | {'AVG CONF':<10} | {'TOTAL DETS':<10}")
        print("-" * 70)

        if not self.class_detection_counts:
            print("  No objects detected in camera field of view.")
        else:
            for cname in sorted(self.class_detection_counts.keys()):
                count = self.class_detection_counts[cname]
                rate = (count / max(1, self.total_frames)) * 100.0
                confs = self.class_confidences[cname]
                avg_conf = sum(confs) / max(1, len(confs))
                tot = self.class_total_instances[cname]
                print(
                    f"{cname:<15} | {count:3d}/{self.total_frames:3d} frames     | {rate:5.1f}% | {avg_conf:7.2f}    | {tot:5d}"
                )

        print("=" * 70 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Evaluate real Gazebo YOLO object detections')
    parser.add_argument('--frames', '-f', type=int, default=50, help='Number of frames to evaluate (default: 50)')
    parser.add_argument('--duration', '-d', type=float, default=30.0, help='Maximum duration in seconds (default: 30)')
    args = parser.parse_args()

    rclpy.init()
    evaluator = RealGazeboDetectionEvaluator(target_frames=args.frames, max_duration=args.duration)

    try:
        while rclpy.ok() and not evaluator.done:
            rclpy.spin_once(evaluator, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.print_summary()
        evaluator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
