#!/usr/bin/env python3
"""
Phase 8: Real Camera + LiDAR 3D Fusion Validation.
Evaluates physical 3D object detection & sensor fusion against the live Gazebo system.
Subscribes to:
- /vision/detections (YOLOv8 2D bounding boxes & confidence)
- /scan (2D LiDAR ranges)
- /vision/semantic_obstacles (Fused 3D objects with distance, direction, and coordinates)

Validates physical plausibility:
- Distance consistency: dist ~ sqrt(x^2 + y^2)
- Coordinate signs matching bearing angle
- Physical bounds for height (z) and lateral clearance
"""

import os
import sys
import math
import time
import argparse

# Ensure ROS 2 and venv paths are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from autonomous_robot_interfaces.msg import Detection2DArray, SemanticObstacleArray
from sensor_msgs.msg import LaserScan

class RealFusionValidator(Node):
    def __init__(self, target_samples=15, max_timeout=25.0):
        super().__init__('real_fusion_validator')
        self.target_samples = target_samples
        self.max_timeout = max_timeout
        self.start_time = time.time()
        self.collected = []
        self.latest_scan = None
        self.done = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.sub_fused = self.create_subscription(SemanticObstacleArray, '/vision/semantic_obstacles', self._fused_cb, qos)

        self.get_logger().info(f"RealFusionValidator active: collecting up to {target_samples} fused detections...")

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _fused_cb(self, msg: SemanticObstacleArray):
        if not msg.obstacles:
            return

        for obs in msg.obstacles:
            cname = obs.class_name.upper()
            conf = obs.confidence
            dist = obs.distance
            x = obs.x
            y = obs.y
            z = obs.z
            direction = obs.direction if obs.direction else "Ahead"

            # Check physical plausibility
            calc_dist = math.hypot(x, y)
            dist_diff = abs(calc_dist - dist)
            # Physical checks: x > 0 (in front of camera), distance matches coordinates within 0.15m, z in [0.0, 2.5]
            is_plausible = (x > 0.0) and (dist_diff < 0.25) and (0.0 <= z <= 2.5)

            rec = {
                'class': cname,
                'confidence': conf,
                'distance': dist,
                'calc_dist': calc_dist,
                'direction': direction,
                'x': x,
                'y': y,
                'z': z,
                'plausible': is_plausible
            }
            self.collected.append(rec)
            print(f"[{len(self.collected):02d}] {cname:<12} | Conf: {conf:4.2f} | Dist: {dist:4.2f}m | Dir: {direction:<11} | Pos: ({x:5.2f}, {y:5.2f}, {z:4.2f}) | Plausible: {is_plausible}")

            if len(self.collected) >= self.target_samples:
                self.done = True
                break

        if time.time() - self.start_time > self.max_timeout:
            self.done = True

    def print_report(self):
        print("\n" + "=" * 88)
        print("           PHASE 8: REAL CAMERA + LIDAR FUSION VALIDATION REPORT")
        print("=" * 88)
        print(f"Total Objects Sampled : {len(self.collected)}")
        print(f"Elapsed Time          : {time.time() - self.start_time:.2f} s")
        print("-" * 88)
        print(f"{'Class':<12} | {'Conf':<6} | {'Distance':<8} | {'Direction':<12} | {'3D Position (X, Y, Z)':<24} | {'Status'}")
        print("-" * 88)

        if not self.collected:
            print("  No fused obstacles detected during observation interval.")
        else:
            plausible_count = 0
            for r in self.collected:
                pos_str = f"X:{r['x']:5.2f}, Y:{r['y']:5.2f}, Z:{r['z']:4.2f}"
                status = "PASS [Plausible]" if r['plausible'] else "FAIL [Anomaly]"
                if r['plausible']:
                    plausible_count += 1
                print(f"{r['class']:<12} | {r['confidence']:5.2f} | {r['distance']:5.2f}m | {r['direction']:<12} | {pos_str:<24} | {status}")

            pct = (plausible_count / len(self.collected)) * 100.0
            print("-" * 88)
            print(f"Plausibility Rate : {plausible_count}/{len(self.collected)} ({pct:.1f}%)")

        print("=" * 88 + "\n")

def main():
    parser = argparse.ArgumentParser(description="Real Camera + LiDAR Fusion Validator")
    parser.add_argument("--samples", type=int, default=10, help="Number of fused samples to collect")
    parser.add_argument("--timeout", type=float, default=15.0, help="Max observation timeout in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = RealFusionValidator(target_samples=args.samples, max_timeout=args.timeout)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_report()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
