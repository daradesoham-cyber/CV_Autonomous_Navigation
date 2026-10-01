#!/usr/bin/env python3
"""
Record the perception -> costmap chain during a Nav2 run (diagnostics for the Phase 3 Nav2 regression):
/vision/semantic_obstacles (class, distance, bearing, x, y, dynamic, direction), the robot map pose
(TF map->base_link) and Gazebo ground-truth robot pose, until --seconds elapse.
Output: jsonl, one line per semantic-obstacle message.
Usage: python3 record_semantic_during_nav.py --out FILE --seconds 150
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from autonomous_robot_interfaces.msg import SemanticObstacleArray


class Rec(Node):
    def __init__(self, path):
        super().__init__("v3_semantic_nav_recorder")
        self.f = open(path, "w")
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(SemanticObstacleArray, "/vision/semantic_obstacles", self.cb, 20)

    def cb(self, m):
        pose = None
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            q = t.transform.rotation
            pose = [round(t.transform.translation.x, 3), round(t.transform.translation.y, 3),
                    round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3)]
        except Exception:
            pass
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.f.write(json.dumps({"stamp": st, "robot_map_pose": pose, "obstacles": [
            {"class": o.class_name, "distance": round(o.distance, 3), "bearing_deg": round(math.degrees(o.bearing), 1),
             "x": round(o.x, 3), "y": round(o.y, 3), "radius": o.safety_radius, "dynamic": o.is_dynamic,
             "direction": o.direction} for o in m.obstacles]}) + "\n")
        self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=150)
    a = ap.parse_args()
    rclpy.init()
    n = Rec(a.out)
    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.f.close()


if __name__ == "__main__":
    main()
