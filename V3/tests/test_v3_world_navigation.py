#!/usr/bin/env python3
"""
V3 Phase 1 navigation-geometry smoke test (not a benchmark).

Requires: V3 world + V2.6 localization (V3 map) + V2.6 Nav2 running, robot at the V2.6 spawn (0, -11, 1.57).
Sends real NavigateToPose goals through the V3 prop layout:
    spawn -> collection_point -> laboratory_test_point
Records Nav2 result status, duration, map-frame path length, minimum LiDAR range (and where), and the
map-frame trace. Position is the AMCL-localized map->base_link TF (raw /odom is not used for
position because the sensor test teleports the robot, which /odom does not follow). Nothing is
inferred: a leg passes only if Nav2 reports SUCCEEDED and the final map-frame position is within
0.5 m of the goal. --reset teleports the robot to the V2.6 spawn and re-initialises AMCL first.
Evidence: V3/docs/evidence/phase1_navigation.json
"""
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROPS = json.load(open(os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")))
OUT = os.path.join(V3_ROOT, "docs", "evidence", "phase1_navigation.json")
LEG_TIMEOUT = 300.0
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


class NavTest(Node):
    def __init__(self):
        super().__init__("v3_navigation_smoke_test")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pose, self.trace, self.min_scan = None, [], float("inf")
        self.min_scan_at = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform.translation
        except Exception:
            return
        self.pose = (t.x, t.y)
        if not self.trace or math.hypot(t.x - self.trace[-1][0], t.y - self.trace[-1][1]) > 0.05:
            self.trace.append((round(t.x, 3), round(t.y, 3)))

    def on_scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and r > m.range_min and (best is None or r < best[0]):
                best = (r, math.degrees(m.angle_min + i * m.angle_increment))
        if best and best[0] < self.min_scan:
            self.min_scan = best[0]
            self.min_scan_at = {"pose": [round(v, 2) for v in self.pose] if self.pose else None,
                                "beam_deg": round(best[1], 1)}

    def reset_to_spawn(self):
        import subprocess
        subprocess.run(["gz", "service", "-s", "/world/realistic_facility_world/set_pose", "--reqtype", "gz.msgs.Pose",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
                        'name: "autonomous_robot", position: {x: 0.0, y: -11.0, z: 0.1}, orientation: {z: 0.7071, w: 0.7071}'],
                       check=True, capture_output=True)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.y = -11.0
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = 0.7071, 0.7071
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        end = time.time() + 5
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.5)

    def spin_until(self, fut, timeout):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.done()

    def go(self, name, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.trace, self.min_scan, self.min_scan_at = [], float("inf"), None
        t0 = time.time()
        send = self.nav.send_goal_async(goal)
        self.spin_until(send, 10)
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"leg": name, "status": "REJECTED"}
        res = handle.get_result_async()
        finished = self.spin_until(res, LEG_TIMEOUT)
        status = STATUS.get(res.result().status, str(res.result().status)) if finished else "TIMEOUT"
        if not finished:
            handle.cancel_goal_async()
        length = sum(math.dist(a, b) for a, b in zip(self.trace, self.trace[1:]))
        err = math.hypot(self.pose[0] - x, self.pose[1] - y) if self.pose else None
        ok = status == "SUCCEEDED" and err is not None and err <= 0.5
        return {"leg": name, "goal": [x, y, round(yaw, 3)], "nav2_status": status, "duration_s": round(time.time() - t0, 1),
                "path_length_m": round(length, 2), "final_map_error_m": round(err, 3) if err is not None else None,
                "min_lidar_range_m": round(self.min_scan, 3), "min_lidar_at": self.min_scan_at, "result": "PASS" if ok else "FAIL",
                "map_trace": self.trace}


def main():
    rclpy.init()
    n = NavTest()
    if not n.nav.wait_for_server(timeout_sec=60):
        print("navigate_to_pose action server not available")
        return 1
    if "--reset" in sys.argv:
        n.reset_to_spawn()
    c, t = PROPS["collection_point"], PROPS["laboratory_test_point"]
    legs = []
    for name, loc in (("spawn->collection_point", c), ("collection_point->laboratory_test_point", t)):
        r = n.go(name, loc["x"], loc["y"], loc["yaw"])
        legs.append(r)
        print({k: v for k, v in r.items() if k != "map_trace"})
        if r.get("result") != "PASS":
            break
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"legs": legs}, open(OUT, "w"), indent=2)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if len(legs) == 2 and all(l.get("result") == "PASS" for l in legs) else 1


if __name__ == "__main__":
    sys.exit(main())
