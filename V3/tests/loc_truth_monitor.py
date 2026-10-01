#!/usr/bin/env python3
"""Log Gazebo ground truth (robot + traffic models) against AMCL at 2 Hz: loc_truth_monitor.py OUT.jsonl DURATION_S"""
import json, math, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node as GzNode

out, dur = sys.argv[1], float(sys.argv[2])
truth, lock, amcl = {}, threading.Lock(), [None]
def on_pose(m):
    with lock:
        for p in m.pose:
            if p.name == 'autonomous_robot' or p.name.startswith('dynamic_'):
                truth[p.name] = (round(p.position.x, 3), round(p.position.y, 3))
g = GzNode(); g.subscribe(Pose_V, '/world/realistic_facility_world/pose/info', on_pose)
rclpy.init(); n = Node('loc_truth_monitor')
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: amcl.__setitem__(0, (m.pose.pose.position.x, m.pose.pose.position.y)), q)
t0 = time.time(); f = open(out, 'w'); worst = 0.0
while time.time() - t0 < dur:
    end = time.time() + 0.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    with lock:
        t = dict(truth)
    r, a = t.get('autonomous_robot'), amcl[0]
    err = math.dist(r, a) if r and a else None
    near = {k: round(math.dist(r, v), 2) for k, v in t.items() if k != 'autonomous_robot' and r and math.dist(r, v) < 2.0}
    worst = max(worst, err or 0)
    f.write(json.dumps({'t': round(time.time() - t0, 1), 'truth': r, 'amcl': a and (round(a[0], 3), round(a[1], 3)), 'err': err and round(err, 3), 'near': near}) + '\n'); f.flush()
print('max localization error m', round(worst, 3))
