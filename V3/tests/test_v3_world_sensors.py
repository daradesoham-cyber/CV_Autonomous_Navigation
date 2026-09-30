#!/usr/bin/env python3
"""
V3 Phase 1 runtime sensor validation (requires the V3 world running:
    ros2 launch hospital_logistics v3_world.launch.py headless:=true).

S1  /camera/image_raw and /scan publish (message rate measured over 5 s)
S2  robot is teleported to each viewpoint (gz set_pose service); a camera frame is saved and the
    LiDAR range toward each nearby prop is compared with the ray/footprint distance computed from the
    prop's collision geometry (hospital_logistics_props.json). A prop counts as LiDAR-visible when a
    beam inside its angular span returns within 0.30 m of the geometric distance.
    Props lower than the LiDAR plane (z = 0.235 m) or above it only (e.g. wall plaques) are reported
    as 'not in LiDAR plane', not as failures.
Evidence: V3/docs/evidence/phase1_sensors/*.png and phase1_sensor_validation.json
"""
import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVID = os.path.join(V3_ROOT, "docs", "evidence", "phase1_sensors")
PROPS = json.load(open(os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")))
WORLD = "realistic_facility_world"
LIDAR_X = 0.10   # lidar.xacro origin relative to base_link
LIDAR_Z = 0.235  # 0.06 chassis offset + 0.175
TOL = 0.30

VIEWPOINTS = [
    ("ward_entry_south", 6.0, -7.3, -math.pi / 2),
    ("collection_point_west", 5.0, -9.0, math.pi),
    ("collection_point_south", 5.0, -9.0, -math.pi / 2),
    ("reception_east", 0.5, -10.8, 0.0),
    ("reception_north", 0.0, -11.0, math.pi / 2),
    ("lab_junction_northwest", -5.0, 7.0, 2.36),
    ("test_point_north", -7.0, 9.5, math.pi / 2),
    ("storage_west", -8.0, 3.0, math.pi),
    ("corridor_west_person", -3.0, -5.0, math.pi),
    ("corridor_east_trolley", 4.0, -5.0, 0.0),
]


class Grab(Node):
    def __init__(self):
        super().__init__("v3_sensor_validation")
        self.img, self.scan, self.n_img, self.n_scan = None, None, 0, 0
        self.create_subscription(Image, "/camera/image_raw", self.on_img, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_img(self, m):
        self.img, self.n_img = m, self.n_img + 1

    def on_scan(self, m):
        self.scan, self.n_scan = m, self.n_scan + 1


def spin_for(node, seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def set_pose(x, y, yaw):
    req = (f'name: "autonomous_robot", position: {{x: {x}, y: {y}, z: 0.1}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    out = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                          "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                         capture_output=True, text=True)
    return "true" in out.stdout


def ray_box(ox, oy, ang, fp):
    dx, dy = math.cos(ang), math.sin(ang)
    tmin, tmax = 0.0, 1e9
    for o, d, lo, hi in ((ox, dx, fp["xmin"], fp["xmax"]), (oy, dy, fp["ymin"], fp["ymax"])):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmin <= tmax else None


def save_png(msg, path):
    import cv2
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "rgb8":
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, arr)
    return float(arr.std())


def occluders():
    sys.path.insert(0, os.path.dirname(V3_ROOT))
    from scripts.build_realistic_facility import WALLS, PILLARS, FURNITURE
    boxes = [(n, {"xmin": x - sx / 2, "xmax": x + sx / 2, "ymin": y - sy / 2, "ymax": y + sy / 2})
             for n, x, y, sx, sy in list(WALLS) + list(PILLARS)]
    boxes += [(f[0], {"xmin": f[1] - f[7] / 2, "xmax": f[1] + f[7] / 2, "ymin": f[2] - f[8] / 2, "ymax": f[2] + f[8] / 2})
              for f in FURNITURE if f[0] not in PROPS["replaced_placeholders"] and f[9] >= LIDAR_Z]
    boxes += [(p["name"], p["footprint"]) for p in PROPS["props"]
              if p["footprint"]["height"] >= LIDAR_Z and not p["name"].startswith(("v3_sign_", "v3_payload"))]
    return boxes


OCCLUDERS = occluders()


def lidar_check(scan, rx, ry, yaw, max_range=6.0):
    lx, ly = rx + LIDAR_X * math.cos(yaw), ry + LIDAR_X * math.sin(yaw)
    rows = []
    for p in PROPS["props"]:
        fp = p["footprint"]
        cx, cy = (fp["xmin"] + fp["xmax"]) / 2, (fp["ymin"] + fp["ymax"]) / 2
        if math.hypot(cx - lx, cy - ly) > max_range:
            continue
        in_plane = fp["height"] >= LIDAR_Z and not p["name"].startswith(("v3_sign_", "v3_payload"))
        best, any_beam = None, False
        for i, r in enumerate(scan.ranges):
            a = scan.angle_min + i * scan.angle_increment
            expect = ray_box(lx, ly, yaw + a, fp)
            if expect is None:
                continue
            any_beam = True
            # geometric line of sight: skip beams where another obstacle is nearer than this prop
            blocked = any((t := ray_box(lx, ly, yaw + a, ob)) is not None and t < expect - TOL
                          for n, ob in OCCLUDERS if n != p["name"])
            if blocked:
                continue
            err = abs(r - expect) if math.isfinite(r) else float("inf")
            if best is None or err < best[0]:
                best = (err, round(float(r), 3), round(expect, 3), round(math.degrees(a), 1))
        if not any_beam:
            continue
        if not in_plane:
            status = "not in LiDAR plane"
        elif best is None:
            status = "OCCLUDED (no line of sight)"
        else:
            status = "VISIBLE" if best[0] <= TOL else "MISMATCH"
        if best is None:
            best = (None, None, None, None)
        rows.append({"prop": p["name"], "model": p["model"], "beam_deg": best[3], "measured_m": best[1],
                     "geometric_m": best[2], "status": status})
    return rows


def main():
    os.makedirs(EVID, exist_ok=True)
    rclpy.init()
    node = Grab()
    spin_for(node, 5.0)
    rates = {"camera_hz": node.n_img / 5.0, "scan_hz": node.n_scan / 5.0}
    result = {"S1_rates": rates, "S2_viewpoints": []}
    ok = rates["camera_hz"] > 1 and rates["scan_hz"] > 1 and node.img is not None and node.scan is not None
    print("S1", rates)
    if ok:
        for name, x, y, yaw in VIEWPOINTS:
            teleported = set_pose(x, y, yaw)
            spin_for(node, 2.5)  # let the robot settle and fresh frames arrive
            png = os.path.join(EVID, f"{name}.png")
            std = save_png(node.img, png)
            rows = lidar_check(node.scan, x, y, yaw)
            valid = sum(1 for r in node.scan.ranges if math.isfinite(r))
            result["S2_viewpoints"].append({"viewpoint": name, "pose": [x, y, round(yaw, 3)], "teleported": teleported,
                                            "image": os.path.relpath(png, V3_ROOT), "image_pixel_std": round(std, 1),
                                            "scan_finite_beams": valid, "props": rows})
            vis = [r["prop"] for r in rows if r["status"] == "VISIBLE"]
            other = [(r["prop"], r["status"]) for r in rows if r["status"] not in ("VISIBLE",)]
            print(f"{name:26s} tp={teleported} img_std={std:5.1f} finite={valid}\n    visible={vis}\n    other={other}")
    json.dump(result, open(os.path.join(EVID, "..", "phase1_sensor_validation.json"), "w"), indent=2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
