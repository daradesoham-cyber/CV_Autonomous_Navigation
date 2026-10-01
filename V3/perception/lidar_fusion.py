"""
Camera-LiDAR association for V3 (pure functions, no ROS).

This is a faithful port of the V2.6 association in
ros2_ws/src/autonomous_robot_perception/autonomous_robot_perception/lidar_camera_fusion_node.py
(camera_ray_to_lidar_angle, extract_robust_cluster_distance and the angular-envelope part of
detections_callback), with the same default parameters, so the V2.6 behaviour can be evaluated
against Gazebo ground truth and reused by V3 nodes without modifying V2.6:
  - camera bearing of a box column: linear mapping (0.5 - u/W) * hfov   (as V2.6)
  - LiDAR angular envelope over depths {0.4, 0.8, 1.5, 3, 5} m with the camera-LiDAR baseline
    (camera 0.12 m ahead of the LiDAR), +-0.03 rad tolerance
  - ranges in the envelope -> gap clustering (0.35 m) -> closest cluster with >= 3 points ->
    20th percentile = object front-surface range
  - position in base_link: (0.10 + d cos b, d sin b)  (LiDAR is 0.10 m ahead of base_link)
"""
import math

import numpy as np

HFOV = 1.15
IMG_W = 640.0
CAM_DX_FROM_LIDAR = 0.12
CAM_DY_FROM_LIDAR = 0.0
LIDAR_X_IN_BASE = 0.10
CLUSTER_TOL = 0.35
MIN_CLUSTER_PTS = 3


def camera_ray_to_lidar_angle(theta_cam, depth):
    return math.atan2(CAM_DY_FROM_LIDAR + depth * math.sin(theta_cam), CAM_DX_FROM_LIDAR + depth * math.cos(theta_cam))


def robust_cluster_distance(valid_ranges):
    if len(valid_ranges) < MIN_CLUSTER_PTS:
        return -1.0
    r = np.sort(valid_ranges)
    clusters, cur = [], [r[0]]
    for v in r[1:]:
        if v - cur[-1] <= CLUSTER_TOL:
            cur.append(v)
        else:
            if len(cur) >= MIN_CLUSTER_PTS:
                clusters.append(cur)
            cur = [v]
    if len(cur) >= MIN_CLUSTER_PTS:
        clusters.append(cur)
    if not clusters:
        return -1.0
    return float(np.percentile(clusters[0], 20))


def fuse_box(x_min, x_max, ranges, angle_min, angle_inc, range_min, range_max):
    """Returns dict(distance, bearing, base_xy, n_beams, n_valid) for one detection box."""
    ranges = np.asarray(ranges, dtype=float)
    xc, bw = (x_min + x_max) / 2.0, x_max - x_min
    bearing_cam = (0.5 - xc / IMG_W) * HFOV
    span = (bw / IMG_W) * HFOV
    lo, hi = bearing_cam - span / 2, bearing_cam + span / 2
    angles = [camera_ray_to_lidar_angle(a, d) for d in (0.4, 0.8, 1.5, 3.0, 5.0) for a in (lo, hi, bearing_cam)]
    amin, amax = min(angles) - 0.03, max(angles) + 0.03
    i0, i1 = int((amin - angle_min) / angle_inc), int((amax - angle_min) / angle_inc)
    i0, i1 = sorted((i0, i1))
    i0, i1 = max(0, min(i0, len(ranges) - 1)), max(0, min(i1, len(ranges) - 1))
    sl = ranges[i0:i1 + 1]
    valid = sl[(sl >= range_min) & (sl <= range_max) & np.isfinite(sl)]
    d = robust_cluster_distance(valid)
    b = camera_ray_to_lidar_angle(bearing_cam, d) if d > 0 else bearing_cam
    base_xy = (LIDAR_X_IN_BASE + d * math.cos(b), d * math.sin(b)) if d > 0 else None
    return {"distance": d, "bearing": b, "base_xy": base_xy, "n_beams": int(i1 - i0 + 1), "n_valid": int(len(valid))}


def fuse_box_debug(x_min, x_max, ranges, angle_min, angle_inc, range_min, range_max):
    """Same result as fuse_box(), plus the beam indices of the envelope and of the selected cluster
    (for failure analysis only; identical selection logic)."""
    res = fuse_box(x_min, x_max, ranges, angle_min, angle_inc, range_min, range_max)
    r = np.asarray(ranges, dtype=float)
    xc, bw = (x_min + x_max) / 2.0, x_max - x_min
    bearing_cam = (0.5 - xc / IMG_W) * HFOV
    span = (bw / IMG_W) * HFOV
    lo, hi = bearing_cam - span / 2, bearing_cam + span / 2
    angles = [camera_ray_to_lidar_angle(a, d) for d in (0.4, 0.8, 1.5, 3.0, 5.0) for a in (lo, hi, bearing_cam)]
    amin, amax = min(angles) - 0.03, max(angles) + 0.03
    i0, i1 = sorted((int((amin - angle_min) / angle_inc), int((amax - angle_min) / angle_inc)))
    i0, i1 = max(0, min(i0, len(r) - 1)), max(0, min(i1, len(r) - 1))
    idx = np.arange(i0, i1 + 1)
    valid = idx[(r[idx] >= range_min) & (r[idx] <= range_max) & np.isfinite(r[idx])]
    res["envelope_idx"] = valid
    res["selected_idx"] = np.array([], dtype=int)
    if res["distance"] > 0 and len(valid):
        rs = np.sort(r[valid])
        clusters, cur = [], [rs[0]]
        for v in rs[1:]:
            if v - cur[-1] <= CLUSTER_TOL:
                cur.append(v)
            else:
                if len(cur) >= MIN_CLUSTER_PTS:
                    clusters.append(cur)
                cur = [v]
        if len(cur) >= MIN_CLUSTER_PTS:
            clusters.append(cur)
        c0 = clusters[0]
        res["selected_idx"] = valid[(r[valid] >= c0[0]) & (r[valid] <= c0[-1])]
    return res
