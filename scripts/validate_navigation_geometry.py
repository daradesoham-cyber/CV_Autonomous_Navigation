#!/usr/bin/env python3
"""
V2.6 Navigation Geometry & Route Feasibility Validator.
Validates all topological nodes and edges against the physical facility world.
Checks swept footprint clearances, wall intersections, and obstacle buffers.
Produces PASS / FAIL metrics and outputs V2.6_TOPOLOGY_GEOMETRY_AUDIT.md.
"""

import os
import sys
import math
import sqlite3
import numpy as np

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
sys.path.insert(0, PROJECT_ROOT)

from scripts.build_realistic_facility import WALLS, PILLARS, FURNITURE

ROBOT_WIDTH = 0.390
ROBOT_DIAMETER = 0.440
ROBOT_RADIUS = 0.220
INFLATION_RADIUS = 0.300
MIN_CLEAR_MARGIN = 0.080  # Extra margin beyond physical radius for safe passing
PASS_THRESHOLD = ROBOT_RADIUS + MIN_CLEAR_MARGIN  # 0.30m (matches inflation_radius)

DB_PATH = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db")
OUT_MD = os.path.join(PROJECT_ROOT, "V2.6_TOPOLOGY_GEOMETRY_AUDIT.md")

def get_obstacle_boxes():
    boxes = []
    for name, wx, wy, sx, sy in WALLS:
        boxes.append({
            'name': name, 'type': 'wall',
            'xmin': wx - sx/2.0, 'xmax': wx + sx/2.0,
            'ymin': wy - sy/2.0, 'ymax': wy + sy/2.0
        })
    for name, px, py, sx, sy in PILLARS:
        boxes.append({
            'name': name, 'type': 'pillar',
            'xmin': px - sx/2.0, 'xmax': px + sx/2.0,
            'ymin': py - sy/2.0, 'ymax': py + sy/2.0
        })
    for f in FURNITURE:
        name = f[0]; fx, fy = f[1], f[2]; sx, sy = f[7], f[8]
        boxes.append({
            'name': name, 'type': 'furniture',
            'xmin': fx - sx/2.0, 'xmax': fx + sx/2.0,
            'ymin': fy - sy/2.0, 'ymax': fy + sy/2.0
        })
    return boxes

def point_to_box_dist(px, py, b):
    dx = max(0.0, b['xmin'] - px, px - b['xmax'])
    dy = max(0.0, b['ymin'] - py, py - b['ymax'])
    return math.hypot(dx, dy)

def is_point_inside(px, py, b):
    return (b['xmin'] <= px <= b['xmax']) and (b['ymin'] <= py <= b['ymax'])

def check_edge_feasibility(x1, y1, x2, y2, obstacles):
    length = math.hypot(x2 - x1, y2 - y1)
    if length < 1e-4:
        return True, 999.0, None, "Zero-length edge"

    num_samples = max(10, int(math.ceil(length / 0.05)))
    min_dist_along_edge = 999.0
    critical_obstacle = None
    colliding = False

    for s in np.linspace(0.0, 1.0, num_samples):
        px = x1 + s * (x2 - x1)
        py = y1 + s * (y2 - y1)
        for obs in obstacles:
            if is_point_inside(px, py, obs):
                return False, 0.0, obs['name'], f"Collides inside {obs['name']}"
            d = point_to_box_dist(px, py, obs)
            if d < min_dist_along_edge:
                min_dist_along_edge = d
                critical_obstacle = obs['name']

    if min_dist_along_edge < ROBOT_RADIUS:
        return False, min_dist_along_edge, critical_obstacle, f"Clearance {min_dist_along_edge:.2f}m < robot radius ({ROBOT_RADIUS}m)"
    elif min_dist_along_edge < PASS_THRESHOLD:
        return True, min_dist_along_edge, critical_obstacle, f"Restricted clearance ({min_dist_along_edge:.2f}m)"
    else:
        return True, min_dist_along_edge, critical_obstacle, f"Clear ({min_dist_along_edge:.2f}m)"

def validate_topology():
    print("=" * 70)
    print("  RUNNING TOPOLOGICAL GRAPH ROUTE FEASIBILITY AUDIT")
    print("=" * 70)

    obstacles = get_obstacle_boxes()
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    # Load Nodes
    nodes = {r[0]: {'name': r[1], 'x': r[2], 'y': r[3]} for r in cur.execute("SELECT id, name, x, y FROM nodes").fetchall()}

    # Load Edges
    edges_raw = cur.execute("SELECT from_node, to_node, is_blocked, is_dead_end FROM edges").fetchall()

    edge_results = []
    pass_count = 0
    fail_count = 0
    warn_count = 0

    for u, v, blocked, dead_end in edges_raw:
        if u not in nodes or v not in nodes:
            continue
        nu, nv = nodes[u], nodes[v]
        feasible, min_clear, crit_obs, note = check_edge_feasibility(nu['x'], nu['y'], nv['x'], nv['y'], obstacles)

        if not feasible:
            status = "FAIL"
            fail_count += 1
        elif min_clear < PASS_THRESHOLD:
            status = "RESTRICTED"
            warn_count += 1
        else:
            status = "PASS"
            pass_count += 1

        length = math.hypot(nv['x'] - nu['x'], nv['y'] - nu['y'])
        edge_results.append({
            'edge': f"{u} -> {v}",
            'from_node': u,
            'to_node': v,
            'length_m': round(length, 2),
            'min_clearance_m': round(min_clear, 2),
            'status': status,
            'critical_obstacle': crit_obs,
            'note': note
        })

    conn.close()

    # Generate Markdown Report
    with open(OUT_MD, 'w') as f:
        f.write("# V2.6 Topological Graph Geometry & Clearance Audit\n\n")
        f.write(f"**Audit Status:** COMPLETE  \n")
        f.write(f"**Total Directed Edges Audited:** {len(edge_results)}  \n")
        f.write(f"**Results:** {pass_count} PASS, {warn_count} RESTRICTED, {fail_count} FAIL  \n\n")
        f.write("---\n\n")
        f.write("## Edge-by-Edge Traversability Matrix\n\n")
        f.write("| Edge | Length (m) | Min Clearance (m) | Status | Critical Obstacle | Diagnostic Assessment |\n")
        f.write("| :--- | :---: | :---: | :---: | :--- | :--- |\n")
        for r in edge_results:
            f.write(f"| `{r['edge']}` | {r['length_m']:.2f} | **{r['min_clearance_m']:.2f}** | **{r['status']}** | {r['critical_obstacle']} | {r['note']} |\n")

    print(f"Topology Audit Finished:")
    print(f"  Total Edges: {len(edge_results)}")
    print(f"  PASS:        {pass_count}")
    print(f"  RESTRICTED:  {warn_count}")
    print(f"  FAIL:        {fail_count}")
    print(f"Saved full topology audit to {OUT_MD}")

    return fail_count == 0

if __name__ == '__main__':
    success = validate_topology()
    sys.exit(0 if success else 1)
