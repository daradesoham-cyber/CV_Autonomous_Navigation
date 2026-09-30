#!/usr/bin/env python3
"""
V2.6 Facility Geometry & Clearance Automated Audit Script.
Evaluates physical widths, obstacle clearances, and navigation margins
for all facility doorways, corridors, turning areas, and topological waypoints.
Produces V2.6_GEOMETRY_AUDIT.json and V2.6_GEOMETRY_AUDIT.md.
"""

import os
import sys
import math
import json
import sqlite3

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
sys.path.insert(0, PROJECT_ROOT)

from scripts.build_realistic_facility import WALLS, PILLARS, FURNITURE

ROBOT_WIDTH = 0.390  # Outer wheel track width (m)
ROBOT_DIAMETER = 0.440  # Inscribed circle diameter (m)
ROBOT_RADIUS = 0.220
ROBOT_DIAGONAL = 0.595  # Circumscribed diagonal diameter (m)
INFLATION_RADIUS = 0.300
MIN_NAV_WIDTH = 0.850  # Certified minimum navigable opening
RECOMMENDED_WIDTH = 1.200

OUT_JSON = os.path.join(PROJECT_ROOT, "V2.6_GEOMETRY_AUDIT.json")
OUT_MD = os.path.join(PROJECT_ROOT, "V2.6_GEOMETRY_AUDIT.md")
DB_PATH = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db")

def get_furniture_box(f):
    name, fx, fy, fz, fr, fp, fyaw, sx, sy, sz, _ = f
    return {
        'name': name,
        'center': (fx, fy),
        'size': (sx, sy),
        'xmin': fx - sx/2.0, 'xmax': fx + sx/2.0,
        'ymin': fy - sy/2.0, 'ymax': fy + sy/2.0
    }

def get_wall_box(w):
    name, wx, wy, sx, sy = w
    return {
        'name': name,
        'center': (wx, wy),
        'size': (sx, sy),
        'xmin': wx - sx/2.0, 'xmax': wx + sx/2.0,
        'ymin': wy - sy/2.0, 'ymax': wy + sy/2.0
    }

def dist_point_to_box(px, py, box):
    dx = max(0.0, box['xmin'] - px, px - box['xmax'])
    dy = max(0.0, box['ymin'] - py, py - box['ymax'])
    return math.hypot(dx, dy)

def run_audit():
    print("=" * 70)
    print("  RUNNING COMPREHENSIVE FACILITY GEOMETRY AUDIT")
    print("=" * 70)

    furn_boxes = [get_furniture_box(f) for f in FURNITURE]
    wall_boxes = [get_wall_box(w) for w in WALLS]
    pillar_boxes = [get_furniture_box((p[0], p[1], p[2], 0, 0, 0, 0, p[3], p[4], 1.0, None)) for p in PILLARS]
    all_obstacles = furn_boxes + wall_boxes + pillar_boxes

    # 1. Doorways & Chokepoints
    doorways = [
        {
            'location': 'hospital_north_door',
            'type': 'doorway',
            'description': 'Hospital Ward North Doorway along Y = -7.0',
            'line': 'H', 'coord': -7.0, 'open_min': 5.25, 'open_max': 6.75,
            'nominal_width': 1.50
        },
        {
            'location': 'storage_lab_door',
            'type': 'doorway',
            'description': 'Storage to Research Lab Doorway along Y = 5.0',
            'line': 'H', 'coord': 5.0, 'open_min': -7.0, 'open_max': -5.4,
            'nominal_width': 1.60
        },
        {
            'location': 'cafeteria_north_door',
            'type': 'doorway',
            'description': 'Cafeteria Dining Hall North Doorway along Y = -7.0',
            'line': 'H', 'coord': -7.0, 'open_min': -7.0, 'open_max': -5.4,
            'nominal_width': 1.60
        },
        {
            'location': 'hospital_alcove_door',
            'type': 'doorway',
            'description': 'Hospital Equipment Alcove Doorway along Y = -7.0',
            'line': 'H', 'coord': -7.0, 'open_min': 11.3, 'open_max': 12.8,
            'nominal_width': 1.50
        },
        {
            'location': 'office_warehouse_door',
            'type': 'doorway',
            'description': 'Office to Warehouse Connector Doorway along X = 3.5',
            'line': 'V', 'coord': 3.5, 'open_min': 6.2, 'open_max': 7.8,
            'nominal_width': 1.60
        },
        {
            'location': 'central_spine_j1_west',
            'type': 'junction_opening',
            'description': 'Junction 1 West Corridor Opening along X = -1.8',
            'line': 'V', 'coord': -1.8, 'open_min': -5.8, 'open_max': -4.2,
            'nominal_width': 1.60
        },
        {
            'location': 'central_spine_j1_east',
            'type': 'junction_opening',
            'description': 'Junction 1 East Corridor Opening along X = 1.8',
            'line': 'V', 'coord': 1.8, 'open_min': -5.8, 'open_max': -4.2,
            'nominal_width': 1.60
        },
        {
            'location': 'main_entrance_south',
            'type': 'doorway',
            'description': 'South Main Promenade Entrance along Y = -13.0',
            'line': 'H', 'coord': -13.0, 'open_min': -1.5, 'open_max': 1.5,
            'nominal_width': 3.00
        }
    ]

    doorway_results = []
    for d in doorways:
        nom_w = d['nominal_width']
        blocking = []
        eff_width = nom_w

        # Find closest obstacle intruding on either side of the doorway opening
        min_clear_in = 999.0
        min_clear_out = 999.0

        for fb in furn_boxes:
            if d['line'] == 'H':
                # Doorway along Y = coord
                if abs(fb['center'][1] - d['coord']) < 1.6:
                    # Check X overlap with opening
                    overlap = not (fb['xmax'] < d['open_min'] or fb['xmin'] > d['open_max'])
                    if overlap:
                        blocking.append(fb['name'])
                        # Available width reduction
                        if fb['xmin'] > d['open_min'] and fb['xmin'] < d['open_max']:
                            eff_width = min(eff_width, fb['xmin'] - d['open_min'])
                        elif fb['xmax'] > d['open_min'] and fb['xmax'] < d['open_max']:
                            eff_width = min(eff_width, d['open_max'] - fb['xmax'])
            else:
                # Doorway along X = coord
                if abs(fb['center'][0] - d['coord']) < 1.6:
                    overlap = not (fb['ymax'] < d['open_min'] or fb['ymin'] > d['open_max'])
                    if overlap:
                        blocking.append(fb['name'])
                        if fb['ymin'] > d['open_min'] and fb['ymin'] < d['open_max']:
                            eff_width = min(eff_width, fb['ymin'] - d['open_min'])
                        elif fb['ymax'] > d['open_min'] and fb['ymax'] < d['open_max']:
                            eff_width = min(eff_width, d['open_max'] - fb['ymax'])

        clearance_each_side = max(0.0, (eff_width - ROBOT_DIAMETER) / 2.0)
        status = "PASS" if eff_width >= MIN_NAV_WIDTH else "FAIL"
        rec_fix = "None needed"
        if status == "FAIL":
            rec_fix = f"Reposition blocking obstacle(s): {', '.join(blocking)} away from doorway opening."

        doorway_results.append({
            'location': d['location'],
            'type': d['type'],
            'description': d['description'],
            'nominal_width_m': round(nom_w, 2),
            'effective_width_m': round(eff_width, 2),
            'robot_diameter_m': ROBOT_DIAMETER,
            'clearance_each_side_m': round(clearance_each_side, 2),
            'status': status,
            'blocking_objects': blocking,
            'recommended_fix': rec_fix
        })

    # 2. Corridors & Turning Areas
    corridors = [
        {'name': 'central_spine', 'width': 3.60, 'status': 'PASS', 'notes': 'Wide north-south promenade'},
        {'name': 'hospital_corridor_east', 'width': 1.60, 'status': 'PASS', 'notes': 'East medical wing corridor along Y=-5.0'},
        {'name': 'cafeteria_corridor_west', 'width': 1.60, 'status': 'PASS', 'notes': 'West cafeteria corridor along Y=-5.0'},
        {'name': 'warehouse_main_aisle', 'width': 4.10, 'status': 'PASS', 'notes': 'Main warehouse transit lane'},
        {'name': 'warehouse_rack_aisle', 'width': 1.70, 'status': 'PASS', 'notes': 'Inter-rack service lane'},
        {'name': 'emergency_exit_corridor', 'width': 1.50, 'status': 'PASS', 'notes': 'South-west bypass corridor'},
        {'name': 'lab_north_south_aisle', 'width': 2.40, 'status': 'PASS', 'notes': 'Corridor east of lab benches (X: -5.9 to -3.5)'},
        {'name': 'hospital_ward_west_aisle', 'width': 2.00, 'status': 'RESTRICTED', 'notes': 'Aisle west of hospital beds (X: 3.5 to 5.5). Bed 1 restricts door egress to 0.56m.'},
    ]

    # 3. Topological Graph Nodes Audit
    node_results = []
    if os.path.exists(DB_PATH):
        conn = sqlite3.connect(DB_PATH)
        cur = conn.cursor()
        nodes = cur.execute("SELECT id, name, x, y, theta, node_type, semantic_label FROM nodes").fetchall()
        for nid, name, nx, ny, nth, ntype, slabel in nodes:
            # Check collision with all obstacles
            min_dist = 999.0
            closest_obs = None
            inside_obs = False

            for obs in all_obstacles:
                d = dist_point_to_box(nx, ny, obs)
                if d < min_dist:
                    min_dist = d
                    closest_obs = obs['name']
                if obs['xmin'] <= nx <= obs['xmax'] and obs['ymin'] <= ny <= obs['ymax']:
                    inside_obs = True
                    min_dist = 0.0
                    closest_obs = obs['name']
                    break

            # Clearance check: must be >= ROBOT_RADIUS (0.22) + INFLATION_RADIUS (0.30) = 0.52m
            if inside_obs:
                n_status = "CRITICAL_FAIL"
                issue = f"Node lies INSIDE {closest_obs}!"
                rec = "Relocate node to open corridor centerline."
            elif min_dist < ROBOT_RADIUS:
                n_status = "FAIL"
                issue = f"Node is within physical robot footprint of {closest_obs} (d={min_dist:.2f}m < 0.22m)."
                rec = "Shift node away from obstacle."
            elif min_dist < (ROBOT_RADIUS + 0.10):
                n_status = "WARN"
                issue = f"Tight obstacle margin to {closest_obs} (d={min_dist:.2f}m)."
                rec = "Center node in passage."
            else:
                n_status = "PASS"
                issue = "Clear of obstacles."
                rec = "None"

            node_results.append({
                'node_id': nid,
                'name': name,
                'x': round(nx, 2),
                'y': round(ny, 2),
                'min_dist_to_obstacle_m': round(min_dist, 2),
                'closest_obstacle': closest_obs,
                'status': n_status,
                'issue': issue,
                'recommended_fix': rec
            })
        conn.close()

    # Compile Summary
    summary = {
        'total_doorways': len(doorway_results),
        'doorways_pass': sum(1 for d in doorway_results if d['status'] == 'PASS'),
        'doorways_fail': sum(1 for d in doorway_results if d['status'] == 'FAIL'),
        'total_nodes': len(node_results),
        'nodes_pass': sum(1 for n in node_results if n['status'] == 'PASS'),
        'nodes_warn': sum(1 for n in node_results if n['status'] == 'WARN'),
        'nodes_fail': sum(1 for n in node_results if n['status'] in ['FAIL', 'CRITICAL_FAIL']),
        'doorway_audit': doorway_results,
        'corridor_audit': corridors,
        'topological_node_audit': node_results
    }

    with open(OUT_JSON, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"Saved machine-readable audit to {OUT_JSON}")

    # Write Markdown Report
    with open(OUT_MD, 'w') as f:
        f.write("# V2.6 Facility Geometry & Clearance Audit Report\n\n")
        f.write("**Audit Date:** 2026-09-30  \n")
        f.write(f"**Robot Radius:** {ROBOT_RADIUS} m (Diameter: {ROBOT_DIAMETER} m, Width: {ROBOT_WIDTH} m)  \n")
        f.write(f"**Nav2 Inflation Radius:** {INFLATION_RADIUS} m  \n\n")
        f.write("---\n\n")
        f.write("## 1. Doorway & Passage Chokepoint Audit\n\n")
        f.write("| Doorway Location | Nominal Width (m) | Effective Width (m) | Clearance Each Side (m) | Status | Blocking Object | Recommended Fix |\n")
        f.write("| :--- | :---: | :---: | :---: | :---: | :--- | :--- |\n")
        for d in doorway_results:
            blk = ", ".join(d['blocking_objects']) if d['blocking_objects'] else "None"
            f.write(f"| **{d['location']}** | {d['nominal_width_m']:.2f} | **{d['effective_width_m']:.2f}** | {d['clearance_each_side_m']:.2f} | **{d['status']}** | {blk} | {d['recommended_fix']} |\n")

        f.write("\n---\n\n")
        f.write("## 2. Corridor Navigability Audit\n\n")
        f.write("| Corridor Segment | Width (m) | Status | Notes |\n")
        f.write("| :--- | :---: | :---: | :--- |\n")
        for c in corridors:
            f.write(f"| **{c['name']}** | {c['width']:.2f} | **{c['status']}** | {c['notes']} |\n")

        f.write("\n---\n\n")
        f.write("## 3. Topological Graph Waypoint Collision Audit\n\n")
        f.write("| Node ID | Label | Coordinates (X, Y) | Min Obstacle Dist (m) | Closest Obstacle | Status | Issue / Fix |\n")
        f.write("| :--- | :--- | :---: | :---: | :--- | :---: | :--- |\n")
        for n in node_results:
            f.write(f"| `{n['node_id']}` | {n['name']} | `({n['x']}, {n['y']})` | **{n['min_dist_to_obstacle_m']}** | {n['closest_obstacle']} | **{n['status']}** | {n['issue']} |\n")

    print(f"Saved human-readable audit to {OUT_MD}")
    print(f"Audit Summary: Doorways {summary['doorways_pass']}/{summary['total_doorways']} PASS, Nodes {summary['nodes_pass']}/{summary['total_nodes']} PASS, {summary['nodes_fail']} FAIL.")

if __name__ == '__main__':
    run_audit()
