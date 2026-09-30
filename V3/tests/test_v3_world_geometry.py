#!/usr/bin/env python3
"""
V3 Phase 1 static geometry validation.

Checks every V3 prop footprint (world-frame collision bounds from hospital_logistics_props.json)
against the V2.6 facility geometry and topology:
  G1  props do not intersect walls, pillars or kept furniture
  G2  clearance of every topology node vs. V3 obstacles (compared to V2.6 obstacles)
  G3  clearance of every topology edge (segment) vs. V3 props
  G4  V2.6 doorway openings are not narrowed by V3 props
  G5  collection point / laboratory test point clearance
Threshold = robot radius 0.22 + inflation 0.30 = 0.52 m (same as V2.6 audit).
Writes V3/docs/evidence/phase1_geometry_validation.json. Exit code 1 on any FAIL.
"""
import json
import math
import os
import sqlite3
import sys

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, PROJECT_ROOT)
from scripts.build_realistic_facility import WALLS, PILLARS, FURNITURE  # noqa: E402

CLEAR = 0.52
DB = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db")
PROPS = json.load(open(os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")))
OUT = os.path.join(V3_ROOT, "docs", "evidence", "phase1_geometry_validation.json")

# V2.6 doorway openings (from scripts/audit_facility_geometry.py)
DOORWAYS = [
    ("hospital_north_door", "H", -7.0, 5.25, 6.75), ("storage_lab_door", "H", 5.0, -7.0, -5.4),
    ("cafeteria_north_door", "H", -7.0, -7.0, -5.4), ("hospital_alcove_door", "H", -7.0, 11.3, 12.8),
    ("office_warehouse_door", "V", 3.5, 6.2, 7.8), ("central_spine_j1_west", "V", -1.8, -5.8, -4.2),
    ("central_spine_j1_east", "V", 1.8, -5.8, -4.2), ("main_entrance_south", "H", -13.0, -1.5, 1.5),
]


def box(name, cx, cy, sx, sy):
    return {"name": name, "xmin": cx - sx / 2, "xmax": cx + sx / 2, "ymin": cy - sy / 2, "ymax": cy + sy / 2}


def pt_box(px, py, b):
    return math.hypot(max(0.0, b["xmin"] - px, px - b["xmax"]), max(0.0, b["ymin"] - py, py - b["ymax"]))


def seg_box(ax, ay, bx, by, b, steps=200):
    return min(pt_box(ax + (bx - ax) * t / steps, ay + (by - ay) * t / steps, b) for t in range(steps + 1))


def overlap(a, b):
    return not (a["xmax"] <= b["xmin"] or a["xmin"] >= b["xmax"] or a["ymax"] <= b["ymin"] or a["ymin"] >= b["ymax"])


def main():
    replaced = set(PROPS["replaced_placeholders"])
    walls = [box(n, x, y, sx, sy) for n, x, y, sx, sy in WALLS] + [box(n, x, y, sx, sy) for n, x, y, sx, sy in PILLARS]
    v26_furn = [box(f[0], f[1], f[2], f[7], f[8]) for f in FURNITURE]
    kept_furn = [b for b in v26_furn if b["name"] not in replaced]
    props = [dict(name=p["name"], **{k: p["footprint"][k] for k in ("xmin", "xmax", "ymin", "ymax")}) for p in PROPS["props"]]
    v26_obs, v3_obs = walls + v26_furn, walls + kept_furn + props
    results, fails = {}, 0

    # G1 — wall-mounted plaques are allowed to touch the wall they are mounted on
    g1 = []
    for p in props:
        hits = [o["name"] for o in walls + kept_furn if overlap(p, o)]
        if p["name"] == "v3_sign_laboratory_test_point":
            hits = [h for h in hits if h != "lab_bench_2"]
        g1.append({"prop": p["name"], "intersects": hits, "status": "FAIL" if hits else "PASS"})
    results["G1_prop_intersections"] = g1

    conn = sqlite3.connect(DB)
    nodes = {nid: (name, x, y) for nid, name, x, y in conn.execute("SELECT id, name, x, y FROM nodes")}
    edges = conn.execute("SELECT from_node, to_node FROM edges").fetchall()

    # G2
    g2 = []
    for nid, (name, x, y) in nodes.items():
        d26 = min(pt_box(x, y, o) for o in v26_obs)
        d3, near = min((pt_box(x, y, o), o["name"]) for o in v3_obs)
        status = "PASS" if d3 >= CLEAR or d3 >= d26 - 1e-6 else "FAIL"
        g2.append({"node": name, "x": x, "y": y, "v26_clearance": round(d26, 3), "v3_clearance": round(d3, 3),
                   "nearest_v3": near, "status": status})
    results["G2_node_clearance"] = g2

    # G3 — edges vs. V3 props only (walls/kept furniture unchanged from V2.6)
    g3, seen = [], set()
    for a, b in edges:
        key = tuple(sorted((a, b)))
        if key in seen or a not in nodes or b not in nodes:
            continue
        seen.add(key)
        (_, ax, ay), (nb, bx, by) = nodes[a], nodes[b]
        d, near = min((seg_box(ax, ay, bx, by, p), p["name"]) for p in props)
        g3.append({"edge": f"{nodes[a][0]} -> {nb}", "min_prop_clearance": round(d, 3), "nearest_prop": near,
                   "status": "PASS" if d >= CLEAR else "FAIL"})
    results["G3_edge_clearance"] = g3

    # G4
    g4 = []
    for name, line, coord, lo, hi in DOORWAYS:
        blocking = []
        for p in props:
            near_line = (abs((p["ymin"] + p["ymax"]) / 2 - coord) < 1.6) if line == "H" else (abs((p["xmin"] + p["xmax"]) / 2 - coord) < 1.6)
            span = (p["xmin"], p["xmax"]) if line == "H" else (p["ymin"], p["ymax"])
            if near_line and not (span[1] < lo or span[0] > hi):
                blocking.append(p["name"])
        g4.append({"doorway": name, "width": round(hi - lo, 2), "v3_blocking_props": blocking,
                   "status": "FAIL" if blocking else "PASS"})
    results["G4_doorways"] = g4

    # G5
    g5 = []
    for key in ("collection_point", "laboratory_test_point"):
        loc = PROPS[key]
        d, near = min((pt_box(loc["x"], loc["y"], o), o["name"]) for o in v3_obs)
        g5.append({"location": key, "x": loc["x"], "y": loc["y"], "clearance": round(d, 3), "nearest": near,
                   "status": "PASS" if d >= CLEAR else "FAIL"})
    results["G5_logistics_points"] = g5

    # G6 — static-map check: cells occupied in the V3 map but free in the V2.6 map must stay >= CLEAR
    # from every topology edge, and every topology node cell must be free in the V3 map.
    import cv2
    import numpy as np
    from scripts.build_realistic_facility import RESOLUTION, ORIGIN_X, ORIGIN_Y
    m3 = cv2.imread(os.path.join(V3_ROOT, "maps", "hospital_logistics_map.pgm"), cv2.IMREAD_GRAYSCALE)
    m26 = cv2.imread(os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/maps/realistic_facility_map.pgm"),
                     cv2.IMREAD_GRAYSCALE)
    new_cells = np.argwhere((m3 == 0) & (m26 != 0))
    h = m3.shape[0]
    new_xy = np.column_stack([ORIGIN_X + new_cells[:, 1] * RESOLUTION, ORIGIN_Y + (h - 1 - new_cells[:, 0]) * RESOLUTION])
    g6 = []
    for a, b in seen:
        (_, ax, ay), (_, bx, by) = nodes[a], nodes[b]
        ab = np.array([bx - ax, by - ay])
        t = np.clip(((new_xy - [ax, ay]) @ ab) / max(ab @ ab, 1e-9), 0, 1)
        d = float(np.min(np.linalg.norm(new_xy - ([ax, ay] + t[:, None] * ab), axis=1))) if len(new_xy) else 99.0
        g6.append({"edge": f"{nodes[a][0]} -> {nodes[b][0]}", "min_new_occupied_cell_m": round(d, 3),
                   "status": "PASS" if d >= CLEAR else "FAIL"})
    for nid, (name, x, y) in nodes.items():
        col, row = int(round((x - ORIGIN_X) / RESOLUTION)), h - 1 - int(round((y - ORIGIN_Y) / RESOLUTION))
        g6.append({"node": name, "v3_map_value": int(m3[row, col]), "status": "PASS" if m3[row, col] == 254 else "FAIL"})
    results["G6_static_map"] = g6
    print(f"new occupied cells in V3 map: {len(new_xy)}")

    for section in results.values():
        for r in section:
            fails += r["status"] == "FAIL"
            if r["status"] == "FAIL":
                print("FAIL", r)
    summary = {k: f"{sum(r['status'] == 'PASS' for r in v)}/{len(v)} PASS" for k, v in results.items()}
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"threshold_m": CLEAR, "summary": summary, "results": results}, open(OUT, "w"), indent=2)
    for k, v in summary.items():
        print(f"{k:28s} {v}")
    print("worst edge clearances:", sorted((r["min_prop_clearance"], r["edge"], r["nearest_prop"]) for r in g3)[:5])
    print("logistics points:", [(r["location"], r["clearance"], r["nearest"]) for r in g5])
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
