#!/usr/bin/env python3
"""
V2.6 Phase 2 — Step 4: Hospital Start Condition Experiment.
Compares 3 hospital start poses under controlled conditions:
  Condition 0 (Baseline):  x=6.5, y=-9.5, yaw=-1.57 (Facing SOUTH into bed 2, exit is NORTH)
  Condition 1 (Cand 1):    x=6.5, y=-9.5, yaw=+1.57 (Between beds, facing NORTH towards doorway)
  Condition 2 (Cand 2):    x=6.0, y=-9.0, yaw=+1.57 (Center aisle, facing NORTH towards doorway)

Measures:
  - first 10s behavior (distance travelled, local planner failures, recovery triggers)
  - total recoveries (spin/backup/wait)
  - total replans
  - minimum clearance
  - door clearance / exit success (crossing y >= -7.0)
  - collision count
  - overall trajectory stability
Saves results to results/v26_phase2/hospital_start_experiments.json.
"""

import os
import sys
import time
import math
import json
import subprocess
import urllib.request
import urllib.parse
from typing import Dict, Any, List

API_BASE = "http://127.0.0.1:5050"
PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUTPUT_JSON = os.path.join(PROJECT_ROOT, "results/v26_phase2/hospital_start_experiments.json")

def euler_to_quat(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

def get_telemetry() -> Dict[str, Any]:
    try:
        req = urllib.request.Request(f"{API_BASE}/api/telemetry")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def send_goal(goal_label: str) -> Dict[str, Any]:
    data = json.dumps({"goal": goal_label}).encode('utf-8')
    req = urllib.request.Request(f"{API_BASE}/api/goal", data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode('utf-8'))

def send_control(action: str):
    data = json.dumps({"action": action}).encode('utf-8')
    req = urllib.request.Request(f"{API_BASE}/api/control", data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def set_robot_pose(x: float, y: float, yaw: float):
    print(f"  [POSE] Setting Gazebo robot pose to ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)...")
    _, _, qz, qw = euler_to_quat(yaw)
    cmd = [
        "gz", "service",
        "--service", "/world/realistic_facility_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'name: "autonomous_robot", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}'
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    # Sync AMCL
    amcl_script = os.path.join(PROJECT_ROOT, "scripts/set_start.py")
    t_wait = time.time()
    converged = False
    last_pub = 0.0

    while time.time() - t_wait < 20.0:
        now = time.time()
        if now - last_pub >= 2.0:
            last_pub = now
            amcl_cmd = [
                sys.executable,
                amcl_script,
                "--x", str(x),
                "--y", str(y),
                "--yaw", str(yaw)
            ]
            subprocess.run(amcl_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        telem = get_telemetry()
        rp = telem.get('robot_pose', {})
        px, py = rp.get('x', 999.0), rp.get('y', 999.0)
        if math.hypot(px - x, py - y) < 0.6:
            print(f"  [OK] AMCL confirmed pose at ({px:.2f}, {py:.2f})!")
            converged = True
            break
        time.sleep(0.3)

    if not converged:
        print(f"  [WARN] AMCL pose ({px:.2f}, {py:.2f}) did not fully converge within 20s.")
    time.sleep(1.0)

def run_hospital_test(name: str, x: float, y: float, yaw: float, duration_sec: float = 40.0) -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print(f"  RUNNING HOSPITAL START TEST: {name}")
    print(f"  Coordinates: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad / {math.degrees(yaw):.1f} deg)")
    print("=" * 70)

    # Abort previous and reset
    send_control("ABORT")
    time.sleep(1.5)

    # Set pose
    set_robot_pose(x, y, yaw)

    # Goal: STORAGE (requires leaving hospital ward through north doorway at y=-7.0)
    print("  Dispatching goal: 'STORAGE'...")
    send_goal("STORAGE")
    time.sleep(1.0)

    t_start = time.time()
    min_clearance = 10.0
    first_10s_dist = 0.0
    first_10s_state = "UNKNOWN"
    first_10s_recoveries = 0
    first_10s_replans = 0
    cleared_doorway = False
    time_to_doorway = -1.0
    recoveries_count = 0
    replans_count = 0
    ttc_yields = 0
    samples = []
    collisions = 0

    while time.time() - t_start < duration_sec:
        t_cur = time.time() - t_start
        telem = get_telemetry()
        pose = telem.get('robot_pose', {})
        rx = pose.get('x', 0.0)
        ry = pose.get('y', 0.0)
        dist_trav = telem.get('distance_travelled', 0.0)
        state = telem.get('state', 'UNKNOWN')
        cur_replans = telem.get('replans_count', 0)
        diag = telem.get('diagnostics', {})
        cur_recoveries = diag.get('recovery_count', 0)
        lidar_min = telem.get('lidar', {}).get('min_distance', 10.0)
        ttc = telem.get('ttc', -1.0)

        if 0.05 < lidar_min < min_clearance:
            min_clearance = lidar_min
            if lidar_min < 0.15:
                collisions += 1

        if ttc > 0 and ttc < 1.8:
            ttc_yields += 1

        if cur_replans > replans_count:
            replans_count = cur_replans

        if cur_recoveries > recoveries_count:
            recoveries_count = cur_recoveries

        # Check doorway clearance (doorway is at y = -7.0)
        if ry >= -7.0 and not cleared_doorway:
            cleared_doorway = True
            time_to_doorway = round(t_cur, 2)
            print(f"  >>> [DOORWAY CLEARED] at t={t_cur:.1f}s, pos=({rx:.2f}, {ry:.2f})!")

        if t_cur <= 10.2:
            first_10s_dist = dist_trav
            first_10s_state = state
            first_10s_recoveries = cur_recoveries
            first_10s_replans = cur_replans

        samples.append({
            't': round(t_cur, 2),
            'x': round(rx, 2),
            'y': round(ry, 2),
            'dist': round(dist_trav, 2),
            'state': state,
            'min_lidar': round(lidar_min, 2),
            'replans': cur_replans,
            'recoveries': cur_recoveries
        })

        if int(t_cur * 2) % 10 == 0:
            print(f"  [{t_cur:4.1f}s] Pos: ({rx:.2f}, {ry:.2f}) | Dist: {dist_trav:.2f}m | State: {state:<10} | Clear: {lidar_min:.2f}m | Replans: {cur_replans} | Recov: {cur_recoveries}")

        time.sleep(0.25)

    send_control("ABORT")
    time.sleep(1.0)

    result = {
        'name': name,
        'start_pose': {'x': x, 'y': y, 'yaw': yaw, 'yaw_deg': round(math.degrees(yaw), 1)},
        'goal': 'STORAGE',
        'test_duration_sec': duration_sec,
        'first_10s_behavior': {
            'distance_m': round(first_10s_dist, 2),
            'state': first_10s_state,
            'recoveries': first_10s_recoveries,
            'replans': first_10s_replans
        },
        'cleared_doorway': cleared_doorway,
        'time_to_doorway_sec': time_to_doorway,
        'final_distance_travelled_m': round(dist_trav, 2),
        'min_clearance_m': round(min_clearance, 2),
        'total_recoveries': recoveries_count,
        'total_replans': replans_count,
        'total_ttc_yields': ttc_yields,
        'collisions': collisions,
        'outcome': 'DOORWAY_CLEARED' if cleared_doorway else 'TRAPPED_IN_WARD',
        'samples_count': len(samples)
    }
    return result

def main():
    print("=" * 75)
    print("      V2.6 PHASE 2 — HOSPITAL START CONDITION CONTROLLED EXPERIMENT")
    print("=" * 75)

    conditions = [
        {
            'name': 'Baseline_Facing_South',
            'x': 6.5,
            'y': -9.5,
            'yaw': -1.57,  # South: directly towards bed 2
            'desc': 'Baseline pose from V2.5 (yaw=-1.57, facing away from doorway)'
        },
        {
            'name': 'Candidate1_Facing_North',
            'x': 6.5,
            'y': -9.5,
            'yaw': 1.57,   # North: towards doorway
            'desc': 'Same (x, y) between beds, but oriented toward doorway (yaw=+1.57)'
        },
        {
            'name': 'Candidate2_Center_Aisle',
            'x': 6.0,
            'y': -9.0,
            'yaw': 1.57,   # Center aisle, facing North towards doorway
            'desc': 'Shifted to ward center aisle (x=6.0, y=-9.0, yaw=+1.57)'
        }
    ]

    all_results = []
    for cond in conditions:
        r = run_hospital_test(cond['name'], cond['x'], cond['y'], cond['yaw'], duration_sec=35.0)
        r['description'] = cond['desc']
        all_results.append(r)
        time.sleep(2.0)

    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(all_results, f, indent=2)

    print("\n" + "=" * 75)
    print("             HOSPITAL START EXPERIMENT SUMMARY TABLE")
    print("=" * 75)
    print(f"| {'Condition':<25} | {'Start Pose':<20} | {'10s Dist':<9} | {'Doorway':<10} | {'MinClear':<9} | {'Replans':<8} | {'Recov':<6} | {'Outcome':<15} |")
    print("|" + "-" * 27 + "|" + "-" * 22 + "|" + "-" * 11 + "|" + "-" * 12 + "|" + "-" * 11 + "|" + "-" * 10 + "|" + "-" * 8 + "|" + "-" * 17 + "|")
    for r in all_results:
        p_str = f"({r['start_pose']['x']},{r['start_pose']['y']},{r['start_pose']['yaw_deg']}°)"
        door_str = f"{r['time_to_doorway_sec']}s" if r['cleared_doorway'] else "NO"
        print(f"| {r['name']:<25} | {p_str:<20} | {r['first_10s_behavior']['distance_m']:>7.2f}m | {door_str:<10} | {r['min_clearance_m']:>7.2f}m | {r['total_replans']:>8} | {r['total_recoveries']:>6} | {r['outcome']:<15} |")
    print("=" * 75)
    print(f"[OK] Saved experimental results to {OUTPUT_JSON}\n")

if __name__ == '__main__':
    main()
