#!/usr/bin/env python3
"""
V2.6 Phase 2 — Step 1: Nav2 Inflation & Clearance Controlled Experiment.
Tests 4 inflation values across 4 critical facility zones:
  Inflation Values:
    - 0.38 m (Baseline)
    - 0.32 m (Candidate 1)
    - 0.30 m (Candidate 2)
    - 0.28 m (Candidate 3)
  Facility Zones:
    1. Hospital Ward:   HOSPITAL -> STORAGE (tight doorway & bed corridor)
    2. Warehouse:       WAREHOUSE -> EXIT   (industrial aisle & dynamic forklift zone)
    3. Storage:         STORAGE -> OFFICE   (storage racks & central spine)
    4. Lab / Long Path: RECEPTION -> ROOM A (narrow east bypass & lab entryway)

Measures:
  - Success / Completion
  - Travel time (s)
  - Distance travelled (m)
  - Effective velocity (m/s)
  - Minimum clearance (m) [Safety Threshold: >= 0.25m]
  - Collisions (Must be 0)
  - Local planner failures / controller patience exceeded
  - Total replans
  - Total recoveries
  - Total TTC yields

Saves structured results to results/v26_phase2/clearance_experiments.json.
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
OUTPUT_JSON = os.path.join(PROJECT_ROOT, "results/v26_phase2/clearance_experiments.json")

LOCATIONS = {
    'hospital': {'x': 6.0, 'y': -9.0, 'yaw': 1.57, 'label': 'HOSPITAL'},
    'storage': {'x': -6.5, 'y': 3.0, 'yaw': 1.57, 'label': 'STORAGE'},
    'warehouse': {'x': 8.0, 'y': 7.0, 'yaw': 1.57, 'label': 'WAREHOUSE'},
    'exit': {'x': -12.5, 'y': -9.0, 'yaw': -1.57, 'label': 'EXIT'},
    'office': {'x': 0.0, 'y': 10.0, 'yaw': 1.57, 'label': 'OFFICE'},
    'reception': {'x': 0.0, 'y': -8.6, 'yaw': 1.57, 'label': 'RECEPTION'},
    'room_a': {'x': -7.0, 'y': 9.5, 'yaw': 1.57, 'label': 'ROOM A'},
}

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
    time.sleep(0.4)

    amcl_script = os.path.join(PROJECT_ROOT, "scripts/set_start.py")
    t_wait = time.time()
    converged = False
    last_pub = 0.0

    while time.time() - t_wait < 15.0:
        now = time.time()
        if now - last_pub >= 2.0:
            last_pub = now
            subprocess.run([
                sys.executable, amcl_script,
                "--x", str(x), "--y", str(y), "--yaw", str(yaw)
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        telem = get_telemetry()
        rp = telem.get('robot_pose', {})
        px, py = rp.get('x', 999.0), rp.get('y', 999.0)
        if math.hypot(px - x, py - y) < 0.6:
            converged = True
            break
        time.sleep(0.3)

    time.sleep(0.8)

def set_nav2_inflation(radius: float) -> bool:
    print(f"\n[CONFIG] Setting Nav2 costmap inflation_radius to {radius:.2f} m...")
    cmd1 = [
        "bash", "-c",
        f"source /opt/ros/lyrical/setup.bash && ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius {radius} && ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius {radius}"
    ]
    res = subprocess.run(cmd1, capture_output=True, text=True)
    if "successful" in res.stdout:
        print(f"[OK] Nav2 inflation_radius successfully set to {radius:.2f} m.")
        time.sleep(1.0)
        return True
    else:
        print(f"[WARN] Failed to set inflation parameter: {res.stderr}")
        return False

def run_single_test(start_key: str, goal_key: str, inflation_radius: float, timeout_sec: float = 65.0) -> Dict[str, Any]:
    start_info = LOCATIONS[start_key]
    goal_info = LOCATIONS[goal_key]
    start_label = start_info['label']
    goal_label = goal_info['label']

    print(f"\n  --- Test: {start_label} -> {goal_label} (Inflation: {inflation_radius:.2f}m) ---")

    # Reset
    send_control("ABORT")
    time.sleep(1.2)

    # Teleport to start
    set_robot_pose(start_info['x'], start_info['y'], start_info['yaw'])

    # Send goal
    send_goal(goal_label)
    time.sleep(0.8)

    t_start = time.time()
    min_clearance = 10.0
    replans_count = 0
    recoveries_count = 0
    ttc_yields = 0
    result = "TIMEOUT"
    stationary_duration = 0.0
    last_check_time = t_start
    prev_dist = 0.0
    collisions = 0
    local_planner_failures = 0

    while time.time() - t_start < timeout_sec:
        t_cur = time.time() - t_start
        telem = get_telemetry()
        pose = telem.get('robot_pose', {})
        rx, ry = pose.get('x', 0.0), pose.get('y', 0.0)
        rem = telem.get('distance_remaining', 999.0)
        dist_trav = telem.get('distance_travelled', 0.0)
        m_status = telem.get('mission_status', 'UNKNOWN')
        state = telem.get('state', 'UNKNOWN')
        diag = telem.get('diagnostics', {})
        cur_replans = telem.get('replans_count', 0)
        cur_recov = diag.get('recovery_count', 0)
        lidar_min = telem.get('lidar', {}).get('min_distance', 10.0)
        ttc = telem.get('ttc', -1.0)
        v_lin = telem.get('linear_velocity', 0.0)

        if 0.05 < lidar_min < min_clearance:
            min_clearance = lidar_min
            if lidar_min < 0.15:
                collisions += 1

        if ttc > 0 and ttc < 1.8:
            ttc_yields += 1

        if cur_replans > replans_count:
            replans_count = cur_replans

        if cur_recov > recoveries_count:
            recoveries_count = cur_recov

        if state == "RECOVERY":
            local_planner_failures += 1

        dt = time.time() - last_check_time
        last_check_time = time.time()
        if abs(dist_trav - prev_dist) < 0.02:
            stationary_duration += dt
        prev_dist = dist_trav

        # Check goal arrival
        if m_status in ["COMPLETED", "REACHED", "ARRIVED", "GOAL_REACHED"] or (rem < 0.6 and dist_trav > 2.0):
            result = "SUCCESS"
            print(f"  >>> [MISSION SUCCESS] Arrived in {t_cur:.1f}s, dist: {dist_trav:.2f}m, min clearance: {min_clearance:.2f}m")
            break

        if int(t_cur) % 10 == 0 and dt < 0.2:
            print(f"    [{t_cur:4.1f}s] Pos: ({rx:.1f}, {ry:.1f}) | Dist: {dist_trav:.1f}m | Rem: {rem:.1f}m | Clear: {lidar_min:.2f}m | Vel: {v_lin:.2f}m/s | Replans: {cur_replans} | Recov: {cur_recov}")

        time.sleep(0.25)

    duration = round(time.time() - t_start, 2)
    avg_speed = round(dist_trav / duration, 3) if duration > 0 else 0.0

    send_control("ABORT")
    time.sleep(1.0)

    test_record = {
        'zone': start_key,
        'start': start_label,
        'goal': goal_label,
        'inflation_radius_m': inflation_radius,
        'result': result,
        'time_sec': duration,
        'distance_m': round(dist_trav, 2),
        'avg_speed_mps': avg_speed,
        'stationary_time_sec': round(stationary_duration, 2),
        'min_clearance_m': round(min_clearance, 2),
        'safe_clearance_preserved': (min_clearance >= 0.25),
        'collisions': collisions,
        'replans_count': replans_count,
        'recovery_events': recoveries_count,
        'ttc_yields': ttc_yields,
        'local_planner_failures': local_planner_failures
    }
    return test_record

def main():
    print("=" * 75)
    print("   V2.6 PHASE 2 — NAV2 INFLATION / CLEARANCE CONTROLLED EXPERIMENT")
    print("=" * 75)

    inflation_candidates = [0.38, 0.32, 0.30, 0.28]
    test_missions = [
        ('hospital', 'storage'),     # Zone 1: Hospital Ward
        ('warehouse', 'exit'),       # Zone 2: Warehouse
        ('storage', 'office'),       # Zone 3: Storage Central Spine
        ('reception', 'room_a'),     # Zone 4: Lab / East Bypass
    ]

    all_results = []

    for infl in inflation_candidates:
        set_nav2_inflation(infl)
        for s, g in test_missions:
            r = run_single_test(s, g, infl, timeout_sec=55.0)
            all_results.append(r)
            time.sleep(1.5)

    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(all_results, f, indent=2)

    # Print summary comparison table
    print("\n" + "=" * 90)
    print("           NAV2 INFLATION EXPERIMENT SUMMARY TABLE ACROSS 4 ZONES")
    print("=" * 90)
    print(f"| {'Inflation':<9} | {'Mission':<22} | {'Result':<8} | {'Time':<7} | {'Dist':<7} | {'Speed':<8} | {'MinClear':<9} | {'Safe?':<6} | {'Replans':<8} | {'Recov':<6} |")
    print("|" + "-" * 11 + "|" + "-" * 24 + "|" + "-" * 10 + "|" + "-" * 9 + "|" + "-" * 9 + "|" + "-" * 10 + "|" + "-" * 11 + "|" + "-" * 8 + "|" + "-" * 10 + "|" + "-" * 8 + "|")
    for r in all_results:
        m_str = f"{r['start']}->{r['goal']}"
        safe_str = "YES" if r['safe_clearance_preserved'] else "NO"
        print(f"| {r['inflation_radius_m']:>7.2f}m | {m_str:<22} | {r['result']:<8} | {r['time_sec']:>5.1f}s | {r['distance_m']:>5.1f}m | {r['avg_speed_mps']:>6.2f}m/s| {r['min_clearance_m']:>7.2f}m | {safe_str:<6} | {r['replans_count']:>8} | {r['recovery_events']:>6} |")
    print("=" * 90)
    print(f"[OK] Full experiment results saved to: {OUTPUT_JSON}\n")

if __name__ == '__main__':
    main()
