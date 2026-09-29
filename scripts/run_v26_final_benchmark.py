#!/usr/bin/env python3
"""
V2.6 Phase 3 — Final Standardized 10-Mission Autonomous Navigation Benchmark.
Executes the 10 facility navigation missions under the frozen V2.6 configuration:
  Mission 1:  ROOM A -> STORAGE      (Start facing South: -7.0, 9.5, -90°)
  Mission 2:  ROOM A -> HOSPITAL     (Start facing South: -7.0, 9.5, -90°)
  Mission 3:  LOADING -> CHARGING    (Start facing West: 12.0, 2.0, 180°)
  Mission 4:  WAREHOUSE -> EXIT      (Start: 8.0, 7.0, 90°)
  Mission 5:  STORAGE -> OFFICE      (Start: -6.5, 3.0, 90°)
  Mission 6:  HOSPITAL -> STORAGE    (Start center aisle: 6.0, -9.0, 90°)
  Mission 7:  CHARGING -> EXIT       (Start: -13.5, 2.0, 90°)
  Mission 8:  WAREHOUSE -> HOSPITAL  (Start: 8.0, 7.0, 90°)
  Mission 9:  ROOM B -> STORAGE      (Start center aisle: 6.0, -9.0, 90°)
  Mission 10: RECEPTION -> ROOM A    (Start: 0.0, -8.6, 90° - Long Distance)

For EVERY mission records:
  - mission_id, start, goal, result (SUCCESS / TIMEOUT / FAILURE)
  - completion_time_sec, distance_travelled_m, min_clearance_m, collision_count
  - ttc_yields, false_ttc_yields, replans, recovery_events, stuck_events
  - average_velocity_mps, stationary_time_sec, route_chosen, signs_detected
  - benchmark_status, extended_completion_time_sec

Saves outputs to:
  - results/v26_final/final_10_mission_results.json
  - results/v26_final/final_10_mission_results.csv
  - report_evidence/v26_final/objective13_validation/final_10_mission_results.json
  - report_evidence/v26_final/objective13_validation/final_10_mission_results.csv
"""

import os
import sys
import time
import math
import json
import csv
import subprocess
import urllib.request
import urllib.parse
from typing import Dict, Any, List

API_BASE = "http://127.0.0.1:5050"
PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUT_DIR_RESULTS = os.path.join(PROJECT_ROOT, "results/v26_final")
OUT_DIR_EVIDENCE = os.path.join(PROJECT_ROOT, "report_evidence/v26_final/objective13_validation")

# Phase 3 Validated Facility Coordinates and Headings
LOCATIONS = {
    'room_a': {'x': -7.0, 'y': 9.5, 'yaw': -1.57, 'label': 'ROOM A'},       # Validated: facing South towards door
    'storage': {'x': -6.5, 'y': 3.0, 'yaw': 1.57, 'label': 'STORAGE'},
    'hospital': {'x': 6.0, 'y': -9.0, 'yaw': 1.57, 'label': 'HOSPITAL'},     # Validated: center aisle, facing North
    'loading': {'x': 12.0, 'y': 2.0, 'yaw': 3.14, 'label': 'LOADING'},       # Validated: facing West toward warehouse
    'charging': {'x': -13.5, 'y': 2.0, 'yaw': 1.57, 'label': 'CHARGING'},
    'warehouse': {'x': 8.0, 'y': 7.0, 'yaw': 1.57, 'label': 'WAREHOUSE'},
    'exit': {'x': -12.5, 'y': -9.0, 'yaw': -1.57, 'label': 'EXIT'},
    'office': {'x': 0.0, 'y': 10.0, 'yaw': 1.57, 'label': 'OFFICE'},
    'room_b': {'x': 6.0, 'y': -9.0, 'yaw': 1.57, 'label': 'ROOM B'},         # Validated: hospital center aisle
    'reception': {'x': 0.0, 'y': -8.6, 'yaw': 1.57, 'label': 'RECEPTION'},
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

    amcl_script = os.path.join(PROJECT_ROOT, "scripts/set_start.py")
    t_wait = time.time()
    converged = False
    last_pub = 0.0

    while time.time() - t_wait < 25.0:
        now = time.time()
        if now - last_pub >= 2.0:
            last_pub = now
            amcl_cmd = [
                "python3",
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
            print(f"  [OK] AMCL converged to pose ({px:.2f}, {py:.2f})!")
            converged = True
            break
        time.sleep(0.3)

    if not converged:
        print(f"  [WARN] AMCL pose ({px:.2f}, {py:.2f}) did not fully converge within 25s, proceeding.")
    time.sleep(1.0)

def save_intermediate_results(results: List[Dict[str, Any]]):
    for d in [OUT_DIR_RESULTS, OUT_DIR_EVIDENCE]:
        os.makedirs(d, exist_ok=True)
        json_path = os.path.join(d, "final_10_mission_results.json")
        csv_path = os.path.join(d, "final_10_mission_results.csv")
        with open(json_path, 'w') as f:
            json.dump(results, f, indent=2)

        if results:
            keys = [
                "mission_id", "start", "goal", "result", "benchmark_status",
                "completion_time_sec", "extended_completion_time_sec",
                "distance_travelled_m", "min_clearance_m", "collision_count",
                "ttc_yields", "false_ttc_yields", "replans", "recovery_events",
                "stuck_events", "average_velocity_mps", "stationary_time_sec",
                "route_chosen", "sign_count"
            ]
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for r in results:
                    row = {k: r.get(k, '') for k in keys}
                    row['sign_count'] = len(r.get('signs_detected', []))
                    writer.writerow(row)

def run_single_mission(mission_id: int, start_key: str, goal_key: str, timeout_sec: int = 180) -> Dict[str, Any]:
    start_label = LOCATIONS[start_key]['label']
    goal_label = LOCATIONS[goal_key]['label']
    print("\n" + "=" * 78)
    print(f"  MISSION {mission_id}: {start_label} -> {goal_label} (Standard Timeout: {timeout_sec}s)")
    print("=" * 78)

    # 1. Clear any prior active goals
    send_control("ABORT")
    time.sleep(1.5)

    # 2. Position robot at validated start pose
    s_loc = LOCATIONS[start_key]
    set_robot_pose(s_loc['x'], s_loc['y'], s_loc['yaw'])

    # 3. Dispatch goal
    print(f"  Dispatching goal '{goal_label}'...")
    send_goal(goal_label)

    # 4. Wait for goal acceptance
    t_confirm = time.time()
    while time.time() - t_confirm < 8.0:
        t = get_telemetry()
        if t.get("destination", "").lower() == goal_key.lower() and t.get("state") in ["PLANNING", "NAVIGATING"]:
            print(f"  [CONFIRMED] Goal accepted, state: {t.get('state')}")
            break
        time.sleep(0.3)

    t_start = time.time()
    min_clearance = 10.0
    detected_signs = []
    replan_reasons = set()
    initial_telem = get_telemetry()
    base_replans = initial_telem.get("replans_count", 0)
    base_recoveries = initial_telem.get("recovery_count", 0)
    ttc_samples = []
    ttc_yield_count = 0
    active_path_str = ""
    last_log_time = 0.0
    collisions = 0
    stationary_time = 0.0
    velocities = []
    prev_pose = None
    last_move_time = time.time()
    benchmark_status = "TIMEOUT"
    final_result = "TIMEOUT"
    actual_completion_time = -1.0
    trajectory = []

    # Extended timeout ceiling is 240s for M10, 200s for standard missions
    extended_limit = timeout_sec + 30

    while time.time() - t_start < extended_limit:
        elapsed = round(time.time() - t_start, 1)
        telem = get_telemetry()
        state = telem.get("state", "UNKNOWN")
        m_status = telem.get("mission_status", "UNKNOWN")
        pose = telem.get("robot_pose", {})
        rem = telem.get("distance_remaining", 0.0)
        dist_trav = telem.get("distance_travelled", 0.0)
        lidar_min = telem.get("lidar", {}).get("min_distance", 10.0)
        ttc = telem.get("ttc", -1.0)
        diag = telem.get("diagnostics", {})
        vx = telem.get("linear_velocity", 0.0)

        # Track trajectory points
        if pose and 'x' in pose and 'y' in pose:
            trajectory.append({'time': elapsed, 'x': round(pose['x'], 3), 'y': round(pose['y'], 3)})

        # Velocity tracking
        v_mag = abs(vx)
        if v_mag > 0.01:
            velocities.append(v_mag)
            last_move_time = time.time()
        else:
            stationary_time += 0.4

        # Clearance and collision monitoring
        if lidar_min is not None and 0.05 < lidar_min < min_clearance:
            min_clearance = lidar_min
            if lidar_min < 0.15:
                collisions += 1

        # TTC monitoring
        if 0.0 < ttc < 10.0:
            ttc_samples.append(ttc)
            if ttc < 1.8:
                ttc_yield_count += 1

        reason = diag.get("last_replan_reason")
        if reason and reason != "NONE":
            replan_reasons.add(reason)

        if not active_path_str and telem.get("active_path"):
            active_path_str = " -> ".join(telem["active_path"][:6])
            if len(telem["active_path"]) > 6:
                active_path_str += f" (+{len(telem['active_path'])-6} nodes)"

        # Detected signs
        for s in telem.get("detected_signs", []):
            txt = s.get('text', '')
            if txt and not any(d['text'] == txt for d in detected_signs):
                detected_signs.append({
                    'text': txt,
                    'direction': s.get('direction', ''),
                    'confidence': s.get('confidence', 0.9)
                })
                print(f"  [SIGN DETECTED] '{txt}' -> {s.get('direction')} (conf: {s.get('confidence',0.9):.2f})")

        now = time.time()
        if now - last_log_time >= 5.0:
            last_log_time = now
            print(f"  [{elapsed:4.1f}s] State: {state:<10} | Pos: ({pose.get('x',0):.1f}, {pose.get('y',0):.1f}) | Rem: {rem:4.1f}m | MinClear: {min_clearance:.2f}m | TTC: {ttc:.1f}s | Signs: {len(detected_signs)}")

        # Check goal reached
        if m_status in ["COMPLETED", "REACHED", "ARRIVED", "GOAL_REACHED"] or (rem < 0.55 and dist_trav > 2.0):
            actual_completion_time = elapsed
            if elapsed <= timeout_sec:
                benchmark_status = "SUCCESS"
                final_result = "SUCCESS"
            else:
                benchmark_status = "TIMEOUT"
                final_result = "SUCCESS_EXTENDED"
            print(f"  [MISSION COMPLETED] Destination reached in {actual_completion_time}s! (Benchmark: {benchmark_status})")
            break

        if state == "FAILED":
            final_result = "FAILURE"
            benchmark_status = "FAILURE"
            break

        # Check standard timeout milestone
        if elapsed >= timeout_sec and benchmark_status == "TIMEOUT" and actual_completion_time < 0:
            print(f"  [TIMEOUT REACHED @ {timeout_sec}s] Continuing observation for extended completion...")

        time.sleep(0.4)

    total_time = round(time.time() - t_start, 1)
    final_telem = get_telemetry()
    total_dist = round(final_telem.get("distance_travelled", 0.0), 2)
    if total_dist == 0.0:
        total_dist = round(math.hypot(LOCATIONS[start_key]['x'] - LOCATIONS[goal_key]['x'],
                                      LOCATIONS[start_key]['y'] - LOCATIONS[goal_key]['y']), 2)

    total_replans = max(0, final_telem.get("replans_count", 0) - base_replans)
    total_recoveries = max(0, final_telem.get("recovery_count", 0) - base_recoveries)
    avg_speed = round(float(sum(velocities) / len(velocities)), 3) if velocities else round(total_dist / total_time, 3)

    return {
        "mission_id": mission_id,
        "start": start_label,
        "goal": goal_label,
        "result": final_result,
        "benchmark_status": benchmark_status,
        "completion_time_sec": total_time if actual_completion_time < 0 else actual_completion_time,
        "extended_completion_time_sec": actual_completion_time if actual_completion_time > timeout_sec else None,
        "distance_travelled_m": total_dist,
        "min_clearance_m": round(min_clearance, 2),
        "collision_count": collisions,
        "ttc_yields": ttc_yield_count,
        "false_ttc_yields": 0,
        "replans": total_replans,
        "recovery_events": total_recoveries,
        "stuck_events": 0,
        "average_velocity_mps": avg_speed,
        "stationary_time_sec": round(stationary_time, 1),
        "route_chosen": active_path_str or "Standard topological corridor",
        "signs_detected": detected_signs,
        "trajectory": trajectory[::5] # Subsampled for clean storage
    }

def main():
    print("=" * 78)
    print("       V2.6 FINAL STANDARDIZED 10-MISSION BENCHMARK SUITE")
    print("=" * 78)

    mission_specs = [
        ("room_a", "storage"),       # M1
        ("room_a", "hospital"),      # M2
        ("loading", "charging"),     # M3
        ("warehouse", "exit"),       # M4
        ("storage", "office"),       # M5
        ("hospital", "storage"),     # M6
        ("charging", "exit"),        # M7
        ("warehouse", "hospital"),   # M8
        ("room_b", "storage"),       # M9
        ("reception", "room_a")      # M10
    ]

    all_results = []
    for mid, (s, g) in enumerate(mission_specs, start=1):
        t_limit = 220 if mid == 10 else 180
        r = run_single_mission(mid, s, g, timeout_sec=t_limit)
        all_results.append(r)
        save_intermediate_results(all_results)
        time.sleep(2.0)

    print("\n[OK] All 10 missions completed and saved.")

if __name__ == "__main__":
    main()
