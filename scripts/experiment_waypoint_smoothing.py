#!/usr/bin/env python3
"""
V2.6 Phase 2 — Step 3: Waypoint & Topological Navigation Smoothing Experiment.
Compares:
  - Mode A (Baseline): Waypoint transitions with explicit cancel_goal_async() and 0.15s sleep
  - Mode B (Smoothed): Preemptive waypoint handoff without cancel brake
Runs on multi-node route: RECEPTION -> ROOM A (8 topological nodes).
Measures:
  - Waypoints traversed
  - Time spent per waypoint
  - Total stationary duration
  - Average velocity
  - Total travel time
  - Replans per waypoint
Saves results to results/v26_phase2/waypoint_experiments.json.
"""

import os
import sys
import time
import math
import json
import urllib.request
from typing import Dict, Any, List

API_BASE = "http://127.0.0.1:5050"
PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUTPUT_JSON = os.path.join(PROJECT_ROOT, "results/v26_phase2/waypoint_experiments.json")

def get_telemetry() -> Dict[str, Any]:
    try:
        req = urllib.request.Request(f"{API_BASE}/api/telemetry")
        with urllib.request.urlopen(req, timeout=2) as resp:
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
        with urllib.request.urlopen(req, timeout=2) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def run_waypoint_test(test_name: str, duration_sec: float = 65.0) -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print(f"  RUNNING WAYPOINT SMOOTHING EXPERIMENT: {test_name}")
    print("  Route: RECEPTION -> ROOM A (8 nodes across west corridor)")
    print("=" * 70)

    send_control("ABORT")
    time.sleep(1.0)

    # Teleport to start
    set_cmd = [
        "gz", "service",
        "--service", "/world/realistic_facility_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", 'name: "autonomous_robot", position: {x: 0.0, y: -8.6, z: 0.1}, orientation: {z: 0.707, w: 0.707}'
    ]
    import subprocess
    subprocess.run(set_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    amcl_script = os.path.join(PROJECT_ROOT, "scripts/set_start.py")
    subprocess.run([sys.executable, amcl_script, "--x", "0.0", "--y", "-8.6", "--yaw", "1.57"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    send_goal("ROOM A")
    time.sleep(0.5)

    t_start = time.time()
    waypoints_visited = []
    waypoint_times = {}
    stationary_time = 0.0
    prev_dist = 0.0
    last_node = ""
    node_entry_time = t_start
    result = "TIMEOUT"
    min_clear = 10.0
    replans = 0

    while time.time() - t_start < duration_sec:
        t_cur = time.time() - t_start
        telem = get_telemetry()
        cur_node = telem.get('current_node', '')
        tgt_node = telem.get('target_node', '')
        m_status = telem.get('mission_status', '')
        dist = telem.get('distance_travelled', 0.0)
        v = telem.get('linear_velocity', 0.0)
        clear = telem.get('lidar', {}).get('min_distance', 10.0)
        cur_replans = telem.get('replans_count', 0)

        if 0.05 < clear < min_clear:
            min_clear = clear

        if cur_replans > replans:
            replans = cur_replans

        if v < 0.05:
            stationary_time += 0.25

        if cur_node and cur_node != last_node:
            now = time.time()
            if last_node:
                waypoint_times[last_node] = round(now - node_entry_time, 2)
            waypoints_visited.append(cur_node)
            last_node = cur_node
            node_entry_time = now
            print(f"  [{t_cur:4.1f}s] Reached Node: '{cur_node}' | Next Target: '{tgt_node}' | Vel: {v:.2f}m/s | Dist: {dist:.1f}m")

        if m_status in ["COMPLETED", "REACHED", "ARRIVED", "GOAL_REACHED"] or (cur_node == "room_a" and dist > 15.0):
            result = "SUCCESS"
            print(f"  >>> [MISSION SUCCESS] Arrived at Room A in {t_cur:.1f}s!")
            break

        time.sleep(0.25)

    duration = round(time.time() - t_start, 2)
    avg_speed = round(dist / duration, 3) if duration > 0 else 0.0

    send_control("ABORT")

    return {
        'test_name': test_name,
        'result': result,
        'duration_sec': duration,
        'distance_m': round(dist, 2),
        'avg_speed_mps': avg_speed,
        'stationary_time_sec': round(stationary_time, 2),
        'waypoints_count': len(waypoints_visited),
        'waypoints_visited': waypoints_visited,
        'waypoint_times': waypoint_times,
        'avg_time_per_waypoint_sec': round(duration / max(1, len(waypoints_visited)), 2),
        'min_clearance_m': round(min_clear, 2),
        'total_replans': replans
    }

def main():
    res = run_waypoint_test("Baseline_Handoff_Evaluation", duration_sec=55.0)
    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
    with open(OUTPUT_JSON, 'w') as f:
        json.dump([res], f, indent=2)

    print("\n" + "=" * 70)
    print("         WAYPOINT SMOOTHING BASELINE EVALUATION SUMMARY")
    print("=" * 70)
    print(f"  Result:                      {res['result']}")
    print(f"  Duration:                    {res['duration_sec']} s")
    print(f"  Distance Travelled:          {res['distance_m']} m")
    print(f"  Average Speed:               {res['avg_speed_mps']} m/s")
    print(f"  Stationary Time:             {res['stationary_time_sec']} s")
    print(f"  Waypoints Traversed:         {res['waypoints_count']} nodes")
    print(f"  Avg Time per Waypoint:       {res['avg_time_per_waypoint_sec']} s/node")
    print(f"  Total Replans:               {res['total_replans']}")
    print("=" * 70)
    print(f"[OK] Saved baseline results to {OUTPUT_JSON}\n")

if __name__ == '__main__':
    main()
