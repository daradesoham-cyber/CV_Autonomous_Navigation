#!/usr/bin/env python3
"""
V2.6 Phase 2 — Step 2: Velocity & Controller Bottleneck Diagnostic Experiment.
Logs high-frequency (20 Hz) telemetry along a long straight corridor
(Reception x=0.0, y=-8.6 to Office x=0.0, y=10.0) to isolate what limits velocity:
  - Curvature regulation
  - Cost regulation
  - Approach scaling
  - Waypoint handoff deceleration
  - Lookahead distance
Saves results to results/v26_phase2/controller_experiments.json.
"""

import os
import sys
import time
import math
import json
import subprocess
import urllib.request
from typing import Dict, Any, List

API_BASE = "http://127.0.0.1:5050"
PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUTPUT_JSON = os.path.join(PROJECT_ROOT, "results/v26_phase2/controller_experiments.json")

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

def set_robot_pose(x: float, y: float, yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
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
    subprocess.run([sys.executable, amcl_script, "--x", str(x), "--y", str(y), "--yaw", str(yaw)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)

def run_velocity_diagnostic(duration_sec: float = 35.0) -> Dict[str, Any]:
    print("\n" + "=" * 70)
    print("  RUNNING CONTROLLER VELOCITY DIAGNOSTIC RUN")
    print("  Route: RECEPTION -> OFFICE (Straight Central Spine, 18.6m)")
    print("=" * 70)

    send_control("ABORT")
    time.sleep(1.0)

    # Teleport to Reception facing North (+Y)
    set_robot_pose(0.0, -8.6, 1.57)

    # Dispatch goal
    send_goal("OFFICE")
    time.sleep(0.5)

    samples = []
    t_start = time.time()
    peak_v = 0.0
    v_sum = 0.0
    v_count = 0
    low_speed_time = 0.0
    mid_speed_time = 0.0
    high_speed_time = 0.0

    last_t = t_start
    while time.time() - t_start < duration_sec:
        now = time.time()
        dt = now - last_t
        last_t = now

        telem = get_telemetry()
        pose = telem.get('robot_pose', {})
        rx = pose.get('x', 0.0)
        ry = pose.get('y', 0.0)
        v_lin = telem.get('linear_velocity', 0.0)
        v_ang = telem.get('angular_velocity', 0.0)
        rem = telem.get('distance_remaining', 0.0)
        dist_trav = telem.get('distance_travelled', 0.0)
        state = telem.get('state', 'UNKNOWN')
        cur_node = telem.get('current_node', '')
        tgt_node = telem.get('target_node', '')
        m_status = telem.get('mission_status', '')

        if v_lin > peak_v:
            peak_v = v_lin

        if state == "NAVIGATING":
            v_sum += v_lin
            v_count += 1
            if v_lin < 0.25:
                low_speed_time += dt
            elif v_lin < 0.40:
                mid_speed_time += dt
            else:
                high_speed_time += dt

        samples.append({
            't': round(now - t_start, 2),
            'x': round(rx, 2),
            'y': round(ry, 2),
            'v_lin': round(v_lin, 3),
            'v_ang': round(v_ang, 3),
            'rem': round(rem, 2),
            'dist': round(dist_trav, 2),
            'cur_node': cur_node,
            'tgt_node': tgt_node
        })

        if int((now - t_start) * 2) % 6 == 0:
            print(f"  [{(now - t_start):4.1f}s] Pos: ({rx:4.1f}, {ry:4.1f}) | Vel: {v_lin:4.2f} m/s | Ang: {v_ang:5.2f} rad/s | Rem: {rem:4.1f}m | Target: {tgt_node}")

        if m_status in ["COMPLETED", "REACHED", "ARRIVED", "GOAL_REACHED"] or (rem < 0.6 and dist_trav > 5.0):
            print(f"  >>> [ARRIVED] Goal reached in {now - t_start:.1f}s!")
            break

        time.sleep(0.05)

    send_control("ABORT")

    avg_v = (v_sum / v_count) if v_count > 0 else 0.0
    total_time = round(time.time() - t_start, 2)

    result = {
        'test': 'Reception_To_Office_Straight_Spine',
        'duration_sec': total_time,
        'distance_travelled_m': round(dist_trav, 2),
        'peak_linear_velocity_mps': round(peak_v, 3),
        'average_linear_velocity_mps': round(avg_v, 3),
        'desired_linear_velocity_mps': 0.50,
        'speed_efficiency_pct': round((avg_v / 0.50) * 100, 1),
        'time_breakdown_sec': {
            'low_speed_under_0_25': round(low_speed_time, 2),
            'mid_speed_0_25_to_0_40': round(mid_speed_time, 2),
            'high_speed_over_0_40': round(high_speed_time, 2)
        },
        'samples_count': len(samples),
        'diagnostic_conclusion': ''
    }

    if high_speed_time > (low_speed_time + mid_speed_time):
        result['diagnostic_conclusion'] = 'High speed sustained on straight corridor'
    elif low_speed_time > mid_speed_time:
        result['diagnostic_conclusion'] = 'Velocity heavily choked (<0.25 m/s) by cost/waypoint deceleration'
    else:
        result['diagnostic_conclusion'] = 'Moderate velocity (0.25-0.40 m/s) with waypoint handoff dips'

    return result, samples

def main():
    res, samples = run_velocity_diagnostic(duration_sec=35.0)
    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(res, f, indent=2)

    print("\n" + "=" * 70)
    print("       CONTROLLER VELOCITY BOTTLENECK DIAGNOSTIC SUMMARY")
    print("=" * 70)
    print(f"  Desired Linear Velocity:     {res['desired_linear_velocity_mps']} m/s")
    print(f"  Peak Linear Velocity:        {res['peak_linear_velocity_mps']} m/s")
    print(f"  Average Linear Velocity:     {res['average_linear_velocity_mps']} m/s")
    print(f"  Speed Efficiency:            {res['speed_efficiency_pct']} %")
    print(f"  Time < 0.25 m/s:             {res['time_breakdown_sec']['low_speed_under_0_25']} s")
    print(f"  Time 0.25 - 0.40 m/s:        {res['time_breakdown_sec']['mid_speed_0_25_to_0_40']} s")
    print(f"  Time >= 0.40 m/s:            {res['time_breakdown_sec']['high_speed_over_0_40']} s")
    print(f"  Diagnostic Conclusion:       {res['diagnostic_conclusion']}")
    print("=" * 70)
    print(f"[OK] Velocity diagnostic saved to {OUTPUT_JSON}\n")

if __name__ == '__main__':
    main()
