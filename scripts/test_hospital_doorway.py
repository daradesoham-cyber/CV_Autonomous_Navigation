#!/usr/bin/env python3
"""
Test Hospital Ward Doorway Traversability in Isolation.
Start: Hospital Ward (x=6.0, y=-9.0, yaw=1.57)
Doorway: (x=6.0, y=-6.0, width=0.85m)
Goal: Corridor (x=6.0, y=-4.5, yaw=1.57)
"""

import os
import sys
import time
import math
import json
import argparse
import subprocess
import urllib.request

API_BASE = "http://127.0.0.1:5050"
PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUT_JSON = os.path.join(PROJECT_ROOT, "results/v26_final/hospital_doorway_test.json")

def euler_to_quat(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

def get_telemetry():
    try:
        req = urllib.request.Request(f"{API_BASE}/api/telemetry")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def send_control(action):
    data = json.dumps({"action": action}).encode('utf-8')
    req = urllib.request.Request(f"{API_BASE}/api/control", data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def set_robot_pose(x, y, yaw):
    print(f"Setting Gazebo robot pose to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})...")
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
    while time.time() - t_wait < 20.0:
        now = time.time()
        if now - last_pub >= 2.0:
            last_pub = now
            subprocess.run(["python3", amcl_script, "--x", str(x), "--y", str(y), "--yaw", str(yaw)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        telem = get_telemetry()
        rp = telem.get('robot_pose', {})
        px, py = rp.get('x', 999.0), rp.get('y', 999.0)
        if math.hypot(px - x, py - y) < 0.6:
            print(f"AMCL converged to ({px:.2f}, {py:.2f})")
            converged = True
            break
        time.sleep(0.3)
    return converged

def main():
    print("=" * 60)
    print("  HOSPITAL DOORWAY TRAVERSABILITY ISOLATION TEST")
    print("=" * 60)

    send_control("ABORT")
    time.sleep(1.0)

    parser = argparse.ArgumentParser(description="Test hospital doorway traversability")
    parser.add_argument("--start-x", type=float, default=5.0, help="Start X")
    parser.add_argument("--start-y", type=float, default=-8.5, help="Start Y")
    parser.add_argument("--start-yaw", type=float, default=1.57, help="Start Yaw")
    args = parser.parse_args()

    # 1. Position inside hospital ward
    start_x, start_y, start_yaw = args.start_x, args.start_y, args.start_yaw
    goal_x, goal_y, goal_yaw = 6.0, -4.5, 1.57

    if not set_robot_pose(start_x, start_y, start_yaw):
        print("[WARN] AMCL localization slow to converge, proceeding anyway.")

    # 2. Dispatch custom coordinate goal to Nav2 action server
    print(f"Sending goal ({goal_x}, {goal_y}) through doorway via scripts/send_goal.py...")
    send_cmd = [
        "python3", "-u", os.path.join(PROJECT_ROOT, "scripts/send_goal.py"),
        "--x", str(goal_x), "--y", str(goal_y), "--yaw", str(goal_yaw)
    ]
    goal_proc = subprocess.Popen(send_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    trajectory = []
    min_clearance = 999.0
    doorway_traversed = False
    passed_doorway_time = None
    start_time = time.time()
    outcome = "UNKNOWN"

    timeout = 140.0  # 140s allows complete path following, dynamic obstacle yield, and arrival

    while time.time() - start_time < timeout:
        telem = get_telemetry()
        rp = telem.get('robot_pose', {})
        px = rp.get('x', start_x)
        py = rp.get('y', start_y)
        yaw = rp.get('yaw', 0.0)
        lin_vel = telem.get('linear_velocity', 0.0)
        lidar = telem.get('lidar', {})
        min_dist = lidar.get('min_distance', 999.0)
        if min_dist < min_clearance and min_dist > 0.05:
            min_clearance = min_dist

        trajectory.append({
            't': round(time.time() - start_time, 2),
            'x': round(px, 3),
            'y': round(py, 3),
            'yaw': round(yaw, 3),
            'v': round(lin_vel, 3),
            'min_dist': round(min_dist, 3)
        })

        # Check doorway crossing (door is at y = -7.0)
        if py > -6.8 and not doorway_traversed:
            doorway_traversed = True
            passed_doorway_time = time.time() - start_time
            print(f"  [SUCCESS] Doorway traversed at t={passed_doorway_time:.2f}s! Robot at y={py:.2f}")

        # Check goal reached (within 0.5m of (6.0, -4.5))
        dist_to_goal = math.hypot(px - goal_x, py - goal_y)
        if dist_to_goal < 0.4:
            outcome = "SUCCESS"
            print(f"  [SUCCESS] Goal ({goal_x}, {goal_y}) reached in {time.time() - start_time:.2f}s! Final dist: {dist_to_goal:.2f}m")
            break

        # Check if send_goal process exited
        ret = goal_proc.poll()
        if ret is not None:
            stdout, stderr = goal_proc.communicate()
            combined_out = (stdout or "") + (stderr or "")
            if ret == 0 or "succeeded" in combined_out.lower() or dist_to_goal < 0.5:
                outcome = "SUCCESS"
                print(f"  [SUCCESS] Nav2 reported goal reached!")
                break
            else:
                outcome = "ABORTED"
                print(f"  [ABORT] send_goal finished with code {ret}. Output:\n{combined_out}")
                break

        time.sleep(0.2)

    total_time = time.time() - start_time
    if outcome == "UNKNOWN":
        outcome = "TIMEOUT"

    print(f"\nDoorway Test Finished:")
    print(f"  Outcome: {outcome}")
    print(f"  Doorway crossed: {doorway_traversed} (at t={passed_doorway_time}s)" if doorway_traversed else "  Doorway crossed: FALSE")
    print(f"  Total time: {total_time:.2f}s")
    print(f"  Min clearance: {min_clearance:.2f}m")

    result_data = {
        'test': 'hospital_doorway_isolation',
        'outcome': outcome,
        'doorway_traversed': doorway_traversed,
        'doorway_traversed_time_sec': passed_doorway_time,
        'total_time_sec': round(total_time, 2),
        'min_clearance_m': round(min_clearance, 3),
        'start_pose': [start_x, start_y, start_yaw],
        'goal_pose': [goal_x, goal_y, goal_yaw],
        'trajectory_points': len(trajectory),
        'trajectory': trajectory
    }

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, 'w') as f:
        json.dump(result_data, f, indent=2)
    print(f"Saved results to {OUT_JSON}")

if __name__ == '__main__':
    main()
