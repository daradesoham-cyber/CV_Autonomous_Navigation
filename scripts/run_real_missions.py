#!/usr/bin/env python3
"""
Automated Real Navigation Mission Runner for Phase 3 Validation.
Executes 5 real Gazebo navigation missions:
1. ROOM A -> STORAGE
2. ROOM A -> CHARGING
3. LOADING -> EXIT
4. HOSPITAL -> WAREHOUSE
5. RECEPTION -> ROOM A

Captures real-time telemetry from ROS 2 and queries ground-truth SQLite database
for actual travel metrics.
"""

import os
import sys
import time
import math
import json
import sqlite3
import subprocess
import urllib.request
import urllib.parse

DB_PATH = "/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db"
API_BASE = "http://127.0.0.1:5050"

LOCATIONS = {
    'room_a': {'x': -7.5, 'y': 9.5, 'yaw': 1.57},
    'storage': {'x': -7.0, 'y': 3.0, 'yaw': 1.57},
    'charging': {'x': -13.0, 'y': 2.0, 'yaw': 1.57},
    'loading': {'x': 12.0, 'y': 2.0, 'yaw': 0.0},
    'exit': {'x': -13.0, 'y': -9.0, 'yaw': -1.57},
    'hospital': {'x': 6.5, 'y': -9.5, 'yaw': -1.57},
    'warehouse': {'x': 8.0, 'y': 7.0, 'yaw': 1.57},
    'reception': {'x': 0.0, 'y': -8.0, 'yaw': 1.57},
    'start': {'x': 0.0, 'y': -11.0, 'yaw': 1.57},
}

def euler_to_quat(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

def set_robot_pose(x, y, yaw):
    print(f"Setting Gazebo robot pose to ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)...")
    _, _, qz, qw = euler_to_quat(yaw)
    cmd = [
        "gz", "service",
        "--service", "/world/realistic_facility_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", f'name: "autonomous_robot", position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}'
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    # Sync AMCL initial pose
    amcl_cmd = [
        "python3",
        "/home/soham-darade/CV_Autonomous_Navigation/scripts/set_start.py",
        "--x", str(x),
        "--y", str(y),
        "--yaw", str(yaw)
    ]
    subprocess.run(amcl_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for AMCL pose to confirm localization near target
    t_wait = time.time()
    while time.time() - t_wait < 15.0:
        t = get_telemetry()
        rp = t.get('robot_pose', {})
        px, py = rp.get('x', 999.0), rp.get('y', 999.0)
        if math.hypot(px - x, py - y) < 1.0:
            print(f"[OK] Robot successfully confirmed localization at ({px:.2f}, {py:.2f})!")
            break
        time.sleep(0.5)
    time.sleep(1.0)

def get_telemetry():
    try:
        req = urllib.request.Request(f"{API_BASE}/api/telemetry")
        with urllib.request.urlopen(req, timeout=3) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return {}

def send_goal(goal_label):
    data = json.dumps({"goal": goal_label}).encode('utf-8')
    req = urllib.request.Request(f"{API_BASE}/api/goal", data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode('utf-8'))

def get_latest_sqlite_journey():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute("SELECT * FROM journeys ORDER BY id DESC LIMIT 1")
    row = c.fetchone()
    conn.close()
    return dict(row) if row else {}

def run_mission(start_name, goal_name, timeout_sec=240):
    print("\n" + "=" * 70)
    print(f"  EXECUTING MISSION: {start_name.upper()} -> {goal_name.upper()}")
    print("=" * 70)

    # Ensure previous mission is reset
    telem = get_telemetry()
    if telem.get("mission_status") not in ["IDLE", "GOAL_REACHED", "COMPLETED", "UNKNOWN"]:
        abort_data = json.dumps({"action": "ABORT"}).encode('utf-8')
        req = urllib.request.Request(f"{API_BASE}/api/control", data=abort_data, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as resp:
                pass
        except Exception:
            pass
        time.sleep(1.0)

    start_loc = LOCATIONS[start_name]
    set_robot_pose(start_loc['x'], start_loc['y'], start_loc['yaw'])

    # Send goal
    print(f"Dispatching goal '{goal_name.upper()}' to Navigation Decision Engine...")
    resp = send_goal(goal_name)
    print(f"Goal dispatch response: {resp}")

    t_start = time.time()
    last_print = 0
    route_selected = "N/A"
    planned_dist = 0.0

    while time.time() - t_start < timeout_sec:
        telem = get_telemetry()
        state = telem.get("mission_status", "UNKNOWN")
        pose = telem.get("robot_pose", {})
        rem = telem.get("distance_remaining", 0.0)
        trav = telem.get("distance_travelled", 0.0)
        clear = telem.get("lidar", {}).get("min_distance", 0.0)
        replans = telem.get("replans_count", 0)

        if route_selected == "N/A" and telem.get("alternative_routes"):
            for r in telem["alternative_routes"]:
                if r.get("is_selected"):
                    route_selected = r.get("route_id", "Route A")
                    planned_dist = r.get("distance", 0.0)

        if time.time() - last_print >= 2.5:
            last_print = time.time()
            elapsed = time.time() - t_start
            px, py = pose.get('x', 0.0), pose.get('y', 0.0)
            print(f"[{elapsed:4.1f}s] State: {state:12} | Pose: ({px:5.2f}, {py:5.2f}) | Rem: {rem:4.1f}m | Trav: {trav:4.1f}m | Clear: {clear:4.2f}m | Replans: {replans}")

        # Check for mission completion after minimum warm-up
        if time.time() - t_start > 8.0:
            if (state in ["GOAL_REACHED", "COMPLETED"]) or (state == "IDLE" and rem <= 0.8):
                print(f"[SUCCESS] Robot reached goal '{goal_name.upper()}'!")
                time.sleep(2.0)
                break

        if state in ["FAILED", "ABORTED"]:
            print(f"[FAILURE] Mission entered state {state}!")
            break

        time.sleep(0.5)

    time.sleep(1.5)
    journey = get_latest_sqlite_journey()
    return journey

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Run real Gazebo navigation missions")
    parser.add_argument("--start", type=str, default="room_a", help="Start location name")
    parser.add_argument("--goal", type=str, default="storage", help="Goal location name")
    parser.add_argument("--all", action="store_true", help="Run all 5 standard missions")
    args = parser.parse_args()

    if args.all:
        missions = [
            ("room_a", "storage"),
            ("room_a", "charging"),
            ("loading", "exit"),
            ("hospital", "warehouse"),
            ("reception", "room_a")
        ]
    else:
        missions = [(args.start, args.goal)]

    results = []
    for s, g in missions:
        res = run_mission(s, g)
        results.append(res)
        time.sleep(2.0)

    print("\n" + "=" * 90)
    print("                PHASE 3: REAL GAZEBO NAVIGATION RESULTS SUMMARY")
    print("=" * 90)
    print(f"{'Mission':<22} | {'Dist(Plan/Act)':<14} | {'Time':<7} | {'AvgSpd':<7} | {'MinClr':<7} | {'Obs':<4} | {'Replan':<6} | {'Status'}")
    print("-" * 90)

    for r in results:
        start = (r.get('start_label') or r.get('start_node') or 'N/A').upper()
        goal = (r.get('goal_label') or r.get('goal_node') or 'N/A').upper()
        name = f"{start} -> {goal}"
        act_d = r.get('distance_travelled', 0.0) or r.get('actual_distance', 0.0) or 0.0
        plan_d = r.get('planned_distance', 0.0) or act_d
        dist_str = f"{plan_d:4.1f}m / {act_d:4.1f}m"
        t_sec = r.get('travel_time', 0.0) or r.get('travel_time_sec', 0.0) or 0.0
        spd = r.get('average_speed', 0.0) or (act_d / t_sec if t_sec > 0 else 0.0)
        clr = r.get('min_lidar_clearance', 0.0) or r.get('min_clearance', 0.0) or 0.0
        obs = r.get('obstacles_encountered', 0)
        rep = r.get('replans_count', 0)
        status = "PASS" if r.get('success') else "FAIL"

        print(f"{name:<22} | {dist_str:<14} | {t_sec:5.1f}s | {spd:4.2f}m/s | {clr:4.2f}m | {obs:<4} | {rep:<6} | {status}")

    print("=" * 90 + "\n")

if __name__ == '__main__':
    main()
