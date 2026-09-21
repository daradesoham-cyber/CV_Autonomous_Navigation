#!/usr/bin/env python3
"""
Comprehensive Automated Experiment Runner for Semantic Memory-Based Autonomous Navigation.
Executes all test scenarios, evaluates YOLOv8 + LiDAR fusion, verifies topological routing,
measures replanning latency and memory persistence, and outputs a formatted benchmark report.
"""

import os
import sys
import time
import subprocess

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
VENV_PYTHON = os.path.join(PROJECT_ROOT, ".venv/bin/python")


def run_scenario(name: str, script_relpath: str) -> dict:
    print(f"\n{'#' * 60}")
    print(f"RUNNING EXPERIMENT: {name}")
    print(f"{'#' * 60}")

    cmd = f"source /opt/ros/lyrical/setup.bash && source {PROJECT_ROOT}/ros2_ws/install/setup.bash && {VENV_PYTHON} {PROJECT_ROOT}/{script_relpath}"

    start_t = time.time()
    res = subprocess.run(
        cmd,
        shell=True,
        executable="/bin/bash",
        cwd=PROJECT_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    duration = time.time() - start_t
    print(res.stdout)

    success = (res.returncode == 0)
    return {
        'name': name,
        'script': script_relpath,
        'success': success,
        'duration_sec': round(duration, 3),
        'output': res.stdout
    }


def main():
    print("=" * 70)
    print("AUTONOMOUS NAVIGATION SYSTEM - EXPERIMENT SUITE")
    print("Environment: ROS 2 Lyrical | Gazebo Sim 10.5 | NVIDIA RTX 3050 (cuda:0)")
    print("=" * 70)

    scenarios = [
        ("Scenario 1: Semantic Sign Perception & Guidance", "scripts/test_scenario_1_sign_navigation.py"),
        ("Scenario 2: Dynamic Obstacle Blockage & Replanning", "scripts/test_scenario_2_dynamic_replanning.py"),
        ("Scenario 3: Dead End Avoidance & Pruning", "scripts/test_scenario_3_dead_end_avoidance.py"),
        ("Scenario 4: Navigation Memory & SQLite Persistence", "scripts/test_scenario_4_memory_persistence.py"),
        ("Scenario 5: 16-Point Realistic Facility Evaluation", "scripts/test_realistic_facility.py"),
    ]

    results = []
    for name, script in scenarios:
        res = run_scenario(name, script)
        results.append(res)

    print("\n" + "=" * 70)
    print("EXPERIMENT SUITE SUMMARY REPORT")
    print("=" * 70)
    print(f"{'Scenario Name':<45} | {'Status':<8} | {'Duration (s)':<12}")
    print("-" * 70)

    all_passed = True
    for r in results:
        status_str = "PASSED" if r['success'] else "FAILED"
        if not r['success']:
            all_passed = False
        print(f"{r['name']:<45} | {status_str:<8} | {r['duration_sec']:<12.3f}")

    print("=" * 70)
    if all_passed:
        print("[ALL EXPERIMENTS COMPLETED SUCCESSFULLY - 100% PASS RATE]")
    else:
        print("[SOME EXPERIMENTS FAILED - REVIEW LOGS ABOVE]")

    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()
