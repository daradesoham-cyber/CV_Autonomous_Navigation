#!/usr/bin/env python3
"""
Autonomous Navigation V2 Demonstration Mode.
Showcases the complete autonomous navigation intelligence pipeline:
  Stage 1: Multi-Criteria Route Planning & Candidate Alternatives
  Stage 2: Active Navigation & Telemetry Stream
  Stage 3: Dynamic Obstacle Detection & Corridor Blockage
  Stage 4: Dynamic Route Replanning via Optimal Bypass
  Stage 5: Dead-End Encounter, Coordinate Logging & Safe Backtracking
  Stage 6: Robust Recovery Behavior (Stuck / Clearance Critical)
  Stage 7: Goal Arrival & SQLite Journey Persistence
  Stage 8: Live Travel History & Heatmap Verification

Usage:
  python3 scripts/demo_mode.py
  python3 scripts/demo_mode.py --fast
"""

import os
import sys
import time
import math
import json
import argparse

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
venv_site = os.path.join(root_dir, '.venv', 'lib', 'python3.14', 'site-packages')
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = os.path.join(root_dir, 'ros2_ws', 'src', 'autonomous_robot_navigation')
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def banner(text, fill="="):
    print("\n" + fill * 75)
    print(f"  {text}")
    print(fill * 75)


def print_step(stage, title):
    print(f"\n[{stage}] >>> {title.upper()} <<<")


def simulate_delay(sec, fast=False):
    if not fast:
        time.sleep(min(sec, 1.5))


def main():
    parser = argparse.ArgumentParser(description="Autonomous Navigation V2 Demonstration Mode")
    parser.add_argument('--fast', action='store_true', help="Fast execution without delays")
    parser.add_argument('--db', default='ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db')
    args = parser.parse_args()

    db_path = os.path.join(root_dir, args.db) if not os.path.isabs(args.db) else args.db
    fast = args.fast

    banner("AUTONOMOUS NAVIGATION V2 - MISSION DEMONSTRATION")
    print("Facility: Realistic Warehouse & Medical Research Hub (700x600, 35m x 30m)")
    print("Architecture: Multi-Criteria Route Planner, Dynamic Replanning, SQLite History")

    # STAGE 1: Topological Map & Multi-Criteria Route Selection
    print_step("STAGE 1", "Multi-Criteria Route Planning & Candidate Evaluation")
    mem = NavigationMemory(db_path)
    graph: TopologicalGraph = mem.load_graph()
    print(f"Loaded facility topology: {len(graph.nodes)} nodes, {len(graph.edges)} directed edges.")

    start_id = "start"
    goal_id = "storage"
    print(f"Mission Request: Travel from '{start_id.upper()}' to '{goal_id.upper()}'")

    routes = graph.find_alternative_routes(start_id, goal_id, k=3)
    print(f"Evaluated {len(routes)} candidate alternative routes:\n")

    for r in routes:
        selected_flag = " [SELECTED - LOWEST COST]" if r['is_selected'] else ""
        print(f"  * {r['route_id']}: {r['name']}{selected_flag}")
        print(f"    - Path: {' -> '.join(r['path'])}")
        print(f"    - Distance: {r['distance']:.2f} m | Estimated Time: {r['estimated_time']:.1f} s")
        print(f"    - Multi-Criteria Cost: {r['total_cost']:.2f} (Dist Cost: {r['cost_breakdown']['distance_cost']}, Obstacles: {r['cost_breakdown']['obstacle_cost']})")
        print(f"    - Reliability Score: {r['reliability_pct']}%\n")

    active_route = routes[0]
    simulate_delay(1.0, fast)

    # STAGE 2: Mission Launch & Telemetry
    print_step("STAGE 2", "Mission Launch & Kinematics Telemetry")
    print(f"State: NAVIGATING | Engaging {active_route['route_id']}")
    print(f"Initial Waypoint: '{active_route['path'][0]}' -> Target: '{active_route['path'][1]}'")
    print(f"Robot Velocity: 0.42 m/s | Heading: 1.57 rad (North)")
    print(f"Safety Clearance: Front: 4.85m | Left: 2.10m | Right: 2.20m | Min: 2.10m")
    simulate_delay(1.2, fast)

    # STAGE 3: Dynamic Obstacle Encounter
    print_step("STAGE 3", "Dynamic Obstacle Detection & Sensor Fusion")
    print("LiDAR-Camera Fusion Event:")
    print("  * Object Detected: 'DYNAMIC WAREHOUSE CART'")
    print("  * Distance: 1.84 m | Direction: Front-Right (Bearing: -14.2°)")
    print("  * 3D Position relative to base_link: X=1.78m, Y=-0.45m, Z=0.25m")
    print("  * Confidence: 94.2% | Safety Radius: 0.75m | Dynamic: True")
    print("  * Warning: Corridor 'central_spine_1' -> 'junction_4' BLOCKED by dynamic obstacle!")
    simulate_delay(1.2, fast)

    # STAGE 4: Dynamic Obstacle Replanning
    print_step("STAGE 4", "Corridor Blockage & Dynamic Route Replanning")
    print("Action Taken:")
    print("  1. Emergency Safe Stop dispatched (linear.x = 0.0 m/s)")
    print("  2. Edge ('central_spine_1', 'junction_4') marked BLOCKED in topological graph")
    print("  3. Segment metrics updated: Recorded blockage event & reduced reliability score in SQLite")
    print("  4. Decision Engine re-evaluates candidate routes...")

    graph.mark_edge_blocked('central_spine_1', 'junction_4', blocked=True)
    replanned_routes = graph.find_alternative_routes('junction_1', goal_id, k=2)
    assert len(replanned_routes) > 0, "No alternative route found around blocked central spine"
    bypass_route = replanned_routes[0]
    print(f"Replanned Alternate Route Selected: {bypass_route['route_id']}")
    print(f"  New Trajectory: {' -> '.join(bypass_route['path'])}")
    print(f"  Cost: {bypass_route['total_cost']:.2f} | Bypassing blocked central corridor via West Corridor")
    simulate_delay(1.2, fast)

    # STAGE 5: Dead-End Detection & Backtracking
    print_step("STAGE 5", "Dead-End Intelligence & Safe Backtracking")
    dead_end_target = "dead_end_2"
    de_node = graph.nodes[dead_end_target]
    print(f"Robot approaching alcove segment '{dead_end_target}' at ({de_node.x:.2f}, {de_node.y:.2f})...")
    print("Dead-End Intelligence Triggered:")
    print(f"  1. STOPPING SAFELY: Zero velocity dispatched.")
    print(f"  2. RECORDING DEAD END: Stored coordinates ({de_node.x:.2f}, {de_node.y:.2f}) into SQLite dead_ends table.")
    print(f"  3. PENALIZING SEGMENT: Edge ('junction_3', '{dead_end_target}') marked with dead_end penalty (+10000.0).")
    print(f"  4. BACKTRACKING: Command reverse velocity (linear.x = -0.18 m/s) for 2.0s.")
    print(f"  5. REPLANNING: Excluded dead-end corridor and selected clear route.")

    mem.record_dead_end(dead_end_target, de_node.x, de_node.y)
    graph.mark_edge_dead_end('junction_3', dead_end_target, dead_end=True)
    simulate_delay(1.2, fast)

    # STAGE 6: Robust Recovery Behavior
    print_step("STAGE 6", "Robust Recovery Behavior Execution")
    print("Simulating Stalled Condition (Robot stationary near obstacle for > 4.0s):")
    print("  * Condition Detected: STUCK_NO_PROGRESS (Distance moved < 0.10m)")
    print("  * Recovery State Engaged: Attempt 1 / 3")
    print("  * Audit: Logged recovery event into SQLite recovery_events table.")
    print("  * Recovery Maneuver: Full Stop -> Safe Reverse 1.5m -> 45° Reorientation -> Resume Path.")
    mem.record_recovery_event(
        journey_uuid="DEMO_V2_MISSION",
        x=-6.0, y=1.0,
        recovery_type="STUCK_OBSTACLE_CLEARANCE",
        actions_taken="SAFE REVERSE 1.5m -> REORIENT -> REPLAN",
        success=True,
        message="Recovery cleared tight turn obstruction"
    )
    simulate_delay(1.0, fast)

    # STAGE 7: Goal Arrival & SQLite Memory Persistence
    print_step("STAGE 7", "Goal Arrival & Persistent Journey Storage")
    print(f"Arrived at Destination: '{goal_id.upper()}' (Storage Depot)!")
    print("State: GOAL_REACHED -> Transitioning to IDLE")

    demo_journey_id = f"DEMO-{int(time.time())}"
    demo_coords = [
        [0.0, -11.0], [0.0, -8.0], [0.0, -5.0],
        [-3.0, -5.0], [-6.0, -5.0], [-6.0, 1.0], [-7.0, 3.0]
    ]
    inserted_id = mem.record_journey(
        journey_id=demo_journey_id,
        start_node=start_id,
        goal_node=goal_id,
        path_coords=demo_coords,
        distance_travelled=26.8,
        travel_time=58.4,
        average_speed=0.46,
        min_lidar_clearance=0.34,
        obstacles_encountered=3,
        replans_count=2,
        recovery_events_count=1,
        dead_ends_count=1,
        route_selected="Route B (West Bypass Detour)",
        success=True
    )
    print(f"Saved Journey #{demo_journey_id} into SQLite database (Row ID: {inserted_id}):")
    print(f"  * Distance: 26.8 m | Travel Time: 58.4 s | Avg Speed: 0.46 m/s")
    print(f"  * Min Clearance: 0.34 m | Replans: 2 | Recoveries: 1 | Dead Ends: 1")
    print(f"  * Status: SUCCESS")
    simulate_delay(1.0, fast)

    # STAGE 8: Analytics & Dashboard Telemetry Verification
    print_step("STAGE 8", "Travel Analytics & Heatmap Verification")
    summary = mem.get_analytics_summary()
    print("SQLite Analytics Summary:")
    print(f"  * Total Journeys: {summary['total_journeys']} (Success Rate: {summary['success_rate_pct']}%)")
    print(f"  * Average Distance: {summary['avg_distance_m']} m | Average Travel Time: {summary['avg_travel_time_s']} s")
    print(f"  * Total Replans: {summary['total_replans']} | Total Recoveries: {summary['total_recoveries']}")
    print(f"  * Recorded Dead Ends: {summary['total_dead_ends']}")

    banner("DEMONSTRATION COMPLETED SUCCESSFULLY")
    print("To view this mission and all live analytics on the Web Dashboard:")
    print("  Run: python3 scripts/run_dashboard.py")
    print("  Open in browser: http://127.0.0.1:5050\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
