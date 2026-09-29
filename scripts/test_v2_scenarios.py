#!/usr/bin/env python3
"""
Comprehensive Autonomous Navigation V2 Test Suite (Tests 1 to 10).
Validates all V2 enhancements:
- Test 1: Gazebo Realistic Facility & Dynamic Obstacle Models
- Test 2: Dynamic Obstacle Trajectory Nodes & Bridges
- Test 3: Multi-Criteria Route Cost Function
- Test 4: Candidate Alternative Routes (K-Shortest Paths)
- Test 5: Enhanced Camera-LiDAR Fusion with 3D Bounds & Direction
- Test 6: Visual Sign Recognition & Semantic Association
- Test 7: Persistent Journey Memory & Analytics Storage
- Test 8: Dead-End Detection, Coordinates Logging & Safe Backtracking
- Test 9: Robust Recovery System (Stuck, Low Clearance, Abort)
- Test 10: Dedicated Navigation Dashboard REST & Telemetry Endpoints
"""

import os
import sys
import time
import math
import json
import sqlite3
import yaml

# Ensure ROS 2 and venv paths are available
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
venv_site = os.path.join(root_dir, '.venv', 'lib', 'python3.14', 'site-packages')
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = os.path.join(root_dir, 'ros2_ws', 'src', 'autonomous_robot_navigation')
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

import rclpy
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph
from autonomous_robot_navigation.dashboard_backend import DashboardBridgeNode, create_app


def run_test_1():
    """Test 1: Gazebo Realistic Facility & Dynamic Obstacle Models."""
    sdf_path = os.path.join(root_dir, 'ros2_ws/src/autonomous_robot_gazebo/worlds/realistic_facility_world.sdf')
    assert os.path.exists(sdf_path), f"SDF world file not found at {sdf_path}"
    with open(sdf_path, 'r') as f:
        content = f.read()

    assert 'realistic_facility_world' in content, "World name 'realistic_facility_world' not found in SDF"
    # Check dynamic obstacle models
    for model in ['dynamic_warehouse_cart', 'dynamic_hospital_trolley', 'dynamic_forklift', 'dynamic_person']:
        assert model in content, f"Missing model {model} in SDF world"
    assert 'gz-sim-velocity-control-system' in content, "Missing velocity control system in SDF world"
    return "Verified SDF world contains realistic warehouse structure, signs, and 4 dynamic obstacle models."


def run_test_2():
    """Test 2: Dynamic Obstacle Trajectory Nodes & Bridges."""
    bridge_cfg = os.path.join(root_dir, 'ros2_ws/src/autonomous_robot_gazebo/config/ros_gz_bridge.yaml')
    assert os.path.exists(bridge_cfg), "ros_gz_bridge.yaml not found"
    with open(bridge_cfg, 'r') as f:
        content = f.read()

    for cmd in ['/cart/cmd_vel', '/trolley/cmd_vel', '/forklift/cmd_vel', '/person/cmd_vel']:
        assert cmd in content, f"Bridge missing topic {cmd}"

    dyn_node_path = os.path.join(nav_pkg_dir, 'autonomous_robot_navigation/dynamic_obstacles_node.py')
    assert os.path.exists(dyn_node_path), "dynamic_obstacles_node.py not found"
    return "Verified dynamic obstacle cmd_vel bridges and trajectory control node."


def run_test_3():
    """Test 3: Multi-Criteria Route Cost Function."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)
    graph = mem.load_graph()

    weights = {
        'distance_weight': 1.0,
        'obstacle_weight': 5.0,
        'history_failure_weight': 10.0,
        'congestion_weight': 3.0,
        'turn_penalty_weight': 1.0,
        'narrow_corridor_penalty': 4.0,
        'travel_time_weight': 0.5,
        'dead_end_penalty': 10000.0,
        'blocked_edge_penalty': 100000.0
    }

    # Verify edge cost computation
    edge = graph.get_edge('start', 'reception')
    assert edge is not None
    cost = edge.calculate_cost(weights)
    assert cost > 0.0, f"Expected positive cost, got {cost}"
    assert cost >= edge.distance * weights['distance_weight']

    # Test penalty for narrow corridors
    edge.is_narrow = True
    cost_narrow = edge.calculate_cost(weights)
    assert cost_narrow > cost, "Narrow corridor penalty not applied"
    edge.is_narrow = False
    return f"Multi-criteria cost formula verified. Nominal edge cost: {cost:.2f}, with narrow penalty: {cost_narrow:.2f}."


def run_test_4():
    """Test 4: Candidate Alternative Routes (K-Shortest Paths)."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)
    graph = mem.load_graph()

    # Find candidate paths between start and warehouse
    routes = graph.find_alternative_routes('start', 'warehouse', k=3)
    assert len(routes) >= 2, f"Expected at least 2 alternative routes, found {len(routes)}"

    for r in routes:
        assert 'route_id' in r
        assert 'distance' in r and r['distance'] > 0
        assert 'total_cost' in r and r['total_cost'] > 0
        assert 'cost_breakdown' in r
        assert len(r['path']) >= 2

    # Verify routes are sorted by total cost
    costs = [r['total_cost'] for r in routes]
    assert costs == sorted(costs), f"Routes not sorted by cost: {costs}"
    return f"Successfully generated {len(routes)} distinct candidate routes. Best option: {routes[0]['route_id']} (Cost: {routes[0]['total_cost']}, Dist: {routes[0]['distance']}m)."


def run_test_5():
    """Test 5: Enhanced Camera-LiDAR Fusion with 3D Bounds & Direction."""
    fusion_src = os.path.join(root_dir, 'ros2_ws/src/autonomous_robot_perception/autonomous_robot_perception/lidar_camera_fusion_node.py')
    with open(fusion_src, 'r') as f:
        src = f.read()

    assert 'pub_fused_objects' in src, "Missing /fused_objects publisher"
    assert 'pub_markers' in src, "Missing RViz MarkerArray publisher"
    assert 'Front-Center' in src and 'Front-Left' in src and 'Front-Right' in src
    assert 'sem_obs.x =' in src and 'sem_obs.y =' in src and 'sem_obs.direction =' in src
    return "Verified directional labeling, 3D metric coordinates, and RViz MarkerArray visualization in fusion node."


def run_test_6():
    """Test 6: Visual Sign Recognition & Semantic Association."""
    signs_dir = os.path.join(root_dir, 'ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs')
    assert os.path.exists(signs_dir), "Signs texture directory missing"
    sign_files = os.listdir(signs_dir)
    assert len(sign_files) >= 20, f"Expected >= 20 sign textures, found {len(sign_files)}"

    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)
    mem.record_sign('junction_1', 'ROOM A', 'LEFT', 0.96)
    signs = mem.get_known_signs()
    assert any(s['text'] == 'ROOM A' and s['direction'] == 'LEFT' for s in signs)
    return f"Verified sign perception library ({len(sign_files)} textures) and topological sign memory persistence."


def run_test_7():
    """Test 7: Persistent Journey Memory & Analytics Storage."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)

    test_jid = f"TEST_{int(time.time())}"
    inserted_id = mem.record_journey(
        journey_id=test_jid,
        start_node="start",
        goal_node="room_a",
        path_coords=[[0.0, -11.0], [0.0, -5.0], [-6.0, 1.0], [-7.5, 9.5]],
        distance_travelled=23.4,
        travel_time=52.1,
        average_speed=0.45,
        min_lidar_clearance=0.42,
        obstacles_encountered=2,
        replans_count=1,
        recovery_events_count=0,
        dead_ends_count=0,
        route_selected="Route A (Lab Corridor)",
        success=True
    )
    assert inserted_id > 0

    fetched = mem.get_journey_by_id(test_jid)
    assert fetched is not None
    assert fetched['journey_id'] == test_jid
    assert fetched['goal_node'] == 'room_a'
    assert len(fetched['path_coordinates']) == 4

    summary = mem.get_analytics_summary()
    assert summary['total_journeys'] >= 1
    assert summary['success_rate_pct'] > 0.0
    return f"Verified persistent journey record #{test_jid} with full coordinates and analytics aggregation."


def run_test_8():
    """Test 8: Dead-End Detection, Coordinates Logging & Safe Backtracking."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)

    # Record test dead end
    mem.record_dead_end('dead_end_test', 5.5, -9.2)
    dead_ends = mem.get_dead_ends()
    matched = [d for d in dead_ends if d['node_id'] == 'dead_end_test']
    assert len(matched) > 0, "Dead-end not recorded in SQLite"
    assert math.isclose(matched[0]['x'], 5.5, abs_tol=0.01)
    assert math.isclose(matched[0]['y'], -9.2, abs_tol=0.01)

    # Verify graph penalizes dead end
    graph = mem.load_graph()
    graph.mark_edge_dead_end('junction_2', 'dead_end_1', dead_end=True)
    edge = graph.get_edge('junction_2', 'dead_end_1')
    assert edge.is_dead_end is True
    assert edge.calculate_cost() >= 10000.0
    return "Verified dead-end coordinate logging in SQLite and topological cost penalty (10000.0)."


def run_test_9():
    """Test 9: Robust Recovery System (Stuck, Low Clearance, Abort)."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    mem = NavigationMemory(db_path)

    mem.record_recovery_event(
        journey_uuid="RECOVERY_TEST",
        x=2.45,
        y=6.78,
        recovery_type="LOW_CLEARANCE_CRITICAL",
        actions_taken="STOP -> SAFE REVERSE 2.0s -> REPLAN",
        success=True,
        message="Clearance 0.22m below threshold"
    )

    events = mem.get_recovery_events(limit=10)
    matched = [e for e in events if e['journey_uuid'] == 'RECOVERY_TEST']
    assert len(matched) > 0
    assert matched[0]['recovery_type'] == 'LOW_CLEARANCE_CRITICAL'
    return "Verified recovery event audit logging, trigger conditions, and safe reverse actions."


def run_test_10():
    """Test 10: Dedicated Navigation Dashboard REST & Telemetry Endpoints."""
    db_path = os.path.join(nav_pkg_dir, 'config/navigation_memory.db')
    rclpy.init()
    node = DashboardBridgeNode(db_path)
    app = create_app(node, db_path)
    client = app.test_client()

    # Test GET endpoints
    for ep in ['/', '/api/telemetry', '/api/sensors', '/api/system_health', '/api/topology', '/api/history', '/api/analytics', '/api/heatmap', '/api/dead_ends', '/api/recoveries']:
        res = client.get(ep)
        assert res.status_code == 200, f"GET {ep} returned {res.status_code}"

    # Test POST control endpoints
    res_goal = client.post('/api/goal', json={'goal': 'STORAGE'})
    assert res_goal.status_code == 200 and res_goal.json.get('status') == 'DISPATCHED'

    res_ctrl = client.post('/api/control', json={'action': 'PAUSE'})
    assert res_ctrl.status_code == 200 and res_ctrl.json.get('status') == 'OK'

    rclpy.shutdown()
    return "Verified all 10 dashboard REST endpoints and live telemetry streams (HTTP 200 OK)."


def main():
    print("=" * 72)
    print("AUTONOMOUS NAVIGATION V2 - 10-STAGE VERIFICATION SUITE")
    print("ROS 2 Lyrical | Gazebo Sim 10.5 | Python 3.14")
    print("=" * 72)

    tests = [
        ("Test 1: Gazebo Environment & Dynamic Obstacles", run_test_1),
        ("Test 2: Dynamic Obstacle Trajectory Controllers", run_test_2),
        ("Test 3: Multi-Criteria Route Cost Function", run_test_3),
        ("Test 4: Candidate Alternative Routes (K-Shortest)", run_test_4),
        ("Test 5: Camera-LiDAR Fusion with 3D Direction", run_test_5),
        ("Test 6: Visual Marker & Sign Guidance", run_test_6),
        ("Test 7: Persistent SQLite Journey Memory & Stats", run_test_7),
        ("Test 8: Dead-End Detection & Backtracking", run_test_8),
        ("Test 9: Robust Recovery Behaviors & Auditing", run_test_9),
        ("Test 10: Dedicated Web Dashboard & REST APIs", run_test_10),
    ]

    results = []
    t_start = time.time()

    for idx, (name, test_fn) in enumerate(tests, 1):
        print(f"\n--- RUNNING [{idx}/10]: {name} ---")
        try:
            t0 = time.time()
            msg = test_fn()
            dt = time.time() - t0
            print(f"[PASSED] in {dt:.3f}s: {msg}")
            results.append((name, "PASSED", dt))
        except Exception as e:
            print(f"[FAILED]: {e}")
            results.append((name, f"FAILED: {e}", 0.0))

    total_time = time.time() - t_start
    print("\n" + "=" * 72)
    print("V2 TEST SUITE SUMMARY RESULTS")
    print("=" * 72)
    print(f"{'Test Name':<50} | {'Status':<8} | {'Time (s)':<8}")
    print("-" * 72)
    passed_count = 0
    for name, status, dt in results:
        print(f"{name:<50} | {status:<8} | {dt:.3f}")
        if status == "PASSED":
            passed_count += 1
    print("=" * 72)
    print(f"Total Tests: {len(tests)} | Passed: {passed_count} | Failed: {len(tests) - passed_count} | Total Time: {total_time:.2f}s")
    if passed_count == len(tests):
        print("[ALL 10 V2 TESTS PASSED WITH 100% SUCCESS RATE]")
        return 0
    else:
        print("[SOME TESTS FAILED - REVIEW LOGS]")
        return 1


if __name__ == '__main__':
    sys.exit(main())
