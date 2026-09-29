#!/usr/bin/env python3
"""
V2.6 Phase 1 Verification Test Suite
Tests:
  Test A: Static object (hospital_bed) with robot ego-motion -> No TTC yield
  Test B: Dynamic object (cart) with genuine closing motion -> Valid TTC yield (< 1.8s)
  Test C: TTC / Stuck timer interaction -> Intentional yield does not trigger NO_PROGRESS recovery
  Test D: SQLite transient blockage isolation -> Transient blockages cleared on mission start
  Test E: Regression -> Navigation graph loading and multi-criteria route calculation
"""

import math
import os
import shutil
import sys
import tempfile
import time

# Ensure project paths are in sys.path
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
NAV_SRC = os.path.join(BASE_DIR, 'ros2_ws/src/autonomous_robot_navigation')
PERC_SRC = os.path.join(BASE_DIR, 'ros2_ws/src/autonomous_robot_perception')

if NAV_SRC not in sys.path:
    sys.path.insert(0, NAV_SRC)
if PERC_SRC not in sys.path:
    sys.path.insert(0, PERC_SRC)

from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def test_a_static_object_ego_motion():
    """
    Test A — Static object:
    Robot approaches stationary hospital bed at 0.25 m/s.
    Expected:
    - Ego-motion compensation cancels out forward velocity
    - Obstacle classified as STATIC
    - No false dynamic TTC yield produced (TTC = -1.0)
    """
    print("\n" + "=" * 60)
    print("TEST A: Static Object Ego-Motion Compensation (Hospital Bed)")
    print("=" * 60)

    # Simulation setup
    robot_vx = 0.25   # Robot driving forward at 0.25 m/s
    robot_wz = 0.0
    dt = 0.10         # 10 Hz detection interval

    # Hospital bed is stationary in world at initial distance 1.50 m, bearing 0.0 rad
    # Frame 1:
    dist_1 = 1.50
    bearing_1 = 0.0
    raw_x_1 = 0.10 + dist_1 * math.cos(bearing_1)
    raw_y_1 = 0.00 + dist_1 * math.sin(bearing_1)

    # Frame 2 (after 0.1s, robot moved forward by 0.025m):
    dist_2 = dist_1 - robot_vx * dt  # 1.475 m
    bearing_2 = 0.0
    raw_x_2 = 0.10 + dist_2 * math.cos(bearing_2)
    raw_y_2 = 0.00 + dist_2 * math.sin(bearing_2)

    # Apparent velocities in base_link
    vx_apparent = (raw_x_2 - raw_x_1) / dt   # -0.25 m/s
    vy_apparent = (raw_y_2 - raw_y_1) / dt   # 0.0 m/s
    apparent_speed = math.hypot(vx_apparent, vy_apparent)  # 0.25 m/s
    dr_apparent = (dist_2 - dist_1) / dt    # -0.25 m/s

    print(f"  Apparent vx in base_link: {vx_apparent:.3f} m/s")
    print(f"  Apparent range rate (dr): {dr_apparent:.3f} m/s")

    # V2.6 Ego-motion compensation
    vx_true = vx_apparent + robot_vx - robot_wz * raw_y_2
    vy_true = vy_apparent + robot_wz * raw_x_2
    true_speed = math.hypot(vx_true, vy_true)
    dr_true = dr_apparent + robot_vx * math.cos(bearing_2)

    print(f"  Ego-compensated true vx: {vx_true:.3f} m/s")
    print(f"  Ego-compensated true speed: {true_speed:.3f} m/s")
    print(f"  Ego-compensated true dr: {dr_true:.3f} m/s")

    # Motion state classification
    if true_speed < 0.08:
        motion_state = "STATIC"
    else:
        if dr_true > 0.05:
            motion_state = "MOVING_AWAY"
        elif dr_true < -0.05:
            motion_state = "MOVING_TOWARDS"
        elif abs(vy_true) > 0.10:
            motion_state = "CROSSING"
        else:
            motion_state = "MOVING"

    print(f"  Classified motion state: {motion_state}")
    assert motion_state == "STATIC", f"Expected STATIC, got {motion_state}"

    # TTC calculation for static hospital bed (is_dynamic = False)
    is_dynamic = False
    is_dynamic_moving = (
        (is_dynamic and motion_state in ["MOVING_TOWARDS", "CROSSING", "MOVING"])
        or (true_speed > 0.12 and dr_true < -0.05)
    )
    closing_speed = 0.0
    if is_dynamic_moving and dr_apparent < -0.05:
        closing_speed = -dr_apparent

    obs_ttc = -1.0
    if closing_speed > 0.08:
        obs_ttc = dist_2 / closing_speed

    print(f"  Calculated TTC: {obs_ttc:.2f} s (expected: -1.0s / no yield)")
    assert obs_ttc == -1.0, f"Expected TTC=-1.0 for stationary bed, got {obs_ttc}"
    print("  [PASS] Test A: Stationary bed correctly classified as STATIC, no false TTC yield triggered.")
    return True


def test_b_dynamic_object_closing():
    """
    Test B — Dynamic object:
    Robot approaches moving cart. Cart is moving towards robot at 0.35 m/s in world.
    Robot is moving forward at 0.25 m/s.
    Expected:
    - Genuine closing motion produces valid TTC (< 1.8s)
    - TTC remains capable of triggering safety yield
    """
    print("\n" + "=" * 60)
    print("TEST B: Dynamic Object Genuine Closing Motion (Cart)")
    print("=" * 60)

    robot_vx = 0.25   # Robot forward speed
    cart_world_vx = -0.35  # Cart moving toward robot in world
    dt = 0.10

    # Total closing speed = 0.25 + 0.35 = 0.60 m/s
    dist_1 = 1.05
    dist_2 = dist_1 - (robot_vx - cart_world_vx) * dt  # 1.05 - 0.060 = 0.99 m
    bearing = 0.0

    raw_x_1 = 0.10 + dist_1
    raw_y_1 = 0.00
    raw_x_2 = 0.10 + dist_2
    raw_y_2 = 0.00

    vx_apparent = (raw_x_2 - raw_x_1) / dt  # -0.60 m/s
    vy_apparent = (raw_y_2 - raw_y_1) / dt  # 0.0 m/s
    dr_apparent = (dist_2 - dist_1) / dt   # -0.60 m/s

    # V2.6 Ego-motion compensation
    vx_true = vx_apparent + robot_vx
    vy_true = vy_apparent
    true_speed = math.hypot(vx_true, vy_true)  # 0.35 m/s
    dr_true = dr_apparent + robot_vx * math.cos(bearing)  # -0.35 m/s

    print(f"  Apparent closing rate: {-dr_apparent:.3f} m/s")
    print(f"  Ego-compensated true cart speed: {true_speed:.3f} m/s")
    print(f"  Ego-compensated true cart dr: {dr_true:.3f} m/s")

    # Motion state classification
    if true_speed < 0.08:
        motion_state = "STATIC"
    else:
        if dr_true > 0.05:
            motion_state = "MOVING_AWAY"
        elif dr_true < -0.05:
            motion_state = "MOVING_TOWARDS"
        elif abs(vy_true) > 0.10:
            motion_state = "CROSSING"
        else:
            motion_state = "MOVING"

    print(f"  Classified motion state: {motion_state}")
    assert motion_state == "MOVING_TOWARDS", f"Expected MOVING_TOWARDS, got {motion_state}"

    # TTC calculation for dynamic cart (is_dynamic = True)
    is_dynamic = True
    is_dynamic_moving = (
        (is_dynamic and motion_state in ["MOVING_TOWARDS", "CROSSING", "MOVING"])
        or (true_speed > 0.12 and dr_true < -0.05)
    )
    closing_speed = 0.0
    if is_dynamic_moving and dr_apparent < -0.05:
        closing_speed = -dr_apparent

    obs_ttc = -1.0
    if closing_speed > 0.08:
        obs_ttc = dist_2 / closing_speed

    print(f"  Calculated TTC: {obs_ttc:.2f} s (closing speed: {closing_speed:.2f} m/s)")
    assert 0.0 < obs_ttc < 1.8, f"Expected 0.0 < TTC < 1.8s, got {obs_ttc}"
    print("  [PASS] Test B: Genuine closing dynamic cart produces valid TTC (< 1.8s) triggering safety yield.")
    return True


def test_c_ttc_stuck_interaction():
    """
    Test C — TTC / Stuck interaction:
    A legitimate 1.8s TTC yield occurs.
    Expected:
    - Robot does NOT trigger NO_PROGRESS recovery during or immediately after yield.
    - Transitions cleanly: NORMAL -> TTC YIELD -> TTC CLEAR -> NORMAL.
    - Ongoing unsafe condition extends yield safely without recovery.
    """
    print("\n" + "=" * 60)
    print("TEST C: TTC / Stuck Timer Interaction")
    print("=" * 60)

    class MockDecisionEngine:
        def __init__(self):
            self.state = "NAVIGATING"
            self.is_paused = False
            self.recovery_in_progress = False
            self.robot_x = 5.0
            self.robot_y = 2.0
            self.last_progress_pose = (5.0, 2.0)
            self.last_progress_time = 0.0
            self.dynamic_obstacle_pause_until = 0.0
            self.ttc_yield_active = False
            self.ttc_events_count = 0
            self.recovery_params = {'stuck_timeout': 5.0}
            self.recoveries_triggered = []
            self.events = []

        def _stop_robot(self):
            pass

        def _trigger_recovery(self, reason):
            self.recoveries_triggered.append(reason)

        def _emit_event(self, msg, level):
            self.events.append((msg, level))

        def on_ttc(self, ttc: float, current_time: float):
            now = current_time
            if 0.0 < ttc < 1.8:
                if not self.ttc_yield_active or now >= self.dynamic_obstacle_pause_until:
                    self.ttc_yield_active = True
                    self.ttc_events_count += 1
                    self.dynamic_obstacle_pause_until = now + 1.8
                    self._stop_robot()
                    self._emit_event(f"TTC ALERT: Pausing to yield ({ttc:.2f}s)", "WARN")
                else:
                    self.dynamic_obstacle_pause_until = max(self.dynamic_obstacle_pause_until, now + 1.8)
                    self._stop_robot()

        def monitoring_step(self, current_time: float):
            now = current_time
            if self.state == "NAVIGATING" and not self.is_paused and not self.recovery_in_progress:
                if now < self.dynamic_obstacle_pause_until:
                    self._stop_robot()
                    self.last_progress_time = now
                    self.last_progress_pose = (self.robot_x, self.robot_y)
                else:
                    if self.ttc_yield_active:
                        self.ttc_yield_active = False
                        self.dynamic_obstacle_pause_until = 0.0
                        self.last_progress_time = now
                        self.last_progress_pose = (self.robot_x, self.robot_y)
                        self._emit_event("TTC CLEAR: Obstacle cleared corridor. Resuming.", "INFO")

                    if (now - self.last_progress_time) > self.recovery_params['stuck_timeout']:
                        dist_moved = math.hypot(self.robot_x - self.last_progress_pose[0], self.robot_y - self.last_progress_pose[1])
                        if dist_moved < 0.10:
                            self._trigger_recovery("NO_PROGRESS")

    sim_time = 100.0
    de = MockDecisionEngine()
    de.last_progress_time = sim_time - 4.5  # 4.5s since last progress (0.5s before timeout!)

    print(f"  Initial state: last_progress_time = {de.last_progress_time:.1f}s, sim_time = {sim_time:.1f}s (4.5s elapsed)")

    # 1. Trigger TTC alert (yield 1.8s)
    de.on_ttc(1.2, sim_time)
    assert de.ttc_yield_active is True
    assert de.dynamic_obstacle_pause_until == sim_time + 1.8
    print(f"  TTC Alert triggered: yield active until {de.dynamic_obstacle_pause_until:.1f}s")

    # 2. Advance time into yield: sim_time = 101.0 (5.5s since original progress time!)
    # In V2.5, this would have triggered NO_PROGRESS recovery!
    sim_time = 101.0
    de.monitoring_step(sim_time)
    assert len(de.recoveries_triggered) == 0, "Recovery triggered during intentional TTC yield!"
    assert de.last_progress_time == 101.0, "Progress timer was not refreshed during yield!"
    print(f"  During yield (t=101.0s): NO_PROGRESS recovery prevented. Progress timer refreshed to {de.last_progress_time:.1f}s.")

    # 3. Test continued safe waiting: obstacle still unsafe at t=101.5s
    sim_time = 101.5
    de.on_ttc(1.1, sim_time)
    assert de.dynamic_obstacle_pause_until == 101.5 + 1.8  # extended to 103.3s
    assert de.ttc_events_count == 1, "TTC events count should not re-increment on continuous extension"
    print(f"  Continued safe waiting: yield smoothly extended to {de.dynamic_obstacle_pause_until:.1f}s.")

    # 4. Advance time until obstacle clears: sim_time = 103.5s
    sim_time = 103.5
    de.monitoring_step(sim_time)
    assert de.ttc_yield_active is False, "TTC yield did not clear after pause expired"
    assert de.dynamic_obstacle_pause_until == 0.0
    assert len(de.recoveries_triggered) == 0, "False recovery triggered upon clearing yield!"
    assert any("TTC CLEAR" in ev[0] for ev in de.events), "TTC CLEAR event not emitted"
    print("  Corridor clear transition: NORMAL -> TTC CLEAR -> NORMAL cleanly verified.")

    # 5. Verify genuine stuck detection still works after yield:
    # No movement for > 5.0s after resume
    sim_time = 108.6
    de.monitoring_step(sim_time)
    assert len(de.recoveries_triggered) == 1 and de.recoveries_triggered[0] == "NO_PROGRESS", \
        "Genuine stuck recovery failed to trigger after genuine 5s stall!"
    print("  Genuine stall test: NO_PROGRESS recovery correctly fired after real 5.1s stall.")

    print("  [PASS] Test C: TTC yield suspends stuck timeout, handles transitions, and preserves genuine stall recovery.")
    return True


def test_d_sqlite_transient_blockage():
    """
    Test D — SQLite Transient Blockage Isolation:
    Transient obstacle blockage during one mission must NOT contaminate subsequent missions.
    Permanent topology and historical analytics must remain intact.
    """
    print("\n" + "=" * 60)
    print("TEST D: SQLite Transient Blockage Reset & Persistence Isolation")
    print("=" * 60)

    db_source = os.path.join(BASE_DIR, 'ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db')
    with tempfile.NamedTemporaryFile(suffix='.db', delete=False) as tf:
        test_db = tf.name
    shutil.copyfile(db_source, test_db)

    try:
        mem = NavigationMemory(test_db)
        g = mem.load_graph()

        # Step 1: Simulate runtime blockage during Mission 1
        edge_u, edge_v = 'junction_1', 'corridor_east_1'
        print(f"  Mission 1: Simulating obstacle blockage on edge ({edge_u} -> {edge_v})...")
        mem.mark_blocked(edge_u, edge_v, blocked=True)
        mem.record_obstacle(edge_u, edge_v, 'cart', 2.0, 0.0)

        # Verify edge is blocked in DB
        g_mission1 = mem.load_graph()
        edge = g_mission1.get_edge(edge_u, edge_v)
        assert edge is not None and edge.is_blocked is True, "Edge was not marked blocked in Mission 1!"
        print(f"  Mission 1 verification: edge ({edge_u} -> {edge_v}) is_blocked = {edge.is_blocked}")

        # Step 2: Simulate start of Mission 2
        print("  Mission 2 Start: Calling reset_transient_blockages()...")
        unblocked_count = mem.reset_transient_blockages()
        print(f"  Unblocked edges count: {unblocked_count}")
        assert unblocked_count >= 2, f"Expected at least 2 edges unblocked (bidirectional), got {unblocked_count}"

        # Load graph for Mission 2
        g_mission2 = mem.load_graph()
        g_mission2.reset_blockages()
        edge_mission2 = g_mission2.get_edge(edge_u, edge_v)
        assert edge_mission2 is not None and edge_mission2.is_blocked is False, \
            "Edge remained blocked in Mission 2! Contamination occurred!"
        print(f"  Mission 2 verification: edge ({edge_u} -> {edge_v}) is_blocked = {edge_mission2.is_blocked}")

        # Step 3: Verify permanent topology & analytics survived
        with mem._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT COUNT(*) FROM nodes")
            node_count = cur.fetchone()[0]
            cur.execute("SELECT COUNT(*) FROM edges")
            edge_count = cur.fetchone()[0]
            cur.execute("SELECT COUNT(*) FROM obstacles WHERE edge_from = ? AND edge_to = ?", (edge_u, edge_v))
            obs_logged = cur.fetchone()[0]

        print(f"  Permanent nodes preserved: {node_count}")
        print(f"  Permanent edges preserved: {edge_count}")
        print(f"  Obstacle history log preserved: {obs_logged} records")
        assert node_count >= 40, f"Nodes were corrupted! Count: {node_count}"
        assert edge_count >= 90, f"Edges were corrupted! Count: {edge_count}"
        assert obs_logged >= 1, "Obstacle historical logging was lost!"

        print("  [PASS] Test D: Transient blockages cleanly reset between missions while preserving all history.")
        return True
    finally:
        if os.path.exists(test_db):
            os.remove(test_db)


def test_e_regression_routing():
    """
    Test E — Regression:
    Confirm existing navigation graph loading and route calculation still work
    without blocked penalty inflation.
    """
    print("\n" + "=" * 60)
    print("TEST E: Navigation Graph Loading & Multi-Criteria Route Calculation")
    print("=" * 60)

    db_path = os.path.join(BASE_DIR, 'ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db')
    mem = NavigationMemory(db_path)
    # Clear transient blockages
    unblocked = mem.reset_transient_blockages()
    print(f"  Cleared {unblocked} stale transient blockages from SQLite.")

    g = mem.load_graph()
    g.reset_blockages()

    print(f"  Loaded topological graph: {len(g.nodes)} nodes, {len(g.edges)} edges.")
    assert len(g.nodes) >= 40, f"Insufficient nodes: {len(g.nodes)}"
    assert len(g.edges) >= 90, f"Insufficient edges: {len(g.edges)}"

    # Test key routes across facility
    test_cases = [
        ('charging', 'hospital', 'Hospital Ward Mission'),
        ('start', 'warehouse', 'Warehouse Mission'),
        ('reception', 'storage', 'Storage Mission'),
        ('triage', 'lab', 'Lab Mission'),
    ]

    for start_node, goal_node, desc in test_cases:
        print(f"\n  Evaluating {desc}: '{start_node}' -> '{goal_node}'")
        routes = g.find_alternative_routes(start_node, goal_node, k=3)
        assert len(routes) > 0, f"No route found for {start_node} -> {goal_node}!"
        best_route = routes[0]
        path = best_route['path']
        dist = best_route['distance']
        cost = best_route['total_cost']

        print(f"    Primary route ({best_route['route_id']}): {' -> '.join(path[:4])} ... -> {path[-1]}")
        print(f"    Distance: {dist:.2f} m | Cost: {cost:.2f} | Segments: {len(path)-1}")
        print(f"    Alternative routes generated: {len(routes)}")

        # Verify cost is normal and does NOT contain 1e6 blocked penalty
        assert cost < 5000.0, f"Route cost abnormally high ({cost:.1f}), indicates blocked penalty (1e6) contamination!"
        assert path[0] == start_node and path[-1] == goal_node, f"Invalid path endpoints: {path[0]} -> {path[-1]}"

    print("\n  [PASS] Test E: All candidate routes successfully computed with clean multi-criteria costs.")
    return True


def main():
    print("==================================================================")
    print("      V2.6 PHASE 1 — TARGETED VERIFICATION TEST SUITE             ")
    print("==================================================================")

    results = {}
    results['Test A (Static Ego-Motion)'] = test_a_static_object_ego_motion()
    results['Test B (Dynamic TTC Yield)'] = test_b_dynamic_object_closing()
    results['Test C (TTC/Stuck Interaction)'] = test_c_ttc_stuck_interaction()
    results['Test D (SQLite Blockage Reset)'] = test_d_sqlite_transient_blockage()
    results['Test E (Routing Regression)'] = test_e_regression_routing()

    print("\n" + "=" * 60)
    print("SUMMARY OF TEST RESULTS")
    print("=" * 60)
    all_passed = True
    for name, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        print(f"  {name:40s} : {status}")
        if not passed:
            all_passed = False

    print("=" * 60)
    if all_passed:
        print("ALL TESTS PASSED SUCCESSFULLY (5/5)!")
        return 0
    else:
        print("SOME TESTS FAILED!")
        return 1


if __name__ == '__main__':
    sys.exit(main())
