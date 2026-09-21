#!/usr/bin/env python3
"""
Scenario 2: Corridor Blockage & Dynamic Replanning Test.
Verifies that:
1. Dynamic obstacle detection marks edge blocked in memory.
2. Topological planner detects blockage and computes alternate route.
3. Bypass corridor is selected when primary corridor is obstructed.
4. Edge unblocking restores normal routing.
"""

import sys
import os

sys.path.insert(0, '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation')
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def test_dynamic_replanning():
    print("==================================================")
    print("TEST: Scenario 2 - Dynamic Replanning & Bypass")
    print("==================================================")

    mem = NavigationMemory()
    mem.reset_memory()
    graph = mem.load_graph()

    is_realistic = 'wh_junction_6' in graph.nodes

    if is_realistic:
        # Case 1: Route to Warehouse in Realistic Facility
        initial_path = graph.get_shortest_path('start', 'warehouse')
        print(f"[RUN] Initial path start -> warehouse: {' -> '.join(initial_path)}")

        print("[RUN] Simulating blockage on primary corridor (wh_junction_6 -> warehouse)...")
        graph.mark_edge_blocked('wh_junction_6', 'warehouse', blocked=True)
        replanned_path = graph.get_shortest_path('start', 'warehouse')
        print(f"[OK] Replanned path around blockage: {' -> '.join(replanned_path)}")
        assert 'office_wh_bypass' in replanned_path, "Replanned path did not use North office-warehouse bypass!"
        assert replanned_path[-1] == 'warehouse', "Replanned path did not reach warehouse!"

        # Case 2: West Bypass Corridor
        print("\n[RUN] Testing West Bypass corridor in realistic facility...")
        init_lab_path = graph.get_shortest_path('junction_7', 'lab')
        print(f"[RUN] Initial path junction_7 -> lab: {' -> '.join(init_lab_path)}")
        assert init_lab_path == ['junction_7', 'junction_8', 'lab']

        print("[RUN] Blocking junction_7 -> junction_8 (simulating dynamic cart/pallet)...")
        graph.mark_edge_blocked('junction_7', 'junction_8', blocked=True)
        bypass_path = graph.get_shortest_path('junction_7', 'lab')
        print(f"[OK] Replanned route via West Bypass: {' -> '.join(bypass_path)}")
        assert 'west_bypass_south' in bypass_path, "Did not route via west_bypass_south!"
        assert 'west_bypass_north' in bypass_path, "Did not route via west_bypass_north!"

        # Case 3: Restore / Unblock
        print("\n[RUN] Unblocking junction_7 -> junction_8...")
        graph.mark_edge_blocked('junction_7', 'junction_8', blocked=False)
        restored_path = graph.get_shortest_path('junction_7', 'lab')
        print(f"[OK] Restored path: {' -> '.join(restored_path)}")
        assert restored_path == ['junction_7', 'junction_8', 'lab']
    else:
        # Complex layout
        initial_path = graph.get_shortest_path('junction_2', 'warehouse')
        print(f"[RUN] Initial path junction_2 -> warehouse: {' -> '.join(initial_path)}")
        graph.mark_edge_blocked('junction_2', 'junction_5', blocked=True)
        replanned_path = graph.get_shortest_path('junction_2', 'warehouse')
        print(f"[OK] Replanned path around blockage: {' -> '.join(replanned_path)}")
        assert 'warehouse' in replanned_path

        graph.mark_edge_blocked('junction_4', 'storage', blocked=True)
        bypass_path = graph.get_shortest_path('junction_4', 'storage')
        assert 'west_bypass_mid' in bypass_path

    print("\n>>> SCENARIO 2 TEST PASSED SUCCESSFULLY <<<\n")
    return True


if __name__ == '__main__':
    success = test_dynamic_replanning()
    sys.exit(0 if success else 1)
