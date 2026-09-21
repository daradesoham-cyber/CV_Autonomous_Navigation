#!/usr/bin/env python3
"""
Scenario 4: Memory Persistence & Sign Database Verification Test.
Verifies that:
1. SQLite database initializes topological graph correctly.
2. Sign observations are properly persisted and retrievable.
3. Edge blockages and traversal counts persist across reloads.
4. Graph state reloaded from disk matches in-memory modifications.
"""

import os
import sys
import time
import tempfile

# Add project path
sys.path.insert(0, '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation')
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def test_memory_persistence():
    print("==================================================")
    print("TEST: Scenario 4 - Navigation Memory Persistence")
    print("==================================================")

    with tempfile.NamedTemporaryFile(suffix='.db', delete=False) as tf:
        temp_db_path = tf.name

    try:
        # Step 1: Initialize DB
        mem = NavigationMemory(temp_db_path)
        g1 = mem.load_graph()
        print(f"[OK] Initialized database with {len(g1.nodes)} nodes and {len(g1.edges)} edges.")
        assert len(g1.nodes) >= 20, "Expected at least 20 topological nodes"
        assert len(g1.edges) >= 40, "Expected at least 40 edges"

        is_realistic = 'reception' in g1.nodes
        test_edge_u, test_edge_v = ('start', 'reception') if is_realistic else ('start', 'junction_1')
        obs_edge_u, obs_edge_v = ('wh_junction_6', 'warehouse') if is_realistic else ('junction_2', 'junction_5')

        # Step 2: Record Sign Observations
        print("[RUN] Recording sign observations...")
        mem.record_sign('junction_1', 'HOSPITAL', 'RIGHT', 0.94)
        mem.record_sign('junction_2', 'WAREHOUSE', 'STRAIGHT', 0.91)
        mem.record_sign('junction_3', 'STORAGE', 'LEFT', 0.88)
        mem.record_sign('junction_4', 'CAFETERIA', 'RIGHT', 0.92)

        signs = mem.get_known_signs()
        print(f"[OK] Retrieved {len(signs)} signs from database.")
        assert len(signs) == 4, f"Expected 4 signs, got {len(signs)}"
        assert signs[0]['text'] in ['CAFETERIA', 'STORAGE', 'WAREHOUSE', 'HOSPITAL']

        # Step 3: Record Edge Traversal & Obstacle Blockage
        print(f"[RUN] Recording successful traversal on ({test_edge_u} -> {test_edge_v}) and obstacle on ({obs_edge_u} -> {obs_edge_v})...")
        mem.record_traversal(test_edge_u, test_edge_v, success=True, duration=3.2)
        mem.record_obstacle(obs_edge_u, obs_edge_v, 'cart', 3.0, 3.0)

        # Step 4: Reload from DB in a fresh NavigationMemory instance
        print("[RUN] Simulating node restart: loading fresh instance from disk...")
        mem_reloaded = NavigationMemory(temp_db_path)
        g2 = mem_reloaded.load_graph()

        edge_trav = g2.get_edge(test_edge_u, test_edge_v)
        assert edge_trav is not None and edge_trav.traversal_count >= 1, "Traversal count did not persist!"
        print(f"[OK] Persisted traversal count for '{test_edge_u}' -> '{test_edge_v}': {edge_trav.traversal_count}")

        edge_blocked = g2.get_edge(obs_edge_u, obs_edge_v)
        assert edge_blocked is not None and edge_blocked.is_blocked is True, "Edge blocked status did not persist!"
        print(f"[OK] Persisted blockage for '{obs_edge_u}' -> '{obs_edge_v}'.")

        # Step 5: Test Reset
        print("[RUN] Testing memory reset...")
        mem_reloaded.reset_memory()
        g3 = mem_reloaded.load_graph()
        assert g3.get_edge(obs_edge_u, obs_edge_v).is_blocked is False, "Blocked status not cleared after reset!"
        assert len(mem_reloaded.get_known_signs()) == 0, "Signs not cleared after reset!"
        print("[OK] Reset successfully cleared dynamic states while preserving topology.")

        print("\n>>> SCENARIO 4 TEST PASSED SUCCESSFULLY <<<\n")
        return True

    finally:
        if os.path.exists(temp_db_path):
            os.remove(temp_db_path)


if __name__ == '__main__':
    success = test_memory_persistence()
    sys.exit(0 if success else 1)
