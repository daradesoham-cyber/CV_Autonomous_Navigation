#!/usr/bin/env python3
"""
Scenario 3: Dead End Avoidance & Pruning Test.
Verifies that:
1. Dead-end nodes are correctly classified in topological memory.
2. Shortest path routing strictly avoids cul-de-sacs.
3. Dead-end edge penalties prevent backtracking traps.
"""

import sys
import tempfile
import os

sys.path.insert(0, '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation')
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


def test_dead_end_avoidance():
    print("==================================================")
    print("TEST: Scenario 3 - Dead End Avoidance & Pruning")
    print("==================================================")

    mem = NavigationMemory()
    graph = mem.load_graph()

    dead_ends = [nid for nid, n in graph.nodes.items() if n.node_type == 'dead_end']
    print(f"[OK] Identified {len(dead_ends)} dead-end nodes: {dead_ends}")
    assert len(dead_ends) >= 3, f"Expected at least 3 dead ends, found {len(dead_ends)}"
    assert 'dead_end_1' in dead_ends
    assert 'dead_end_2' in dead_ends
    assert 'dead_end_3' in dead_ends

    # Verify that paths to all destinations avoid dead ends
    destinations = ['hospital', 'warehouse', 'office', 'lab', 'storage', 'cafeteria', 'exit']
    for dest in destinations:
        path = graph.get_shortest_path('start', dest)
        assert path is not None, f"Failed to find path to {dest}"
        for de in dead_ends:
            assert de not in path, f"Dead end '{de}' was incorrectly included in path to '{dest}': {path}"
        print(f"[OK] Path to '{dest}' ({len(path)} nodes: {' -> '.join(path)}) cleanly avoids all dead ends.")

    # Verify dead end penalty
    edge_de1 = graph.get_edge('junction_2', 'dead_end_1')
    assert edge_de1.is_dead_end is True
    assert edge_de1.get_effective_cost() >= 1e5, "Dead end edge cost is not penalized!"
    print(f"[OK] Dead-end edge cost penalty verified: {edge_de1.get_effective_cost():.0f}")

    print("\n>>> SCENARIO 3 TEST PASSED SUCCESSFULLY <<<\n")
    return True


if __name__ == '__main__':
    success = test_dead_end_avoidance()
    sys.exit(0 if success else 1)
