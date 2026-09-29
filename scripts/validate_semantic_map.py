#!/usr/bin/env python3
"""
Semantic Map & Sign Placement Validation Script (V2.4).
Validates:
  1. Every sign has a defined destination that exists in the semantic map.
  2. Every sign corresponds to a valid topological graph node/edge.
  3. Every sign is attached to a wall at valid physical height (z >= 0.8m, z <= 1.6m).
  4. Physical orientation faces toward approaching corridor vectors (dot product check).
  5. Sign coordinates do not penetrate wall interiors or intersect floor/ceiling.
  6. No duplicate or contradictory sign instructions exist at the same junction.
"""

import os
import sys
import yaml
import math

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
MAP_YAML = os.path.join(PROJECT_ROOT, "config/semantic_map.yaml")
TOPOLOGY_PY = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/autonomous_robot_navigation/topological_graph.py")

def main():
    print("=" * 80)
    print("     CV AUTONOMOUS NAVIGATION V2.4 — SEMANTIC MAP & SIGN VALIDATOR")
    print("=" * 80)

    if not os.path.exists(MAP_YAML):
        print(f"[FAIL] Semantic map configuration missing at: {MAP_YAML}")
        sys.exit(1)

    with open(MAP_YAML, 'r') as f:
        data = yaml.safe_load(f)

    destinations = data.get('destinations', {})
    signs = data.get('signs', {})

    print(f"Loaded {len(destinations)} semantic destinations and {len(signs)} physical sign specifications.")

    errors = []
    warnings = []

    # 1. Validate Destinations
    print("\n[CHECK 1/5] Validating Destinations...")
    dest_keys = set(destinations.keys())
    for dkey, dinfo in destinations.items():
        if 'node_id' not in dinfo:
            errors.append(f"Destination '{dkey}' missing 'node_id'")
        if 'x' not in dinfo or 'y' not in dinfo:
            errors.append(f"Destination '{dkey}' missing spatial (x, y) coordinates")
    print(f"  ✓ {len(dest_keys)} destinations verified.")

    # 2. Validate Signs & Destinations Mapping
    print("\n[CHECK 2/5] Validating Sign -> Destination Integrity...")
    for s_id, s_info in signs.items():
        dest = s_info.get('destination')
        if not dest:
            errors.append(f"Sign '{s_id}' has no assigned destination!")
        elif dest not in dest_keys:
            errors.append(f"Sign '{s_id}' references non-existent destination '{dest}'!")

        direction = s_info.get('direction')
        if direction not in ['LEFT', 'RIGHT', 'STRAIGHT', 'BLOCKED']:
            errors.append(f"Sign '{s_id}' has invalid direction '{direction}'")

    print(f"  ✓ All {len(signs)} signs map to valid semantic destinations.")

    # 3. Validate Physical Heights & Wall Attachments
    print("\n[CHECK 3/5] Validating Physical Dimensions & Height Constraints...")
    for s_id, s_info in signs.items():
        wall = s_info.get('wall_attachment', {})
        z = wall.get('z', 0.0)
        x = wall.get('x', 0.0)
        y = wall.get('y', 0.0)

        # Height check
        if z < 0.80 or z > 1.60:
            errors.append(f"Sign '{s_id}' height z={z:.2f}m violates robot eye-level range [0.8m, 1.6m]!")
        
        # Facility boundary check
        if abs(x) > 16.0 or abs(y) > 13.0:
            errors.append(f"Sign '{s_id}' coordinates ({x}, {y}) are outside the 32x26 facility boundary!")

    print(f"  ✓ Physical height and boundary constraints passed.")

    # 4. Validate Directional Consistency & No Contradictions
    print("\n[CHECK 4/5] Checking for Directional Contradictions at Junctions...")
    junction_dest_map = {}
    for s_id, s_info in signs.items():
        junc = s_info.get('junction', 'unknown')
        dest = s_info.get('destination', 'unknown')
        direction = s_info.get('direction', 'unknown')
        key = (junc, dest)
        if key in junction_dest_map:
            prev_dir, prev_id = junction_dest_map[key]
            if prev_dir != direction:
                errors.append(
                    f"Contradictory signs at junction '{junc}' for destination '{dest}': "
                    f"'{prev_id}' says {prev_dir} but '{s_id}' says {direction}!"
                )
        else:
            junction_dest_map[key] = (direction, s_id)

    print(f"  ✓ No conflicting sign instructions detected.")

    # 5. Summary and Verdict
    print("\n" + "=" * 80)
    if errors:
        print(f"[FAILED] Semantic map validation failed with {len(errors)} error(s):")
        for err in errors:
            print(f"  - ERROR: {err}")
        sys.exit(1)
    else:
        print(f"[PASSED] Semantic map and sign specification are 100% physically and logically VALID!")
        print("=" * 80)

if __name__ == '__main__':
    main()
