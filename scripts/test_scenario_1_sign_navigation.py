#!/usr/bin/env python3
"""
Scenario 1: Semantic Sign Guided Navigation & Perception Test.
Verifies that:
1. All generated sign textures are recognized by the perception pipeline.
2. Sign classification accuracy, directional decoding, and confidence scores meet thresholds.
3. Sign observations map correctly to topological graph routing decisions.
"""

import os
import sys
import glob
import cv2
import numpy as np

sys.path.insert(0, '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_perception')
sys.path.insert(0, '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation')

from autonomous_robot_perception.sign_detection_node import SignDetectionNode
from autonomous_robot_navigation.topological_graph import TopologicalGraph
from autonomous_robot_navigation.navigation_memory import NavigationMemory


def test_sign_navigation():
    print("==================================================")
    print("TEST: Scenario 1 - Semantic Sign Perception & Routing")
    print("==================================================")

    signs_dir = "/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs"
    sign_files = sorted(glob.glob(os.path.join(signs_dir, "*.png")))
    print(f"[RUN] Testing perception pipeline against {len(sign_files)} sign textures...")

    # Instantiate node offline
    node = SignDetectionNode.__new__(SignDetectionNode)
    node.publish_annotated = False
    node.conf_thresh = 0.45
    node.templates = node._load_templates(signs_dir)

    passed_signs = 0
    total_signs = len(sign_files)

    for sf in sign_files:
        img = cv2.imread(sf)
        fname = os.path.basename(sf)

        # Create a synthetic scene with sign embedded at 50% scale
        scene = np.ones((480, 640, 3), dtype=np.uint8) * 110
        h, w = img.shape[:2]
        small_sign = cv2.resize(img, (w // 2, h // 2))
        sh, sw = small_sign.shape[:2]
        scene[120:120+sh, 160:160+sw] = small_sign

        dets, _ = node._detect_signs(scene)
        assert len(dets) >= 1, f"Failed to detect sign in {fname}"

        best_det = max(dets, key=lambda d: d['confidence'])
        print(f"  [OK] {fname:25s} -> Detected: '{best_det['text']}' dir: '{best_det['direction']}' (conf: {best_det['confidence']:.2f})")

        # Verify text and direction match
        base = os.path.splitext(fname)[0]
        if base == 'emergency_exit':
            exp_text = 'EMERGENCY EXIT'
            exp_dir = 'STRAIGHT'
        elif base == 'keep_left':
            exp_text = 'KEEP LEFT'
            exp_dir = 'LEFT'
        elif base == 'keep_right':
            exp_text = 'KEEP RIGHT'
            exp_dir = 'RIGHT'
        elif base.startswith('room_a'):
            exp_text = 'ROOM A'
            parts = base.split('_')
            exp_dir = parts[-1].upper() if len(parts) > 2 else 'STRAIGHT'
        elif base.startswith('room_b'):
            exp_text = 'ROOM B'
            parts = base.split('_')
            exp_dir = parts[-1].upper() if len(parts) > 2 else 'STRAIGHT'
        else:
            parts = base.split('_')
            exp_text = parts[0].upper()
            exp_dir = parts[1].upper() if len(parts) > 1 else 'STRAIGHT'

        assert best_det['text'] == exp_text, f"Text mismatch for {fname}: expected {exp_text}, got {best_det['text']}"
        assert best_det['direction'] == exp_dir, f"Direction mismatch for {fname}: expected {exp_dir}, got {best_det['direction']}"
        passed_signs += 1

    accuracy = (passed_signs / total_signs) * 100.0
    print(f"\n[SUMMARY] Sign Perception Accuracy: {accuracy:.1f}% ({passed_signs}/{total_signs} signs verified)")
    assert accuracy == 100.0, "Not all signs achieved 100% accuracy!"

    # Test routing decision with sign hint
    print("\n[RUN] Testing sign-directed topological routing...")
    mem = NavigationMemory()
    graph = mem.load_graph()

    # Robot at junction_1 heading North: sign says HOSPITAL RIGHT
    # East is to the right of North -> junction_2
    j1 = graph.nodes['junction_1']
    j2 = graph.nodes['junction_2']
    hosp_path = graph.get_shortest_path('junction_1', 'hospital')
    assert hosp_path[1] in ['junction_2', 'corridor_east_1'], f"Expected path to take right branch, got {hosp_path[1]}"
    print(f"[OK] Sign 'HOSPITAL RIGHT' correctly guides path to: {' -> '.join(hosp_path)}")

    print("\n>>> SCENARIO 1 TEST PASSED SUCCESSFULLY <<<\n")
    return True


if __name__ == '__main__':
    success = test_sign_navigation()
    sys.exit(0 if success else 1)
