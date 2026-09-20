#!/usr/bin/env python3
"""
Synthetic Dataset Generator for Gazebo Simulation Objects.
Captures frames from /camera/image_raw and generates YOLO-format annotations.
"""
import os
import sys
import yaml

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import cv2
import numpy as np

def create_dataset_structure():
    base_dir = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_objects"
    for split in ["train", "val"]:
        os.makedirs(os.path.join(base_dir, "images", split), exist_ok=True)
        os.makedirs(os.path.join(base_dir, "labels", split), exist_ok=True)

    yaml_content = {
        'path': base_dir,
        'train': 'images/train',
        'val': 'images/val',
        'names': {
            0: 'person',
            1: 'chair',
            2: 'table',
            3: 'box',
            4: 'traffic_cone'
        }
    }

    yaml_path = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_data.yaml"
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)

    print(f"Dataset structure and YAML created at: {yaml_path}")

if __name__ == '__main__':
    create_dataset_structure()
