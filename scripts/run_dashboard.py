#!/usr/bin/env python3
"""
Launcher for Autonomous Navigation V2 Dashboard.
Runs on port 5050.
"""
import os
import sys

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
venv_site = os.path.join(root_dir, '.venv', 'lib', 'python3.14', 'site-packages')
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = os.path.join(root_dir, 'ros2_ws', 'src', 'autonomous_robot_navigation')
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

from autonomous_robot_navigation.dashboard_backend import main

if __name__ == '__main__':
    main()
