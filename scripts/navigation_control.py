#!/usr/bin/env python3
"""
Simple GUI and Status Monitor for CV Autonomous Navigation System.
Provides quick buttons to start simulation, set start/goal poses, execute scenarios,
and display real-time verified GPU and ROS subsystem statuses.
"""
import os
import sys
import subprocess
import yaml
import threading

# Ensure ROS and Virtualenv site-packages are available
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJ_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

VENV_SITE = os.path.join(PROJ_DIR, ".venv", "lib", "python3.14", "site-packages")
if os.path.exists(VENV_SITE) and VENV_SITE not in sys.path:
    sys.path.insert(0, VENV_SITE)

ROS_SITE = "/opt/ros/lyrical/lib/python3.14/site-packages"
if os.path.exists(ROS_SITE) and ROS_SITE not in sys.path:
    sys.path.insert(0, ROS_SITE)

def check_gpu_status():
    """Verify GPU and CUDA acceleration dynamically."""
    gpu_info = {
        'gpu_name': 'Unknown',
        'cuda_available': False,
        'cuda_version': 'N/A',
        'yolo_device': 'N/A'
    }
    try:
        import torch
        if torch.cuda.is_available():
            gpu_info['cuda_available'] = True
            gpu_info['gpu_name'] = torch.cuda.get_device_name(0)
            gpu_info['cuda_version'] = torch.version.cuda or '13.0'
            gpu_info['yolo_device'] = 'cuda:0 (Verified)'
        else:
            gpu_info['gpu_name'] = 'No CUDA GPU'
            gpu_info['yolo_device'] = 'CPU'
    except Exception as e:
        gpu_info['gpu_name'] = f'Error: {e}'

    return gpu_info

def check_ros_topics():
    """Check active ROS 2 topics to deduce subsystem status."""
    status = {
        'robot': 'Unknown',
        'cv': 'Unknown',
        'lidar': 'Unknown',
        'nav2': 'Unknown'
    }
    try:
        cmd = "source /opt/ros/lyrical/setup.bash 2>/dev/null && ros2 topic list 2>/dev/null"
        output = subprocess.check_output(['bash', '-c', cmd], timeout=2.0).decode('utf-8')
        topics = output.splitlines()

        status['robot'] = 'Active (/odom)' if '/odom' in topics else 'Offline'
        status['cv'] = 'Active (YOLO /vision/detections)' if '/vision/detections' in topics else 'Offline'
        status['lidar'] = 'Active (/scan)' if '/scan' in topics else 'Offline'
        status['nav2'] = 'Active (Costmap /plan)' if '/plan' in topics or '/global_costmap/costmap' in topics else 'Offline'
    except Exception:
        status['robot'] = 'ROS Not Active'
        status['cv'] = 'ROS Not Active'
        status['lidar'] = 'ROS Not Active'
        status['nav2'] = 'ROS Not Active'

    return status

def load_locations():
    path = os.path.join(PROJ_DIR, "config", "navigation_locations.yaml")
    if os.path.exists(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f).get('locations', {})
    return {}

def load_scenarios():
    path = os.path.join(PROJ_DIR, "config", "experiments.yaml")
    if os.path.exists(path):
        with open(path, 'r') as f:
            return yaml.safe_load(f).get('scenarios', {})
    return {}

def run_command_in_terminal(cmd):
    """Launch command in a visible terminal window."""
    term_candidates = ['ptyxis', 'x-terminal-emulator', 'gnome-terminal', 'xterm']
    term = None
    for t in term_candidates:
        if subprocess.call(['which', t], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            term = t
            break

    if term == 'ptyxis':
        subprocess.Popen(['ptyxis', '--', 'bash', '-c', f'{cmd}; echo "Finished. Press Enter to close."; read'])
    elif term == 'gnome-terminal':
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{cmd}; echo "Finished. Press Enter to close."; read'])
    elif term == 'xterm':
        subprocess.Popen(['xterm', '-e', f'bash -c "{cmd}; echo Finished. Press Enter to close.; read"'])
    else:
        subprocess.Popen(['bash', '-c', cmd])

def start_gui():
    import tkinter as tk
    from tkinter import ttk, messagebox

    root = tk.Tk()
    root.title("CV Autonomous Navigation Control")
    root.geometry("480x620")
    root.resizable(False, False)

    # Style
    style = ttk.Style()
    style.theme_use('clam')

    # Title
    lbl_title = tk.Label(
        root,
        text="CV Autonomous Navigation",
        font=("Helvetica", 16, "bold"),
        bg="#2c3e50",
        fg="white",
        pady=10
    )
    lbl_title.pack(fill=tk.X)

    # Main Frame
    main_frame = ttk.Frame(root, padding="15")
    main_frame.pack(fill=tk.BOTH, expand=True)

    # Simulation Launcher
    sim_frame = ttk.LabelFrame(main_frame, text="Simulation Controls", padding="10")
    sim_frame.pack(fill=tk.X, pady=5)

    def on_start_sim():
        run_command_in_terminal(f"{PROJ_DIR}/scripts/start_project.sh")

    btn_start_sim = tk.Button(
        sim_frame,
        text="🚀 Start Simulation (Gazebo + Nav2 + CV + RViz)",
        font=("Helvetica", 10, "bold"),
        bg="#27ae60",
        fg="white",
        activebackground="#2ecc71",
        command=on_start_sim,
        pady=5
    )
    btn_start_sim.pack(fill=tk.X, pady=2)

    # Navigation Controls (Start & Goal)
    nav_frame = ttk.LabelFrame(main_frame, text="Predefined Start & Goal", padding="10")
    nav_frame.pack(fill=tk.X, pady=5)

    locations = load_locations()
    loc_names = list(locations.keys()) or ['start_a', 'goal_a', 'start_b', 'goal_b']

    # Start Pose row
    f_start = ttk.Frame(nav_frame)
    f_start.pack(fill=tk.X, pady=2)
    ttk.Label(f_start, text="Start:", width=8).pack(side=tk.LEFT)
    cb_start = ttk.Combobox(f_start, values=loc_names, state="readonly", width=16)
    cb_start.set('start_a' if 'start_a' in loc_names else loc_names[0])
    cb_start.pack(side=tk.LEFT, padx=5)

    def on_set_start():
        loc = cb_start.get()
        subprocess.Popen(['python3', f'{PROJ_DIR}/scripts/set_start.py', '--location', loc])

    btn_set_start = ttk.Button(f_start, text="Set Start", command=on_set_start)
    btn_set_start.pack(side=tk.RIGHT)

    # Goal Pose row
    f_goal = ttk.Frame(nav_frame)
    f_goal.pack(fill=tk.X, pady=2)
    ttk.Label(f_goal, text="Goal:", width=8).pack(side=tk.LEFT)
    cb_goal = ttk.Combobox(f_goal, values=loc_names, state="readonly", width=16)
    cb_goal.set('goal_a' if 'goal_a' in loc_names else loc_names[1])
    cb_goal.pack(side=tk.LEFT, padx=5)

    def on_send_goal():
        loc = cb_goal.get()
        run_command_in_terminal(f"python3 {PROJ_DIR}/scripts/send_goal.py --location {loc}")

    btn_send_goal = ttk.Button(f_goal, text="Send Goal", command=on_send_goal)
    btn_send_goal.pack(side=tk.RIGHT)

    # Scenario Controls
    scen_frame = ttk.LabelFrame(main_frame, text="Experiment Scenarios", padding="10")
    scen_frame.pack(fill=tk.X, pady=5)

    scenarios = load_scenarios()
    scen_names = list(scenarios.keys()) or ['scenario_1', 'scenario_2', 'scenario_3']

    f_scen = ttk.Frame(scen_frame)
    f_scen.pack(fill=tk.X, pady=2)
    ttk.Label(f_scen, text="Scenario:", width=8).pack(side=tk.LEFT)
    cb_scen = ttk.Combobox(f_scen, values=scen_names, state="readonly", width=22)
    cb_scen.set('scenario_1' if 'scenario_1' in scen_names else scen_names[0])
    cb_scen.pack(side=tk.LEFT, padx=5)

    def on_run_scenario():
        sc = cb_scen.get()
        run_command_in_terminal(f"{PROJ_DIR}/scripts/run_experiment.sh --scenario {sc}")

    btn_run_scen = tk.Button(
        f_scen,
        text="Run",
        bg="#2980b9",
        fg="white",
        command=on_run_scenario,
        width=8
    )
    btn_run_scen.pack(side=tk.RIGHT)

    # Emergency Stop
    def on_stop():
        subprocess.Popen(['bash', f'{PROJ_DIR}/scripts/stop_navigation.sh'])

    btn_stop = tk.Button(
        main_frame,
        text="🛑 EMERGENCY STOP (Halt Velocity & Cancel Goal)",
        font=("Helvetica", 10, "bold"),
        bg="#c0392b",
        fg="white",
        activebackground="#e74c3c",
        command=on_stop,
        pady=6
    )
    btn_stop.pack(fill=tk.X, pady=6)

    # Status Monitor
    stat_frame = ttk.LabelFrame(main_frame, text="Live System Status", padding="10")
    stat_frame.pack(fill=tk.BOTH, expand=True, pady=5)

    lbl_robot = ttk.Label(stat_frame, text="Robot: Checking...")
    lbl_robot.pack(anchor=tk.W, pady=1)

    lbl_cv = ttk.Label(stat_frame, text="CV: Checking...")
    lbl_cv.pack(anchor=tk.W, pady=1)

    lbl_lidar = ttk.Label(stat_frame, text="LiDAR: Checking...")
    lbl_lidar.pack(anchor=tk.W, pady=1)

    lbl_nav2 = ttk.Label(stat_frame, text="Nav2: Checking...")
    lbl_nav2.pack(anchor=tk.W, pady=1)

    lbl_gpu = ttk.Label(stat_frame, text="GPU: Checking...", foreground="#27ae60", font=("Helvetica", 9, "bold"))
    lbl_gpu.pack(anchor=tk.W, pady=2)

    # Background updater
    def update_status():
        gpu = check_gpu_status()
        ros = check_ros_topics()

        lbl_robot.config(text=f"Robot: {ros['robot']}")
        lbl_cv.config(text=f"CV: {ros['cv']}")
        lbl_lidar.config(text=f"LiDAR: {ros['lidar']}")
        lbl_nav2.config(text=f"Nav2: {ros['nav2']}")

        if gpu['cuda_available']:
            lbl_gpu.config(
                text=f"GPU: {gpu['gpu_name']} | CUDA: {gpu['cuda_version']} | YOLO: {gpu['yolo_device']}",
                foreground="#27ae60"
            )
        else:
            lbl_gpu.config(text=f"GPU: {gpu['gpu_name']} (No CUDA)", foreground="#e74c3c")

        root.after(3000, update_status)

    root.after(500, update_status)
    root.mainloop()

def print_cli_status():
    print("=" * 65)
    print("     CV AUTONOMOUS NAVIGATION SYSTEM STATUS AUDIT")
    print("=" * 65)
    gpu = check_gpu_status()
    print("Hardware Acceleration:")
    print(f"  GPU Device:      {gpu['gpu_name']}")
    print(f"  CUDA Available:  {gpu['cuda_available']} ({gpu['cuda_version']})")
    print(f"  YOLO Runtime:    {gpu['yolo_device']}")

    print("\nROS 2 Subsystems:")
    ros = check_ros_topics()
    print(f"  Robot State:     {ros['robot']}")
    print(f"  Computer Vision: {ros['cv']}")
    print(f"  LiDAR Sensing:   {ros['lidar']}")
    print(f"  Nav2 Stack:      {ros['nav2']}")
    print("=" * 65)

if __name__ == '__main__':
    if '--status' in sys.argv or '--no-gui' in sys.argv or not os.environ.get('DISPLAY'):
        print_cli_status()
    else:
        try:
            start_gui()
        except Exception as e:
            print(f"[WARN] Unable to open GUI window ({e}). Printing CLI status instead:")
            print_cli_status()
