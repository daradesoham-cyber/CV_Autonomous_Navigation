#!/usr/bin/env python3
"""
Plots genuine recorded robot trajectories from v26_mission_validation_results.json.
STRICT DATA INTEGRITY:
- Uses ONLY raw, timestamped recorded coordinates from the robot.
- NO synthetic waypoints.
- NO random noise injection (np.random).
- If trajectory data is missing, marks as TRAJECTORY EVIDENCE UNAVAILABLE.
"""

import os
import sys
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
RESULTS_JSON = os.path.join(PROJECT_ROOT, "results/v26_final/v26_mission_validation_results.json")
OUT_DIR = os.path.join(PROJECT_ROOT, "report_evidence/v26_final/objective01_navigation")
os.makedirs(OUT_DIR, exist_ok=True)

ZONES = [
    ("Reception & Entrance", (-6, -12), 12, 5, '#e0f2fe', '#0284c7'),
    ("Medical Hospital Ward", (3, -12), 11, 6, '#fee2e2', '#ef4444'),
    ("Industrial Warehouse", (4, 4), 10, 8, '#fef3c7', '#d97706'),
    ("Research Lab (Room A)", (-14, 6), 11, 6, '#f3e8ff', '#9333ea'),
    ("Storage Depot", (-14, 0), 10, 5, '#ede9fe', '#7c3aed'),
    ("Executive Office", (-4, 8), 7, 4, '#dcfce7', '#16a34a'),
    ("AGV Charging Station", (-15, 0), 3, 4, '#e2e8f0', '#64748b'),
    ("Cargo Loading Bay", (10, 0), 5, 4, '#f1f5f9', '#475569')
]

def draw_base_facility(ax):
    ax.set_facecolor('#f8fafc')
    boundary = patches.Rectangle((-16, -13), 32, 26, linewidth=2.5, edgecolor='#1e293b', facecolor='#ffffff', zorder=1)
    ax.add_patch(boundary)

    for label, (zx, zy), zw, zh, fc, ec in ZONES:
        rect = patches.Rectangle((zx, zy), zw, zh, linewidth=1.2, edgecolor=ec, facecolor=fc, alpha=0.55, zorder=2)
        ax.add_patch(rect)
        ax.text(zx + zw/2, zy + zh/2, label, fontsize=9.5, fontweight='bold', color=ec, ha='center', va='center', zorder=3)

    spine = patches.Rectangle((-1.5, -7), 3, 15, linewidth=1, linestyle=':', edgecolor='#94a3b8', facecolor='none', zorder=2)
    ax.add_patch(spine)
    ax.text(0, -0.5, "Central Spine Corridor", fontsize=8.5, color='#64748b', rotation=90, ha='center', va='center', zorder=3)

    ax.set_xlim(-16.5, 16.5)
    ax.set_ylim(-13.5, 13.5)
    ax.grid(True, linestyle=':', alpha=0.5, color='#94a3b8')
    ax.set_xlabel("X Coordinate (meters)", fontsize=10.5, fontweight='semibold')
    ax.set_ylabel("Y Coordinate (meters)", fontsize=10.5, fontweight='semibold')

def plot_single_mission_trajectory(mission_data, out_filename, title):
    traj = mission_data.get("trajectory", [])
    fig, ax = plt.subplots(figsize=(11, 9), dpi=300)
    draw_base_facility(ax)

    s_pose = mission_data["start_pose"]
    g_pose = mission_data["goal_pose"]
    ax.scatter(s_pose['x'], s_pose['y'], color='#22c55e', s=160, marker='o', edgecolors='white', linewidth=2,
               label=f"Start: {mission_data['start_location']} ({s_pose['x']:.1f}, {s_pose['y']:.1f})", zorder=7)
    ax.scatter(g_pose['x'], g_pose['y'], color='#dc2626', s=180, marker='*', edgecolors='black', linewidth=1.5,
               label=f"Goal: {mission_data['goal_location']} ({g_pose['x']:.1f}, {g_pose['y']:.1f})", zorder=7)

    if traj and len(traj) > 1:
        xs = [pt['x'] for pt in traj]
        ys = [pt['y'] for pt in traj]
        outcome_color = '#0284c7' if mission_data['success'] else '#ef4444'
        label_text = f"Actual Recorded Motion ({mission_data['path_distance_m']}m, {mission_data['elapsed_time_s']}s)"
        ax.plot(xs, ys, color=outcome_color, linewidth=2.5, label=label_text, zorder=6)
        # Final pose reached
        ax.scatter(xs[-1], ys[-1], color='#a855f7', s=120, marker='X', edgecolors='black', linewidth=1.5,
                   label=f"Final Position: ({xs[-1]:.2f}, {ys[-1]:.2f})", zorder=8)
    else:
        ax.text(0, 0, "TRAJECTORY EVIDENCE UNAVAILABLE\n(Robot remained stationary or no motion recorded)",
                fontsize=12, color='#ef4444', fontweight='bold', ha='center', va='center', zorder=10,
                bbox=dict(boxstyle='round,pad=0.5', facecolor='#fee2e2', edgecolor='#ef4444'))

    outcome_str = "SUCCESS" if mission_data['success'] else f"FAILURE ({mission_data['termination_reason']})"
    ax.set_title(f"{title}\nActual Live Trajectory | Outcome: {outcome_str}", fontsize=12, fontweight='bold', pad=12)
    ax.legend(loc='lower left', frameon=True, facecolor='white', framealpha=0.95, fontsize=9)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, out_filename)
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[OK] Saved real trajectory plot to {out_path}")

def plot_overview_all(all_missions):
    fig, ax = plt.subplots(figsize=(12, 10), dpi=300)
    draw_base_facility(ax)

    colors = ['#0284c7', '#059669', '#d97706', '#dc2626', '#9333ea', '#0891b2', '#ea580c', '#4f46e5', '#16a34a', '#e11d48']

    for m in all_missions:
        mid = m["mission_id"]
        traj = m.get("trajectory", [])
        c = colors[(mid - 1) % len(colors)]
        if traj and len(traj) > 1:
            xs = [pt['x'] for pt in traj]
            ys = [pt['y'] for pt in traj]
            status = "PASS" if m['success'] else "FAIL"
            ax.plot(xs, ys, color=c, linewidth=2.0, alpha=0.85,
                    label=f"M{mid}: {m['start_location']} -> {m['goal_location']} [{status}] ({m['path_distance_m']}m)")

    ax.set_title("V2.6 Verified Live Trajectory Multi-Mission Overview\nGenuine Robot Odometry Tracks from Automated Forensic Benchmark",
                 fontsize=13, fontweight='bold', pad=15)
    ax.legend(loc='lower left', frameon=True, facecolor='white', framealpha=0.95, fontsize=8.5)
    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "real_multitask_trajectories_overview.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"[OK] Saved real overview trajectory plot to {out_path}")

def main():
    if not os.path.exists(RESULTS_JSON):
        print(f"[ERROR] Results JSON not found at {RESULTS_JSON}")
        sys.exit(1)

    with open(RESULTS_JSON, 'r') as f:
        missions = json.load(f)

    for m in missions:
        mid = m["mission_id"]
        fname = f"real_trajectory_mission_{mid:02d}_{m['start_location'].lower()}_to_{m['goal_location'].lower()}.png"
        title = f"Mission {mid}: {m['start_location']} → {m['goal_location']}"
        plot_single_mission_trajectory(m, fname, title)

    plot_overview_all(missions)
    print("[OK] All real trajectory figures rendered from live telemetry data.")

if __name__ == "__main__":
    main()
