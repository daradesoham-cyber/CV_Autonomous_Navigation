#!/usr/bin/env python3
"""
Generate Presentation & Publication Quality Visualization Plots.
Reads experimental results from results/*.csv and outputs plots to results/plots/.

Generated plots:
1. results/plots/detection_metrics.png - Precision, Recall, mAP50, mAP50-95 comparison
2. results/plots/inference_speed.png - Latency and FPS throughput on RTX 3050 GPU
3. results/plots/ablation_navigation.png - Navigation Success Rate, Replans, Collisions
4. results/plots/clearance_and_path.png - Path Length vs Obstacle Clearance across modalities
5. results/plots/fusion_accuracy.png - LiDAR-Camera Distance Estimation Accuracy & Error
"""
import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def set_plot_style():
    plt.rcParams.update({
        'font.size': 12,
        'axes.labelsize': 13,
        'axes.titlesize': 14,
        'xtick.labelsize': 11,
        'ytick.labelsize': 11,
        'legend.fontsize': 11,
        'figure.titlesize': 16,
        'figure.dpi': 300,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--'
    })

def plot_detection_metrics(csv_path, out_path):
    if not os.path.exists(csv_path):
        print(f"[SKIP] {csv_path} not found.")
        return
    df = pd.read_csv(csv_path)

    metrics = ['Precision', 'Recall', 'mAP50', 'mAP50-95']
    models = df['Model'].tolist()

    x = np.arange(len(metrics))
    width = 0.35

    fig, ax = plt.subplots(figsize=(9, 5.5))
    colors = ['#4A90E2', '#50E3C2']

    for i, model_name in enumerate(models):
        row = df[df['Model'] == model_name].iloc[0]
        vals = [float(row[m]) for m in metrics]
        offset = (i - 0.5) * width
        bars = ax.bar(x + offset, vals, width, label=model_name, color=colors[i % len(colors)], edgecolor='black', alpha=0.88)
        # Add labels on top of bars
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax.set_ylabel('Score [0.0 - 1.0]')
    ax.set_title('Object Detection Benchmark: Pretrained vs Custom YOLOv8n (Gazebo Test Split)', pad=15)
    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontweight='bold')
    ax.set_ylim(0, 1.15)
    ax.legend(loc='upper right', frameon=True)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    print(f"[SAVED] {out_path}")

def plot_inference_speed(csv_path, out_path):
    if not os.path.exists(csv_path):
        print(f"[SKIP] {csv_path} not found.")
        return
    df = pd.read_csv(csv_path)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))
    models = [m.replace(' (COCO)', '').replace(' (Gazebo Fine-Tuned)', '') for m in df['Model']]
    fps_vals = [float(x) for x in df['FPS']]
    latency_vals = [float(x) for x in df['Latency_ms']]

    # FPS Bar Chart
    bars1 = ax1.bar(models, fps_vals, color=['#3498db', '#2ecc71'], edgecolor='black', width=0.45)
    ax1.set_ylabel('Throughput (Frames Per Second)')
    ax1.set_title('Inference Throughput (FPS) on RTX 3050')
    ax1.set_ylim(0, max(fps_vals) * 1.25)
    for bar in bars1:
        h = bar.get_height()
        ax1.annotate(f'{h:.1f} FPS', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    # Latency Bar Chart
    bars2 = ax2.bar(models, latency_vals, color=['#e67e22', '#e74c3c'], edgecolor='black', width=0.45)
    ax2.set_ylabel('Latency (milliseconds)')
    ax2.set_title('Inference Latency (ms) on RTX 3050')
    ax2.set_ylim(0, max(latency_vals) * 1.35)
    for bar in bars2:
        h = bar.get_height()
        ax2.annotate(f'{h:.2f} ms', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    plt.suptitle('NVIDIA GeForce RTX 3050 Laptop GPU (4 GB VRAM) Benchmark', y=1.02)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    print(f"[SAVED] {out_path}")

def plot_ablation_navigation(csv_path, out_path):
    if not os.path.exists(csv_path):
        print(f"[SKIP] {csv_path} not found.")
        return
    df = pd.read_csv(csv_path)

    modalities = df['Modality'].tolist()
    success_rates = [float(x) for x in df['Success_Rate_pct']]
    replans = [int(x) for x in df['Replans']]
    collisions = [int(x) for x in df['Collisions']]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Success Rate
    colors = ['#e74c3c' if s < 80 else '#f39c12' if s < 95 else '#27ae60' for s in success_rates]
    bars1 = ax1.bar(modalities, success_rates, color=colors, edgecolor='black', width=0.5)
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('Navigation Success Rate Across Modalities')
    ax1.set_ylim(0, 115)
    for bar in bars1:
        h = bar.get_height()
        ax1.annotate(f'{h:.1f}%', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    # Replans and Collisions
    x = np.arange(len(modalities))
    width = 0.35
    b_rep = ax2.bar(x - width/2, replans, width, label='Replans', color='#3498db', edgecolor='black')
    b_col = ax2.bar(x + width/2, collisions, width, label='Collisions', color='#e74c3c', edgecolor='black')
    ax2.set_xticks(x)
    ax2.set_xticklabels(modalities)
    ax2.set_ylabel('Count')
    ax2.set_title('Replans vs Collisions')
    ax2.legend(loc='upper right')
    ax2.set_ylim(0, max(max(replans), max(collisions)) + 3)

    for bar in b_rep:
        h = bar.get_height()
        ax2.annotate(f'{int(h)}', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontweight='bold')
    for bar in b_col:
        h = bar.get_height()
        ax2.annotate(f'{int(h)}', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 2), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    plt.suptitle('Ablation Study: Navigation Robustness in Complex 30x30m World', y=1.02)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    print(f"[SAVED] {out_path}")

def plot_clearance_and_path(csv_path, out_path):
    if not os.path.exists(csv_path):
        print(f"[SKIP] {csv_path} not found.")
        return
    df = pd.read_csv(csv_path)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))
    modalities = df['Modality'].tolist()
    path_len = [float(x) for x in df['Avg_Path_Length_m']]
    clearance = [float(x) for x in df['Avg_Min_Clearance_m']]

    # Path Length
    bars1 = ax1.bar(modalities, path_len, color='#9b59b6', edgecolor='black', width=0.45)
    ax1.set_ylabel('Average Path Length (meters)')
    ax1.set_title('Average Executed Path Length')
    ax1.set_ylim(0, max(path_len) * 1.25)
    for bar in bars1:
        h = bar.get_height()
        ax1.annotate(f'{h:.2f} m', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    # Min Clearance
    bars2 = ax2.bar(modalities, clearance, color='#1abc9c', edgecolor='black', width=0.45)
    ax2.set_ylabel('Minimum Clearance (meters)')
    ax2.set_title('Average Minimum Clearance to Obstacles')
    ax2.set_ylim(0, max(clearance) * 1.35)
    for bar in bars2:
        h = bar.get_height()
        ax2.annotate(f'{h:.2f} m', xy=(bar.get_x() + bar.get_width()/2, h),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    plt.suptitle('Navigation Efficiency & Safety Margin Analysis', y=1.02)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    print(f"[SAVED] {out_path}")

def plot_fusion_accuracy(csv_path, out_path):
    if not os.path.exists(csv_path):
        print(f"[SKIP] {csv_path} not found.")
        return
    df = pd.read_csv(csv_path)

    gt = df['ground_truth_m'].astype(float).values
    est = df['estimated_distance_m'].astype(float).values
    err = df['absolute_error_m'].astype(float).values

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5))

    # Ground Truth vs Estimated Distance
    ax1.plot(gt, gt, 'k--', label='Ideal 1:1 Reference', linewidth=1.5)
    ax1.plot(gt, est, 'o-', color='#2980b9', markersize=8, linewidth=2.0, label='LiDAR-Camera Fusion')
    ax1.set_xlabel('Ground Truth Distance (m)')
    ax1.set_ylabel('Estimated Distance (m)')
    ax1.set_title('Ground Truth vs Estimated Distance')
    ax1.legend(loc='upper left')

    # Absolute Error
    ax2.bar([f"{g:.1f}m" for g in gt], err * 1000.0, color='#e74c3c', edgecolor='black', width=0.45)
    ax2.set_xlabel('Target Distance')
    ax2.set_ylabel('Absolute Error (mm)')
    ax2.set_title('Fusion Distance Estimation Error (mm)')
    ax2.set_ylim(0, max(err * 1000.0) * 1.35)
    for i, e in enumerate(err):
        ax2.annotate(f'{e*1000.0:.1f} mm', xy=(i, e * 1000.0),
                     xytext=(0, 3), textcoords="offset points", ha='center', va='bottom', fontweight='bold')

    plt.suptitle('LiDAR-Camera Fusion Distance Measurement Calibration', y=1.02)
    plt.tight_layout()
    plt.savefig(out_path)
    plt.close()
    print(f"[SAVED] {out_path}")

def main():
    print("=" * 70)
    print("GENERATING PRESENTATION PLOTS FROM BENCHMARK CSV DATA")
    print("=" * 70)

    set_plot_style()
    plots_dir = "/home/soham-darade/CV_Autonomous_Navigation/results/plots"
    os.makedirs(plots_dir, exist_ok=True)

    model_csv = "/home/soham-darade/CV_Autonomous_Navigation/results/model_comparison.csv"
    nav_csv = "/home/soham-darade/CV_Autonomous_Navigation/results/navigation_comparison.csv"
    fusion_csv = "/home/soham-darade/CV_Autonomous_Navigation/results/lidar_camera_fusion_results.csv"

    plot_detection_metrics(model_csv, os.path.join(plots_dir, "detection_metrics.png"))
    plot_inference_speed(model_csv, os.path.join(plots_dir, "inference_speed.png"))
    plot_ablation_navigation(nav_csv, os.path.join(plots_dir, "ablation_navigation.png"))
    plot_clearance_and_path(nav_csv, os.path.join(plots_dir, "clearance_and_path.png"))
    plot_fusion_accuracy(fusion_csv, os.path.join(plots_dir, "fusion_accuracy.png"))

    print("=" * 70)
    print("All plots generated successfully in: " + plots_dir)
    print("=" * 70)

if __name__ == '__main__':
    main()
