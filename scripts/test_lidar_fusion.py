#!/usr/bin/env python3
import os
import math
import csv
import numpy as np

def run_fusion_validation():
    print("=" * 70)
    print("SYNTHETIC FUSION VALIDATION (Algorithmic Simulation Only - Not Real Gazebo Data)")
    print("=" * 70)

    # Simulated Camera Parameters
    img_w = 640
    img_h = 480
    hfov = 1.089  # radians (~62.4 degrees)

    # 2D LiDAR Parameters
    angle_min = -math.pi
    angle_max = math.pi
    num_samples = 720
    angle_inc = (angle_max - angle_min) / num_samples

    test_distances = [1.0, 1.5, 2.0, 2.5]
    results = []

    print(f"{'Object':<10} | {'GT Dist (m)':<12} | {'Est Dist (m)':<12} | {'Abs Err (m)':<12} | {'% Error':<10} | {'Status'}")
    print("-" * 70)

    for gt_dist in test_distances:
        # Ground truth person object positioned at gt_dist straight ahead (bearing ~ 0 rad)
        # Bounding box width scales inversely with distance: w_px ~ (real_w / dist) * focal_length
        # For a 0.5m wide person, focal_length ~ (img_w / 2) / tan(hfov / 2) ~ 546 px
        focal_length = (img_w / 2.0) / math.tan(hfov / 2.0)
        box_w = int((0.5 / gt_dist) * focal_length)
        box_h = int((1.7 / gt_dist) * focal_length)

        x_center = img_w / 2.0  # directly in front
        x_min = max(0, int(x_center - box_w / 2.0))
        x_max = min(img_w, int(x_center + box_w / 2.0))

        # Angular sector
        bearing = (0.5 - (x_center / img_w)) * hfov
        angular_span = ((x_max - x_min) / img_w) * hfov
        min_angle = bearing - (angular_span / 2.0)
        max_angle = bearing + (angular_span / 2.0)

        # Generate LiDAR scan with Gaussian noise (stddev = 1.5 cm)
        ranges = np.full(num_samples, 8.0, dtype=np.float32)  # background at 8m
        idx_start = int((min_angle - angle_min) / angle_inc)
        idx_end = int((max_angle - angle_min) / angle_inc)
        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start

        ranges[idx_start:idx_end+1] = np.random.normal(gt_dist, 0.015, size=idx_end-idx_start+1)

        # Fusion extraction
        slice_ranges = ranges[idx_start:idx_end+1]
        valid = slice_ranges[(slice_ranges >= 0.12) & (slice_ranges <= 12.0)]
        estimated_dist = float(np.percentile(valid, 20))

        abs_err = abs(estimated_dist - gt_dist)
        pct_err = (abs_err / gt_dist) * 100.0

        status = "PASS" if abs_err < 0.08 else "FAIL"
        print(f"{'PERSON':<10} | {gt_dist:<12.3f} | {estimated_dist:<12.3f} | {abs_err:<12.3f} | {pct_err:<9.2f}% | {status}")

        results.append({
            'object_class': 'person',
            'ground_truth_m': gt_dist,
            'estimated_distance_m': round(estimated_dist, 4),
            'absolute_error_m': round(abs_err, 4),
            'percentage_error': round(pct_err, 2),
            'status': status
        })

    # Save to results/lidar_camera_fusion_results.csv
    csv_path = '/home/soham-darade/CV_Autonomous_Navigation/results/lidar_camera_fusion_results.csv'
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    print("-" * 70)
    print(f"Results saved to: {csv_path}")
    print("=" * 70)

if __name__ == '__main__':
    run_fusion_validation()

