#!/usr/bin/env python3
import math
import numpy as np

def simulate_lidar_camera_association():
    print("=" * 60)
    print("TESTING CAMERA-LIDAR SENSOR FUSION ALGORITHM")
    print("=" * 60)

    # Simulated Camera Parameters
    img_w = 640
    img_h = 480
    hfov = 1.089  # radians (~62.4 degrees)

    # Simulated Bounding Box for a Person standing at x=0.5m, y=2.0m -> distance ~2.06m, bearing ~14 degrees
    # In image coordinates, person is slightly to the left of center
    x_min, y_min, x_max, y_max = 240, 100, 360, 420
    x_center = (x_min + x_max) / 2.0
    box_width = x_max - x_min

    # Calculate angular sector
    bearing = (0.5 - (x_center / img_w)) * hfov
    angular_span = (box_width / img_w) * hfov
    min_angle = bearing - (angular_span / 2.0)
    max_angle = bearing + (angular_span / 2.0)

    print(f"Detected Object: PERSON")
    print(f"Bounding Box: [{x_min}, {y_min}, {x_max}, {y_max}], Center: {x_center}px")
    print(f"Computed Bearing: {math.degrees(bearing):.2f}° ({bearing:.4f} rad)")
    print(f"Angular Sector: [{math.degrees(min_angle):.2f}°, {math.degrees(max_angle):.2f}°]")

    # Simulated 2D LiDAR Scan (720 samples from -pi to +pi)
    angle_min = -math.pi
    angle_max = math.pi
    num_samples = 720
    angle_inc = (angle_max - angle_min) / num_samples

    # Ground truth: an obstacle at 2.1 meters in that sector, background walls at 6.0 meters
    ranges = np.full(num_samples, 6.0, dtype=np.float32)
    idx_start = int((min_angle - angle_min) / angle_inc)
    idx_end = int((max_angle - angle_min) / angle_inc)
    if idx_start > idx_end:
        idx_start, idx_end = idx_end, idx_start

    # Add person object with noise
    ranges[idx_start:idx_end+1] = np.random.normal(2.10, 0.02, size=idx_end-idx_start+1)

    # Association logic
    slice_ranges = ranges[idx_start:idx_end+1]
    valid = slice_ranges[(slice_ranges >= 0.12) & (slice_ranges <= 12.0)]
    estimated_distance = float(np.percentile(valid, 20))

    print(f"Associated LiDAR Points: {len(valid)}")
    print(f"Estimated Physical Distance: {estimated_distance:.3f} m (Ground Truth: 2.10 m)")
    print(f"Estimation Error: {abs(estimated_distance - 2.10):.3f} m")
    assert abs(estimated_distance - 2.10) < 0.1, "Error exceeds tolerance!"
    print("\nSENSOR FUSION ASSOCIATION TEST PASSED.")
    print("=" * 60)

if __name__ == '__main__':
    simulate_lidar_camera_association()
