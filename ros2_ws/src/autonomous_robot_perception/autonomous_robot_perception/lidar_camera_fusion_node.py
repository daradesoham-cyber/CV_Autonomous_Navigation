#!/usr/bin/env python3
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from autonomous_robot_interfaces.msg import (
    Detection2DArray,
    ObstacleWarning,
    SemanticObstacle,
    SemanticObstacleArray
)

# Configurable semantic safety parameters
# V2.4 semantic safety parameters — includes all 10 V2.4 classes
SEMANTIC_SAFETY_CONFIG = {
    # V2.4 classes
    'person': {'radius': 0.90, 'is_dynamic': True},
    'cart': {'radius': 0.75, 'is_dynamic': True},
    'forklift': {'radius': 1.10, 'is_dynamic': True},
    'pallet': {'radius': 0.60, 'is_dynamic': False},
    'box': {'radius': 0.45, 'is_dynamic': False},
    'obstacle': {'radius': 0.50, 'is_dynamic': False},
    'door': {'radius': 0.70, 'is_dynamic': False},
    'charging_station': {'radius': 0.65, 'is_dynamic': False},
    'hospital_bed': {'radius': 0.85, 'is_dynamic': False},
    'directional_sign': {'radius': 0.05, 'is_dynamic': False},  # sign: not a physical obstacle
    # V2.3 legacy class names (kept for backward compatibility)
    'chair': {'radius': 0.55, 'is_dynamic': False},
    'dining table': {'radius': 0.65, 'is_dynamic': False},
    'table': {'radius': 0.65, 'is_dynamic': False},
    'suitcase': {'radius': 0.45, 'is_dynamic': False},
    'traffic cone': {'radius': 0.50, 'is_dynamic': False},
    'cone': {'radius': 0.50, 'is_dynamic': False},
    'shelf': {'radius': 0.60, 'is_dynamic': False},
    'bed': {'radius': 0.85, 'is_dynamic': False},
}
DEFAULT_CONFIG = {'radius': 0.50, 'is_dynamic': False}

class LidarCameraFusionNode(Node):
    """
    Synchronized LiDAR-Camera Fusion Node.

    Geometry & Assumptions (V2.4 Updated):
    - Robot Base: base_link
    - LiDAR: laser_link mounted at xyz=(0.10, 0, 0.175) relative to base_link (yaw=0)
    - Camera: camera_link mounted at xyz=(0.22, 0, 0.35) relative to base_link, rpy=(0, -0.05, 0)
    - Camera Optical: camera_optical_link (Z forward, X right, Y down)
    - V2.4 FOV: horizontal_fov = 1.15 rad (~65.9 deg), updated from V2.3 1.089 rad
    - Camera height: 0.35m above base_link (base_link is 0.06m above floor → camera ~0.41m AGL)
    - Sign eye-level: 1.10m AGL → signs are ~0.69m above camera optical center
    - Relative Baseline: Camera is located dx = 0.12m ahead of LiDAR on the robot centerline (dy = 0.0m).
    - Parallax: For an object at range R and bearing theta_cam from the camera,
      its coordinates relative to the LiDAR are:
        x_L = dx + R * cos(theta_cam)
        y_L = dy + R * sin(theta_cam)
        theta_L = atan2(y_L, x_L)
      At close range (0.5m - 2.0m), this 0.12m baseline induces up to ~6 deg angular shift.
      This node accounts for this translation across the expected detection depth interval.
    """
    def __init__(self):
        super().__init__('lidar_camera_fusion_node')

        self.declare_parameter('camera_hfov_rad', 1.15)  # V2.4: 65.9 deg (was 1.089 in V2.3)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('critical_distance_m', 2.5)
        self.declare_parameter('max_sync_age_ms', 100.0)  # Max scan-camera time disparity in ms
        self.declare_parameter('camera_dx_from_lidar', 0.12)  # Longitudinal baseline (m)
        self.declare_parameter('camera_dy_from_lidar', 0.00)  # Lateral baseline (m)
        self.declare_parameter('min_valid_cluster_points', 3)  # Rejects isolated noise/spurs
        self.declare_parameter('cluster_distance_tolerance_m', 0.35)  # Max gap within obstacle surface

        self.hfov = self.get_parameter('camera_hfov_rad').get_parameter_value().double_value
        self.img_w = self.get_parameter('image_width').get_parameter_value().double_value
        self.crit_dist = self.get_parameter('critical_distance_m').get_parameter_value().double_value
        self.max_sync_age_ms = self.get_parameter('max_sync_age_ms').get_parameter_value().double_value
        self.cam_dx = self.get_parameter('camera_dx_from_lidar').get_parameter_value().double_value
        self.cam_dy = self.get_parameter('camera_dy_from_lidar').get_parameter_value().double_value
        self.min_cluster_pts = self.get_parameter('min_valid_cluster_points').get_parameter_value().integer_value
        self.cluster_tol = self.get_parameter('cluster_distance_tolerance_m').get_parameter_value().double_value

        # Short history buffer of LaserScan messages (stores ~2s at 20 Hz)
        self.scan_buffer = deque(maxlen=40)

        # Cross-frame object tracker (V2.2 Section 9 & 10)
        self.tracks = {}
        self.next_track_id = 1

        # Subscribers
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_detections = self.create_subscription(Detection2DArray, '/vision/detections', self.detections_callback, 10)

        # Publishers
        self.pub_objects = self.create_publisher(Detection2DArray, '/vision/objects', 10)
        self.pub_obstacles = self.create_publisher(ObstacleWarning, '/vision/obstacles', 10)
        self.pub_semantic = self.create_publisher(SemanticObstacleArray, '/vision/semantic_obstacles', 10)
        self.pub_fused_objects = self.create_publisher(SemanticObstacleArray, '/fused_objects', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/vision/fused_object_markers', 10)
        self.pub_ttc = self.create_publisher(Float32, '/vision/ttc', 10)

        self.get_logger().info(
            f'LidarCameraFusionNode active: max_sync_age={self.max_sync_age_ms:.1f}ms, '
            f'baseline dx={self.cam_dx:.2f}m, dy={self.cam_dy:.2f}m, '
            f'clustering tol={self.cluster_tol:.2f}m (min_pts={self.min_cluster_pts})'
        )

    def scan_callback(self, msg: LaserScan):
        self.scan_buffer.append(msg)

    def get_synchronized_scan(self, det_stamp):
        """Find the LaserScan in history whose timestamp is closest to det_stamp."""
        if not self.scan_buffer:
            return None

        det_sec = det_stamp.sec + det_stamp.nanosec * 1e-9
        best_scan = None
        min_diff = float('inf')

        for scan in self.scan_buffer:
            scan_sec = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            diff = abs(scan_sec - det_sec)
            if diff < min_diff:
                min_diff = diff
                best_scan = scan

        max_allowable_sec = self.max_sync_age_ms / 1000.0
        if min_diff > max_allowable_sec:
            self.get_logger().warning(
                f'Timestamp synchronization: closest scan time delta is {min_diff*1000.0:.1f}ms '
                f'(exceeds max_sync_age_ms={self.max_sync_age_ms:.1f}ms). Rejecting stale scan association.',
                throttle_duration_sec=2.0
            )
            return None

        return best_scan

    def camera_ray_to_lidar_angle(self, theta_cam, depth):
        """Convert a camera optical ray bearing at given depth to LiDAR coordinate frame angle."""
        x_l = self.cam_dx + depth * math.cos(theta_cam)
        y_l = self.cam_dy + depth * math.sin(theta_cam)
        return math.atan2(y_l, x_l)

    def extract_robust_cluster_distance(self, valid_ranges):
        """
        Cluster candidate ranges and extract a robust near-surface percentile
        from the closest consistent cluster, rejecting isolated outliers and background bleed.
        """
        if len(valid_ranges) < self.min_cluster_pts:
            return -1.0

        sorted_r = np.sort(valid_ranges)

        # Segment into clusters using spatial jump threshold
        clusters = []
        cur_cluster = [sorted_r[0]]

        for r in sorted_r[1:]:
            if (r - cur_cluster[-1]) <= self.cluster_tol:
                cur_cluster.append(r)
            else:
                if len(cur_cluster) >= self.min_cluster_pts:
                    clusters.append(cur_cluster)
                cur_cluster = [r]
        if len(cur_cluster) >= self.min_cluster_pts:
            clusters.append(cur_cluster)

        if not clusters:
            return -1.0

        # Select closest consistent cluster (the primary obstacle face)
        closest_cluster = clusters[0]

        # Use 20th percentile within the closest cluster for robust front-surface distance
        surface_dist = float(np.percentile(closest_cluster, 20))
        return surface_dist

    def detections_callback(self, msg: Detection2DArray):
        scan = self.get_synchronized_scan(msg.header.stamp)
        if scan is None:
            return

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = np.array(scan.ranges)

        updated_detections = Detection2DArray()
        updated_detections.header = msg.header

        semantic_array = SemanticObstacleArray()
        semantic_array.header = msg.header
        closing_ttcs = []

        marker_array = MarkerArray()
        marker_id = 0

        for det in msg.detections:
            x_center = (det.x_min + det.x_max) / 2.0
            box_width = det.x_max - det.x_min

            # Angular projection in camera frame (0 is straight ahead, positive left)
            bearing_cam = (0.5 - (x_center / self.img_w)) * self.hfov
            angular_span_cam = (box_width / self.img_w) * self.hfov

            min_angle_cam = bearing_cam - (angular_span_cam / 2.0)
            max_angle_cam = bearing_cam + (angular_span_cam / 2.0)

            # Account for camera-LiDAR baseline across depth range [0.3m, 6.0m]
            # to compute the accurate LiDAR angular envelope
            depth_samples = [0.4, 0.8, 1.5, 3.0, 5.0]
            lidar_angles = []
            for d in depth_samples:
                lidar_angles.append(self.camera_ray_to_lidar_angle(min_angle_cam, d))
                lidar_angles.append(self.camera_ray_to_lidar_angle(max_angle_cam, d))
                lidar_angles.append(self.camera_ray_to_lidar_angle(bearing_cam, d))

            min_lidar_angle = min(lidar_angles) - 0.03  # Add small 0.03 rad tolerance
            max_lidar_angle = max(lidar_angles) + 0.03

            idx_start = int((min_lidar_angle - angle_min) / angle_inc)
            idx_end = int((max_lidar_angle - angle_min) / angle_inc)

            if idx_start > idx_end:
                idx_start, idx_end = idx_end, idx_start

            idx_start = max(0, min(idx_start, len(ranges) - 1))
            idx_end = max(0, min(idx_end, len(ranges) - 1))

            slice_ranges = ranges[idx_start:idx_end + 1]
            valid_mask = (
                (slice_ranges >= scan.range_min) &
                (slice_ranges <= scan.range_max) &
                np.isfinite(slice_ranges)
            )
            valid_ranges = slice_ranges[valid_mask]

            # Extract robust cluster distance
            distance = self.extract_robust_cluster_distance(valid_ranges)

            # Convert distance back to base_link / robot frame bearing
            if distance > 0:
                # Correct bearing for the estimated distance
                effective_bearing = float(self.camera_ray_to_lidar_angle(bearing_cam, distance))
            else:
                effective_bearing = float(bearing_cam)

            det.distance = distance
            det.bearing = effective_bearing
            updated_detections.detections.append(det)

            # Retrieve semantic classification parameters
            cls_name = det.class_name.lower()
            cfg = SEMANTIC_SAFETY_CONFIG.get(cls_name, DEFAULT_CONFIG)
            safety_radius = cfg['radius']
            is_dynamic = cfg['is_dynamic']

            if distance > 0.1 and distance <= self.crit_dist:
                raw_x = float(0.10 + distance * math.cos(effective_bearing))
                raw_y = float(0.00 + distance * math.sin(effective_bearing))

                now_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

                # Match with existing track
                matched_id = None
                min_match_dist = 0.65
                for tid, tr in self.tracks.items():
                    if tr['class_name'] == det.class_name:
                        track_d = math.hypot(raw_x - tr['x'], raw_y - tr['y'])
                        if track_d < min_match_dist:
                            min_match_dist = track_d
                            matched_id = tid

                if matched_id is not None:
                    # Update matched track with temporal smoothing & velocity estimation
                    tr = self.tracks[matched_id]
                    dt = max(0.02, now_ts - tr['last_time'])
                    vx = (raw_x - tr['x']) / dt
                    vy = (raw_y - tr['y']) / dt
                    speed = math.hypot(vx, vy)
                    dr = (distance - tr['distance']) / dt

                    # Temporal smoothing (exponential moving average)
                    smooth_x = 0.65 * tr['x'] + 0.35 * raw_x
                    smooth_y = 0.65 * tr['y'] + 0.35 * raw_y
                    smooth_dist = 0.65 * tr['distance'] + 0.35 * distance

                    # Classify motion state (Section 9)
                    if speed < 0.08:
                        motion_state = "STATIC"
                    else:
                        if dr > 0.05:
                            motion_state = "MOVING_AWAY"
                        elif dr < -0.05:
                            motion_state = "MOVING_TOWARDS"
                        elif abs(vy) > 0.12:
                            motion_state = "CROSSING"
                        else:
                            motion_state = "MOVING"

                    if abs(smooth_y) > 0.85:
                        motion_state = "OUTSIDE_PATH"

                    tr['x'] = smooth_x
                    tr['y'] = smooth_y
                    tr['distance'] = smooth_dist
                    tr['vx'] = vx
                    tr['vy'] = vy
                    tr['speed'] = speed
                    tr['last_time'] = now_ts
                    tr['motion_state'] = motion_state
                    tr['frames'] += 1

                    pos_x = smooth_x
                    pos_y = smooth_y
                    final_dist = smooth_dist
                    track_id = matched_id
                else:
                    # Create new track
                    track_id = self.next_track_id
                    self.next_track_id += 1
                    motion_state = "MOVING_TOWARDS" if is_dynamic else "STATIC"
                    if abs(raw_y) > 0.85:
                        motion_state = "OUTSIDE_PATH"

                    self.tracks[track_id] = {
                        'id': track_id,
                        'class_name': det.class_name,
                        'x': raw_x,
                        'y': raw_y,
                        'distance': distance,
                        'vx': 0.0,
                        'vy': 0.0,
                        'speed': 0.0,
                        'last_time': now_ts,
                        'motion_state': motion_state,
                        'frames': 1
                    }
                    pos_x = raw_x
                    pos_y = raw_y
                    final_dist = distance

                # Determine human-readable direction
                deg = math.degrees(effective_bearing)
                if deg > 45.0:
                    dir_base = "Left"
                elif deg > 10.0:
                    dir_base = "Front-Left"
                elif deg < -45.0:
                    dir_base = "Right"
                elif deg < -10.0:
                    dir_base = "Front-Right"
                else:
                    dir_base = "Front-Center"

                # Time-To-Collision (TTC) calculation for dynamic obstacles
                obs_ttc = -1.0
                closing_speed = 0.0
                if matched_id is not None:
                    if dr < -0.05:
                        closing_speed = -dr
                elif motion_state in ["MOVING_TOWARDS", "CROSSING"] and is_dynamic:
                    closing_speed = 0.35  # Initial closing speed assumption for dynamic object

                if closing_speed > 0.05 and pos_x > 0.05 and abs(pos_y) < 0.85:
                    obs_ttc = float(final_dist / closing_speed)
                    closing_ttcs.append(obs_ttc)

                # Formatted direction string with motion state, track ID, and TTC
                if obs_ttc > 0.0:
                    direction_str = f"{dir_base} | {motion_state} | ID:{track_id} | TTC:{obs_ttc:.1f}s"
                else:
                    direction_str = f"{dir_base} | {motion_state} | ID:{track_id}"
                pos_z = 0.25

                # Semantic Obstacle Message
                sem_obs = SemanticObstacle()
                sem_obs.header = msg.header
                sem_obs.header.frame_id = 'base_link'
                sem_obs.class_name = det.class_name
                sem_obs.confidence = float(det.confidence)
                sem_obs.distance = float(final_dist)
                sem_obs.bearing = float(effective_bearing)
                sem_obs.safety_radius = float(safety_radius)
                sem_obs.is_dynamic = bool(is_dynamic or motion_state in ["MOVING", "MOVING_TOWARDS", "CROSSING"])
                sem_obs.direction = direction_str
                sem_obs.x = pos_x
                sem_obs.y = pos_y
                sem_obs.z = pos_z
                semantic_array.obstacles.append(sem_obs)

                # Obstacle Warning Message
                warn = ObstacleWarning()
                warn.header = msg.header
                warn.obstacle_type = det.class_name
                warn.distance = float(final_dist)
                warn.bearing = float(effective_bearing)
                warn.requires_clearance = bool(sem_obs.is_dynamic)
                self.pub_obstacles.publish(warn)

                # Marker 1: 3D bounding box / shape
                shape_marker = Marker()
                shape_marker.header.frame_id = 'base_link'
                shape_marker.header.stamp = msg.header.stamp
                shape_marker.ns = 'fused_shapes'
                shape_marker.id = marker_id
                marker_id += 1
                shape_marker.type = Marker.CUBE
                shape_marker.action = Marker.ADD
                shape_marker.pose.position.x = pos_x
                shape_marker.pose.position.y = pos_y
                shape_marker.pose.position.z = pos_z
                shape_marker.pose.orientation.w = 1.0
                shape_marker.scale.x = max(0.3, safety_radius * 1.2)
                shape_marker.scale.y = max(0.3, safety_radius * 1.2)
                shape_marker.scale.z = 0.6
                if motion_state == "MOVING_AWAY":
                    shape_marker.color.r = 0.2
                    shape_marker.color.g = 0.8
                    shape_marker.color.b = 0.2
                elif is_dynamic or motion_state in ["MOVING_TOWARDS", "CROSSING"]:
                    shape_marker.color.r = 1.0
                    shape_marker.color.g = 0.2
                    shape_marker.color.b = 0.2
                else:
                    shape_marker.color.r = 1.0
                    shape_marker.color.g = 0.7
                    shape_marker.color.b = 0.0
                shape_marker.color.a = 0.75
                shape_marker.lifetime.sec = 1
                marker_array.markers.append(shape_marker)

                # Marker 2: Floating 3D text label
                text_marker = Marker()
                text_marker.header.frame_id = 'base_link'
                text_marker.header.stamp = msg.header.stamp
                text_marker.ns = 'fused_labels'
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = pos_x
                text_marker.pose.position.y = pos_y
                text_marker.pose.position.z = pos_z + 0.5
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.22
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 0.95
                text_marker.text = f"[{track_id}] {det.class_name.upper()}\n{final_dist:.1f}m | {motion_state}\n({det.confidence*100:.0f}%)"
                text_marker.lifetime.sec = 1
                marker_array.markers.append(text_marker)

        # Prune stale tracks older than 1.2 seconds
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stale_ids = [tid for tid, tr in self.tracks.items() if (curr_time - tr['last_time']) > 1.2]
        for tid in stale_ids:
            del self.tracks[tid]

        # Publish Time-to-Collision (TTC) across closing forward path obstacles
        ttc_msg = Float32()
        ttc_msg.data = float(min(closing_ttcs)) if closing_ttcs else -1.0
        self.pub_ttc.publish(ttc_msg)

        self.pub_objects.publish(updated_detections)
        self.pub_semantic.publish(semantic_array)
        self.pub_fused_objects.publish(semantic_array)
        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
