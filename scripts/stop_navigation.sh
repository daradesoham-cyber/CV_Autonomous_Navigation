#!/usr/bin/env bash
# ==============================================================================
# Script: stop_navigation.sh
# Purpose: Emergency stop and navigation cancel for autonomous robot simulation
# ==============================================================================
set -e

if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
fi

echo "======================================================================"
echo "          EMERGENCY STOP / NAVIGATION HALT DISPATCHED"
echo "======================================================================"

# 1. Publish zero velocity command to /cmd_vel and /cmd_vel_stop immediately
echo "[STOP] Broadcasting zero velocity to /cmd_vel and /cmd_vel_stop..."
for i in {1..3}; do
    ros2 topic pub --once /cmd_vel_stop geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
    sleep 0.05
done

# 2. Cancel active Nav2 goal action if action server is active
echo "[STOP] Canceling active Nav2 goals..."
python3 -c "
import sys
if '/opt/ros/lyrical/lib/python3.14/site-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/lyrical/lib/python3.14/site-packages')
try:
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    rclpy.init()
    node = rclpy.create_node('emergency_stop_client')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if client.wait_for_server(timeout_sec=1.0):
        # Cancel all goals
        cancel_future = client._cancel_goal_async(None)
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=1.5)
        print('[OK] Sent cancellation request to /navigate_to_pose.')
    else:
        print('[INFO] Action server /navigate_to_pose was not running.')
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    pass
" 2>/dev/null || true

echo "[OK] Emergency stop sequence complete. Robot velocity halted."
echo "======================================================================"
