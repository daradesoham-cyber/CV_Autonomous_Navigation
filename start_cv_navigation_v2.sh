#!/usr/bin/env bash
# ==============================================================================
# CV Autonomous Navigation V2.1 — Master System Launcher
# Starts Gazebo Sim, Nav2, Perception, Decision Engine, and Web Dashboard.
# Automatically opens http://127.0.0.1:5050 when online.
# ==============================================================================

set -e

PROJECT_ROOT="/home/soham-darade/CV_Autonomous_Navigation"
ROS_SETUP="/opt/ros/lyrical/setup.bash"
WS_SETUP="$PROJECT_ROOT/ros2_ws/install/setup.bash"
DASHBOARD_URL="http://127.0.0.1:5050"

echo "================================================================="
echo "   STARTING CV AUTONOMOUS NAVIGATION V2.1 (END-TO-END SYSTEM)    "
echo "================================================================="

if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "[ERROR] ROS 2 setup not found at $ROS_SETUP"
    exit 1
fi

if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    echo "[ERROR] Workspace setup not found at $WS_SETUP"
    exit 1
fi

# Ensure previous runs are cleanly stopped
echo "[1/4] Checking existing navigation processes..."
pkill -f "autonomous_robot" || true
pkill -f "dashboard_backend" || true
sleep 1

# Launch full system
echo "[2/4] Launching Gazebo, Nav2, Perception, and Decision Engine..."
ros2 launch autonomous_robot_bringup full_system.launch.py &
LAUNCH_PID=$!

echo "[3/4] Waiting for Web Dashboard (port 5050) to initialize..."
for i in {1..40}; do
    if curl -s -f "$DASHBOARD_URL/api/telemetry" > /dev/null 2>&1; then
        echo "[OK] Dashboard online at $DASHBOARD_URL"
        break
    fi
    sleep 1
done

# Launch browser
echo "[4/4] Opening Web Navigation Dashboard..."
if command -v xdg-open > /dev/null 2>&1; then
    xdg-open "$DASHBOARD_URL" > /dev/null 2>&1 &
elif command -v google-chrome > /dev/null 2>&1; then
    google-chrome "$DASHBOARD_URL" > /dev/null 2>&1 &
fi

echo "================================================================="
echo "  SYSTEM FULLY ONLINE & READY"
echo "  Dashboard: $DASHBOARD_URL"
echo "  RViz2 and Gazebo running."
echo "  Press Ctrl+C to shut down all processes."
echo "================================================================="

# Trap Ctrl+C to cleanly stop all background processes
cleanup() {
    echo ""
    echo "[INFO] Stopping navigation system..."
    kill $LAUNCH_PID 2>/dev/null || true
    "$PROJECT_ROOT/stop_cv_navigation_v2.sh"
    exit 0
}
trap cleanup SIGINT SIGTERM

wait $LAUNCH_PID
