#!/usr/bin/env bash
# ==============================================================================
# Script: start_project.sh
# Purpose: Clean startup launcher for ROS 2 Lyrical + Gazebo Autonomous CV Navigation
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "======================================================================"
echo "    ROS 2 Lyrical + Gazebo Autonomous CV Navigation Launcher"
echo "======================================================================"

# Clean up any stale simulation / ROS 2 processes to ensure exactly one stack runs
cleanup_stale_processes() {
    local stale_pids
    stale_pids=$(pgrep -f "gz sim|parameter_bridge|rviz2|nav2_map_server|nav2_amcl|nav2_planner|nav2_controller|navigation_controller_node|navigation_perception_node" || true)
    if [ -n "$stale_pids" ]; then
        echo "[INFO] Existing simulation/ROS processes detected. Cleaning up stale stack..."
        pkill -f "gz sim" > /dev/null 2>&1 || true
        pkill -f "parameter_bridge" > /dev/null 2>&1 || true
        pkill -f "rviz2" > /dev/null 2>&1 || true
        pkill -f "nav2_" > /dev/null 2>&1 || true
        pkill -f "map_server" > /dev/null 2>&1 || true
        pkill -f "amcl" > /dev/null 2>&1 || true
        pkill -f "autonomous_robot_" > /dev/null 2>&1 || true
        pkill -f "navigation_" > /dev/null 2>&1 || true

        for i in {1..5}; do
            if ! pgrep -f "gz sim|parameter_bridge|rviz2|nav2_map_server|nav2_amcl" > /dev/null 2>&1; then
                break
            fi
            sleep 1
        done

        pkill -9 -f "gz sim" > /dev/null 2>&1 || true
        pkill -9 -f "parameter_bridge" > /dev/null 2>&1 || true
        pkill -9 -f "rviz2" > /dev/null 2>&1 || true
        pkill -9 -f "nav2_" > /dev/null 2>&1 || true
        pkill -9 -f "autonomous_robot_" > /dev/null 2>&1 || true
        pkill -9 -f "navigation_" > /dev/null 2>&1 || true
        sleep 1
        echo "[OK] Cleaned up stale simulation stack."
    fi
}

cleanup_on_exit() {
    echo ""
    echo "[INFO] Terminating simulation stack..."
    cleanup_stale_processes
    echo "[OK] All simulation processes stopped."
}

# Trap termination signals to ensure no orphaned processes remain
trap cleanup_on_exit EXIT INT TERM

# Ensure single clean simulation instance before startup
cleanup_stale_processes

# 1. Check and Source ROS 2 Lyrical
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
    echo "[OK] Sourced ROS 2 Lyrical (/opt/ros/lyrical/setup.bash)"
else
    echo "[ERROR] ROS 2 Lyrical not found at /opt/ros/lyrical/setup.bash"
    exit 1
fi

# 2. Check ROS_DISTRO
if [ "$ROS_DISTRO" != "lyrical" ]; then
    echo "[WARN] Active ROS_DISTRO is '$ROS_DISTRO' (expected 'lyrical')"
else
    echo "[OK] ROS 2 Distribution: $ROS_DISTRO"
fi

# 3. Check Gazebo Sim
if command -v gz &> /dev/null; then
    GZ_VER=$(gz sim --version 2>&1 | head -n 1 || true)
    echo "[OK] Gazebo Sim available: $GZ_VER"
else
    echo "[ERROR] Gazebo (gz) command not found in PATH."
    exit 1
fi

# 4. Check & Source Python AI virtual environment
if [ -d "$PROJ_DIR/.venv" ]; then
    export PATH="$PROJ_DIR/.venv/bin:$PATH"
    echo "[OK] Sourced Python AI venv: $PROJ_DIR/.venv"
fi

# 5. Check Workspace Build Status
INSTALL_SETUP="$PROJ_DIR/ros2_ws/install/setup.bash"
REBUILD=false

for arg in "$@"; do
    if [ "$arg" == "--rebuild" ] || [ "$arg" == "-r" ]; then
        REBUILD=true
    fi
done

if [ "$REBUILD" = true ] || [ ! -f "$INSTALL_SETUP" ]; then
    echo "[INFO] Building ROS 2 workspace (ros2_ws)..."
    cd "$PROJ_DIR/ros2_ws"
    colcon build --symlink-install
    cd "$PROJ_DIR"
    echo "[OK] Colcon build completed."
else
    echo "[OK] Workspace is built: $INSTALL_SETUP"
fi

# Source workspace install
source "$INSTALL_SETUP"
echo "[OK] Sourced workspace setup: $INSTALL_SETUP"

# Parse CLI arguments
WORLD="complex_world"
SLAM="false"

while [[ $# -gt 0 ]]; do
    case $1 in
        --world|-w)
            WORLD="$2"
            shift 2
            ;;
        --slam|-s)
            SLAM="true"
            shift
            ;;
        --rebuild|-r)
            shift
            ;;
        --help|-h)
            echo "Usage: ./start_project.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --world, -w <name>   World name (default: complex_world)"
            echo "  --slam, -s           Run in SLAM mapping mode (default: false, AMCL localization)"
            echo "  --rebuild, -r        Force colcon build before launch"
            echo "  --help, -h           Show this help message"
            exit 0
            ;;
        *)
            echo "[WARN] Unknown argument: $1"
            shift
            ;;
    esac
done

echo "----------------------------------------------------------------------"
echo "Configuration:"
echo "  World:              $WORLD"
echo "  SLAM Mode:          $SLAM"
echo "  Perception:         true (YOLOv8n on cuda:0)"
echo "  Navigation:         true (Nav2 Stack with Costmap Ingestion)"
echo "  RViz:               true"
echo "----------------------------------------------------------------------"
echo "[LAUNCH] Starting Gazebo, Nav2, Perception, and RViz..."

ros2 launch autonomous_robot_bringup bringup.launch.py \
    world:="$WORLD" \
    slam:="$SLAM" \
    navigation:=true \
    perception:=true \
    rviz:=true \
    use_sim_time:=true
