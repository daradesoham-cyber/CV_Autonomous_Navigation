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
