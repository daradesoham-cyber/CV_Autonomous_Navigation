#!/usr/bin/env bash
# Reset Navigation Memory database for autonomous robot
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DB_PATH="$PROJECT_ROOT/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db"

LAYOUT="${1:-realistic}"

echo "=================================================="
echo "Resetting Navigation Memory Database (Layout: $LAYOUT)"
echo "Target DB: $DB_PATH"
echo "=================================================="

if [ -f "$DB_PATH" ]; then
    rm -f "$DB_PATH"
    echo "[OK] Removed existing database: $DB_PATH"
else
    echo "[INFO] No database found at $DB_PATH to delete."
fi

# Re-initialize with specified topology
python3 -c "
import sys
sys.path.insert(0, '$PROJECT_ROOT/ros2_ws/src/autonomous_robot_navigation')
from autonomous_robot_navigation.navigation_memory import NavigationMemory
mem = NavigationMemory('$DB_PATH', layout='$LAYOUT')
print('[OK] Initialized fresh navigation memory with', len(mem.load_graph().nodes), 'nodes and', len(mem.load_graph().edges), 'edges.')
"

echo "=================================================="
echo "Navigation memory reset complete."
echo "=================================================="
