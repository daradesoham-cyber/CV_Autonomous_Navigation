#!/usr/bin/env bash
# ==============================================================================
# Script: run_experiment.sh
# Purpose: Execute repeatable autonomous navigation experiments with Nav2 & CV
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 1. Environment Verification & Sourcing
if [ -f "/opt/ros/lyrical/setup.bash" ]; then
    source /opt/ros/lyrical/setup.bash
else
    echo "[ERROR] ROS 2 Lyrical not found at /opt/ros/lyrical/setup.bash"
    exit 1
fi

if [ -f "$PROJ_DIR/ros2_ws/install/setup.bash" ]; then
    source "$PROJ_DIR/ros2_ws/install/setup.bash"
else
    echo "[ERROR] Workspace not built! Missing $PROJ_DIR/ros2_ws/install/setup.bash"
    echo "Run: cd $PROJ_DIR/ros2_ws && colcon build --symlink-install"
    exit 1
fi

# Ensure Python AI venv is in PATH
if [ -d "$PROJ_DIR/.venv" ]; then
    export PATH="$PROJ_DIR/.venv/bin:$PATH"
fi

START_LOC=""
GOAL_LOC=""
SCENARIO_NAME=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --start|-s)
            START_LOC="$2"
            shift 2
            ;;
        --goal|-g)
            GOAL_LOC="$2"
            shift 2
            ;;
        --scenario)
            SCENARIO_NAME="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: ./scripts/run_experiment.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --scenario <name>       Predefined scenario from config/experiments.yaml"
            echo "                          (e.g. scenario_1, scenario_2, scenario_3, cv_dynamic_obstacle_test)"
            echo "  --start, -s <location>  Start location (e.g. start_a, start_b, start_c)"
            echo "  --goal, -g <location>   Goal location (e.g. goal_a, goal_b, goal_c)"
            echo "  --help, -h              Show this help message"
            echo ""
            echo "Example:"
            echo "  ./scripts/run_experiment.sh --start start_a --goal goal_a"
            echo "  ./scripts/run_experiment.sh --scenario scenario_1"
            exit 0
            ;;
        *)
            echo "[WARN] Unknown argument: $1"
            shift
            ;;
    esac
done

# Resolve scenario if provided
if [ -n "$SCENARIO_NAME" ]; then
    SCENARIO_FILE="$PROJ_DIR/config/experiments.yaml"
    if [ ! -f "$SCENARIO_FILE" ]; then
        echo "[ERROR] Experiments config not found: $SCENARIO_FILE"
        exit 1
    fi

    PARSED_START=$(python3 -c "import yaml; data=yaml.safe_load(open('$SCENARIO_FILE')); print(data.get('scenarios',{}).get('$SCENARIO_NAME',{}).get('start',''))")
    PARSED_GOAL=$(python3 -c "import yaml; data=yaml.safe_load(open('$SCENARIO_FILE')); print(data.get('scenarios',{}).get('$SCENARIO_NAME',{}).get('goal',''))")

    if [ -z "$PARSED_START" ] || [ -z "$PARSED_GOAL" ]; then
        echo "[ERROR] Scenario '$SCENARIO_NAME' not found in $SCENARIO_FILE"
        exit 1
    fi
    START_LOC="$PARSED_START"
    GOAL_LOC="$PARSED_GOAL"
    echo "[SCENARIO] Selected scenario: $SCENARIO_NAME"
fi

if [ -z "$START_LOC" ] || [ -z "$GOAL_LOC" ]; then
    echo "[ERROR] Both --start and --goal (or --scenario) must be specified!"
    echo "Run with --help for usage."
    exit 1
fi

echo "======================================================================"
echo "          AUTONOMOUS NAVIGATION EXPERIMENT RUNNER"
echo "======================================================================"
echo "Start Location: $START_LOC"
echo "Goal Location:  $GOAL_LOC"
echo "Timestamp:      $(date '+%Y-%m-%d %H:%M:%S')"
echo "----------------------------------------------------------------------"

# 2. Set Initial Pose for AMCL
echo "[STEP 1/3] Publishing initial pose for AMCL ($START_LOC)..."
python3 "$SCRIPT_DIR/set_start.py" --location "$START_LOC"
sleep 1.5

# 3. Send Goal and Monitor Execution
echo "[STEP 2/3] Dispatching Nav2 goal ($GOAL_LOC)..."
START_TIME=$(date +%s)
STATUS="FAILED"

if python3 "$SCRIPT_DIR/send_goal.py" --location "$GOAL_LOC"; then
    STATUS="SUCCEEDED"
    echo "[OK] Navigation to $GOAL_LOC completed successfully!"
else
    STATUS="FAILED"
    echo "[WARN] Navigation to $GOAL_LOC was not completed successfully."
fi

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

# 4. Record Results
RESULTS_LOG="$PROJ_DIR/results/experiments_log.csv"
mkdir -p "$PROJ_DIR/results"

if [ ! -f "$RESULTS_LOG" ]; then
    echo "timestamp,scenario,start,goal,status,duration_s" > "$RESULTS_LOG"
fi

SCENARIO_ENTRY="${SCENARIO_NAME:-custom}"
echo "$(date '+%Y-%m-%dT%H:%M:%S'),$SCENARIO_ENTRY,$START_LOC,$GOAL_LOC,$STATUS,$DURATION" >> "$RESULTS_LOG"

echo "----------------------------------------------------------------------"
echo "[STEP 3/3] Experiment Summary:"
echo "  Scenario:     $SCENARIO_ENTRY"
echo "  Start:        $START_LOC"
echo "  Goal:         $GOAL_LOC"
echo "  Outcome:      $STATUS"
echo "  Duration:     ${DURATION}s"
echo "  Log Saved To: $RESULTS_LOG"
echo "======================================================================"
echo "[INFO] Simulation and RViz remain active for inspection."
