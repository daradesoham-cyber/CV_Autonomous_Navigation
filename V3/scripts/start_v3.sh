#!/usr/bin/env bash
# =============================================================================
# V3 Hospital Logistics - start the complete system
#   Gazebo world + robot, localization, Nav2, decision engine, V3 perception
#   (YOLO + camera-LiDAR fusion + costmap obstacles), mission manager, web UI.
#
# Usage: start_v3.sh [--headless] [--traffic] [--no-browser] [--port N]
# Env:   V3_PORT (default 8080), V3_HEADLESS=1, V3_NO_BROWSER=1, V3_READY_TIMEOUT (s, default 240)
# All paths are derived from this script's location (independent of the current directory).
# =============================================================================
set -o pipefail
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
V3_ROOT="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$V3_ROOT")"
WS="$PROJECT_ROOT/ros2_ws"
RUN_DIR="$V3_ROOT/run"
PORT="${V3_PORT:-8080}"
HEADLESS="${V3_HEADLESS:-0}"
TRAFFIC=false
NO_BROWSER="${V3_NO_BROWSER:-0}"
READY_TIMEOUT="${V3_READY_TIMEOUT:-240}"
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS=1 ;;
    --traffic) TRAFFIC=true ;;
    --no-browser) NO_BROWSER=1 ;;
    --port) PORT="$2"; shift ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
  shift
done

say()  { echo -e "[V3] $*"; }
fail() { echo -e "[V3][ERROR] $*" >&2; }

# ---------------------------------------------------------------- environment
if [ ! -f /opt/ros/lyrical/setup.bash ]; then fail "ROS 2 Lyrical not found at /opt/ros/lyrical"; exit 1; fi
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then fail "workspace not built: $WS/install/setup.bash missing (run: cd $WS && colcon build)"; exit 1; fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export V3_ROOT
export V3_VENV_SITE="$("$PROJECT_ROOT/.venv/bin/python" -c 'import site;print(site.getsitepackages()[0])' 2>/dev/null)"

# ---------------------------------------------------------------- dependency checks
missing=0
for pkg in hospital_logistics autonomous_robot_navigation autonomous_robot_perception autonomous_robot_gazebo \
           autonomous_robot_description autonomous_robot_interfaces ros_gz_sim ros_gz_bridge nav2_amcl nav2_controller; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || { fail "ROS package missing: $pkg"; missing=1; }
done
for f in "$V3_ROOT/worlds/v3_hospital_world.sdf" "$V3_ROOT/maps/v3_hospital_map.yaml" "$V3_ROOT/config/v3_hospital_semantic_map.yaml" \
         "$V3_ROOT/models_trained/v3_yolov8n_from_v25.pt" "$V3_ROOT/config/logistics_locations.yaml" "$V3_ROOT/ui/index.html"; do
  [ -f "$f" ] || { fail "required file missing: $f"; missing=1; }
done
[ -n "$V3_VENV_SITE" ] && [ -d "$V3_VENV_SITE" ] || { fail "python venv not found at $PROJECT_ROOT/.venv (needed for YOLO/Flask)"; missing=1; }
command -v gz >/dev/null || { fail "gz (Gazebo Sim) not found"; missing=1; }
[ "$missing" = 0 ] || exit 1

# ---------------------------------------------------------------- single instance
mkdir -p "$RUN_DIR"
if [ -f "$RUN_DIR/v3.pgid" ]; then
  OLD=$(cat "$RUN_DIR/v3.pgid")
  if [ -n "$OLD" ] && kill -0 -- "-$OLD" 2>/dev/null; then
    say "V3 is already running (process group $OLD). Use stop_v3.sh or restart_v3.sh."
    exit 3
  fi
  rm -f "$RUN_DIR/v3.pgid"
fi
if pgrep -f "v3_hospital_world[.]sdf|hospital_logistics_world[.]sdf|realistic_facility_world[.]sdf" >/dev/null; then
  fail "a Gazebo facility world is already running outside this V3 instance (V2.6 or a test). Stop it first."
  exit 3
fi
if ss -ltn "sport = :$PORT" 2>/dev/null | grep -q LISTEN; then
  fail "port $PORT is already in use. Choose another with --port N."
  exit 3
fi

# ---------------------------------------------------------------- runtime data
mkdir -p "$V3_ROOT/data"
if [ ! -f "$V3_ROOT/data/v3_hospital_navigation_memory.db" ]; then
  # NEW hospital topology (v3_hospital_layout.py), empty learned history
  "$PROJECT_ROOT/.venv/bin/python" "$SCRIPT_DIR/build_v3_hospital_topology.py" || { fail "could not build the hospital topology DB"; exit 1; }
fi

# ---------------------------------------------------------------- launch
STAMP=$(date +%Y%m%d-%H%M%S)
LOG_DIR="$V3_ROOT/logs/$STAMP"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$V3_ROOT/logs/latest"
GUI=false; [ "$HEADLESS" = 1 ] && GUI=true
say "starting V3 (headless=$GUI, traffic=$TRAFFIC, port=$PORT); logs: $LOG_DIR"
T0=$(date +%s)
setsid ros2 launch hospital_logistics v3_full_system.launch.py headless:=$GUI traffic:=$TRAFFIC web_port:=$PORT \
  > "$LOG_DIR/launch.log" 2>&1 < /dev/null &
LPID=$!
PGID=$(ps -o pgid= -p "$LPID" | tr -d ' ')
echo "$PGID" > "$RUN_DIR/v3.pgid"
echo "$PORT" > "$RUN_DIR/v3.port"
say "launch process group: $PGID"

cleanup_failed() {
  fail "$1"
  fail "last lines of $LOG_DIR/launch.log:"
  tail -n 25 "$LOG_DIR/launch.log" >&2
  "$SCRIPT_DIR/stop_v3.sh" --quiet
  exit 1
}

# ---------------------------------------------------------------- wait for readiness
say "waiting for the system to become READY (timeout ${READY_TIMEOUT}s)..."
STATE=""
for _ in $(seq 1 "$READY_TIMEOUT"); do
  kill -0 "$LPID" 2>/dev/null || cleanup_failed "the launch process exited during startup"
  if grep -qE "process has died|\[ERROR\].*(Failed|failed to)" "$LOG_DIR/launch.log"; then
    grep -E "process has died" "$LOG_DIR/launch.log" | head -3 >&2
  fi
  STATE=$(curl -s --max-time 1 "http://127.0.0.1:$PORT/api/state" | python3 -c 'import sys,json;print(json.load(sys.stdin)["mission"].get("state",""))' 2>/dev/null)
  [ "$STATE" = "READY" ] && break
  sleep 1
done
[ "$STATE" = "READY" ] || cleanup_failed "system did not reach READY within ${READY_TIMEOUT}s (last mission state: '${STATE:-no response}')"
T1=$(date +%s)
echo "$((T1 - T0))" > "$LOG_DIR/startup_seconds"

LAN_IP=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
say "V3 is READY after $((T1 - T0)) s"
echo ""
echo "  V3 Hospital Logistics UI"
echo "  Local: http://localhost:$PORT"
if [ -n "$LAN_IP" ]; then
  echo "  LAN:   http://$LAN_IP:$PORT"
else
  echo "  LAN:   (no LAN interface with a default route found)"
fi
echo "  Stop:  $SCRIPT_DIR/stop_v3.sh"
echo ""
if [ "$NO_BROWSER" != 1 ] && command -v xdg-open >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  xdg-open "http://localhost:$PORT" >/dev/null 2>&1 &
fi
exit 0
