#!/usr/bin/env bash
# V3 Hospital Logistics - health check. Prints PASS/FAIL per item; exit code 0 only if all pass.
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
V3_ROOT="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$V3_ROOT")"
WS="$PROJECT_ROOT/ros2_ws"
PORT="${V3_PORT:-$(cat "$V3_ROOT/run/v3.port" 2>/dev/null || echo 8080)}"
FAILS=0
res() { if [ "$1" = 0 ]; then printf "  %-34s PASS  %s\n" "$2" "$3"; else printf "  %-34s FAIL  %s\n" "$2" "$3"; FAILS=$((FAILS+1)); fi; }

echo "V3 health check ($(date '+%F %T'))"
[ -f /opt/ros/lyrical/setup.bash ]; r=$?; res $r "ROS 2 environment" "/opt/ros/lyrical"
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash 2>/dev/null
[ -f "$WS/install/setup.bash" ]; r=$?; res $r "V3 workspace" "$WS/install"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
ros2 pkg prefix hospital_logistics >/dev/null 2>&1; res $? "hospital_logistics package" ""

pgrep -f "v3_hospital_world[.]sdf" >/dev/null; res $? "Gazebo (V3 world)" ""

STATE_JSON=$(curl -s --max-time 3 "http://127.0.0.1:$PORT/api/state")
[ -n "$STATE_JSON" ]; res $? "Web UI (localhost:$PORT)" ""
LAN_IP=$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
if [ -n "$LAN_IP" ]; then
  code=$(curl -s -o /dev/null -w '%{http_code}' --max-time 3 "http://$LAN_IP:$PORT/api/state"); [ "$code" = 200 ]
  res $? "Web UI (LAN $LAN_IP:$PORT)" "HTTP $code"
else
  res 1 "Web UI (LAN)" "no LAN interface found"
fi

jget() { echo "$STATE_JSON" | python3 -c "import sys,json;d=json.load(sys.stdin);print($1)" 2>/dev/null; }
chk_rate() { v=$(jget "d['system']['topics']['$2']['hz']"); python3 -c "import sys;sys.exit(0 if float('${v:-0}')>=$3 else 1)" 2>/dev/null; res $? "$1" "${v:-?} Hz"; }
chk_rate "Gazebo clock" clock 5
chk_rate "Camera topic" camera 5
chk_rate "LiDAR topic" scan 5
chk_rate "YOLO detections" detections 1
chk_rate "Camera-LiDAR fusion" fusion 1
chk_rate "Costmap obstacle node" costmap 1
chk_rate "Decision engine status" decision_engine 0.5
chk_rate "Mission manager state" mission 0.5
MS=$(jget "d['mission'].get('state')"); [ -n "$MS" ] && [ "$MS" != "None" ]; res $? "Mission manager" "state=$MS"

# DDS discovery under load misses nodes in a single pass (measured: 10-15 of ~26 nodes per daemon-free
# pass); use the union of the ROS daemon view and three daemon-free passes. Liveness of each V3 node is
# verified independently by the topic-rate checks above (the daemon can keep stale entries).
NODES=$( (timeout 20 ros2 node list 2>/dev/null; for _ in 1 2 3; do timeout 20 ros2 node list --no-daemon 2>/dev/null; done) | sort -u)
for n in v3_detection_node v3_spatial_fusion_node v3_costmap_obstacle_node v3_mission_manager v3_web_server \
         decision_engine_node amcl map_server controller_server planner_server bt_navigator; do
  echo "$NODES" | grep -q "/$n$"; res $? "node /$n" ""
done
for n in controller_server planner_server bt_navigator amcl map_server; do
  s=$(timeout 10 ros2 lifecycle get "/$n" 2>/dev/null | awk '{print $1}'); [ "$s" = active ]; res $? "Nav2 lifecycle /$n" "${s:-unknown}"
done
timeout 10 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q Translation; res $? "TF map -> base_link" ""

python3 -c "import sqlite3;c=sqlite3.connect('$V3_ROOT/data/v3_hospital_missions.db');c.execute('select count(*) from missions').fetchone()" 2>/dev/null
res $? "Mission history DB" "$V3_ROOT/data/v3_hospital_missions.db"
python3 -c "import sqlite3;c=sqlite3.connect('$V3_ROOT/data/v3_hospital_navigation_memory.db');assert c.execute('select count(*) from nodes').fetchone()[0]>0" 2>/dev/null
res $? "Navigation memory DB" "$V3_ROOT/data/v3_hospital_navigation_memory.db"

echo ""
if [ "$FAILS" = 0 ]; then echo "HEALTH: PASS"; exit 0; else echo "HEALTH: FAIL ($FAILS item(s))"; exit 1; fi
