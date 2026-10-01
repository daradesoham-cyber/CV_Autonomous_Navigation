#!/usr/bin/env bash
# Nav2 route trials (spawn -> collection_point -> laboratory_test_point) with a live perception chain.
# Requires: V3 world, V2.6 localization (V3 map) + navigation, navigation_perception_node running.
# usage: nav_perception_trials.sh <v3|v26> <trials> <outdir>
set -o pipefail
CHAIN=$1; N=${2:-3}; OUT=$3
V3="$(cd "$(dirname "$0")/.." && pwd)"; PROJ="$(dirname "$V3")"
source /opt/ros/lyrical/setup.bash; source "$PROJ/ros2_ws/install/setup.bash"
mkdir -p "$OUT"
kill_pat() { for p in $(ps -eo pid,args | awk -v pat="$1" '$0 ~ pat && !/awk/ && !/nav_perception_trials/ {print $1}'); do kill "$p" 2>/dev/null; done; }
# stop both chains' detection/fusion nodes, then start the requested chain
kill_pat "lib/hospital_logistics/v3_detection_node"; kill_pat "lib/hospital_logistics/v3_spatial_fusion_node"
kill_pat "lib/autonomous_robot_perception/object_detection_node"; kill_pat "lib/autonomous_robot_perception/lidar_camera_fusion_node"
sleep 3
if [ "$CHAIN" = "v3" ]; then
  setsid ros2 run hospital_logistics v3_detection_node --ros-args -p use_sim_time:=true > "$OUT/det.log" 2>&1 &
  setsid ros2 run hospital_logistics v3_spatial_fusion_node --ros-args -p use_sim_time:=true > "$OUT/fus.log" 2>&1 &
else
  setsid ros2 run autonomous_robot_perception object_detection_node --ros-args -p use_sim_time:=true > "$OUT/det.log" 2>&1 &
  setsid ros2 run autonomous_robot_perception lidar_camera_fusion_node --ros-args -p use_sim_time:=true > "$OUT/fus.log" 2>&1 &
fi
sleep 25
for i in $(seq 1 "$N"); do
  timeout 200 python3 "$V3/tests/record_semantic_during_nav.py" --out "$OUT/semantic_trial$i.jsonl" --seconds 190 > /dev/null 2>&1 &
  REC=$!
  timeout 200 python3 "$V3/tests/test_v3_world_navigation.py" --reset > "$OUT/trial$i.log" 2>&1
  cp "$V3/docs/evidence/phase1_navigation.json" "$OUT/trial$i.json" 2>/dev/null
  (cd "$PROJ" && git checkout -- V3/docs/evidence/phase1_navigation.json)
  wait $REC 2>/dev/null
  echo "TRIAL $CHAIN $i: $(grep -o "'leg': '[^']*'.*'nav2_status': '[A-Z]*'.*'duration_s': [0-9.]*" "$OUT/trial$i.log" | sed "s/'goal'.*'nav2_status'/'nav2_status'/" | tr '\n' ' ')"
done
