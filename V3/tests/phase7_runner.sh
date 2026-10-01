#!/usr/bin/env bash
# Phase 7 controlled tracker/costmap tests on the real runtime world, for one perception chain.
#   chain v3 : V3 detection + V3 spatial fusion (world-frame tracker) + V3 costmap obstacle node
#   chain v26: V3 detection + V2.6 lidar_camera_fusion_node        + V2.6 navigation_perception_node
# (same detector for both, so only the tracker/costmap path differs). No Nav2: the robot is driven by
# cmd_vel and objects by teleport/cmd_vel (test_v3_runtime_fusion.py). Output: V3/docs/evidence/phase7/<chain>/
# usage: [SCENARIOS="..."] phase7_runner.sh <v3|v26>
set -o pipefail
CHAIN=$1
V3="$(cd "$(dirname "$0")/.." && pwd)"; PROJ="$(dirname "$V3")"
OUT="$V3/docs/evidence/phase7/$CHAIN"; mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash; source "$PROJ/ros2_ws/install/setup.bash"
export V3_ROOT="$V3"
PIDS=()
start() { setsid "$@" > "$OUT/$(basename "$2")_$3.log" 2>&1 < /dev/null & PIDS+=($!); }
setsid ros2 launch hospital_logistics v3_world.launch.py headless:=true > "$OUT/world.log" 2>&1 < /dev/null &
WORLD=$!
for _ in $(seq 1 60); do grep -q "Entity creation successful" "$OUT/world.log" && break; sleep 2; done
sleep 3
setsid ros2 run hospital_logistics v3_detection_node --ros-args -p use_sim_time:=true > "$OUT/det.log" 2>&1 < /dev/null & PIDS+=($!)
if [ "$CHAIN" = v3 ]; then
  setsid ros2 run hospital_logistics v3_spatial_fusion_node --ros-args -p use_sim_time:=true > "$OUT/fusion.log" 2>&1 < /dev/null & PIDS+=($!)
  setsid ros2 run hospital_logistics v3_costmap_obstacle_node --ros-args -p use_sim_time:=true > "$OUT/costmap.log" 2>&1 < /dev/null & PIDS+=($!)
else
  setsid ros2 run autonomous_robot_perception lidar_camera_fusion_node --ros-args -p use_sim_time:=true > "$OUT/fusion.log" 2>&1 < /dev/null & PIDS+=($!)
  setsid ros2 run autonomous_robot_perception navigation_perception_node --ros-args -p use_sim_time:=true > "$OUT/costmap.log" 2>&1 < /dev/null & PIDS+=($!)
fi
sleep 25
SCS="${SCENARIOS:-S3_robot_toward_bed B_static_trolley_robot_moving S1_person_crossing S2_trolley_approaching S4_robot_past_bed E_robot_passing_close_to_bed F_robot_passing_close_to_trolley H_multiple_obstacles_robot_moving}"
for sc in $SCS; do
  timeout 600 python3 "$V3/tests/test_v3_runtime_fusion.py" "$sc" "--out-name=../phase7/$CHAIN/$sc.json" > "$OUT/$sc.stdout" 2>&1
  echo "SCENARIO $CHAIN $sc rc=$?"
done
for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done
kill -INT -- "-$WORLD" 2>/dev/null; sleep 8
for p in "${PIDS[@]}" "$WORLD"; do kill -KILL -- "-$p" 2>/dev/null; done
echo "DONE $CHAIN"
