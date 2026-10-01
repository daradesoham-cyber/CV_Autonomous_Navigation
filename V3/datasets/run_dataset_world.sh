#!/usr/bin/env bash
# Start the V3 dataset-rendering world headless (GPU rendering, no robot, no ROS nodes).
set -eo pipefail
V3_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/lyrical/setup.bash   # (not under 'set -u': the ROS setup reads unset variables)
export GZ_SIM_RESOURCE_PATH="$V3_ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
python3 "$V3_ROOT/datasets/build_dataset_world.py"
exec gz sim -s -r -v 2 "$V3_ROOT/worlds/hospital_logistics_dataset_world.sdf"
