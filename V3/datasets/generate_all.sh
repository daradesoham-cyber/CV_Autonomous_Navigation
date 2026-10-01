#!/usr/bin/env bash
# Full V3 dataset generation. Starts a FRESH dataset world for every stage and every CHUNK frames
# (a long-running server degraded after ~1500 frames and rendered stale scenes; see
# V3_DATASET_GENERATION_REPORT.md). Resumable: each split continues from its metadata line count.
# Seeds are fixed so the dataset is reproducible; splits use disjoint seeds.
set -eo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/lyrical/setup.bash
OUT="${1:-$D/v3_hospital_cv}"
LOG_DIR="${LOG_DIR:-/tmp}"
WORLD_PID=""

start_world() {
  "$D/run_dataset_world.sh" > "$LOG_DIR/v3_dataset_world.log" 2>&1 &
  WORLD_PID=$!
  for _ in $(seq 1 60); do
    gz topic -l 2>/dev/null | grep -q "^/v3ds/seg/labels_map$" && sleep 3 && return 0
    sleep 2
  done
  echo "dataset world did not start" >&2; return 1
}
stop_world() {
  [ -n "$WORLD_PID" ] && kill "$WORLD_PID" 2>/dev/null || true
  pkill -f "hospital_logistics_dataset_world[.]sdf" 2>/dev/null || true
  sleep 3
  WORLD_PID=""
}
trap stop_world EXIT

stage() { stop_world; start_world; python3 "$@"; }

CHUNK=500
done_count() { [ -f "$OUT/meta/$1.jsonl" ] && wc -l < "$OUT/meta/$1.jsonl" || echo 0; }
split_chunks() {  # split count seed
  local split=$1 count=$2 seed=$3 start
  start=$(done_count "$split")
  while [ "$start" -lt "$count" ]; do
    stage "$D/generate_dataset.py" --split "$split" --count "$count" --seed "$seed" --out "$OUT" \
      --start "$start" --end $((start + CHUNK))
    start=$(done_count "$split")
  done
}

[ -f "$D/visibility_calibration.json" ] || stage "$D/calibrate_visibility.py"
split_chunks train 3000 101
split_chunks val   600  202
split_chunks test  500  303
# subsets: one fresh world per subset (10 x 40 frames)
for k in A_hospital_beds B_bed_vs_lab_bench C_bed_vs_reception_desk D_chair_vs_cart E_trolley_vs_forklift_cart \
         F_wheelchair G_iv_stand H_person I_empty_hallway J_cluttered_corridor; do
  [ -f "$OUT/subsets/$k/meta/test.jsonl" ] && [ "$(wc -l < "$OUT/subsets/$k/meta/test.jsonl")" -ge 40 ] && continue
  stage "$D/generate_dataset.py" --split subsets --count 40 --seed 404 --out "$OUT" --only-subset "$k"
done
