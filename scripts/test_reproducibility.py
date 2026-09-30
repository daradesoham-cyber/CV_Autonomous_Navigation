#!/usr/bin/env python3
"""
Sequential Reproducibility Test for Hospital Ward Mission (Mission 6: HOSPITAL -> STORAGE).
Performs Run 1 -> Run 2 -> Run 3 consecutively.
Demonstrates whether hospital ward entrapment is consistently reproducible.
"""

import os
import sys
import json
import time
from run_v26_honest_validation import run_single_mission

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUT_FILE = os.path.join(PROJECT_ROOT, "results/v26_final/reproducibility_test_results.json")

def main():
    print("=================================================================")
    print("   HOSPITAL WARD REPRODUCIBILITY TEST (RUN 1 -> RUN 2 -> RUN 3) ")
    print("=================================================================")

    runs = []
    for run_idx in range(1, 4):
        print(f"\n>>> Executing Run {run_idx} for Mission 6 (HOSPITAL -> STORAGE) <<<")
        res = run_single_mission(6, "hospital", "storage", timeout_sec=140)
        runs.append({
            "run_index": run_idx,
            "success": res["success"],
            "termination_reason": res["termination_reason"],
            "elapsed_time_s": res["elapsed_time_s"],
            "path_distance_m": res["path_distance_m"],
            "average_speed_mps": res["average_speed_mps"],
            "final_pose": res["final_pose"]
        })
        time.sleep(2.0)

    summary = {
        "mission": "Mission 6: HOSPITAL -> STORAGE",
        "total_runs": len(runs),
        "success_count": sum(1 for r in runs if r["success"]),
        "reproducible_failure": all(not r["success"] for r in runs),
        "runs": runs
    }

    with open(OUT_FILE, 'w') as f:
        json.dump(summary, f, indent=2)

    print("\n=================================================================")
    print(f"Hospital Ward Reproducibility Summary: {summary['success_count']} / {len(runs)} Successes")
    print(f"Consistent Entrapment Observed: {summary['reproducible_failure']}")
    print("=================================================================")

if __name__ == "__main__":
    main()
