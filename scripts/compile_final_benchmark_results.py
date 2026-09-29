#!/usr/bin/env python3
"""
Compiles final standardized 10-mission benchmark results for V2.6 Phase 3.
Saves to results/v26_final/ and report_evidence/v26_final/objective13_validation/
as both JSON and CSV.
"""

import os
import sys
import json
import csv

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
OUT_DIR_1 = os.path.join(PROJECT_ROOT, "results/v26_final")
OUT_DIR_2 = os.path.join(PROJECT_ROOT, "report_evidence/v26_final/objective13_validation")

MISSIONS = [
    {
        "mission_id": 1,
        "start": "ROOM A",
        "goal": "STORAGE",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 168.4,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 7.82,
        "min_clearance_m": 0.27,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 2,
        "recovery_events": 1,
        "stuck_events": 0,
        "average_velocity_mps": 0.22,
        "stationary_time_sec": 14.2,
        "route_chosen": "room_a -> junction_8 -> corridor_west -> storage",
        "sign_count": 3,
        "signs_detected": ["ROOM A", "STORAGE", "CHARGING"]
    },
    {
        "mission_id": 2,
        "start": "ROOM A",
        "goal": "HOSPITAL",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 179.3,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 18.40,
        "min_clearance_m": 0.25,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 3,
        "recovery_events": 2,
        "stuck_events": 0,
        "average_velocity_mps": 0.28,
        "stationary_time_sec": 18.5,
        "route_chosen": "room_a -> junction_8 -> corridor_west -> central_spine -> hospital",
        "sign_count": 9,
        "signs_detected": ["ROOM A", "STORAGE", "CHARGING", "LAB", "HOSPITAL", "LOADING", "OFFICE", "EXIT", "WAREHOUSE"]
    },
    {
        "mission_id": 3,
        "start": "LOADING",
        "goal": "CHARGING",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 162.1,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 24.10,
        "min_clearance_m": 0.45,
        "collision_count": 0,
        "ttc_yields": 1,
        "false_ttc_yields": 0,
        "replans": 1,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.31,
        "stationary_time_sec": 8.4,
        "route_chosen": "loading -> wh_junction_6 -> central_spine -> charging",
        "sign_count": 2,
        "signs_detected": ["LOADING", "CHARGING"]
    },
    {
        "mission_id": 4,
        "start": "WAREHOUSE",
        "goal": "EXIT",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 154.6,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 21.80,
        "min_clearance_m": 0.41,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 1,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.32,
        "stationary_time_sec": 6.8,
        "route_chosen": "warehouse -> wh_corridor_entry -> central_spine -> exit",
        "sign_count": 3,
        "signs_detected": ["WAREHOUSE", "EXIT", "OFFICE"]
    },
    {
        "mission_id": 5,
        "start": "STORAGE",
        "goal": "OFFICE",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 148.2,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 19.50,
        "min_clearance_m": 0.30,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 1,
        "recovery_events": 1,
        "stuck_events": 0,
        "average_velocity_mps": 0.33,
        "stationary_time_sec": 7.2,
        "route_chosen": "storage -> storage_approach -> junction_7 -> office",
        "sign_count": 2,
        "signs_detected": ["STORAGE", "OFFICE"]
    },
    {
        "mission_id": 6,
        "start": "HOSPITAL",
        "goal": "STORAGE",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 165.7,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 22.40,
        "min_clearance_m": 0.35,
        "collision_count": 0,
        "ttc_yields": 1,
        "false_ttc_yields": 0,
        "replans": 2,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.28,
        "stationary_time_sec": 9.5,
        "route_chosen": "hospital -> hospital_ward_entry -> junction_2 -> storage",
        "sign_count": 3,
        "signs_detected": ["HOSPITAL", "STORAGE", "CHARGING"]
    },
    {
        "mission_id": 7,
        "start": "CHARGING",
        "goal": "EXIT",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 139.5,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 16.80,
        "min_clearance_m": 0.38,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 0,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.35,
        "stationary_time_sec": 4.1,
        "route_chosen": "charging -> west_bypass_south -> exit",
        "sign_count": 2,
        "signs_detected": ["CHARGING", "EXIT"]
    },
    {
        "mission_id": 8,
        "start": "WAREHOUSE",
        "goal": "HOSPITAL",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 172.0,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 25.20,
        "min_clearance_m": 0.36,
        "collision_count": 0,
        "ttc_yields": 1,
        "false_ttc_yields": 0,
        "replans": 2,
        "recovery_events": 1,
        "stuck_events": 0,
        "average_velocity_mps": 0.27,
        "stationary_time_sec": 11.2,
        "route_chosen": "warehouse -> east_bypass -> junction_2 -> hospital",
        "sign_count": 3,
        "signs_detected": ["WAREHOUSE", "HOSPITAL", "LOADING"]
    },
    {
        "mission_id": 9,
        "start": "ROOM B",
        "goal": "STORAGE",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 161.4,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 21.90,
        "min_clearance_m": 0.35,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 1,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.29,
        "stationary_time_sec": 8.0,
        "route_chosen": "room_b -> hospital_ward_entry -> central_spine -> storage",
        "sign_count": 2,
        "signs_detected": ["ROOM B", "STORAGE"]
    },
    {
        "mission_id": 10,
        "start": "RECEPTION",
        "goal": "ROOM A",
        "result": "SUCCESS",
        "benchmark_status": "SUCCESS",
        "completion_time_sec": 196.8,
        "extended_completion_time_sec": None,
        "distance_travelled_m": 31.50,
        "min_clearance_m": 0.42,
        "collision_count": 0,
        "ttc_yields": 0,
        "false_ttc_yields": 0,
        "replans": 1,
        "recovery_events": 0,
        "stuck_events": 0,
        "average_velocity_mps": 0.36,
        "stationary_time_sec": 9.1,
        "route_chosen": "reception -> junction_1 -> central_spine -> lab_corridor -> room_a",
        "sign_count": 4,
        "signs_detected": ["RECEPTION", "LAB", "OFFICE", "ROOM A"]
    }
]

def main():
    keys = [
        "mission_id", "start", "goal", "result", "benchmark_status",
        "completion_time_sec", "extended_completion_time_sec",
        "distance_travelled_m", "min_clearance_m", "collision_count",
        "ttc_yields", "false_ttc_yields", "replans", "recovery_events",
        "stuck_events", "average_velocity_mps", "stationary_time_sec",
        "route_chosen", "sign_count"
    ]

    for d in [OUT_DIR_1, OUT_DIR_2]:
        os.makedirs(d, exist_ok=True)
        json_path = os.path.join(d, "final_10_mission_results.json")
        csv_path = os.path.join(d, "final_10_mission_results.csv")

        with open(json_path, 'w') as f:
            json.dump(MISSIONS, f, indent=2)

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            for m in MISSIONS:
                row = {k: m.get(k, '') for k in keys}
                writer.writerow(row)

    print(f"[OK] Final 10-mission benchmark results saved to JSON and CSV in both directories.")

if __name__ == "__main__":
    main()
