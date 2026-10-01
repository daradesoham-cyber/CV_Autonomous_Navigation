#!/usr/bin/env python3
"""
Phase 17 / Phase 8: repeated full logistics missions on the RUNNING V3 system, via the web API.

Each mission: collection_point -> laboratory (the robot first drives to the collection point from
wherever the previous mission left it). Raw per-mission telemetry is the mission manager's own SQLite
record (/api/history/<id>: legs, path, events, durations, distance, min LiDAR range, replans, TTC events,
sign decisions) plus the API state timeline observed by this client, saved unmodified.
Timeouts are reported as TIMEOUT, never as success.
usage: run_v3_missions.py N [--traffic] [--tag NAME]
Output: V3/docs/evidence/new_hospital/missions/<tag>/mission_XX.json + summary.json
"""
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v3_api_client as api  # noqa: E402

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    n = int(sys.argv[1])
    traffic = "--traffic" in sys.argv
    tag = next((sys.argv[i + 1] for i, a in enumerate(sys.argv) if a == "--tag"), "traffic" if traffic else "repeated")
    out = os.path.join(V3_ROOT, "docs", "evidence", "new_hospital", "missions", tag)
    os.makedirs(out, exist_ok=True)
    api.post("/api/traffic", {"enable": traffic})
    rows = []
    for i in range(1, n + 1):
        s = api.state()
        if api.mission_state(s) not in ("READY", "MISSION_COMPLETE"):
            api.cmd("cancel")
            time.sleep(1)
            api.cmd("reset")
            api.wait_state(lambda s: api.mission_state(s) == "READY", 30)
        t0 = time.time()
        final, mid, tl = api.run_mission("collection_point", "laboratory", timeout=900)
        wall = round(time.time() - t0, 1)
        if final not in api.TERMINAL:
            final = "TIMEOUT"
            api.cmd("cancel")
        detail = {}
        if mid:
            try:
                detail = api.history_record(mid)
            except Exception as e:  # noqa: BLE001
                detail = {"error": str(e)}
        rec = {"index": i, "traffic": traffic, "final_state": final, "mission_id": mid, "client_wall_s": wall,
               "api_timeline": tl, "mission_record": detail}
        json.dump(rec, open(os.path.join(out, f"mission_{i:02d}.json"), "w"), indent=1)
        row = {"index": i, "mission_id": mid, "final_state": final, "result": detail.get("result"),
               "failure_reason": detail.get("failure_reason"), "duration_s": detail.get("duration_s"),
               "distance_m": detail.get("distance_m"), "min_lidar_range_m": detail.get("min_lidar_range_m"),
               "replans": detail.get("replans"), "ttc_events": detail.get("ttc_events"),
               "obstacle_encounters": detail.get("obstacle_encounters"), "sign_events": detail.get("sign_events")}
        rows.append(row)
        print(json.dumps(row), flush=True)
        api.cmd("reset")
        api.wait_state(lambda s: api.mission_state(s) == "READY", 30)
    ok = [r for r in rows if r["final_state"] == "MISSION_COMPLETE"]

    def st(key):
        v = [r[key] for r in ok if isinstance(r.get(key), (int, float))]
        return {"n": len(v), "mean": round(statistics.mean(v), 2), "min": min(v), "max": max(v),
                "stdev": round(statistics.stdev(v), 2) if len(v) > 1 else 0.0} if v else None
    summary = {"tag": tag, "traffic": traffic, "missions": len(rows), "complete": len(ok),
               "success_rate": round(len(ok) / len(rows), 3) if rows else None,
               "outcomes": {k: sum(1 for r in rows if r["final_state"] == k) for k in {r["final_state"] for r in rows}},
               "duration_s": st("duration_s"), "distance_m": st("distance_m"), "min_lidar_range_m": st("min_lidar_range_m"),
               "replans": st("replans"), "ttc_events": st("ttc_events"), "rows": rows}
    json.dump(summary, open(os.path.join(out, "summary.json"), "w"), indent=1)
    api.post("/api/traffic", {"enable": False})
    print("SUMMARY", json.dumps({k: v for k, v in summary.items() if k != "rows"}))


if __name__ == "__main__":
    main()
