#!/usr/bin/env python3
"""Phase 8 check: during one traffic mission, how far did each moving traffic model travel, and how
close did it come to the robot? Gazebo ground-truth poses (evaluation only), sampled at ~2 Hz.
Output: V3/docs/evidence/phase8/traffic_missions_v3_traffic_node.json (first measurement, before the V3
traffic node existed: traffic_exposure.json). usage: traffic_exposure.py [N]"""
import json, math, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v3_api_client as api  # noqa: E402
from gz.msgs.pose_v_pb2 import Pose_V  # noqa: E402
from gz.transport import Node as GzNode  # noqa: E402
V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MOV = ["dynamic_person", "dynamic_hospital_trolley", "dynamic_warehouse_cart", "dynamic_forklift"]
last, lock = {}, threading.Lock()
def cb(m):
    with lock:
        for p in m.pose:
            if p.name in MOV or p.name == "autonomous_robot":
                last[p.name] = (p.position.x, p.position.y)
n = GzNode(); n.subscribe(Pose_V, "/world/realistic_facility_world/pose/info", cb)
samples, done = [], threading.Event()
def sampler():
    while not done.is_set():
        with lock:
            if "autonomous_robot" in last: samples.append(dict(last))
        time.sleep(0.5)
threading.Thread(target=sampler, daemon=True).start()
N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
api.post("/api/traffic", {"enable": True}); time.sleep(2)
runs = []
for i in range(N):
    samples.clear()
    final, mid, _ = api.run_mission("collection_point", "laboratory", timeout=900)
    rec = api.history_record(mid) if mid else {}
    res = {"mission_id": mid, "final": final, "duration_s": rec.get("duration_s"), "distance_m": rec.get("distance_m"),
           "replans": rec.get("replans"), "ttc_events": rec.get("ttc_events"), "min_lidar_range_m": rec.get("min_lidar_range_m"),
           "failure_reason": rec.get("failure_reason"), "samples": len(samples), "objects": {}}
    snap = list(samples)
    for o in MOV:
        pts = [s[o] for s in snap if o in s]
        if not pts: res["objects"][o] = None; continue
        path = sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
        span = max(math.dist(a, b) for a in pts[::10] for b in pts[::10])
        d = [math.dist(s[o], s["autonomous_robot"]) for s in snap if o in s]
        res["objects"][o] = {"path_travelled_m": round(path, 2), "max_extent_m": round(span, 2),
                             "min_centre_distance_to_robot_m": round(min(d), 2),
                             "samples_within_2m_of_robot": sum(1 for x in d if x < 2.0)}
    runs.append(res)
    print(json.dumps({k: v for k, v in res.items()}), flush=True)
    api.cmd("reset"); api.wait_state(lambda s: api.mission_state(s) == "READY", 30)
done.set(); api.post("/api/traffic", {"enable": False})
out = {"missions": N, "complete": sum(r["final"] == "MISSION_COMPLETE" for r in runs), "runs": runs}
os.makedirs(os.path.join(V3, "docs", "evidence", "phase8"), exist_ok=True)
json.dump(out, open(os.path.join(V3, "docs", "evidence", "phase8", "traffic_missions_v3_traffic_node.json"), "w"), indent=1)
print("COMPLETE", out["complete"], "/", N, flush=True); os._exit(0)
