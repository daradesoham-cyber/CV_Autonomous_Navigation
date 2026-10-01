#!/usr/bin/env python3
"""Phase 19: sample resource use of the RUNNING V3 system for N seconds (default 60).
Per-process CPU% / RSS (psutil from the project venv if present, else /proc), GPU util/memory
(nvidia-smi), topic rates + detection/fusion latency from /api/state. Output: phase19/performance.json"""
import json, os, subprocess, sys, time, urllib.request
N = int(sys.argv[1]) if len(sys.argv) > 1 else 60
V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAT = ("gz sim", "gz-sim", "ruby", "parameter_bridge", "controller_server", "planner_server", "behavior_server", "bt_navigator",
       "amcl", "map_server", "lifecycle_manager", "robot_state_publisher", "v3_", "decision_engine", "dynamic_obstacles", "initial_pose")
HZ = os.sysconf("SC_CLK_TCK")
def procs():
    out = {}
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="ignore")
            if "measure_v3_performance" in cmd or not any(p in cmd for p in PAT):
                continue
            st = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
            rss = int(st[21]) * os.sysconf("SC_PAGE_SIZE")
            name = next((p for p in cmd.split() if any(k in p for k in PAT)), cmd.split()[0]).split("/")[-1][:40]
            out[pid] = (name, int(st[11]) + int(st[12]), rss)
        except Exception:
            pass
    return out
def gpu():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=5).stdout.strip().split(",")
        return [float(x) for x in r]
    except Exception:
        return None
a, t0 = procs(), time.time()
g, st = [], []
for _ in range(N):
    time.sleep(1)
    x = gpu(); x and g.append(x)
    try:
        s = json.load(urllib.request.urlopen("http://127.0.0.1:8080/api/state", timeout=2))
        st.append({"rates": {k: v.get("hz") for k, v in s["system"]["topics"].items()},
                   "det": s["perception"]["detection_timing"], "fusion_ms": (s["perception"]["spatial"] or {}).get("fusion_ms"),
                   "mission": s["mission"]["state"]})
    except Exception:
        pass
b, dt = procs(), time.time() - t0
per = {}
for pid, (name, ticks, rss) in b.items():
    if pid in a:
        e = per.setdefault(name, {"cpu_pct": 0.0, "rss_mb": 0.0, "n": 0})
        e["cpu_pct"] += 100.0 * (ticks - a[pid][1]) / HZ / dt; e["rss_mb"] += rss / 1e6; e["n"] += 1
per = {k: {"cpu_pct": round(v["cpu_pct"], 1), "rss_mb": round(v["rss_mb"], 1), "processes": v["n"]} for k, v in sorted(per.items(), key=lambda kv: -kv[1]["cpu_pct"])}
def agg(vals):
    v = [x for x in vals if isinstance(x, (int, float))]
    return {"mean": round(sum(v) / len(v), 2), "min": round(min(v), 2), "max": round(max(v), 2), "n": len(v)} if v else None
rates = {k: agg([s["rates"].get(k) for s in st]) for k in (st[0]["rates"] if st else {})}
res = {"duration_s": round(dt, 1), "cpu_cores": os.cpu_count(), "mission_states_seen": sorted({s["mission"] for s in st}),
       "total_cpu_pct_of_one_core": round(sum(v["cpu_pct"] for v in per.values()), 1),
       "total_rss_mb": round(sum(v["rss_mb"] for v in per.values()), 1), "per_process": per,
       "gpu": {"util_pct": agg([x[0] for x in g]), "mem_used_mb": agg([x[1] for x in g]), "mem_total_mb": g[0][2] if g else None},
       "topic_rates_hz": rates,
       "detection_ms": {k: agg([(s["det"] or {}).get(k) for s in st]) for k in ("preprocess", "inference", "postprocess")},
       "fusion_ms": agg([s["fusion_ms"] for s in st]),
       "load_avg": os.getloadavg()}
out = os.path.join(V3, "docs", "evidence", "phase19", "performance.json")
json.dump(res, open(out, "w"), indent=1)
print(json.dumps({k: v for k, v in res.items() if k != "per_process"}, indent=1)); print(json.dumps(dict(list(per.items())[:12]), indent=1))
