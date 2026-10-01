#!/usr/bin/env python3
"""Phase 18: failure / anomaly analysis over all recorded mission evidence (phase16 system-test missions are
in the mission DB; phase17/<tag>/mission_XX.json are the raw records). Counts outcomes, replan triggers,
WARN/ERROR events by type, legs, minimum LiDAR range. Output: V3/docs/evidence/phase18/failure_analysis.json"""
import collections, glob, json, os, re
V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
def norm(m):
    m = re.sub(r"\d+(\.\d+)?", "#", m)
    return m[:110]
out = {}
for d in sorted(glob.glob(os.path.join(V3, "docs", "evidence", "phase17", "*"))):
    tag = os.path.basename(d)
    files = sorted(glob.glob(os.path.join(d, "mission_*.json")))
    if not files:
        continue
    outcomes, warn, legres, reasons, replan_reasons = collections.Counter(), collections.Counter(), collections.Counter(), collections.Counter(), collections.Counter()
    minl, replans = [], []
    for f in files:
        r = json.load(open(f)); m = r.get("mission_record") or {}
        outcomes[r["final_state"]] += 1
        if m.get("failure_reason"): reasons[m["failure_reason"]] += 1
        for l in m.get("legs", []): legres[l.get("result")] += 1
        if isinstance(m.get("min_lidar_range_m"), (int, float)): minl.append(m["min_lidar_range_m"])
        replans.append(m.get("replans") or 0)
        ev = m.get("events") or {}
        evl = list(ev.get("mission_events", [])) if isinstance(ev, dict) else list(ev)
        for l in m.get("legs", []):
            evl += [e for e in l.get("decision_engine_events", []) if isinstance(e, dict)]
            for rr in (l.get("route") or {}).get("replan_reasons", []):
                replan_reasons[norm(str(rr))] += 1
        for e in evl:
            if e.get("level") in ("WARN", "ERROR"):
                warn[norm(e.get("msg", ""))] += 1
    out[tag] = {"missions": len(files), "outcomes": dict(outcomes), "failure_reasons": dict(reasons), "leg_results": dict(legres),
                "replans_total": sum(replans), "missions_with_replan": sum(1 for x in replans if x),
                "min_lidar_range_m": {"min": min(minl), "max": max(minl)} if minl else None,
                "replan_reasons": dict(replan_reasons), "warn_error_events_top": warn.most_common(15)}
os.makedirs(os.path.join(V3, "docs", "evidence", "phase18"), exist_ok=True)
json.dump(out, open(os.path.join(V3, "docs", "evidence", "phase18", "failure_analysis.json"), "w"), indent=1)
print(json.dumps(out, indent=1))
