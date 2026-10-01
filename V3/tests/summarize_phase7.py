#!/usr/bin/env python3
"""Tabulate Phase 7 results (V3 vs V2.6 chain) from V3/docs/evidence/phase7/<chain>/<scenario>.json."""
import json, os
D = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "evidence", "phase7")
rows = ["| scenario | chain | static? | TTC msgs | min TTC s | TTC<1.8s | phantom pts >1m | costmap pts | phantom % | GT collision |",
        "|---|---|---|---|---|---|---|---|---|---|"]
for sc in sorted(f[:-5] for f in os.listdir(os.path.join(D, "v3")) if f.endswith(".json")):
    for ch in ("v3", "v26"):
        p = os.path.join(D, ch, sc + ".json")
        if not os.path.exists(p):
            rows.append(f"| {sc} | {ch} | missing |"); continue
        r = json.load(open(p))["scenarios"][0]
        t = r["semantic_obstacles"]["ttc_stats_by_class"]
        n = sum(v["n"] for v in t.values()); lo = sum(v["below_1_8s"] for v in t.values())
        mn = min((v["min_s"] for v in t.values()), default=None)
        pc = r["costmap_phantom_check"]; pe = pc["points_evaluated"]; ph = pc["points_farther_than_1m_from_any_real_object"]
        rows.append(f"| {sc} | {ch} | {r['static_scenario']} | {n} | {mn} | {lo} | {ph} | {pe} | "
                    f"{(100*ph/pe):.1f} | {r['gt_collision']} |" if pe else
                    f"| {sc} | {ch} | {r['static_scenario']} | {n} | {mn} | {lo} | {ph} | 0 | - | {r['gt_collision']} |")
out = "\n".join(rows)
open(os.path.join(D, "phase7_summary.md"), "w").write(out + "\n")
print(out)
