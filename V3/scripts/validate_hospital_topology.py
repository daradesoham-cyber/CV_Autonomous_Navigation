#!/usr/bin/env python3
"""
Check the NEW hospital topology (v3_hospital_layout.py) against an occupancy map.

A node passes if its cell is free after inflating obstacles by the robot radius (+ margin).
An edge passes if a grid path exists in that inflated free space and is at most
max(1.6 * euclid, euclid + 3 m) long (i.e. the two nodes really connect through a nearby opening,
not around half the building). Writes <out>.json and an annotated <out>.png.
Usage: validate_hospital_topology.py [--map V3/maps/v3_hospital_map.yaml] [--radius 0.27] [--out PREFIX]
"""
import argparse
import heapq
import json
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage as nd

HERE = os.path.dirname(os.path.abspath(__file__))
V3 = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import v3_hospital_layout as L  # noqa: E402


def load_map(path):
    meta = yaml.safe_load(open(path))
    img = np.array(Image.open(os.path.join(os.path.dirname(path), meta["image"])))
    return img < 128, meta["resolution"], meta["origin"][:2]


def grid_path_len(free, a, b, limit):
    """Dijkstra (8-connected) from a to b in cells; returns length in cells or None (bounded by limit)."""
    H, W = free.shape
    dist = {a: 0.0}
    pq = [(0.0, a)]
    steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if (r, c) == b:
            return d
        if d > limit or d > dist.get((r, c), 1e18):
            continue
        for dr, dc, w in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and free[rr, cc]:
                nd_ = d + w
                if nd_ < dist.get((rr, cc), 1e18):
                    dist[(rr, cc)] = nd_
                    heapq.heappush(pq, (nd_, (rr, cc)))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(V3, "maps", "v3_hospital_map.yaml"))
    ap.add_argument("--radius", type=float, default=0.27)
    ap.add_argument("--out", default=os.path.join(V3, "docs", "evidence", "new_hospital", "topology_check"))
    a = ap.parse_args()
    occ, res, (ox, oy) = load_map(a.map)
    H = occ.shape[0]
    r = int(math.ceil(a.radius / res))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    free = ~nd.binary_dilation(occ, (xx ** 2 + yy ** 2) <= r * r)

    def cell(x, y):
        return int(round(H - 1 - (y - oy) / res)), int(round((x - ox) / res))

    nodes = {n[0]: n for n in L.NODES}
    rep = {"radius_m": a.radius, "map": a.map, "nodes": {}, "edges": []}
    ok = True
    for nid, n in nodes.items():
        f = bool(free[cell(n[2], n[3])])
        rep["nodes"][nid] = {"x": n[2], "y": n[3], "free": f}
        ok &= f
    for u, v in L.EDGES:
        nu, nv = nodes[u], nodes[v]
        e = math.hypot(nu[2] - nv[2], nu[3] - nv[3])
        lim = max(1.6 * e, e + 3.0) / res
        pl = None
        if rep["nodes"][u]["free"] and rep["nodes"][v]["free"]:
            pl = grid_path_len(free, cell(nu[2], nu[3]), cell(nv[2], nv[3]), lim)
        good = pl is not None and pl <= lim
        rep["edges"].append({"u": u, "v": v, "euclid_m": round(e, 2),
                             "path_m": None if pl is None else round(pl * res, 2), "ok": good})
        ok &= good
    rep["all_ok"] = bool(ok)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(rep, open(a.out + ".json", "w"), indent=1)

    img = Image.fromarray(np.where(occ, 0, np.where(free, 255, 200)).astype(np.uint8)).convert("RGB")
    S = 2
    img = img.resize((img.width * S, img.height * S), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def px(x, y):
        rr, cc = cell(x, y)
        return cc * S, rr * S
    for e in rep["edges"]:
        nu, nv = nodes[e["u"]], nodes[e["v"]]
        d.line([px(nu[2], nu[3]), px(nv[2], nv[3])], fill=(40, 160, 40) if e["ok"] else (230, 0, 0), width=3)
    for nid, n in nodes.items():
        x, y = px(n[2], n[3])
        col = (0, 90, 220) if rep["nodes"][nid]["free"] else (230, 0, 0)
        if nid in ("collection_point", "laboratory"):
            col = (220, 0, 160)
        d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=col)
        d.text((x + 8, y - 6), nid, fill=(0, 0, 0))
    img.save(a.out + ".png")
    for nid, v in rep["nodes"].items():
        if not v["free"]:
            print(f"NODE BLOCKED {nid} ({v['x']}, {v['y']})")
    for e in rep["edges"]:
        if not e["ok"]:
            print(f"EDGE FAIL {e['u']} - {e['v']}: euclid {e['euclid_m']} path {e['path_m']}")
    print(f"nodes {len(nodes)}, edges {len(rep['edges'])}, all_ok={rep['all_ok']} -> {a.out}.json/.png")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
