#!/usr/bin/env python3
"""
Generate the Nav2 occupancy map of the V3 hospital directly from the world's collision geometry.

Every <include> of V3/worlds/v3_hospital_world.sdf (plus the V3 static additions) is resolved to its
model.sdf; every <collision> (mesh with scale, box, cylinder, sphere) is placed with
model pose * link pose * collision pose and sliced horizontally at the given heights (default: the
robot LiDAR plane 0.27 m, plus 0.10 m so low obstacles under the scan plane are kept). The slice
segments/polygons are rasterised at 0.05 m. This is what an ideal planar scan at those heights sees, so
AMCL can match it. Dynamic traffic models are excluded (they move).
Outputs: V3/maps/v3_hospital_map.pgm/.yaml and v3_hospital_map_preview.png
Run with the project venv (trimesh, pycollada, shapely): .venv/bin/python V3/scripts/generate_hospital_map.py
"""
import argparse
import math
import os
import re
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from PIL import Image, ImageDraw

V3 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIRS = [os.path.join(V3, "hospital", "models"), os.path.join(V3, "models")]
DYNAMIC = ("dynamic_",)


def pose6(text):
    v = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))


def mat(p):
    x, y, z, r, pi, ya = p
    return trimesh.transformations.compose_matrix(translate=[x, y, z], angles=[r, pi, ya])


def find_model(uri):
    name = uri.replace("model://", "").strip().split("/")[0]
    for d in MODEL_DIRS:
        p = os.path.join(d, name)
        if os.path.isdir(p):
            return p
    return None


def resolve_mesh(uri, mdir):
    u = uri.strip()
    if u.startswith("model://"):
        rest = u[len("model://"):]
        name, _, rel = rest.partition("/")
        base = find_model(name)
        return os.path.join(base, rel) if base else None
    return os.path.join(mdir, u)


_mesh_cache = {}


def load_mesh(path):
    if path not in _mesh_cache:
        m = trimesh.load(path, force="mesh", process=False)
        if path.lower().endswith(".dae"):
            # trimesh ignores the Collada <unit>; Gazebo applies it (AWS meshes are in centimetres)
            u = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', open(path, errors="ignore").read(4096))
            if u:
                m.apply_scale(float(u.group(1)))
        _mesh_cache[path] = m
    return _mesh_cache[path].copy()


def model_collisions(mdir):
    """yield (trimesh in model frame) for every collision of model.sdf"""
    sdf = os.path.join(mdir, "model.sdf")
    root = ET.parse(sdf).getroot()
    model = root.find("model")
    out = []
    for link in model.findall("link"):
        lp = mat(pose6(link.findtext("pose")))
        for col in link.findall("collision"):
            cp = mat(pose6(col.findtext("pose")))
            g = col.find("geometry")
            m = None
            if g.find("mesh") is not None:
                me = g.find("mesh")
                path = resolve_mesh(me.findtext("uri"), mdir)
                if not path or not os.path.exists(path):
                    continue
                m = load_mesh(path)
                sc = me.findtext("scale")
                if sc:
                    m.apply_scale([float(s) for s in sc.split()])
            elif g.find("box") is not None:
                m = trimesh.creation.box(extents=[float(s) for s in g.find("box").findtext("size").split()])
            elif g.find("cylinder") is not None:
                c = g.find("cylinder")
                m = trimesh.creation.cylinder(radius=float(c.findtext("radius")), height=float(c.findtext("length")), sections=24)
            elif g.find("sphere") is not None:
                m = trimesh.creation.icosphere(radius=float(g.find("sphere").findtext("radius")))
            if m is not None:
                m.apply_transform(lp @ cp)
                out.append(m)
    for inc in model.findall("include"):  # nested includes
        sub = find_model(inc.findtext("uri"))
        if sub:
            T = mat(pose6(inc.findtext("pose")))
            for m in model_collisions(sub):
                m.apply_transform(T)
                out.append(m)
    return out


def world_items(world_path):
    root = ET.parse(world_path).getroot()
    w = root.find("world")
    items = []
    for inc in w.findall("include"):
        name = inc.findtext("name") or ""
        if name.startswith(DYNAMIC):
            continue
        mdir = find_model(inc.findtext("uri"))
        if mdir:
            items.append((name, mdir, mat(pose6(inc.findtext("pose")))))
    for mo in w.findall("model"):  # inline models (V3 additions, ground plane skipped: plane geometry)
        name = mo.get("name")
        if name.startswith(DYNAMIC) or name == "ground_plane":
            continue
        tmp = os.path.join("/tmp", f"_v3map_{name}")
        os.makedirs(tmp, exist_ok=True)
        ET.ElementTree(ET.fromstring(f"<sdf version='1.9'>{ET.tostring(mo, encoding='unicode')}</sdf>")).write(os.path.join(tmp, "model.sdf"))
        items.append((name, tmp, mat(pose6(mo.findtext("pose")))))
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(V3, "worlds", "v3_hospital_world.sdf"))
    ap.add_argument("--heights", default="0.10,0.27")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--out", default=os.path.join(V3, "maps", "v3_hospital_map"))
    a = ap.parse_args()
    heights = [float(h) for h in a.heights.split(",")]
    segs, stats = [], {}
    for name, mdir, T in world_items(a.world):
        n = 0
        for m in model_collisions(mdir):
            m.apply_transform(T)
            zmin, zmax = m.bounds[0][2], m.bounds[1][2]
            for h in heights:
                if not (zmin <= h <= zmax):
                    continue
                try:
                    s = trimesh.intersections.mesh_plane(m, [0, 0, 1], [0, 0, h])
                except Exception:
                    continue
                if len(s):
                    segs.append(s[:, :, :2])
                    n += len(s)
        stats[os.path.basename(mdir)] = stats.get(os.path.basename(mdir), 0) + n
    allp = np.concatenate([s.reshape(-1, 2) for s in segs])
    lo, hi = allp.min(0) - 1.0, allp.max(0) + 1.0
    W, H = int(math.ceil((hi[0] - lo[0]) / a.res)), int(math.ceil((hi[1] - lo[1]) / a.res))
    img = Image.new("L", (W, H), 254)
    d = ImageDraw.Draw(img)

    def px(p):
        return ((p[0] - lo[0]) / a.res, H - 1 - (p[1] - lo[1]) / a.res)
    for s in segs:
        for seg in s:
            d.line([px(seg[0]), px(seg[1])], fill=0, width=2)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    img.save(a.out + ".pgm")
    with open(a.out + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(a.out)}.pgm\nmode: trinary\nresolution: {a.res}\n"
                f"origin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    img.convert("RGB").save(a.out + "_preview.png")
    print(f"map {W}x{H} px, origin ({lo[0]:.2f},{lo[1]:.2f}), extent {hi[0]-lo[0]:.1f} x {hi[1]-lo[1]:.1f} m, "
          f"{sum(len(s) for s in segs)} segments")
    for k, v in sorted(stats.items(), key=lambda kv: -kv[1])[:12]:
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
