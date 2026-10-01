#!/usr/bin/env python3
"""
V3 hospital CV dataset generator: real Gazebo Sim renders + Gazebo-generated ground truth.

Requires the dataset world running (see run_dataset_world.sh):
    GZ_SIM_RESOURCE_PATH=V3/models gz sim -s -r V3/worlds/hospital_logistics_dataset_world.sdf

Ground truth: Gazebo's instance-segmentation camera on the same rig link as the RGB camera (paired by
identical sim timestamps) gives, for every model carrying a Label plugin (label = class_id + 1), its
visible pixels. The label box is the tight box of those pixels; instances that are too small or
estimated mostly occluded are dropped and recorded (rule in gt_labels.py). Gazebo's
boundingbox_camera boxes are recorded as an independent cross-check. Nothing is drawn, guessed or
hand-labelled.

Splits
  train / val : randomized scenes (seeded, disjoint seeds): props relocated around the camera,
                lighting randomized; modes random / hard_negative / sign_marker / empty / clutter.
  test        : the CANONICAL V3 world layout (props at their V3 positions, pool parked), camera at
                robot-camera poses along real Nav2 routes (Phase 1 traces), topology nodes and the
                Phase 1 viewpoints; default lighting (80 %) and dim/bright variants (20 %).
  subsets A-J : targeted challenge sets for the Phase 1 failure modes (canonical layout + controlled
                placements), generated with their own seed, evaluation only.

Usage: python3 generate_dataset.py --split train --count 3000 --seed 1
       python3 generate_dataset.py --split subsets --count 40 --seed 9
"""
import argparse
import json
import math
import os
import random
import sys
import time

import cv2

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gz_scene import GzScene  # noqa: E402
import gt_labels as gt  # noqa: E402
from scripts.build_realistic_facility import WALLS, PILLARS, FURNITURE  # noqa: E402

OUT_ROOT = os.path.join(V3_ROOT, "datasets", "v3_hospital_cv")
WORLD_META = json.load(open(os.path.join(V3_ROOT, "datasets", "dataset_world_objects.json")))
PROPS_META = json.load(open(os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")))
NAV_TRACE = os.path.join(V3_ROOT, "docs", "evidence", "phase1_navigation.json")
CAM_FWD, CAM_Z, CAM_PITCH = 0.22, 0.41, -0.05  # robot camera mount (camera.xacro + base 0.06)
PARK_Z = WORLD_META["park_z"]
RIG = "v3ds_camera_rig"

OBJ = {o["name"]: o for o in WORLD_META["objects"]}
REPLACED = set(PROPS_META["replaced_placeholders"])
FIXED_V3 = {"v3_collection_table", "v3_payload_blood_sample_carrier"}
PLAQUES = ["v3_sign_collection_point", "v3_sign_laboratory_test_point"]


def box(name, x, y, sx, sy):
    return (name, x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2)


WALL_BOXES = [box(n, x, y, sx, sy) for n, x, y, sx, sy in WALLS] + [box(n, x, y, sx, sy) for n, x, y, sx, sy in PILLARS]
FURN_BOXES = [box(f[0], f[1], f[2], f[7], f[8]) for f in FURNITURE if f[0] not in REPLACED]
SIGN_BOXES = [box(o["name"], o["canonical_pose"][0], o["canonical_pose"][1], 0.8, 0.8)
              for o in WORLD_META["objects"] if o["name"].startswith("sign_")]
FIXED_BOXES = []
for p in PROPS_META["props"]:
    if p["name"] in FIXED_V3:
        f = p["footprint"]
        FIXED_BOXES.append((p["name"], f["xmin"], f["ymin"], f["xmax"], f["ymax"]))
STRUCTURE = WALL_BOXES + FURN_BOXES + FIXED_BOXES
HARD_NEG_TARGETS = [b for b in FURN_BOXES] + [b for b in WALL_BOXES if b[0].startswith("pillar")]

MOVABLE = [o["name"] for o in WORLD_META["objects"]
           if o.get("footprint") and o["name"] not in FIXED_V3 and o["name"] not in PLAQUES
           and not o["name"].startswith("sign_") and o["name"] != "ground_plane"
           and (o["kind"] in ("prop", "pool") or o["name"].startswith("dynamic_"))]
POSITIVE = [n for n in MOVABLE if OBJ[n]["class_id"] is not None]
NEGATIVE = [n for n in MOVABLE if OBJ[n]["class_id"] is None]
CLASS_NAMES = ["person", "hospital_bed", "cart", "wheelchair", "iv_stand", "surgical_trolley",
               "directional_sign", "collection_point_marker", "lab_test_point_marker"]


def by_class(cid):
    return [n for n in POSITIVE if OBJ[n]["class_id"] == cid]


def model_of(n):
    return OBJ[n].get("model", n)


# ---------------------------------------------------------------- geometry
def footprint_world(name, x, y, yaw, margin=0.0):
    (ax, ay), (bx, by) = OBJ[name]["footprint"]
    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for px, py in ((ax, ay), (ax, by), (bx, ay), (bx, by)):
        xs.append(x + c * px - s * py)
        ys.append(y + s * px + c * py)
    return (name, min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)


def overlap(a, b):
    return not (a[3] <= b[1] or a[1] >= b[3] or a[4] <= b[2] or a[2] >= b[4])


def point_clear(x, y, boxes, r):
    return all(max(b[1] - x, 0, x - b[3]) ** 2 + max(b[2] - y, 0, y - b[4]) ** 2 >= r * r for b in boxes)


def inside_building(x, y, m=0.3):
    return -16 + m < x < 16 - m and -13 + m < y < 13 - m


def seg_hits(x0, y0, x1, y1, boxes, skip=()):
    for b in boxes:
        if b[0] in skip:
            continue
        t0, t1 = 0.0, 1.0
        ok = True
        for o, d, lo, hi in ((x0, x1 - x0, b[1], b[3]), (y0, y1 - y0, b[2], b[4])):
            if abs(d) < 1e-9:
                if o < lo or o > hi:
                    ok = False
                    break
            else:
                a, c = (lo - o) / d, (hi - o) / d
                t0, t1 = max(t0, min(a, c)), min(t1, max(a, c))
                if t0 > t1:
                    ok = False
                    break
        if ok:
            return True
    return False


def rig_pose(rx, ry, yaw, pitch=CAM_PITCH):
    return (rx + CAM_FWD * math.cos(yaw), ry + CAM_FWD * math.sin(yaw), CAM_Z, 0.0, pitch, yaw)


class Scene:
    """Tracks where every movable object is for one frame."""

    def __init__(self, rng, canonical):
        self.rng = rng
        self.poses = {}
        self.occupied = []
        for n in MOVABLE + PLAQUES:
            p = OBJ[n]["canonical_pose"]
            if canonical and OBJ[n]["kind"] != "pool":
                self.poses[n] = tuple(p)
                if OBJ[n].get("footprint") and n not in PLAQUES:
                    self.occupied.append(footprint_world(n, p[0], p[1], p[5]))
            elif n in PLAQUES:
                self.poses[n] = tuple(p)
            else:
                self.poses[n] = (p[0], p[1], PARK_Z, 0.0, 0.0, 0.0)

    def blocked(self, fp):
        return any(overlap(fp, b) for b in STRUCTURE + self.occupied)

    def place(self, name, x, y, yaw):
        fp = footprint_world(name, x, y, yaw, margin=0.08)
        if not inside_building(x, y) or self.blocked(fp):
            return False
        base_z = OBJ[name]["canonical_pose"][2] if name.startswith("dynamic_") else 0.0
        self.poses[name] = (x, y, base_z, 0.0, 0.0, yaw)
        self.occupied.append(fp)
        return True

    def place_in_view(self, name, rx, ry, yaw, dmin=1.0, dmax=7.0, spread=0.55, tries=40):
        for _ in range(tries):
            d = self.rng.uniform(dmin, dmax)
            a = yaw + self.rng.uniform(-spread, spread)
            x, y = rx + d * math.cos(a), ry + d * math.sin(a)
            if seg_hits(rx, ry, x, y, WALL_BOXES):
                continue
            if self.place(name, x, y, self.rng.uniform(-math.pi, math.pi)):
                return True
        return False

    def place_near(self, name, bx, by, rmin=0.4, rmax=2.0, tries=40):
        for _ in range(tries):
            a, d = self.rng.uniform(-math.pi, math.pi), self.rng.uniform(rmin, rmax)
            if self.place(name, bx + d * math.cos(a), by + d * math.sin(a), self.rng.uniform(-math.pi, math.pi)):
                return True
        return False

    def free_robot_pose(self, tries=400):
        for _ in range(tries):
            x, y = self.rng.uniform(-15.5, 15.5), self.rng.uniform(-12.5, 12.5)
            if point_clear(x, y, STRUCTURE + self.occupied, 0.4):
                return x, y, self.rng.uniform(-math.pi, math.pi)
        raise RuntimeError("no free robot pose")

    def robot_facing(self, tx, ty, dmin, dmax, spread=0.8, jitter=0.3, tries=200, skip=()):
        for _ in range(tries):
            a, d = self.rng.uniform(-math.pi, math.pi), self.rng.uniform(dmin, dmax)
            x, y = tx + d * math.cos(a), ty + d * math.sin(a)
            if not inside_building(x, y, 0.5) or not point_clear(x, y, STRUCTURE + self.occupied, 0.4):
                continue
            if seg_hits(x, y, tx, ty, WALL_BOXES, skip=skip):
                continue
            yaw = math.atan2(ty - y, tx - x) + self.rng.uniform(-jitter, jitter)
            return x, y, yaw
        return None


def pick_balanced(rng, pool, counts, k):
    chosen = []
    cands = list(pool)
    for _ in range(min(k, len(cands))):
        w = [1.0 / (1.0 + counts.get(OBJ[n]["class_id"], 0)) for n in cands]
        n = rng.choices(cands, weights=w)[0]
        chosen.append(n)
        cands.remove(n)
    return chosen


# ---------------------------------------------------------------- lighting
def lighting(rng, mode):
    if mode == "default":
        return {"global": 1.0, "tint": (1.0, 1.0, 1.0), "jitter": [1.0] * len(WORLD_META["lights"])}
    if mode == "dim":
        g = rng.uniform(0.40, 0.60)
    elif mode == "bright":
        g = rng.uniform(1.15, 1.30)
    else:
        g = rng.uniform(0.45, 1.25)
    t = rng.choice([(1.0, 1.0, 1.0), (1.0, 0.93, 0.82), (0.88, 0.95, 1.0)])
    return {"global": g, "tint": t, "jitter": [rng.uniform(0.85, 1.15) for _ in WORLD_META["lights"]]}


def apply_lighting(gz, light):
    for spec, j in zip(WORLD_META["lights"], light["jitter"]):
        rgb = [min(1.0, c * light["global"] * j * t) for c, t in zip(spec["diffuse"], light["tint"])]
        gz.set_light(spec, rgb)


# ---------------------------------------------------------------- scene builders
def scene_random(rng, counts, clutter=False):
    sc = Scene(rng, canonical=False)
    rx, ry, yaw = sc.free_robot_pose()
    k = rng.randint(5, 9) if clutter else rng.randint(1, 4)
    for n in pick_balanced(rng, POSITIVE, counts, k):
        sc.place_in_view(n, rx, ry, yaw, dmax=5.5 if clutter else 7.0)
    for n in rng.sample(NEGATIVE, rng.randint(1, 4) if clutter else rng.randint(0, 3)):
        sc.place_in_view(n, rx, ry, yaw)
    return sc, (rx, ry, yaw)


CONFUSERS = {
    "bench": [1], "desk": [1], "table": [1], "workstation": [1], "rack": [2, 5], "pallet": [5],
    "pillar": [4], "charging": [2], "bench_": [1],
}


def scene_hard_negative(rng, counts):
    sc = Scene(rng, canonical=False)
    use_movable = rng.random() < 0.4
    if use_movable:
        neg = rng.choice(NEGATIVE)
        rx, ry, yaw = sc.free_robot_pose()
        if not sc.place_in_view(neg, rx, ry, yaw, dmin=1.2, dmax=4.5, spread=0.3):
            return scene_random(rng, counts)
        target, (tx, ty) = neg, sc.poses[neg][:2]
    else:
        t = rng.choice(HARD_NEG_TARGETS)
        target, tx, ty = t[0], (t[1] + t[3]) / 2, (t[2] + t[4]) / 2
        pose = sc.robot_facing(tx, ty, 1.2, 5.0, skip=(target,))
        if pose is None:
            return scene_random(rng, counts)
        rx, ry, yaw = pose
    if rng.random() < 0.5:  # confusable positive next to the negative
        cls = next((v for k, v in CONFUSERS.items() if k in target), None) or \
            {"Chair": [2, 3], "dynamic_forklift": [5], "dynamic_warehouse_cart": [5, 2],
             "MetalCabinet": [2], "Drawer": [2], "AdjTable": [1], "StorageRack": [2, 5]}.get(model_of(target), [1, 2])
        cands = [n for c in cls for n in by_class(c) if sc.poses[n][2] == PARK_Z]
        if cands:
            sc.place_near(rng.choice(cands), tx, ty, 0.5, 1.8)
    return sc, (rx, ry, yaw)


def wall_faces():
    faces = []
    for n, x0, y0, x1, y1 in WALL_BOXES:
        if n.startswith("pillar"):
            continue
        if x1 - x0 >= y1 - y0 and x1 - x0 >= 1.4:
            faces += [(n, "y", y1 + 0.02, x0 + 0.6, x1 - 0.6, 1), (n, "y", y0 - 0.02, x0 + 0.6, x1 - 0.6, -1)]
        elif y1 - y0 >= 1.4:
            faces += [(n, "x", x1 + 0.02, y0 + 0.6, y1 - 0.6, 1), (n, "x", x0 - 0.02, y0 + 0.6, y1 - 0.6, -1)]
    return faces


WALL_FACES = wall_faces()


def scene_sign_marker(rng, counts):
    sc = Scene(rng, canonical=False)
    if rng.random() < 0.45:
        s = OBJ[rng.choice([o["name"] for o in WORLD_META["objects"] if o["name"].startswith("sign_")])]
        tx, ty = s["canonical_pose"][:2]
        pose = sc.robot_facing(tx, ty, 1.0, 6.0, jitter=0.35)
    else:
        plaque = rng.choice(PLAQUES)
        for _ in range(60):
            n, axis, c, lo, hi, sgn = rng.choice(WALL_FACES)
            u, z = rng.uniform(lo, hi), rng.uniform(0.55, 1.35)
            if axis == "y":
                x, y, yaw = u, c + sgn * 0.02, 0.0
            else:
                x, y, yaw = c + sgn * 0.02, u, math.pi / 2
            if not inside_building(x, y, 0.1) or any(overlap(("p", x - 0.45, y - 0.45, x + 0.45, y + 0.45), b)
                                                    for b in FURN_BOXES + FIXED_BOXES + WALL_BOXES if b[0] != n):
                continue
            sc.poses[plaque] = (x, y, z, 0.0, 0.0, yaw)
            nx, ny = (0.0, sgn) if axis == "y" else (sgn, 0.0)
            tx, ty = x + 0.3 * nx, y + 0.3 * ny
            break
        else:
            return scene_random(rng, counts)
        pose = None
        for _ in range(100):
            d, a = rng.uniform(1.0, 5.5), rng.uniform(-0.9, 0.9)
            ang = math.atan2(ny, nx) + a
            rx, ry = x + d * math.cos(ang), y + d * math.sin(ang)
            if inside_building(rx, ry, 0.5) and point_clear(rx, ry, STRUCTURE, 0.4) and \
                    not seg_hits(rx, ry, tx, ty, WALL_BOXES):
                pose = (rx, ry, math.atan2(y - ry, x - rx) + rng.uniform(-0.3, 0.3))
                break
    if pose is None:
        return scene_random(rng, counts)
    rx, ry, yaw = pose
    for n in pick_balanced(rng, POSITIVE, counts, rng.randint(0, 2)):
        sc.place_in_view(n, rx, ry, yaw)
    return sc, (rx, ry, yaw)


def scene_empty(rng, counts):
    sc = Scene(rng, canonical=False)
    return sc, sc.free_robot_pose()


def route_poses():
    poses = []
    if os.path.exists(NAV_TRACE):
        for leg in json.load(open(NAV_TRACE))["legs"]:
            tr = leg.get("map_trace", [])
            for i in range(0, len(tr) - 3, 3):
                (x0, y0), (x1, y1) = tr[i], tr[i + 3]
                if math.hypot(x1 - x0, y1 - y0) > 0.05:
                    poses.append((x0, y0, math.atan2(y1 - y0, x1 - x0)))
    return poses


ROUTE_POSES = route_poses()
PHASE1_VIEWPOINTS = [(6.0, -7.3, -math.pi / 2), (5.0, -9.0, math.pi), (5.0, -9.0, -math.pi / 2), (0.5, -10.8, 0.0),
                     (0.0, -11.0, math.pi / 2), (-5.0, 7.0, 2.36), (-7.0, 9.5, math.pi / 2), (-8.0, 3.0, math.pi),
                     (-3.0, -5.0, math.pi), (4.0, -5.0, 0.0)]


def scene_canonical(rng, counts, index):
    sc = Scene(rng, canonical=True)
    # the two V2.6 dynamic obstacles patrol in reality: place them in view or leave at their V2.6 pose
    if index < len(PHASE1_VIEWPOINTS):
        rx, ry, yaw = PHASE1_VIEWPOINTS[index]
    elif rng.random() < 0.7 and ROUTE_POSES:
        rx, ry, yaw = rng.choice(ROUTE_POSES)
        yaw += rng.uniform(-0.25, 0.25)
    else:
        rx, ry, yaw = sc.free_robot_pose()
    for n in ("dynamic_person", "dynamic_hospital_trolley"):
        if rng.random() < 0.5:
            sc.occupied = [b for b in sc.occupied if b[0] != n]
            if not sc.place_in_view(n, rx, ry, yaw, dmin=1.5, dmax=6.0):
                p = OBJ[n]["canonical_pose"]
                sc.poses[n] = tuple(p)
    return sc, (rx, ry, yaw)


# ---- challenge subsets (evaluation only) ----
def s_pool(sc, cid):
    return [n for n in by_class(cid) if sc.poses[n][2] == PARK_Z]


def subset_scene(rng, key, i):
    sc = Scene(rng, canonical=True)
    positive = i % 2 == 0  # alternate "confuser present" / "negative only" frames where relevant
    if key == "A_hospital_beds":
        beds = by_class(1)
        b = rng.choice(beds)
        if OBJ[b]["kind"] == "pool" or rng.random() < 0.5:
            b = rng.choice(s_pool(sc, 1))
            rx, ry, yaw = sc.free_robot_pose()
            if not sc.place_in_view(b, rx, ry, yaw, dmin=1.5, dmax=6.0):
                return subset_scene(rng, key, i)
            return sc, (rx, ry, yaw)
        tx, ty = sc.poses[b][:2]
        return sc, sc.robot_facing(tx, ty, 1.8, 6.0) or sc.free_robot_pose()
    targets = {"B_bed_vs_lab_bench": ["lab_bench_1", "lab_bench_2", "room_a_assembly_bench"],
               "C_bed_vs_reception_desk": ["reception_desk", "office_desk_1", "office_desk_2"]}
    if key in targets:
        tname = rng.choice(targets[key])
        t = next(b for b in FURN_BOXES if b[0] == tname)
        tx, ty = (t[1] + t[3]) / 2, (t[2] + t[4]) / 2
        if positive:
            sc.place_near(rng.choice(s_pool(sc, 1)), tx, ty, 0.8, 2.2)
        pose = sc.robot_facing(tx, ty, 1.5, 4.5, skip=(t[0],))
        return sc, pose or sc.free_robot_pose()
    rx, ry, yaw = sc.free_robot_pose()
    if key == "D_chair_vs_cart":
        for n in rng.sample([n for n in NEGATIVE if model_of(n) == "Chair"], rng.randint(2, 4)):
            sc.place_in_view(n, rx, ry, yaw, dmin=1.2, dmax=4.5, spread=0.4)
        if positive:
            sc.place_in_view(rng.choice(s_pool(sc, 2)), rx, ry, yaw, dmin=1.2, dmax=4.5, spread=0.4)
    elif key == "E_trolley_vs_forklift_cart":
        for n in ("dynamic_forklift", "dynamic_warehouse_cart"):
            sc.occupied = [b for b in sc.occupied if b[0] != n]
            sc.place_in_view(n, rx, ry, yaw, dmin=1.5, dmax=5.0, spread=0.4)
        if positive:
            sc.place_in_view(rng.choice(s_pool(sc, 5)), rx, ry, yaw, dmin=1.2, dmax=4.5, spread=0.4)
    elif key == "F_wheelchair":
        sc.place_in_view(rng.choice(s_pool(sc, 3)), rx, ry, yaw, dmin=1.2, dmax=5.5, spread=0.4)
        for n in rng.sample([n for n in NEGATIVE if model_of(n) == "Chair"], rng.randint(0, 3)):
            sc.place_in_view(n, rx, ry, yaw, dmin=1.2, dmax=5.0)
    elif key == "G_iv_stand":
        p = rng.choice([b for b in WALL_BOXES if b[0].startswith("pillar")])
        tx, ty = (p[1] + p[3]) / 2, (p[2] + p[4]) / 2
        if positive or rng.random() < 0.4:
            sc.place_near(rng.choice(s_pool(sc, 4)), tx, ty, 0.6, 1.6)
        pose = sc.robot_facing(tx, ty, 1.5, 5.0, skip=(p[0],))
        return sc, pose or (rx, ry, yaw)
    elif key == "H_person":
        for n in rng.sample(s_pool(sc, 0) + ["dynamic_person"], rng.randint(1, 3)):
            sc.occupied = [b for b in sc.occupied if b[0] != n]
            sc.place_in_view(n, rx, ry, yaw, dmin=1.2, dmax=7.0)
    elif key == "I_empty_hallway":
        for n in MOVABLE:  # nothing movable anywhere near the camera
            p = OBJ[n]["canonical_pose"]
            if math.hypot(p[0] - rx, p[1] - ry) < 8.0:
                sc.poses[n] = (p[0], p[1], PARK_Z, 0.0, 0.0, 0.0)
    elif key == "J_cluttered_corridor":
        for n in rng.sample([n for n in MOVABLE if sc.poses[n][2] == PARK_Z], rng.randint(6, 9)):
            sc.place_in_view(n, rx, ry, yaw, dmin=1.0, dmax=5.0, spread=0.5)
    return sc, (rx, ry, yaw)


# class that must be visible (kept) in a subset frame: every frame for A/F/H, even-index
# ("confuser present") frames for B/C/D/E/G. Retried up to 25 times; failures are recorded.
SUBSET_REQUIRED = {"A_hospital_beds": 1, "B_bed_vs_lab_bench": 1, "C_bed_vs_reception_desk": 1, "D_chair_vs_cart": 2,
                   "E_trolley_vs_forklift_cart": 5, "F_wheelchair": 3, "G_iv_stand": 4, "H_person": 0}

SUBSETS = ["A_hospital_beds", "B_bed_vs_lab_bench", "C_bed_vs_reception_desk", "D_chair_vs_cart",
           "E_trolley_vs_forklift_cart", "F_wheelchair", "G_iv_stand", "H_person", "I_empty_hallway",
           "J_cluttered_corridor"]


# ---------------------------------------------------------------- capture / write
SYNC_STATS = {"frames": 0, "retries": 0, "discarded": 0, "fail_unexplained": 0, "fail_unverified": 0}
FAIL_LOG = []


def render(gz, sc, robot, light, rng, calib=None, max_tries=5):
    """Apply the scene, then capture a fresh, settled frame (>= 0.5 s sim time after the last frame
    seen before the change, two consecutive identical segmentations) whose labels pass the independent
    projection check against the scene metadata and contain no instance unexplained by it.
    Gazebo acknowledged pose changes of static models that were not yet rendered under load
    (see V3_DATASET_GENERATION_REPORT.md); such frames are retried, then discarded (returns None)."""
    rx, ry, yaw = robot
    poses = dict(sc.poses)
    poses[RIG] = rig_pose(rx, ry, yaw, CAM_PITCH + rng.uniform(-0.02, 0.02))
    placed = {n: [round(v, 3) for v in p] for n, p in poses.items() if n != RIG and p[2] != PARK_Z}
    SYNC_STATS["frames"] += 1
    for attempt in range(max_tries):
        apply_lighting(gz, light)
        before = gz.latest_stamp()
        if not gz.set_poses(poses):
            raise RuntimeError("set_pose_vector failed")
        rgb, boxes, seg, stamp = gz.capture_settled(before + 0.5)
        if calib is None:
            return rgb, boxes, seg, stamp, poses[RIG]
        inst = gt.assign_objects(gt.extract_instances(seg), placed, poses[RIG])
        unexplained = [i for i in inst if i.get("object") is None and i["visible_px"] >= gt.MIN_VISIBLE_PX]
        kept = [gt.decide(dict(i), calib) for i in inst]
        kept = [{"class_id": i["class_id"], "xyxy": i["xyxy"]} for i in kept if i["status"] == "kept"]
        ver = gt.verify_boxes(kept, placed, poses[RIG])
        if not unexplained and "UNVERIFIED" not in ver:
            return rgb, boxes, seg, stamp, poses[RIG]
        SYNC_STATS["fail_unexplained"] += bool(unexplained)
        SYNC_STATS["fail_unverified"] += "UNVERIFIED" in ver
        if len(FAIL_LOG) < 200:
            FAIL_LOG.append({"rig": [round(v, 3) for v in poses[RIG]],
                             "unexplained": [{"class": gt.NAMES[u["class_id"]], "xyxy": u["xyxy"], "px": u["visible_px"]} for u in unexplained],
                             "unverified": [{"class": gt.NAMES[k["class_id"]], "xyxy": k["xyxy"]} for k, v in zip(kept, ver) if v == "UNVERIFIED"]})
        SYNC_STATS["retries"] += 1
    SYNC_STATS["discarded"] += 1
    return None


def build_labels(boxes, seg, placed, rig, calib):
    inst = gt.assign_objects(gt.extract_instances(seg), placed, rig)
    inst = [gt.decide(i, calib) for i in inst]
    bbox_cam = {(int(b.label) - 1, round(b.box.min_corner.x), round(b.box.min_corner.y),
                 round(b.box.max_corner.x), round(b.box.max_corner.y)) for b in boxes.annotated_box}
    for i in inst:  # cross-check against Gazebo's boundingbox_camera (tolerance 1 px)
        x0, y0, x1, y1 = i["xyxy"]
        same = [(max(abs(a - x0), abs(b - y0), abs(cc - x1), abs(d - y1)), (a, b, cc, d))
                for c, a, b, cc, d in bbox_cam if c == i["class_id"]]
        err, nearest = min(same) if same else (None, None)
        i["bbox_cam_match"] = err is not None and err <= 1
        if not i["bbox_cam_match"]:
            i["bbox_cam_nearest"], i["bbox_cam_max_err_px"] = nearest, err
    return inst


def write_frame(out_dir, split, idx, rgb, inst, meta, counts):
    img_dir, lbl_dir = os.path.join(out_dir, "images", split), os.path.join(out_dir, "labels", split)
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    stem = f"{split}_{idx:05d}"
    cv2.imwrite(os.path.join(img_dir, stem + ".jpg"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
    h, w = rgb.shape[:2]
    lines = []
    for i in inst:
        if i["status"] != "kept":
            continue
        x0, y0, x1, y1 = i["xyxy"]
        counts[i["class_id"]] = counts.get(i["class_id"], 0) + 1
        lines.append(f"{i['class_id']} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} {(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}")
    with open(os.path.join(lbl_dir, stem + ".txt"), "w") as f:
        f.write("\n".join(lines) + ("\n" if lines else ""))
    meta.update({"image": f"images/{split}/{stem}.jpg",
                 "boxes": [{"class_id": i["class_id"], "xyxy": i["xyxy"], "object": i.get("object"),
                            "visible_px": i["visible_px"], "vis_est": i.get("vis_est")} for i in inst if i["status"] == "kept"],
                 "dropped_instances": [{k: i.get(k) for k in ("class_id", "xyxy", "object", "visible_px", "vis_est", "status")}
                                       for i in inst if i["status"] != "kept"],
                 "bbox_cam_mismatch": sum(1 for i in inst if not i["bbox_cam_match"]),
                 "bbox_cam_mismatches": [{k: i.get(k) for k in ("class_id", "xyxy", "visible_px", "status",
                                                                "bbox_cam_nearest", "bbox_cam_max_err_px")}
                                         for i in inst if not i["bbox_cam_match"]]})
    return meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", required=True, choices=["train", "val", "test", "subsets"])
    ap.add_argument("--count", type=int, required=True, help="frames (per subset for --split subsets)")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--start", type=int, default=0, help="resume index")
    ap.add_argument("--end", type=int, default=None, help="stop before this index (chunked runs)")
    ap.add_argument("--out", default=OUT_ROOT)
    ap.add_argument("--only-subset", default=None, help="generate a single challenge subset")
    a = ap.parse_args()
    rng = random.Random(a.seed)
    calib = gt.load_calibration()
    if calib is None:
        sys.exit("run calibrate_visibility.py first")
    gz = GzScene()
    t0 = time.time()
    while gz.sim_time < 1.0 and time.time() - t0 < 60:
        time.sleep(0.2)
    counts = {}
    jobs = []
    if a.split == "subsets":
        for key in SUBSETS:
            if a.only_subset and key != a.only_subset:
                continue
            jobs += [(key, i) for i in range(a.count)]
    else:
        jobs = [(a.split, i) for i in range(a.count)]
    for key, i in jobs:
        rng.seed(f"{a.seed}-{key}-{i}")  # per-frame determinism (resumable)
        if a.split == "subsets":
            out_dir, split = os.path.join(a.out, "subsets", key), "test"
        else:
            out_dir, split = a.out, a.split
        meta_path = os.path.join(out_dir, "meta", f"{split}.jsonl")
        if i < a.start or (a.end is not None and i >= a.end):
            continue
        if a.split in ("train", "val"):
            r = rng.random()
            mode = ("random" if r < 0.45 else "hard_negative" if r < 0.65 else "sign_marker" if r < 0.80
                    else "empty" if r < 0.90 else "clutter")
            sc, robot = {"random": lambda: scene_random(rng, counts), "hard_negative": lambda: scene_hard_negative(rng, counts),
                         "sign_marker": lambda: scene_sign_marker(rng, counts), "empty": lambda: scene_empty(rng, counts),
                         "clutter": lambda: scene_random(rng, counts, clutter=True)}[mode]()
            light = lighting(rng, "random")
        elif a.split == "test":
            mode = "canonical"
            sc, robot = scene_canonical(rng, counts, i)
            lmode = "default" if (i < len(PHASE1_VIEWPOINTS) or rng.random() < 0.8) else rng.choice(["dim", "bright"])
            light = lighting(rng, lmode)
        else:
            mode = key
        attempts = 0
        while True:
            attempts += 1
            if a.split == "subsets":
                sc, robot = subset_scene(rng, key, i)
                light = lighting(rng, "default" if rng.random() < 0.7 else rng.choice(["dim", "bright"]))
            r = render(gz, sc, robot, light, rng, calib)
            if r is None:  # could not obtain a verified frame for this scene: resample it
                if a.split in ("train", "val", "test"):
                    rng.seed(f"{a.seed}-{key}-{i}-resample{attempts}")
                    if a.split in ("train", "val"):
                        sc, robot = scene_random(rng, counts)
                        mode = f"{mode.split('+')[0]}+resampled_random"
                    else:
                        sc, robot = scene_canonical(rng, counts, 10**6 + i)
                if attempts >= 25:
                    raise RuntimeError(f"no verified frame for {key} {i}")
                continue
            rgb, boxes, seg, stamp, rig = r
            placed = {n: [round(v, 3) for v in p] for n, p in sc.poses.items() if p[2] != PARK_Z}
            inst = build_labels(boxes, seg, placed, rig, calib)
            need = SUBSET_REQUIRED.get(key) if (a.split == "subsets" and i % 2 == 0) else None
            if key == "A_hospital_beds" or key == "F_wheelchair" or key == "H_person":
                need = SUBSET_REQUIRED[key]
            kept_classes = {x["class_id"] for x in inst if x["status"] == "kept"}
            if need is None or need in kept_classes or attempts >= 25:
                break
        meta = write_frame(out_dir, split, i, rgb, inst,
                           {"index": i, "mode": mode, "seed": a.seed, "sim_stamp": stamp, "attempts": attempts,
                            "target_class_present": (need in kept_classes) if need is not None else None,
                            "robot_pose": [round(v, 3) for v in robot], "rig_pose": [round(v, 4) for v in rig],
                            "lighting": light, "placed_objects": placed}, counts)
        os.makedirs(os.path.dirname(meta_path), exist_ok=True)
        with open(meta_path, "a") as f:
            f.write(json.dumps(meta) + "\n")
        if i % 50 == 0:
            print(f"{key} {i}/{a.count} {time.time() - t0:.0f}s counts={counts} sync={SYNC_STATS}", flush=True)
    print("DONE", a.split, dict(sorted(counts.items())), f"{time.time() - t0:.0f}s", "sync", SYNC_STATS, flush=True)
    stats_path = os.path.join(a.out, "meta", f"sync_stats_{a.split}.jsonl")
    os.makedirs(os.path.dirname(stats_path), exist_ok=True)
    with open(stats_path, "a") as f:  # one line per (chunked) run
        f.write(json.dumps({"start": a.start, "end": a.end, **SYNC_STATS}) + "\n")
    if FAIL_LOG:
        with open(os.path.join(a.out, "meta", f"sync_failures_{a.split}.json"), "w") as f:
            json.dump(FAIL_LOG, f, indent=1)
    gz.close()
    os._exit(0)  # gz-transport Python binding segfaults during interpreter teardown


if __name__ == "__main__":
    main()
