#!/usr/bin/env python3
"""
Build the V3 dataset-rendering world from V3/worlds/hospital_logistics_world.sdf.

Changes (dataset world only; the V3 runtime world is not modified):
  - Gazebo Label system plugin (<label>class_id+1</label>) on every model of a V3 class
    (V3/perception/v3_classes.yaml). Unlabelled models are background / hard negatives.
  - dynamic_* models made static (no VelocityControl): they are repositioned by set_pose instead.
  - an instance pool of extra props, parked below the floor (z = -30) until a frame places them.
  - camera rig 'v3ds_camera_rig': an RGB camera, a boundingbox_camera (2D visible boxes) and an
    instance segmentation camera sharing one link, with the robot camera's intrinsics (640x480, hfov 1.15, clip 0.08-25 m,
    gaussian noise 0.005). The rig is posed at the robot camera pose (base + 0.22 m fwd, 0.41 m
    AGL, pitch -0.05) by the generator.
    plus a level gpu_lidar ('lidar_link', /v3ds/scan) at the robot LiDAR's mount relative to the
    camera, identical to the robot LiDAR spec (720 samples, 360 deg, 0.12-12 m, noise 0.01).
No robot is spawned; the rig replaces it for rendering.
Output: V3/worlds/hospital_logistics_dataset_world.sdf and V3/datasets/dataset_world_objects.json
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(V3_ROOT, "worlds", "hospital_logistics_world.sdf")
OUT = os.path.join(V3_ROOT, "worlds", "hospital_logistics_dataset_world.sdf")
OBJ_OUT = os.path.join(V3_ROOT, "datasets", "dataset_world_objects.json")
sys.path.insert(0, os.path.join(V3_ROOT, "scripts"))
from build_v3_hospital_world import MODELS_DIR, model_collision_bounds  # noqa: E402

CLASSES = yaml.safe_load(open(os.path.join(V3_ROOT, "perception", "v3_classes.yaml")))["classes"]

PARK_Z = -30.0
# Extra instances (model, count). Labelled pool adds instance multiplicity; the unlabelled pool
# provides movable hard negatives (chairs vs cart/wheelchair, BP monitor vs sign/iv_stand, ...).
POOL = [
    ("MalePatientBed", 1), ("TrolleyBed", 2), ("PatientWheelChair", 2), ("IVStand", 3),
    ("BPCart", 1), ("InstrumentCart1", 2), ("SurgicalTrolley", 2),
    ("Scrubs", 2), ("FemaleVisitor", 2), ("MaleVisitorOnPhone", 1),
    ("Chair", 4), ("BloodPressureMonitor", 2), ("MetalCabinet", 1), ("Drawer", 2), ("AdjTable", 1),
    ("StorageRack", 1),
]


def label_for(model_name=None, world_name=None):
    for c in CLASSES:
        if model_name and model_name in c.get("models", []):
            return c["id"]
        if world_name and (world_name in c.get("world_models", [])
                           or (c.get("world_model_prefix") and world_name.startswith(c["world_model_prefix"]))):
            return c["id"]
    return None


def obj_bounds(path, scale=(1, 1, 1), offset=(0, 0, 0)):
    mn, mx = [1e9] * 3, [-1e9] * 3
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                v = [float(t) for t in line.split()[1:4]]
                for i in range(3):
                    mn[i] = min(mn[i], v[i] * scale[i] + offset[i])
                    mx[i] = max(mx[i], v[i] * scale[i] + offset[i])
    return mn, mx


def model_visual_aabb(model):
    """Local AABB (model frame, incl. link pose yaw of 0/90 deg) of all visual meshes of a vendored model."""
    root = ET.parse(os.path.join(MODELS_DIR, model, "model.sdf")).getroot()
    link = root.find(".//link")
    lyaw = float((link.findtext("pose") or "0 0 0 0 0 0").split()[5])
    mn, mx = [1e9] * 3, [-1e9] * 3
    for vis in root.iter("visual"):
        uri = vis.findtext(".//mesh/uri")
        scale = [float(v) for v in (vis.findtext(".//mesh/scale") or "1 1 1").split()]
        a, b = obj_bounds(os.path.join(MODELS_DIR, model, uri), scale)
        for i in range(3):
            mn[i], mx[i] = min(mn[i], a[i]), max(mx[i], b[i])
    if abs(abs(lyaw) - 1.5708) < 0.01:  # 90 deg link yaw swaps x/y extents
        mn[0], mx[0], mn[1], mx[1] = -mx[1], -mn[1], mn[0], mx[0]
    return mn, mx


def inline_visual_aabb(model):
    """Local AABB of an inline model's first visual (box / cylinder / mesh with pose offset)."""
    vis = model.find(".//visual")
    off = [float(v) for v in (vis.findtext("pose") or "0 0 0 0 0 0").split()[:3]]
    box = vis.findtext(".//box/size")
    if box:
        sx, sy, sz = map(float, box.split())
        return [off[0] - sx / 2, off[1] - sy / 2, off[2] - sz / 2], [off[0] + sx / 2, off[1] + sy / 2, off[2] + sz / 2]
    uri = vis.findtext(".//mesh/uri")
    if uri:
        mdl, rel = uri.split("model://")[1].split("/", 1)
        return obj_bounds(os.path.join(MODELS_DIR, mdl, rel), offset=off)
    if vis.find(".//cylinder") is None:
        return None  # plane etc.: not a labelled/movable object
    r = float(vis.findtext(".//cylinder/radius"))
    h = float(vis.findtext(".//cylinder/length"))
    return [off[0] - r, off[1] - r, off[2] - h / 2], [off[0] + r, off[1] + r, off[2] + h / 2]


def inline_footprint(model):
    col = model.find(".//collision")
    if col is None:
        return None
    box = col.findtext(".//box/size")
    if box:
        sx, sy = map(float, box.split()[:2])
        return [(-sx / 2, -sy / 2), (sx / 2, sy / 2)]
    r = col.findtext(".//cylinder/radius")
    if r:
        r = float(r)
        return [(-r, -r), (r, r)]
    return None


def prop_footprint(model):
    pts, _ = model_collision_bounds(model)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return [(min(xs), min(ys)), (max(xs), max(ys))]


def label_plugin(class_id):
    p = ET.Element("plugin", {"filename": "gz-sim-label-system", "name": "gz::sim::systems::Label"})
    ET.SubElement(p, "label").text = str(class_id + 1)
    return p


def camera_block(parent, sensor_type, name, topic):
    s = ET.SubElement(parent, "sensor", {"name": name, "type": sensor_type})
    ET.SubElement(s, "always_on").text = "true"
    ET.SubElement(s, "update_rate").text = "10"
    ET.SubElement(s, "topic").text = topic
    cam = ET.SubElement(s, "camera")
    ET.SubElement(cam, "horizontal_fov").text = "1.15"
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = "640"
    ET.SubElement(img, "height").text = "480"
    if sensor_type == "camera":
        ET.SubElement(img, "format").text = "R8G8B8"
        noise = ET.SubElement(cam, "noise")
        ET.SubElement(noise, "type").text = "gaussian"
        ET.SubElement(noise, "mean").text = "0.0"
        ET.SubElement(noise, "stddev").text = "0.005"
    elif sensor_type == "boundingbox_camera":
        ET.SubElement(cam, "box_type").text = "2d"
    else:  # segmentation: per-pixel instance ids -> visible-pixel ground truth
        ET.SubElement(cam, "segmentation_type").text = "instance"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = "0.08"
    ET.SubElement(clip, "far").text = "25.0"


def main():
    tree = ET.parse(SRC)
    world = tree.getroot().find("world")
    objects = []

    for model in world.findall("model"):
        name = model.get("name")
        if name.startswith("dynamic_"):
            for pl in model.findall("plugin"):
                model.remove(pl)
            st = model.find("static")
            if st is None:
                st = ET.SubElement(model, "static")
            st.text = "true"
        cid = label_for(world_name=name)
        if cid is not None:
            model.append(label_plugin(cid))
        pose = [float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()]
        entry = {"name": name, "kind": "inline", "class_id": cid, "canonical_pose": pose,
                 "footprint": inline_footprint(model)}
        if model.find(".//visual") is not None:
            entry["visual_aabb"] = inline_visual_aabb(model)
        objects.append(entry)

    for inc in world.findall("include"):
        model = inc.findtext("uri").split("model://")[1]
        name = inc.findtext("name")
        cid = label_for(model_name=model)
        if cid is not None:
            inc.append(label_plugin(cid))
        pose = [float(v) for v in inc.findtext("pose").split()]
        objects.append({"name": name, "kind": "prop", "model": model, "class_id": cid, "canonical_pose": pose,
                        "footprint": prop_footprint(model), "visual_aabb": model_visual_aabb(model)})

    for model, count in POOL:
        for i in range(count):
            name = f"pool_{model}_{i}"
            inc = ET.SubElement(world, "include")
            ET.SubElement(inc, "uri").text = f"model://{model}"
            ET.SubElement(inc, "name").text = name
            ET.SubElement(inc, "pose").text = f"{len(objects) * 3.0:.1f} 0 {PARK_Z} 0 0 0"
            ET.SubElement(inc, "static").text = "true"
            cid = label_for(model_name=model)
            if cid is not None:
                inc.append(label_plugin(cid))
            objects.append({"name": name, "kind": "pool", "model": model, "class_id": cid,
                            "canonical_pose": [len(objects) * 3.0, 0, PARK_Z, 0, 0, 0],
                            "footprint": prop_footprint(model), "visual_aabb": model_visual_aabb(model)})

    rig = ET.SubElement(world, "model", {"name": "v3ds_camera_rig"})
    ET.SubElement(rig, "static").text = "true"
    ET.SubElement(rig, "pose").text = f"0 -10.78 0.41 0 -0.05 1.5708"
    link = ET.SubElement(rig, "link", {"name": "link"})
    camera_block(link, "camera", "rgb", "/v3ds/image")
    camera_block(link, "boundingbox_camera", "boxes", "/v3ds/boxes")
    camera_block(link, "segmentation", "seg", "/v3ds/seg")
    # LiDAR at the robot's LiDAR mount relative to the camera (camera is 0.12 m ahead and 0.175 m
    # above the LiDAR; the robot LiDAR is level): offset expressed in the rig frame (pitch -0.05).
    lidar = ET.SubElement(rig, "link", {"name": "lidar_link"})
    p0 = 0.05
    dx, dz = -0.12, -0.175
    ET.SubElement(lidar, "pose").text = (f"{dx * math.cos(p0) + dz * math.sin(p0):.4f} 0 "
                                         f"{-dx * math.sin(p0) + dz * math.cos(p0):.4f} 0 {p0} 0")
    ls = ET.SubElement(lidar, "sensor", {"name": "lidar", "type": "gpu_lidar"})
    for tag, val in (("always_on", "true"), ("update_rate", "10"), ("topic", "/v3ds/scan")):
        ET.SubElement(ls, tag).text = val
    ld = ET.SubElement(ls, "lidar")
    hz = ET.SubElement(ET.SubElement(ld, "scan"), "horizontal")
    for tag, val in (("samples", "720"), ("resolution", "1"), ("min_angle", "-3.14159"), ("max_angle", "3.14159")):
        ET.SubElement(hz, tag).text = val
    rg = ET.SubElement(ld, "range")
    for tag, val in (("min", "0.12"), ("max", "12.0"), ("resolution", "0.01")):
        ET.SubElement(rg, tag).text = val
    nz = ET.SubElement(ld, "noise")
    for tag, val in (("type", "gaussian"), ("mean", "0.0"), ("stddev", "0.01")):
        ET.SubElement(nz, tag).text = val

    lights = []
    for light in world.findall("light"):
        if light.get("type") != "point":
            continue
        att = light.find("attenuation")
        lights.append({"name": light.get("name"),
                       "pose": [float(v) for v in light.findtext("pose").split()],
                       "diffuse": [float(v) for v in light.findtext("diffuse").split()[:3]],
                       "specular": [float(v) for v in light.findtext("specular").split()[:3]],
                       "range": float(att.findtext("range")), "constant": float(att.findtext("constant")),
                       "linear": float(att.findtext("linear")), "quadratic": float(att.findtext("quadratic"))})

    ET.indent(tree, space="  ")
    tree.write(OUT, xml_declaration=True, encoding="unicode")
    json.dump({"park_z": PARK_Z, "lights": lights, "objects": objects}, open(OBJ_OUT, "w"), indent=2)
    n_lab = sum(o["class_id"] is not None for o in objects)
    print(f"{OUT}\n{len(objects)} objects ({n_lab} labelled, {sum(o['kind'] == 'pool' for o in objects)} pool)")


if __name__ == "__main__":
    main()
