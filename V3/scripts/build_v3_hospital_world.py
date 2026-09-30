#!/usr/bin/env python3
"""
V3 Hospital Logistics world builder.

Derives the V3 world from the V2.6 facility builder (scripts/build_realistic_facility.py) without
modifying it or its outputs:
  1. V2.6 FURNITURE minus the placeholder boxes listed in REPLACED_PLACEHOLDERS
  2. + realistic hospital props (V3/models, OpenRobotics Fuel, CC-BY 4.0) from PROPS
  3. + logistics markers (collection point, laboratory test point, payload carrier)
  4. dynamic_person / dynamic_hospital_trolley get realistic visual meshes; their V2.6 collision
     shapes, poses and VelocityControl plugins are unchanged.

The world name stays 'realistic_facility_world' so the V2.6 ros_gz_bridge config (clock topic) applies.

Outputs (V3 only):
  V3/worlds/hospital_logistics_world.sdf
  V3/maps/hospital_logistics_map.{pgm,yaml}
  V3/worlds/hospital_logistics_props.json   (placed props with world-frame collision footprints)
  V3/models/v3_markers/*.png                (marker plaque textures)
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import cv2
from PIL import Image, ImageDraw, ImageFont

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, PROJECT_ROOT)

import scripts.build_realistic_facility as v26  # noqa: E402  (read-only reuse)

MODELS_DIR = os.path.join(V3_ROOT, "models")
WORLD_OUT = os.path.join(V3_ROOT, "worlds", "hospital_logistics_world.sdf")
PROPS_OUT = os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")
MAP_PGM = os.path.join(V3_ROOT, "maps", "hospital_logistics_map.pgm")
MAP_YAML = os.path.join(V3_ROOT, "maps", "hospital_logistics_map.yaml")
MARKER_DIR = os.path.join(MODELS_DIR, "v3_markers")

# Logistics locations. Both reuse V2.6 topology destination nodes that were already validated:
#   'Hospital Medical Ward' (5.0, -9.0) and 'Research Laboratory' (-7.0, 9.5).
COLLECTION_POINT = {"name": "collection_point", "x": 5.0, "y": -9.0, "yaw": math.pi, "v26_node": "Hospital Medical Ward"}
TEST_POINT = {"name": "laboratory_test_point", "x": -7.0, "y": 9.5, "yaw": math.pi / 2, "v26_node": "Research Laboratory"}

REPLACED_PLACEHOLDERS = {
    "hospital_bed_1": "v3_patient_bed_1",
    "hospital_bed_2": "v3_trolley_bed_2",
    "bedside_table_1": "v3_bedside_drawer_1",
    "bedside_table_2": "v3_bedside_drawer_2",
    "waiting_bench_w": "v3_waiting_chair_w1/w2",
    "waiting_bench_e": "v3_waiting_chair_e1/e2",
    "storage_box_stack": "v3_storage_rack",
}

# name, fuel model, x, y, yaw, area, static/dynamic role, CV purpose
PROPS = [
    # --- Hospital ward / COLLECTION POINT ---
    ("v3_patient_bed_1", "MalePatientBed", 7.45, -8.8, 0.0, "ward", "bed with patient; large static obstacle"),
    ("v3_trolley_bed_2", "TrolleyBed", 7.5, -11.0, 0.0, "ward", "hospital bed/trolley bed; static obstacle"),
    ("v3_bedside_drawer_1", "Drawer", 8.95, -8.8, 0.0, "ward", "bedside cabinet; low clutter"),
    ("v3_bedside_drawer_2", "Drawer", 8.95, -11.0, 0.0, "ward", "bedside cabinet; low clutter"),
    ("v3_iv_stand_ward", "IVStand", 9.1, -9.9, 0.0, "ward", "IV stand; thin-pole LiDAR case"),
    ("v3_collection_table", "AdjTable", 4.1, -10.6, math.pi / 2, "ward", "sample collection station table"),
    ("v3_collection_bp_monitor", "BloodPressureMonitor", 4.0, -8.2, 0.0, "ward", "medical device landmark at collection station"),
    ("v3_ward_bp_cart", "BPCart", 5.6, -12.2, 0.0, "ward", "medical cart"),
    ("v3_ward_nurse", "Scrubs", 4.3, -12.2, math.pi / 2, "ward", "static person (nurse)"),
    # --- Reception ---
    ("v3_waiting_chair_w1", "Chair", -2.95, -10.0, 0.0, "reception", "hospital chair"),
    ("v3_waiting_chair_w2", "Chair", -2.30, -10.0, 0.0, "reception", "hospital chair"),
    ("v3_waiting_chair_e1", "Chair", 2.40, -9.9, 0.0, "reception", "hospital chair"),
    ("v3_waiting_chair_e2", "Chair", 3.05, -9.9, 0.0, "reception", "hospital chair"),
    ("v3_reception_nurse", "Scrubs", -2.2, -7.4, -math.pi / 2, "reception", "static person behind reception desk"),
    ("v3_visitor_phone", "MaleVisitorOnPhone", 2.9, -10.8, math.pi, "reception", "static person (visitor)"),
    ("v3_parked_wheelchair", "PatientWheelChair", 2.8, -11.9, 0.0, "reception", "wheelchair"),
    # --- Laboratory / TEST POINT ---
    ("v3_lab_instrument_cart", "InstrumentCart1", -9.0, 12.3, 0.0, "lab", "laboratory instrument cart"),
    ("v3_lab_cabinet", "MetalCabinet", -8.0, 12.55, 0.0, "lab", "medical/lab cabinet"),
    ("v3_lab_surgical_trolley", "SurgicalTrolley", -10.1, 12.4, 0.0, "lab", "medical trolley"),
    # --- Storage ---
    ("v3_storage_rack", "StorageRack", -10.0, 3.0, 0.0, "storage", "medical supply storage rack"),
]

# Realistic visuals for the V2.6 dynamic obstacles: model -> (fuel model, visual mesh, z offset to floor)
DYNAMIC_VISUALS = {
    "dynamic_person": ("Scrubs", "meshes/scrubs.obj", -0.85),
    "dynamic_hospital_trolley": ("SurgicalTrolley", "meshes/trolley.obj", -0.35),
}


def model_collision_bounds(fuel_name):
    """Collision-geometry XY bounds (model frame, including the link pose yaw) from the vendored SDF."""
    root = ET.parse(os.path.join(MODELS_DIR, fuel_name, "model.sdf")).getroot()
    link = root.find(".//link")
    lyaw = float((link.findtext("pose") or "0 0 0 0 0 0").split()[5])
    col = root.find(".//collision")
    box = col.find(".//box/size")
    pts = []
    if box is not None:
        sx, sy, sz = map(float, box.text.split())
        cx, cy, cz = map(float, (col.findtext("pose") or "0 0 0 0 0 0").split()[:3])
        pts = [(cx + dx * sx / 2, cy + dy * sy / 2) for dx in (-1, 1) for dy in (-1, 1)]
        height = sz
    else:
        uri = col.find(".//mesh/uri").text
        scale = [float(s) for s in (col.findtext(".//mesh/scale") or "1 1 1").split()]
        zmax = 0.0
        with open(os.path.join(MODELS_DIR, fuel_name, uri), errors="ignore") as f:
            for line in f:
                if line.startswith("v "):
                    v = [float(t) for t in line.split()[1:4]]
                    pts.append((v[0] * scale[0], v[1] * scale[1]))
                    zmax = max(zmax, v[2] * scale[2])
        height = zmax
    c, s = math.cos(lyaw), math.sin(lyaw)
    return [(c * x - s * y, s * x + c * y) for x, y in pts], height


def world_footprint(fuel_name, x, y, yaw):
    pts, height = model_collision_bounds(fuel_name)
    c, s = math.cos(yaw), math.sin(yaw)
    wx = [x + c * px - s * py for px, py in pts]
    wy = [y + s * px + c * py for px, py in pts]
    return {"xmin": min(wx), "xmax": max(wx), "ymin": min(wy), "ymax": max(wy), "height": height}


def make_plaque_texture(fname, title, subtitle, bg):
    os.makedirs(MARKER_DIR, exist_ok=True)
    img = Image.new("RGB", (800, 400), bg)
    d = ImageDraw.Draw(img)
    try:
        f1 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 96)
        f2 = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 54)
    except OSError:
        f1 = f2 = ImageFont.load_default()
    d.rectangle([12, 12, 787, 387], outline=(255, 255, 255), width=12)
    d.text((400, 160), title, font=f1, fill=(255, 255, 255), anchor="mm")
    d.text((400, 290), subtitle, font=f2, fill=(255, 255, 255), anchor="mm")
    path = os.path.join(MARKER_DIR, fname)
    img.save(path)
    return path


def plaque_sdf(name, x, y, z, yaw, texture):
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>0.8 0.03 0.4</size></box></geometry></collision>
        <visual name='visual'>
          <geometry><box><size>0.8 0.03 0.4</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient>
            <pbr><metal><albedo_map>{texture}</albedo_map><metalness>0</metalness><roughness>0.8</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def floor_pad_sdf(name, x, y, rgb):
    r, g, b = rgb
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.002 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>0.9 0.9 0.004</size></box></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def payload_sdf(x, y, z):
    # Logical payload marker: a closed blood-sample transport carrier sitting on the collection table.
    return f"""
    <model name='v3_payload_blood_sample_carrier'>
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>0.26 0.18 0.14</size></box></geometry></collision>
        <visual name='body'>
          <geometry><box><size>0.26 0.18 0.14</size></box></geometry>
          <material><ambient>0.85 0.1 0.1 1</ambient><diffuse>0.85 0.1 0.1 1</diffuse></material>
        </visual>
        <visual name='lid_band'>
          <pose>0 0 0.072 0 0 0</pose>
          <geometry><box><size>0.27 0.19 0.02</size></box></geometry>
          <material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def build():
    # 1. V2.6 world with placeholders removed
    kept = [f for f in v26.FURNITURE if f[0] not in REPLACED_PLACEHOLDERS]
    original_furniture = v26.FURNITURE
    v26.FURNITURE = kept
    try:
        sdf = v26.generate_sdf()
    finally:
        v26.FURNITURE = original_furniture

    # 2. dynamic obstacle visuals
    root = ET.fromstring(sdf)
    world = root.find("world")
    for model in world.findall("model"):
        spec = DYNAMIC_VISUALS.get(model.get("name"))
        if not spec:
            continue
        fuel, mesh, zoff = spec
        vis = model.find(".//visual")
        for child in list(vis):
            vis.remove(child)
        ET.SubElement(vis, "pose").text = f"0 0 {zoff} 0 0 0"
        geom = ET.SubElement(vis, "geometry")
        m = ET.SubElement(geom, "mesh")
        ET.SubElement(m, "uri").text = f"model://{fuel}/{mesh}"
    sdf = ET.tostring(root, encoding="unicode")

    # 3. props + markers
    placed = []
    blocks = []
    for name, fuel, x, y, yaw, area, purpose in PROPS:
        blocks.append(f"""
    <include>
      <uri>model://{fuel}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>
      <static>true</static>
    </include>""")
        fp = world_footprint(fuel, x, y, yaw)
        placed.append({"name": name, "model": fuel, "x": x, "y": y, "yaw": round(yaw, 4), "area": area,
                       "purpose": purpose, "dynamic": False, "footprint": {k: round(v, 3) for k, v in fp.items()}})

    tex_c = make_plaque_texture("collection_point.png", "COLLECTION", "SAMPLE PICKUP", (180, 30, 40))
    tex_l = make_plaque_texture("laboratory_test_point.png", "LABORATORY", "TEST POINT", (20, 90, 170))
    # collection plaque: on ward west wall (x=3.6 face) facing +x; centre z=0.85 so it is inside the camera
    # vertical FOV from the collection point (camera z=0.41, ~1.4 m from the wall)
    blocks.append(plaque_sdf("v3_sign_collection_point", 3.62, -9.6, 0.85, -math.pi / 2, tex_c))
    # laboratory plaque: standing on lab_bench_2 (top z=0.9), facing south toward the test point
    blocks.append(plaque_sdf("v3_sign_laboratory_test_point", -7.6, 10.95, 1.12, 0.0, tex_l))
    blocks.append(floor_pad_sdf("v3_floor_collection_point", COLLECTION_POINT["x"], COLLECTION_POINT["y"], (0.85, 0.2, 0.2)))
    blocks.append(floor_pad_sdf("v3_floor_laboratory_test_point", TEST_POINT["x"], TEST_POINT["y"], (0.2, 0.45, 0.85)))
    # payload carrier on top of the collection table (AdjTable top at z = 0.80)
    blocks.append(payload_sdf(4.1, -10.3, 0.80 + 0.07))
    placed += [
        {"name": "v3_sign_collection_point", "model": "plaque", "x": 3.62, "y": -9.6, "area": "ward", "dynamic": False,
         "purpose": "collection point landmark plaque",
         "footprint": {"xmin": 3.605, "xmax": 3.635, "ymin": -10.0, "ymax": -9.2, "height": 1.05}},
        {"name": "v3_sign_laboratory_test_point", "model": "plaque", "x": -7.6, "y": 10.95, "area": "lab", "dynamic": False,
         "purpose": "laboratory test point landmark plaque (on lab_bench_2)",
         "footprint": {"xmin": -8.0, "xmax": -7.2, "ymin": 10.935, "ymax": 10.965, "height": 1.32}},
        {"name": "v3_payload_blood_sample_carrier", "model": "box", "x": 4.1, "y": -10.3, "area": "ward", "dynamic": False,
         "purpose": "logical payload marker (on collection table)",
         "footprint": {"xmin": 3.97, "xmax": 4.23, "ymin": -10.39, "ymax": -10.21, "height": 0.94}},
    ]
    sdf = sdf.replace("</world>", "".join(blocks) + "\n  </world>")
    os.makedirs(os.path.dirname(WORLD_OUT), exist_ok=True)
    with open(WORLD_OUT, "w") as f:
        f.write("<?xml version='1.0'?>\n" + sdf if not sdf.startswith("<?xml") else sdf)

    # 4. occupancy map: V2.6 map policy (same corridor clearing) with V3 prop footprints
    map_furniture = list(kept)
    for p in placed:
        fp = p["footprint"]
        map_furniture.append((p["name"], (fp["xmin"] + fp["xmax"]) / 2, (fp["ymin"] + fp["ymax"]) / 2, 0, 0, 0, 0,
                              fp["xmax"] - fp["xmin"], fp["ymax"] - fp["ymin"], fp["height"], None))
    v26.FURNITURE = map_furniture
    try:
        grid = v26.generate_occupancy_map()
    finally:
        v26.FURNITURE = original_furniture
    # V2.6 clears whole room rectangles (ward, lab) after stamping furniture, leaving beds/benches to the
    # LiDAR obstacle layer. The 2D LiDAR (z=0.235 m) only sees the legs of the realistic beds, whose frame
    # underside is at 0.22-0.32 m (below the 0.41 m camera mast), so V3 re-stamps the static prop
    # collision footprints after the clearing. Only the global costmap (static_layer) uses this map.
    for p in placed:
        fp = p["footprint"]
        x1, y1 = v26.world_to_map(fp["xmin"], fp["ymin"])
        x2, y2 = v26.world_to_map(fp["xmax"], fp["ymax"])
        cv2.rectangle(grid, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), 0, -1)
    os.makedirs(os.path.dirname(MAP_PGM), exist_ok=True)
    cv2.imwrite(MAP_PGM, grid)
    with open(MAP_YAML, "w") as f:
        f.write(f"image: {os.path.basename(MAP_PGM)}\nmode: trinary\nresolution: {v26.RESOLUTION}\n"
                f"origin: [{v26.ORIGIN_X}, {v26.ORIGIN_Y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    with open(PROPS_OUT, "w") as f:
        json.dump({"collection_point": COLLECTION_POINT, "laboratory_test_point": TEST_POINT,
                   "replaced_placeholders": REPLACED_PLACEHOLDERS,
                   "dynamic_visuals": {k: v[0] for k, v in DYNAMIC_VISUALS.items()},
                   "props": placed}, f, indent=2)
    print(f"world: {WORLD_OUT}\nmap:   {MAP_PGM}\nprops: {PROPS_OUT} ({len(placed)} placed)")


if __name__ == "__main__":
    build()
