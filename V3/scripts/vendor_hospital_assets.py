#!/usr/bin/env python3
"""
V3 asset vendoring: copy selected OpenRobotics Gazebo Fuel hospital props into V3/models.

Source: https://fuel.gazebosim.org/1.0/OpenRobotics/models/<name> (CC-BY 4.0).
Download first with:
    GZ_FUEL_CACHE_PATH=<cache> gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/<name>

Adaptations for Gazebo Sim 10.5 (nothing else is changed):
  - absolute https://fuel... mesh URIs rewritten to relative meshes/<file> (offline loading)
  - <static>true</static> forced (props are scenery; dynamic obstacles are separate V2.6 models)
  - PatientWheelChair: upstream collision mesh is rotated 90 deg and 0.5 m shorter than the
    visual mesh; it is replaced by a box fitted to the visual mesh bounds.
Upstream zips/thumbnails are not copied.
"""
import argparse
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(V3_ROOT, "models")

# Fuel name -> cache directory name (lower-case in the Fuel cache)
ASSETS = [
    "MalePatientBed", "TrolleyBed", "Drawer", "PatientWheelChair", "IVStand",
    "AdjTable", "BloodPressureMonitor", "BPCart", "MetalCabinet", "StorageRack",
    "SurgicalTrolley", "InstrumentCart1", "Chair", "Scrubs", "FemaleVisitor",
    "MaleVisitorOnPhone",
]


def obj_bounds(path, scale):
    mn, mx = [1e9] * 3, [-1e9] * 3
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                v = [float(t) for t in line.split()[1:4]]
                for i in range(3):
                    mn[i] = min(mn[i], v[i] * scale[i])
                    mx[i] = max(mx[i], v[i] * scale[i])
    return mn, mx


def latest_version_dir(cache, name):
    base = os.path.join(cache, "fuel.gazebosim.org", "openrobotics", "models", name.lower())
    versions = sorted((d for d in os.listdir(base) if d.isdigit()), key=int)
    return os.path.join(base, versions[-1]), versions[-1]


def vendor(cache):
    os.makedirs(MODELS_DIR, exist_ok=True)
    rows = []
    for name in ASSETS:
        src, version = latest_version_dir(cache, name)
        dst = os.path.join(MODELS_DIR, name)
        if os.path.exists(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns("*.zip", "thumbnails"))

        sdf_path = os.path.join(dst, "model.sdf")
        text = open(sdf_path).read()
        text = re.sub(r"https://fuel\.gazebosim\.org/1\.0/openrobotics/models/[^/]+/\d+/files/", "", text,
                      flags=re.IGNORECASE)
        if "<static>" in text:
            text = re.sub(r"<static>\s*\w+\s*</static>", "<static>true</static>", text)
        else:
            text = re.sub(r"(<model name=[\"'][^\"']*[\"']>)", r"\1\n    <static>true</static>", text, count=1)

        if name == "PatientWheelChair":
            root = ET.fromstring(text)
            vis = root.find(".//visual")
            vuri = vis.find(".//mesh/uri").text
            mn, mx = obj_bounds(os.path.join(dst, vuri), [1, 1, 1])
            size = [mx[i] - mn[i] for i in range(3)]
            ctr = [(mx[i] + mn[i]) / 2 for i in range(3)]
            box = (f"<collision name=\"collision\">\n        <pose>{ctr[0]:.3f} {ctr[1]:.3f} {ctr[2]:.3f} 0 0 0</pose>\n"
                   f"        <geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>\n"
                   f"      </collision>")
            text = re.sub(r"<collision name=[\"'][^\"']*[\"']>.*?</collision>", box, text, count=1, flags=re.S)

        open(sdf_path, "w").write(text)
        leftover = re.findall(r"https?://", text)
        rows.append((name, version, "OK" if not leftover else f"REMOTE URI LEFT: {len(leftover)}"))

    with open(os.path.join(MODELS_DIR, "ATTRIBUTION.md"), "w") as f:
        f.write("# V3 hospital model attribution\n\n"
                "All models in this directory are from the OpenRobotics collection on Gazebo Fuel,\n"
                "licensed under Creative Commons Attribution 4.0 International (CC-BY 4.0),\n"
                "https://creativecommons.org/licenses/by/4.0/ .\n\n"
                "Modifications made for V3 (Gazebo Sim 10.5): mesh URIs made relative, models forced static,\n"
                "PatientWheelChair collision mesh replaced by a box fitted to its visual mesh.\n\n"
                "| Model | Fuel URL | Version |\n|---|---|---|\n")
        for name, version, _ in rows:
            f.write(f"| {name} | https://fuel.gazebosim.org/1.0/OpenRobotics/models/{name} | {version} |\n")
    return rows


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", required=True, help="GZ_FUEL_CACHE_PATH used for the download")
    rows = vendor(ap.parse_args().cache)
    for r in rows:
        print(*r)
    sys.exit(0 if all(r[2] == "OK" for r in rows) else 1)
