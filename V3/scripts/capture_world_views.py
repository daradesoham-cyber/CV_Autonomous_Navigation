#!/usr/bin/env python3
"""
Visual check of the running V3 hospital world: spawns temporary camera models in Gazebo (the world's Sensors
system renders them), saves one PNG per view, then removes the cameras.
Usage (ROS/Gazebo env sourced, world running): capture_world_views.py OUT_DIR [view ...]
Views: name -> (x, y, z, roll, pitch, yaw, hfov)
"""
import math
import os
import sys
import time

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.entity_factory_pb2 import EntityFactory
from gz.msgs.entity_pb2 import Entity
from gz.msgs.image_pb2 import Image
from gz.transport import Node

WORLD = "realistic_facility_world"
D = math.pi / 180
VIEWS = {
    "overview_north": (0, -2, 34, 0, 89.9 * D, 90 * D, 1.4),
    "overview_south_half": (0, -21, 17, 0, 89.9 * D, 90 * D, 1.3),
    "overview_north_half": (0, 9, 17, 0, 89.9 * D, 90 * D, 1.3),
    "lobby_reception": (0, 16.5, 2.4, 0, 18 * D, -90 * D, 1.4),
    "lobby_waiting_east": (-2, 12, 2.2, 0, 15 * D, -20 * D, 1.4),
    "west_corridor": (-5, 3.5, 1.7, 0, 8 * D, -90 * D, 1.3),
    "west_ward_collection": (-8.2, -22.5, 1.9, 0, 15 * D, 90 * D, 1.4),
    "laboratory": (7.0, 12.6, 2.0, 0, 18 * D, 40 * D, 1.4),
    "diagnostics_xray": (0, -14.2, 2.4, 0, 30 * D, -90 * D, 1.4),
    "east_ward": (8.2, -23.0, 1.9, 0, 15 * D, 90 * D, 1.4),
    "intersection_west_north": (-5, -5.5, 1.6, 0, 8 * D, -90 * D, 1.3),
}


def cam_sdf(name, pose, hfov):
    x, y, z, r, p, yaw = pose
    return f"""<sdf version='1.9'><model name='{name}'><static>true</static><pose>{x} {y} {z} {r} {p} {yaw}</pose>
<link name='l'><sensor name='cam' type='camera'><topic>/v3_view/{name}</topic><update_rate>5</update_rate><always_on>1</always_on>
<camera><horizontal_fov>{hfov}</horizontal_fov><image><width>1280</width><height>800</height></image>
<clip><near>0.1</near><far>120</far></clip></camera></sensor></link></model></sdf>"""


def main():
    out = sys.argv[1]
    names = sys.argv[2:] or list(VIEWS)
    os.makedirs(out, exist_ok=True)
    node = Node()
    for n in names:
        x, y, z, r, p, yaw, hfov = VIEWS[n]
        cname = f"v3_view_{n}"
        got = {}

        def cb(msg, got=got):
            got["img"] = msg
        node.subscribe(Image, f"/v3_view/{cname}", cb)
        req = EntityFactory()
        req.sdf = cam_sdf(cname, (x, y, z, r, p, yaw), hfov)
        node.request(f"/world/{WORLD}/create", req, EntityFactory, Boolean, 5000)
        t0 = time.time()
        frames = 0
        while time.time() - t0 < 25:
            time.sleep(0.3)
            if "img" in got:
                frames += 1
                got.pop("img") if frames < 4 else None  # let the scene settle a few frames
                if frames >= 4 and "img" in got:
                    break
        msg = got.get("img")
        if msg is None:
            print(f"{n}: no image")
        else:
            from PIL import Image as PImage
            im = PImage.frombytes("RGB", (msg.width, msg.height), msg.data)
            im.save(os.path.join(out, f"{n}.png"))
            print(f"{n}: saved")
        rm = Entity()
        rm.name, rm.type = cname, Entity.MODEL
        node.request(f"/world/{WORLD}/remove", rm, Entity, Boolean, 5000)
        node.unsubscribe(f"/v3_view/{cname}")


if __name__ == "__main__":
    main()
