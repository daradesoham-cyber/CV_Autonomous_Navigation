"""
Thin Gazebo-transport client for the V3 dataset world (in-process, no ROS / CLI subprocesses).

  - set_poses({name: (x, y, z, roll, pitch, yaw)})  -> /world/<w>/set_pose_vector
  - set_light(spec, rgb)                           -> /world/<w>/light_config (full SDF spec, new diffuse)
  - capture(after_sim_time)                         -> (rgb, boxes, seg, sim_time)
    returns the first RGB image, bounding-box message and instance-segmentation labels map whose sim
    timestamps are > after_sim_time and identical, so the labels belong to exactly the rendered frame.
    seg is an (H, W, 3) uint8 array: channel 0 (+256*channel 1) = instance id, channel 2 = label.

Service requests run in a dedicated child process: in gz-transport's Python binding a blocking
request() made in a process that also has Python subscription callbacks timed out every time
(observed 5.0 s timeouts; the same request from a process without subscriptions returns in ~1 ms).
"""
import math
import multiprocessing as mp
import threading
import time

import numpy as np
from gz.msgs.annotated_axis_aligned_2d_box_v_pb2 import AnnotatedAxisAligned2DBox_V
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.clock_pb2 import Clock
from gz.msgs.image_pb2 import Image
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.light_pb2 import Light
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node

WORLD = "realistic_facility_world"


def _stamp(header):
    return header.stamp.sec + header.stamp.nsec * 1e-9


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _request_worker(conn, world):
    node = Node()
    while True:
        item = conn.recv()
        if item is None:
            return
        kind, payload = item
        if kind == "poses":
            req = Pose_V()
            for name, (x, y, z, r, p, yw) in payload.items():
                m = req.pose.add()
                m.name = name
                m.position.x, m.position.y, m.position.z = x, y, z
                qx, qy, qz, qw = quat(r, p, yw)
                m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = qx, qy, qz, qw
            ok, rep = node.request(f"/world/{world}/set_pose_vector/blocking", req, Pose_V, Boolean, 5000)
        else:
            # full light spec (light_config replaces the light): SDF values with a scaled diffuse
            spec, rgb = payload
            req = Light()
            req.name = spec["name"]
            req.type = Light.POINT
            x, y, z = spec["pose"][:3]
            req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, z
            req.pose.orientation.w = 1.0
            req.diffuse.r, req.diffuse.g, req.diffuse.b, req.diffuse.a = (*rgb, 1.0)
            req.specular.r, req.specular.g, req.specular.b, req.specular.a = (*spec["specular"], 1.0)
            req.range = spec["range"]
            req.attenuation_constant = spec["constant"]
            req.attenuation_linear = spec["linear"]
            req.attenuation_quadratic = spec["quadratic"]
            req.intensity = 1.0
            req.cast_shadows = False
            ok, rep = node.request(f"/world/{world}/light_config", req, Light, Boolean, 5000)
        conn.send(bool(ok and rep.data))


class GzScene:
    def __init__(self, world=WORLD, with_scan=False):
        self.world = world
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.worker = ctx.Process(target=_request_worker, args=(child, world), daemon=True)
        self.worker.start()
        self.node = Node()
        self.lock = threading.Lock()
        self.images, self.boxes, self.segs, self.scans = {}, {}, {}, {}
        self.with_scan = with_scan
        self.sim_time = 0.0
        self.node.subscribe(Image, "/v3ds/image", self._on_image)
        self.node.subscribe(AnnotatedAxisAligned2DBox_V, "/v3ds/boxes", self._on_boxes)
        self.node.subscribe(Image, "/v3ds/seg/labels_map", self._on_seg)
        self.node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)
        if with_scan:
            self.node.subscribe(LaserScan, "/v3ds/scan", self._on_scan)

    def _on_clock(self, msg):
        self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_image(self, msg):
        with self.lock:
            self.images[round(_stamp(msg.header), 4)] = msg
            for k in sorted(self.images)[:-20]:
                del self.images[k]

    def _on_seg(self, msg):
        with self.lock:
            self.segs[round(_stamp(msg.header), 4)] = msg
            for k in sorted(self.segs)[:-20]:
                del self.segs[k]

    def _on_scan(self, msg):
        with self.lock:
            self.scans[round(_stamp(msg.header), 4)] = msg
            for k in sorted(self.scans)[:-20]:
                del self.scans[k]

    def _on_boxes(self, msg):
        with self.lock:
            self.boxes[round(_stamp(msg.header), 4)] = msg
            for k in sorted(self.boxes)[:-20]:
                del self.boxes[k]

    def set_poses(self, poses):
        self.conn.send(("poses", poses))
        return self.conn.recv()

    def set_light(self, spec, rgb):
        self.conn.send(("light", (spec, rgb)))
        return self.conn.recv()

    def close(self):
        self.conn.send(None)
        self.worker.join(timeout=5)

    def latest_stamp(self):
        with self.lock:
            return max(self.images) if self.images else 0.0

    def capture_settled(self, after_stamp, timeout=15.0):
        """First pair of consecutive synchronized frames newer than after_stamp whose segmentation
        instance boxes are identical (scene no longer changing between renders); returns the second."""
        end = time.time() + timeout
        prev = self.capture(after_stamp, timeout=max(0.5, end - time.time()))
        while time.time() < end:
            cur = self.capture(prev[3], timeout=max(0.5, end - time.time()))
            if np.array_equal(prev[2], cur[2]):
                return cur
            prev = cur
        raise TimeoutError("scene did not settle")

    def capture(self, after_sim_time, timeout=10.0):
        """Return (rgb, boxes_msg, stamp) for the first synchronized frame rendered after after_sim_time."""
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                common = sorted(k for k in self.images
                                if k in self.boxes and k in self.segs and k > after_sim_time + 1e-6
                                and (not self.with_scan or k in self.scans))
                if common:
                    k = common[0]
                    img, box, seg = self.images[k], self.boxes[k], self.segs[k]
                    rgb = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3).copy()
                    segm = np.frombuffer(seg.data, dtype=np.uint8).reshape(seg.height, seg.width, 3).copy()
                    if self.with_scan:
                        self.last_scan = self.scans[k]
                    return rgb, box, segm, k
            time.sleep(0.01)
        raise TimeoutError("no synchronized image/box/segmentation frame")
