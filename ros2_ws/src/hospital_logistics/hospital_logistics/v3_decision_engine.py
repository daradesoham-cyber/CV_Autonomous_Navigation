"""
V3 entry point for the V2.6 decision engine (unchanged code, imported as a library).

Adapter fix applied before the node is created:
  V2.6 decision_engine_node._signs_callback calls _trigger_recovery(ReplanReason.OBSTACLE_BLOCKED) when a
  CLOSED/BLOCKED sign hits a node on the active route, but ReplanReason has no OBSTACLE_BLOCKED member.
  The AttributeError is swallowed (debug log), so the announced replan never happened (measured in
  V3/tests/test_v3_sign_gating.py case 9). V3 defines the member as ROUTE_BLOCKED, an existing reason.

  Waypoints are normally reached through the proximity ("seamless") check, which calls only
  memory.record_traversal(success=True); route_segments (what route costs are computed from) was updated
  only on failures, so learned history could only degrade (measured in the V3 memory after ~20 completed
  missions: 0 successes, up to 7 failures on the only ward entry segment). V3 also records a successful
  traversal in route_segments (update_segment_metrics(success=True), the call V2.6 makes on its
  Nav2-result path), guarding against double counting when both calls happen for the same edge.

  NEW hospital: the V2.6 engine treats any person/cart within ~2.2 m straight ahead as a blocked corridor
  and replans the topology. The furnished hospital has static visitors, staff, wheelchairs and carts next to
  the corridors; those are part of the static map, so Nav2 already plans around them and a topology replan
  only produces detours (measured in the first mission: 7 replans, 154 m instead of ~95 m). V3 therefore
  drops obstacles that are not moving (not is_dynamic, not MOVING_TOWARDS / CROSSING) and whose map position
  lies on an occupied cell of the static map (within map_match_radius_m). Moving traffic is handled unchanged.
"""
import math
import os
import time

import numpy as np
import yaml

from autonomous_robot_navigation import decision_engine_node as _de

if not hasattr(_de.ReplanReason, 'OBSTACLE_BLOCKED'):
    _de.ReplanReason.OBSTACLE_BLOCKED = _de.ReplanReason.ROUTE_BLOCKED


_orig_init = _de.DecisionEngineNode.__init__


def _init_with_success_learning(self, *a, **kw):
    _orig_init(self, *a, **kw)
    mem = self.memory
    orig_rt, orig_usm = mem.record_traversal, mem.update_segment_metrics
    recent = {}

    def record_traversal(u, v, success=True, *ra, **rkw):
        r = orig_rt(u, v, success, *ra, **rkw)
        if success:
            recent[(u, v)] = time.time()
            orig_usm(u, v, success=True, clearance=self.min_clearance)
        return r

    def update_segment_metrics(u, v, *ua, **ukw):
        success = ukw.get('success', ua[0] if ua else None)
        if success and time.time() - recent.pop((u, v), 0.0) < 1.0:
            return None  # already recorded by record_traversal for this traversal
        return orig_usm(u, v, *ua, **ukw)

    mem.record_traversal, mem.update_segment_metrics = record_traversal, update_segment_metrics


_de.DecisionEngineNode.__init__ = _init_with_success_learning

V3_ROOT = os.environ.get('V3_ROOT', os.path.expanduser('~/CV_Autonomous_Navigation/V3'))
MAP_YAML = os.path.join(V3_ROOT, 'maps', 'v3_hospital_map.yaml')
MAP_MATCH_RADIUS_M = 0.45


def _load_static_map(path):
    meta = yaml.safe_load(open(path))
    with open(os.path.join(os.path.dirname(path), meta['image']), 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), np.uint8).reshape(h, w)
    return img < 128, float(meta['resolution']), meta['origin'][:2]


_STATIC = _load_static_map(MAP_YAML) if os.path.exists(MAP_YAML) else None


def _on_static_map(x, y):
    occ, res, (ox, oy) = _STATIC
    h, w = occ.shape
    c, r = int((x - ox) / res), int(h - 1 - (y - oy) / res)
    k = int(MAP_MATCH_RADIUS_M / res)
    r0, r1, c0, c1 = max(0, r - k), min(h, r + k + 1), max(0, c - k), min(w, c + k + 1)
    return r0 < r1 and c0 < c1 and bool(occ[r0:r1, c0:c1].any())


_orig_obstacles_cb = _de.DecisionEngineNode._obstacles_callback


def _obstacles_without_mapped_statics(self, msg):
    if _STATIC is not None:
        keep = []
        cy, sy = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        for o in msg.obstacles:
            d = getattr(o, 'direction', '')
            moving = o.is_dynamic or 'MOVING_TOWARDS' in d or 'CROSSING' in d
            mx = self.robot_x + cy * o.x - sy * o.y
            my = self.robot_y + sy * o.x + cy * o.y
            if not moving and (o.x != 0.0 or o.y != 0.0) and _on_static_map(mx, my):
                continue  # mapped static prop: Nav2 already plans around it
            keep.append(o)
        del msg.obstacles[:]
        msg.obstacles.extend(keep)
    return _orig_obstacles_cb(self, msg)


_de.DecisionEngineNode._obstacles_callback = _obstacles_without_mapped_statics


def main(args=None):
    _de.main(args)


if __name__ == '__main__':
    main()
