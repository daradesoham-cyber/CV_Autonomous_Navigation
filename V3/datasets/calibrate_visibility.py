#!/usr/bin/env python3
"""
Measure the unoccluded pixel-fill ratio (visible_px / tight box area) of each labelled V3 model.

Each model is rendered ALONE (every other movable object parked) in the open warehouse main aisle
(x 6.8-9.8, y 1-11.5), camera at the robot-camera height facing north, object 1.5-6.5 m ahead with a
random yaw and lateral offset. Frames where the instance touches the image border are skipped.
The median fill per model is the reference used by gt_labels.decide() to estimate visibility.
Output: V3/datasets/visibility_calibration.json
"""
import json
import math
import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_dataset as g  # noqa: E402
import gt_labels as gt  # noqa: E402

SAMPLES = 30


def main():
    rng = random.Random(4242)
    gz = g.GzScene()
    t0 = time.time()
    while gz.sim_time < 1.0 and time.time() - t0 < 60:
        time.sleep(0.2)
    reps = {}
    for n in g.POSITIVE:
        reps.setdefault(gt.model_key(n), n)
    result = {}
    base = g.Scene(rng, canonical=False)
    for model, name in sorted(reps.items()):
        fills = []
        for i in range(SAMPLES * 2):
            if len(fills) >= SAMPLES:
                break
            poses = dict(base.poses)
            d, lat = rng.uniform(1.5, 6.5), rng.uniform(-0.8, 0.8)
            z = g.OBJ[name]["canonical_pose"][2] if name.startswith("dynamic_") else 0.0
            poses[name] = (8.3 + lat, 1.4 + d, z, 0.0, 0.0, rng.uniform(-math.pi, math.pi))
            rig = g.rig_pose(8.3, 1.2, math.pi / 2)
            poses[g.RIG] = rig
            gz.set_poses(poses)
            rgb, boxes, seg, stamp = gz.capture(gz.sim_time + 0.25)
            inst = [x for x in gt.extract_instances(seg) if x["class_id"] == g.OBJ[name]["class_id"]]
            inst = gt.assign_objects(inst, {name: poses[name]}, rig)
            inst = [x for x in inst if x.get("object") == name]
            if len(inst) != 1:
                continue
            x0, y0, x1, y1 = inst[0]["xyxy"]
            if x0 <= 0 or y0 <= 0 or x1 >= gt.W or y1 >= gt.H or inst[0]["visible_px"] < 200:
                continue
            fills.append(inst[0]["fill"])
        result[model] = {"object_used": name, "n": len(fills),
                         "median_fill": round(statistics.median(fills), 4) if fills else None,
                         "p10_fill": round(sorted(fills)[len(fills) // 10], 4) if fills else None,
                         "p90_fill": round(sorted(fills)[(9 * len(fills)) // 10], 4) if fills else None}
        print(model, result[model], flush=True)
    out = {"method": __doc__.strip().splitlines()[0], "samples_per_model": SAMPLES,
           "reference_fill": {m: r["median_fill"] for m, r in result.items() if r["median_fill"]},
           "per_model": result}
    json.dump(out, open(gt.CALIB_PATH, "w"), indent=2)
    print("written", gt.CALIB_PATH, flush=True)
    gz.close()
    os._exit(0)


if __name__ == "__main__":
    main()
