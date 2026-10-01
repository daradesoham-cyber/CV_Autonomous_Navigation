#!/usr/bin/env python3
"""
V3 perception evaluation: V2.5 baseline vs V3 model(s) on the canonical V3 test split and the
challenge subsets A-J. All ground truth comes from the dataset metadata (Gazebo segmentation).

Per evaluation set and model:
  - AP@0.50 and AP@0.50:0.95 per class (all-point interpolated, predictions at conf >= 0.001)
  - at the operating confidence (--conf, default 0.35 = V2.6 global threshold): TP / FP / FN,
    precision, recall per class and total, FP per image
  - FP attribution: every FP is attributed to the scene object it covers (projected 3D bounds of
    placed props, V2.6 furniture, pillars/walls, or a GT object of another class) -> e.g.
    "hospital_bed on lab_bench_1", "forklift on surgical_trolley"
Matching: greedy by confidence, same class, IoU >= 0.5. Not counted as FP (ignored):
  - predictions matching (IoU >= 0.5, same class) an instance the generator dropped as too small /
    mostly occluded (it is in the image but deliberately unlabelled)
  - `person` predictions >= 70 % inside a GT hospital_bed / wheelchair box (occupant is part of the
    object mesh, v3_classes.yaml)
V2.5 predictions of classes outside the V3 taxonomy (forklift, pallet, box, obstacle, door,
charging_station) are always FPs and are reported separately as out-of-taxonomy.
Additionally, ultralytics `val` is run for V3 models on the test split (mAP, P, R, confusion matrix).

Usage: python3 evaluate_v3.py --models v25=models/yolov8n_v25.pt v3=V3/models_trained/X.pt
"""
import argparse
import glob
import json
import math
import os
import sys
from collections import Counter, defaultdict

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

V3_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(V3_ROOT)
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(V3_ROOT, "datasets"))
import gt_labels as gt  # noqa: E402
from scripts.build_realistic_facility import FURNITURE, PILLARS, WALLS  # noqa: E402

DATA_ROOT = os.path.join(V3_ROOT, "datasets", "v3_hospital_cv")
OUT_DIR = os.path.join(V3_ROOT, "docs", "evidence", "phase2_eval")
NAMES = gt.NAMES
IOU_T = 0.5
OCCUPIED = {NAMES.index("hospital_bed"), NAMES.index("wheelchair")}
PROPS_META = json.load(open(os.path.join(V3_ROOT, "worlds", "hospital_logistics_props.json")))
REPLACED = set(PROPS_META["replaced_placeholders"])

# fixed scene structure as 3D boxes (name -> (visual_aabb, pose)) for FP attribution
STRUCT3D = {}
for f in FURNITURE:
    if f[0] not in REPLACED:
        n, x, y, z, _, _, yaw, sx, sy, sz, _ = f
        STRUCT3D[n] = ([[-sx / 2, -sy / 2, -sz / 2], [sx / 2, sy / 2, sz / 2]], [x, y, z, 0, 0, yaw])
for n, x, y, sx, sy in PILLARS:
    STRUCT3D[n] = ([[-sx / 2, -sy / 2, 0], [sx / 2, sy / 2, 2.8]], [x, y, 0, 0, 0, 0])
for n, x, y, sx, sy in WALLS:
    STRUCT3D[n] = ([[-sx / 2, -sy / 2, 0], [sx / 2, sy / 2, 2.8]], [x, y, 0, 0, 0, 0])


def project_aabb(aabb, pose, rig):
    mn, mx = aabb
    corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
    world = corners @ gt.rot(*pose[3:6]).T + np.array(pose[:3])
    cam = (world - np.array(rig[:3])) @ gt.rot(*rig[3:6])
    fwd, right, down = cam[:, 0], -cam[:, 1], -cam[:, 2]
    front = fwd > 0.08
    if not front.any():
        return None
    u = gt.W / 2 + gt.F * right[front] / fwd[front]
    v = gt.H / 2 + gt.F * down[front] / fwd[front]
    b = [u.min(), v.min(), u.max(), v.max()]
    if not front.all():
        b = [min(b[0], 0), min(b[1], 0), max(b[2], gt.W), max(b[3], gt.H)]
    b = [max(0, b[0]), max(0, b[1]), min(gt.W, b[2]), min(gt.H, b[3])]
    return b if b[2] > b[0] and b[3] > b[1] else None


def inside_frac(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    return ix * iy / max(1e-6, (a[2] - a[0]) * (a[3] - a[1]))


def attribute_fp(box, meta, gtb):
    """Name the scene object an FP box lies on (largest overlap with depth ordering by distance)."""
    rig = meta["rig_pose"]
    cands = []
    for g in gtb:  # labelled GT of another class
        cands.append((gt.iou(box, g["xyxy"]), inside_frac(box, g["xyxy"]), f"GT:{NAMES[g['class_id']]}"))
    for n, pose in meta["placed_objects"].items():
        o = gt.OBJ.get(n)
        if o and o["class_id"] is None and "visual_aabb" in o:
            pb = project_aabb(o["visual_aabb"], pose, rig)
            if pb:
                cands.append((gt.iou(box, pb), inside_frac(box, pb), f"neg:{o.get('model', n)}"))
    for n, (aabb, pose) in STRUCT3D.items():
        if math.hypot(pose[0] - rig[0], pose[1] - rig[1]) > 20:
            continue
        pb = project_aabb(aabb, pose, rig)
        if pb:
            kind = "wall" if n.startswith("wall") else n
            cands.append((gt.iou(box, pb), inside_frac(box, pb), f"struct:{kind}"))
    best = max(cands, key=lambda c: (c[0] >= 0.3, c[0], c[1]), default=None)
    if best and (best[0] >= 0.3 or best[1] >= 0.6):
        return best[2]
    return "background"


def load_set(root, split):
    metas = [json.loads(l) for l in open(os.path.join(root, "meta", f"{split}.jsonl"))]
    return [(os.path.join(root, m["image"]), m) for m in metas]


def predict_all(model, items, batch=32):
    preds = []
    for i in range(0, len(items), batch):
        res = model.predict([p for p, _ in items[i:i + batch]], conf=0.001, iou=0.45, imgsz=640, device=0,
                            verbose=False, max_det=100)
        for r in res:
            preds.append([(model.names[int(c)], float(s), [float(v) for v in b])
                          for c, s, b in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist())])
    return preds


def match_image(preds, meta, conf_of):
    """Returns per-prediction records (conf, cls, tp flag / 'ignored') and FN list at IoU 0.5."""
    gtb = meta["boxes"]
    drops = meta.get("dropped_instances", [])
    used = [False] * len(gtb)
    recs = []
    for name, s, box in sorted(preds, key=lambda p: -p[1]):
        cid = NAMES.index(name) if name in NAMES else None
        best, bi = 0.0, -1
        if cid is not None:
            for j, g in enumerate(gtb):
                if not used[j] and g["class_id"] == cid:
                    v = gt.iou(box, g["xyxy"])
                    if v > best:
                        best, bi = v, j
        if best >= IOU_T:
            used[bi] = True
            recs.append({"cls": name, "conf": s, "tp": True, "box": box})
            continue
        ignored = cid is not None and any(d["class_id"] == cid and gt.iou(box, d["xyxy"]) >= IOU_T for d in drops)
        if name == "person" and any(g["class_id"] in OCCUPIED and inside_frac(box, g["xyxy"]) >= 0.7 for g in gtb):
            ignored = True
        recs.append({"cls": name, "conf": s, "tp": False, "ignored": ignored, "box": box})
    fns = [g for j, g in enumerate(gtb) if not used[j]]
    return recs, fns


def ap_from(scores_tp, n_gt):
    if n_gt == 0:
        return None
    if not scores_tp:
        return 0.0
    scores_tp.sort(key=lambda x: -x[0])
    tp = np.cumsum([1 if t else 0 for _, t in scores_tp])
    fp = np.cumsum([0 if t else 1 for _, t in scores_tp])
    rec, prec = tp / n_gt, tp / np.maximum(tp + fp, 1e-9)
    mrec = np.concatenate([[0], rec, [1]])
    mpre = np.concatenate([[1], prec, [0]])
    for i in range(len(mpre) - 2, -1, -1):
        mpre[i] = max(mpre[i], mpre[i + 1])
    idx = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[idx + 1] - mrec[idx]) * mpre[idx + 1]))


def evaluate_set(items, preds, conf_of):
    out = {"images": len(items)}
    n_gt = Counter(g["class_id"] for _, m in items for g in m["boxes"])
    ap50 = {}
    ap5095 = {}
    for cid, name in enumerate(NAMES):
        if n_gt[cid] == 0:
            continue
        aps = []
        for t in np.arange(0.5, 0.96, 0.05):
            st = []
            for (path, m), pr in zip(items, preds):
                p = [x for x in pr if x[0] == name]
                gtb = [g for g in m["boxes"] if g["class_id"] == cid]
                used = [False] * len(gtb)
                for _, s, box in sorted(p, key=lambda x: -x[1]):
                    best, bi = 0.0, -1
                    for j, g in enumerate(gtb):
                        if not used[j]:
                            v = gt.iou(box, g["xyxy"])
                            if v > best:
                                best, bi = v, j
                    if best >= t:
                        used[bi] = True
                        st.append((s, True))
                    else:
                        drops = [d for d in m.get("dropped_instances", []) if d["class_id"] == cid]
                        if any(gt.iou(box, d["xyxy"]) >= IOU_T for d in drops):
                            continue
                        if name == "person" and any(g2["class_id"] in OCCUPIED and inside_frac(box, g2["xyxy"]) >= 0.7
                                                    for g2 in m["boxes"]):
                            continue
                        st.append((s, False))
            aps.append(ap_from(st, n_gt[cid]))
        ap50[name] = round(aps[0], 4)
        ap5095[name] = round(float(np.mean(aps)), 4)
    out["AP50"] = ap50
    out["AP50_95"] = ap5095
    out["mAP50"] = round(float(np.mean(list(ap50.values()))), 4) if ap50 else None
    out["mAP50_95"] = round(float(np.mean(list(ap5095.values()))), 4) if ap5095 else None

    per = defaultdict(lambda: Counter())
    fp_attr = Counter()
    oot = Counter()
    fp_images = 0
    for (path, m), pr in zip(items, preds):
        kept = [p for p in pr if p[1] >= conf_of(p[0])]
        recs, fns = match_image(kept, m, conf_of)
        img_fp = 0
        for r in recs:
            if r["tp"]:
                per[r["cls"]]["TP"] += 1
            elif not r.get("ignored"):
                img_fp += 1
                where = attribute_fp(r["box"], m, m["boxes"])
                fp_attr[f"{r['cls']} on {where}"] += 1
                if r["cls"] in NAMES:
                    per[r["cls"]]["FP"] += 1
                else:
                    oot[r["cls"]] += 1
            else:
                per[r["cls"]]["ignored"] += 1
        for g in fns:
            per[NAMES[g["class_id"]]]["FN"] += 1
        fp_images += img_fp > 0
    tot = Counter()
    cls_out = {}
    for name in NAMES:
        c = per[name]
        if not any(c.values()) and n_gt[NAMES.index(name)] == 0:
            continue
        tot.update({k: c[k] for k in ("TP", "FP", "FN")})
        cls_out[name] = {"TP": c["TP"], "FP": c["FP"], "FN": c["FN"], "GT": n_gt[NAMES.index(name)],
                         "precision": round(c["TP"] / (c["TP"] + c["FP"]), 4) if c["TP"] + c["FP"] else None,
                         "recall": round(c["TP"] / (c["TP"] + c["FN"]), 4) if c["TP"] + c["FN"] else None}
    total_fp = tot["FP"] + sum(oot.values())
    out["per_class"] = cls_out
    out["total"] = {"TP": tot["TP"], "FP_in_taxonomy": tot["FP"], "FP_out_of_taxonomy": sum(oot.values()),
                    "FP_total": total_fp, "FN": tot["FN"],
                    "precision": round(tot["TP"] / (tot["TP"] + total_fp), 4) if tot["TP"] + total_fp else None,
                    "recall": round(tot["TP"] / (tot["TP"] + tot["FN"]), 4) if tot["TP"] + tot["FN"] else None,
                    "FP_per_image": round(total_fp / len(items), 3), "images_with_FP": fp_images}
    out["out_of_taxonomy_predictions"] = dict(oot)
    out["FP_attribution"] = dict(fp_attr.most_common())
    return out


def example_sheet(items, preds_by_model, conf_of_by_model, path, n=4):
    rows = []
    for (img_path, m) in items[:n]:
        tiles = []
        for tag, preds in preds_by_model.items():
            im = cv2.imread(img_path)
            for g in m["boxes"]:
                x0, y0, x1, y1 = map(int, g["xyxy"])
                cv2.rectangle(im, (x0, y0), (x1, y1), (0, 200, 0), 1)
            pr = preds[items.index((img_path, m))]
            for name, s, b in pr:
                if s < conf_of_by_model[tag](name):
                    continue
                x0, y0, x1, y1 = map(int, b)
                cv2.rectangle(im, (x0, y0), (x1, y1), (0, 0, 255), 2)
                cv2.putText(im, f"{name} {s:.2f}", (x0 + 2, max(12, y0 + 14)), 0, 0.5, (0, 0, 255), 2)
            cv2.putText(im, tag, (5, 470), 0, 0.8, (255, 0, 0), 2)
            tiles.append(cv2.resize(im, (400, 300)))
        rows.append(np.hstack(tiles))
    cv2.imwrite(path, np.vstack(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", nargs="+", required=True, help="tag=path.pt")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--out", default=OUT_DIR)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    sets = {"test_canonical": load_set(DATA_ROOT, "test")}
    for d in sorted(glob.glob(os.path.join(DATA_ROOT, "subsets", "*"))):
        sets[os.path.basename(d)] = load_set(d, "test")
    v25_thr = yaml.safe_load(open(os.path.join(PROJECT_ROOT, "config", "perception_v25.yaml")))["yolo"]
    report = {"conf": a.conf, "iou": IOU_T, "models": {}, "sets": {}}
    preds = {}
    conf_fns = {}
    for spec in a.models:
        tag, path = spec.split("=", 1)
        path = path if os.path.isabs(path) else os.path.join(PROJECT_ROOT, path)
        model = YOLO(path)
        report["models"][tag] = {"path": path, "classes": model.names}
        conf_fns[tag] = lambda name, c=a.conf: c
        if tag == "v25_deployed":  # V2.5 at its deployed per-class thresholds
            conf_fns[tag] = lambda name, t=v25_thr: t["class_confidence_thresholds"].get(name, t["global_confidence_threshold"])
        preds[tag] = {k: predict_all(model, v) for k, v in sets.items()}
        if not any(n in ("forklift", "pallet") for n in model.names.values()):  # V3 taxonomy -> ultralytics val
            r = model.val(data=os.path.join(DATA_ROOT, "data.yaml"), split="test", imgsz=640, device=0, conf=0.001,
                          iou=0.6, plots=True, project=a.out, name=f"ultralytics_val_{tag}", exist_ok=True, verbose=False)
            report["models"][tag]["ultralytics_test"] = {
                "mAP50": round(float(r.box.map50), 4), "mAP50_95": round(float(r.box.map), 4),
                "precision": round(float(r.box.mp), 4), "recall": round(float(r.box.mr), 4),
                "per_class_AP50": {model.names[int(c)]: round(float(v), 4) for c, v in zip(r.box.ap_class_index, r.box.ap50)},
                "per_class_AP50_95": {model.names[int(c)]: round(float(v), 4) for c, v in zip(r.box.ap_class_index, r.box.ap)},
                "confusion_matrix_png": os.path.join(a.out, f"ultralytics_val_{tag}", "confusion_matrix.png")}
    for k, items in sets.items():
        report["sets"][k] = {tag: evaluate_set(items, preds[tag][k], conf_fns[tag]) for tag in preds}
        example_sheet(items, {t: preds[t][k] for t in preds}, conf_fns, os.path.join(a.out, f"examples_{k}.png"))
        line = " | ".join(f"{t}: P={r['total']['precision']} R={r['total']['recall']} FP/img={r['total']['FP_per_image']} mAP50={r['mAP50']}"
                          for t, r in report["sets"][k].items())
        print(f"{k:28s} {line}", flush=True)
    json.dump(report, open(os.path.join(a.out, "evaluation.json"), "w"), indent=2)
    print("written", os.path.join(a.out, "evaluation.json"))


if __name__ == "__main__":
    main()
