#!/usr/bin/env python3
"""
Controlled Perception Tests (Test A through Test H) for CV Autonomous Navigation V2.5.
Validates:
- Test A: Empty hallway (rejection of false positives on corridor background)
- Test B: Empty wall (rejection of false hospital_bed detections with calibrated thresholds & geometry filters)
- Test C: Floor / corner / shadow (rejection of shadows, tile lines, wall corners)
- Test D: Actual hospital bed (detection rate and confidences on real beds)
- Test E: Actual cart (detection rate and confidences on warehouse carts & trolleys)
- Test F: Actual forklift (detection rate and confidences on forklifts)
- Test G: Actual person (detection rate and confidences on personnel)
- Test H: Directional signs (near, medium, far, frontal, oblique recognition)
"""

import os
import sys
import glob
import json
import yaml
import cv2
import numpy as np
from ultralytics import YOLO

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
MODEL_PATH = os.path.join(PROJECT_ROOT, "models/yolov8n_v25.pt")
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config/perception_v25.yaml")
VAL_IMAGES_DIR = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset/images/val")
VAL_LABELS_DIR = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset/labels/val")
SUBSETS_DIR = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset/eval_subsets")
OUTPUT_JSON = os.path.join(PROJECT_ROOT, "controlled_perception_tests_v25.json")

def load_config():
    with open(CONFIG_PATH, 'r') as f:
        return yaml.safe_load(f)

def run_filtered_inference(model, img, cfg):
    yolo_cfg = cfg.get('yolo', {})
    class_thresholds = yolo_cfg.get('class_confidence_thresholds', {})
    global_thresh = yolo_cfg.get('global_confidence_threshold', 0.35)
    iou_thresh = yolo_cfg.get('iou_threshold', 0.45)
    min_area = yolo_cfg.get('min_box_area_px2', 400)
    max_area_ratio = yolo_cfg.get('max_box_area_ratio', 0.60)
    max_width_ratio = yolo_cfg.get('max_box_width_ratio', 0.92)

    h_img, w_img = img.shape[:2]
    max_area = max_area_ratio * (w_img * h_img)
    max_width = max_width_ratio * w_img

    # Run YOLO with minimum global threshold
    res = model.predict(img, imgsz=640, conf=min(global_thresh, 0.25), iou=iou_thresh, device=0, verbose=False)[0]

    accepted = []
    for b in res.boxes:
        cls_id = int(b.cls[0].item())
        cls_name = model.names[cls_id]
        conf = float(b.conf[0].item())
        xyxy = b.xyxy[0].cpu().numpy().tolist()
        x1, y1, x2, y2 = xyxy

        # Per-class threshold
        req_conf = class_thresholds.get(cls_name, global_thresh)
        if conf < req_conf:
            continue

        # Area & width filters
        area = (x2 - x1) * (y2 - y1)
        w = (x2 - x1)
        if area < min_area or area > max_area or w > max_width:
            continue

        accepted.append({
            'class_id': cls_id,
            'class_name': cls_name,
            'confidence': round(conf, 3),
            'bbox': [round(v, 1) for v in xyxy],
            'area_px': round(area, 1)
        })

    return accepted

def get_images_by_class(target_class_id, max_count=30):
    matched = []
    all_imgs = sorted(glob.glob(os.path.join(VAL_IMAGES_DIR, "*.jpg")))
    for p in all_imgs:
        stem = os.path.splitext(os.path.basename(p))[0]
        lbl = os.path.join(VAL_LABELS_DIR, stem + ".txt")
        if os.path.exists(lbl):
            with open(lbl, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    if parts and int(parts[0]) == target_class_id:
                        matched.append(p)
                        break
        if len(matched) >= max_count:
            break
    return matched

def main():
    print("=" * 75)
    print("      V2.5 CONTROLLED PERCEPTION SUITE (TEST A THROUGH TEST H)")
    print("=" * 75)

    cfg = load_config()
    model = YOLO(MODEL_PATH)

    results = {}

    # -------------------------------------------------------------
    # Test A: Empty Hallway (Pure corridor perspective backgrounds)
    # -------------------------------------------------------------
    print("\n--- Running Test A: Empty Hallway (Corridor Negative Frames) ---")
    neg_images = sorted(glob.glob(os.path.join(VAL_IMAGES_DIR, "*.jpg")))[:30]
    fp_a = []
    for p in neg_images:
        im = cv2.imread(p)
        dets = run_filtered_inference(model, im, cfg)
        if dets:
            fp_a.append({'file': os.path.basename(p), 'detections': dets})

    results['Test_A_Empty_Hallway'] = {
        'total_evaluated': len(neg_images),
        'false_positives': len(fp_a),
        'fp_rate_pct': round((len(fp_a) / max(1, len(neg_images))) * 100, 2),
        'status': 'PASS' if len(fp_a) == 0 else 'FAIL',
        'details': fp_a
    }
    print(f"  Result: {len(fp_a)} FP on {len(neg_images)} frames ({results['Test_A_Empty_Hallway']['status']})")

    # -------------------------------------------------------------
    # Test B: Empty Wall (Frontal walls, baseboards, lighting gradients)
    # -------------------------------------------------------------
    print("\n--- Running Test B: Empty Wall (Frontal Walls & Lighting Gradients) ---")
    wall_test_cases = []
    # Generate 20 diverse empty wall lighting & baseboard variations
    for h_pos in [150, 200, 250, 300, 350]:
        for light_x in [100, 200, 320, 450]:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            # Wall color: (217, 224, 224), Floor color: (217, 209, 209)
            img[:h_pos, :] = (217, 224, 224)
            img[h_pos:, :] = (217, 209, 209)
            # Add diffuse light gradient on wall
            Y, X = np.ogrid[:h_pos, :640]
            dist_sq = (X - light_x)**2 + (Y - (h_pos - 100))**2
            intensity = np.exp(-dist_sq / (2 * 120**2))
            for c in range(3):
                img[:h_pos, :, c] = np.clip(img[:h_pos, :, c] * (0.7 + 0.3 * intensity), 0, 255).astype(np.uint8)
            # Add skirting board
            cv2.line(img, (0, h_pos), (640, h_pos), (80, 80, 80), 2)
            wall_test_cases.append((f"wall_h{h_pos}_lx{light_x}", img))

    fp_b = []
    for name, im in wall_test_cases:
        dets = run_filtered_inference(model, im, cfg)
        if dets:
            fp_b.append({'case': name, 'detections': dets})

    results['Test_B_Empty_Wall'] = {
        'total_evaluated': len(wall_test_cases),
        'false_positives': len(fp_b),
        'fp_rate_pct': round((len(fp_b) / max(1, len(wall_test_cases))) * 100, 2),
        'status': 'PASS' if len(fp_b) == 0 else 'FAIL',
        'details': fp_b
    }
    print(f"  Result: {len(fp_b)} FP on {len(wall_test_cases)} wall cases ({results['Test_B_Empty_Wall']['status']})")

    # -------------------------------------------------------------
    # Test C: Floor / Corner / Shadow (Flooring grid, corner shadows)
    # -------------------------------------------------------------
    print("\n--- Running Test C: Floor / Corner / Shadow ---")
    corner_cases = []
    for ang in [-0.4, -0.2, 0.0, 0.2, 0.4]:
        for shadow_int in [0.4, 0.6, 0.8]:
            im = np.zeros((480, 640, 3), dtype=np.uint8)
            im[:240, :] = (215, 220, 222)
            im[240:, :] = (175, 180, 185)
            # Corner line
            cx = int(320 + 200 * ang)
            cv2.line(im, (cx, 0), (cx, 240), (80, 85, 90), 3)
            # Dynamic shadow triangle on floor
            pts_sh = np.array([[cx, 240], [cx + 120, 480], [cx - 60, 480]], np.int32)
            shadow_mask = np.zeros((480, 640), dtype=np.uint8)
            cv2.fillPoly(shadow_mask, [pts_sh], 255)
            im[shadow_mask == 255] = (im[shadow_mask == 255] * shadow_int).astype(np.uint8)
            corner_cases.append((f"corner_a{ang}_s{shadow_int}", im))

    fp_c = []
    for name, im in corner_cases:
        dets = run_filtered_inference(model, im, cfg)
        if dets:
            fp_c.append({'case': name, 'detections': dets})

    results['Test_C_Floor_Corner_Shadow'] = {
        'total_evaluated': len(corner_cases),
        'false_positives': len(fp_c),
        'fp_rate_pct': round((len(fp_c) / max(1, len(corner_cases))) * 100, 2),
        'status': 'PASS' if len(fp_c) == 0 else 'FAIL',
        'details': fp_c
    }
    print(f"  Result: {len(fp_c)} FP on {len(corner_cases)} cases ({results['Test_C_Floor_Corner_Shadow']['status']})")

    # -------------------------------------------------------------
    # Test D: Actual Hospital Bed (Ground Truth)
    # -------------------------------------------------------------
    print("\n--- Running Test D: Actual Hospital Bed ---")
    bed_imgs = get_images_by_class(target_class_id=8, max_count=30)
    tp_d, confs_d = 0, []
    for p in bed_imgs:
        im = cv2.imread(p)
        dets = run_filtered_inference(model, im, cfg)
        bed_dets = [d for d in dets if d['class_name'] == 'hospital_bed']
        if bed_dets:
            tp_d += 1
            confs_d.append(max(d['confidence'] for d in bed_dets))

    recall_d = round((tp_d / max(1, len(bed_imgs))) * 100, 1)
    mean_conf_d = round(float(np.mean(confs_d)), 3) if confs_d else 0.0
    results['Test_D_Hospital_Bed'] = {
        'total_samples': len(bed_imgs),
        'true_positives': tp_d,
        'recall_pct': recall_d,
        'mean_confidence': mean_conf_d,
        'status': 'PASS' if recall_d >= 80.0 else 'FAIL'
    }
    print(f"  Result: {tp_d}/{len(bed_imgs)} beds detected ({recall_d}%, Mean Conf: {mean_conf_d})")

    # -------------------------------------------------------------
    # Test E: Actual Cart (Ground Truth)
    # -------------------------------------------------------------
    print("\n--- Running Test E: Actual Cart ---")
    cart_imgs = get_images_by_class(target_class_id=1, max_count=30)
    tp_e, confs_e = 0, []
    for p in cart_imgs:
        im = cv2.imread(p)
        dets = run_filtered_inference(model, im, cfg)
        cart_dets = [d for d in dets if d['class_name'] == 'cart']
        if cart_dets:
            tp_e += 1
            confs_e.append(max(d['confidence'] for d in cart_dets))

    recall_e = round((tp_e / max(1, len(cart_imgs))) * 100, 1)
    mean_conf_e = round(float(np.mean(confs_e)), 3) if confs_e else 0.0
    results['Test_E_Cart'] = {
        'total_samples': len(cart_imgs),
        'true_positives': tp_e,
        'recall_pct': recall_e,
        'mean_confidence': mean_conf_e,
        'status': 'PASS' if recall_e >= 80.0 else 'FAIL'
    }
    print(f"  Result: {tp_e}/{len(cart_imgs)} carts detected ({recall_e}%, Mean Conf: {mean_conf_e})")

    # -------------------------------------------------------------
    # Test F: Actual Forklift (Ground Truth)
    # -------------------------------------------------------------
    print("\n--- Running Test F: Actual Forklift ---")
    fork_imgs = get_images_by_class(target_class_id=2, max_count=30)
    tp_f, confs_f = 0, []
    for p in fork_imgs:
        im = cv2.imread(p)
        dets = run_filtered_inference(model, im, cfg)
        fork_dets = [d for d in dets if d['class_name'] == 'forklift']
        if fork_dets:
            tp_f += 1
            confs_f.append(max(d['confidence'] for d in fork_dets))

    recall_f = round((tp_f / max(1, len(fork_imgs))) * 100, 1)
    mean_conf_f = round(float(np.mean(confs_f)), 3) if confs_f else 0.0
    results['Test_F_Forklift'] = {
        'total_samples': len(fork_imgs),
        'true_positives': tp_f,
        'recall_pct': recall_f,
        'mean_confidence': mean_conf_f,
        'status': 'PASS' if recall_f >= 80.0 else 'FAIL'
    }
    print(f"  Result: {tp_f}/{len(fork_imgs)} forklifts detected ({recall_f}%, Mean Conf: {mean_conf_f})")

    # -------------------------------------------------------------
    # Test G: Actual Person (Ground Truth)
    # -------------------------------------------------------------
    print("\n--- Running Test G: Actual Person ---")
    person_imgs = get_images_by_class(target_class_id=0, max_count=30)
    tp_g, confs_g = 0, []
    for p in person_imgs:
        im = cv2.imread(p)
        dets = run_filtered_inference(model, im, cfg)
        person_dets = [d for d in dets if d['class_name'] == 'person']
        if person_dets:
            tp_g += 1
            confs_g.append(max(d['confidence'] for d in person_dets))

    recall_g = round((tp_g / max(1, len(person_imgs))) * 100, 1)
    mean_conf_g = round(float(np.mean(confs_g)), 3) if confs_g else 0.0
    results['Test_G_Person'] = {
        'total_samples': len(person_imgs),
        'true_positives': tp_g,
        'recall_pct': recall_g,
        'mean_confidence': mean_conf_g,
        'status': 'PASS' if recall_g >= 80.0 else 'FAIL'
    }
    print(f"  Result: {tp_g}/{len(person_imgs)} people detected ({recall_g}%, Mean Conf: {mean_conf_g})")

    # -------------------------------------------------------------
    # Test H: Directional Signs (Near, Medium, Far, Frontal, Oblique)
    # -------------------------------------------------------------
    print("\n--- Running Test H: Directional Signs Across Subsets ---")
    sign_subsets = {
        'near': os.path.join(SUBSETS_DIR, "distance_near"),
        'medium': os.path.join(SUBSETS_DIR, "distance_medium"),
        'far': os.path.join(SUBSETS_DIR, "distance_far"),
        'frontal': os.path.join(SUBSETS_DIR, "angle_frontal"),
        'oblique': os.path.join(SUBSETS_DIR, "angle_oblique")
    }

    sub_res = {}
    for sub_name, sub_path in sign_subsets.items():
        imgs = sorted(glob.glob(os.path.join(sub_path, "images", "*.jpg")))[:20]
        tp_s, confs_s = 0, []
        for p in imgs:
            im = cv2.imread(p)
            dets = run_filtered_inference(model, im, cfg)
            s_dets = [d for d in dets if d['class_name'] == 'directional_sign']
            if s_dets:
                tp_s += 1
                confs_s.append(max(d['confidence'] for d in s_dets))

        rec_s = round((tp_s / max(1, len(imgs))) * 100, 1)
        mean_c_s = round(float(np.mean(confs_s)), 3) if confs_s else 0.0
        sub_res[sub_name] = {
            'samples': len(imgs),
            'detected': tp_s,
            'recall_pct': rec_s,
            'mean_conf': mean_c_s
        }
        print(f"  - Subset {sub_name:<10}: {tp_s}/{len(imgs)} signs detected ({rec_s}%, Mean Conf: {mean_c_s})")

    overall_sign_rec = round(float(np.mean([s['recall_pct'] for s in sub_res.values()])), 1)
    results['Test_H_Directional_Signs'] = {
        'subsets': sub_res,
        'overall_recall_pct': overall_sign_rec,
        'status': 'PASS' if overall_sign_rec >= 75.0 else 'FAIL'
    }

    # Save results
    with open(OUTPUT_JSON, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nAll controlled perception test results written to: {OUTPUT_JSON}")
    print("=" * 75)

if __name__ == '__main__':
    main()
