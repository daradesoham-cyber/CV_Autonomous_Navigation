#!/usr/bin/env python3
"""
Automated Synthetic Dataset Generator for Gazebo Simulation Objects.
Generates images with randomized positions, viewpoints, backgrounds, and lighting,
along with precise ground-truth bounding-box annotations in YOLO format.

Classes:
  0: person
  1: chair
  2: box
  3: cone
  4: pallet
  5: shelf
  6: hospital_bed
  7: cart
"""
import os
import sys
import random
import yaml
import numpy as np
import cv2

CLASSES = [
    'person',
    'chair',
    'box',
    'cone',
    'pallet',
    'shelf',
    'hospital_bed',
    'cart'
]

# Color palettes for synthetic rendering (representing Gazebo materials)
CLASS_COLORS = {
    0: (165, 60, 40),     # Person (blue-ish clothing)
    1: (30, 30, 30),      # Chair (black/dark grey)
    2: (70, 120, 180),    # Box (cardboard brown in BGR)
    3: (0, 120, 255),     # Cone (orange in BGR)
    4: (40, 90, 140),     # Pallet (wood brown in BGR)
    5: (140, 70, 40),     # Shelf (industrial blue in BGR)
    6: (220, 220, 230),   # Hospital bed (light clinical grey/white in BGR)
    7: (30, 30, 220),     # Cart (red in BGR)
}

def draw_object(img, cls_id, cx, cy, w, h):
    """Draw visually structured geometry representing each Gazebo simulation model."""
    color = CLASS_COLORS[cls_id]
    x1 = int(cx - w / 2)
    y1 = int(cy - h / 2)
    x2 = int(cx + w / 2)
    y2 = int(cy + h / 2)

    # Bound coordinates to image
    H, W = img.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(W - 1, x2), min(H - 1, y2)
    bw = x2 - x1
    bh = y2 - y1

    if bw <= 10 or bh <= 10:
        return None

    if cls_id == 0:  # person: head + torso + legs
        head_r = int(bh * 0.12)
        head_cx = (x1 + x2) // 2
        head_cy = y1 + head_r
        cv2.circle(img, (head_cx, head_cy), head_r, (140, 180, 220), -1)  # skin tone
        # Torso
        torso_y1 = y1 + int(bh * 0.22)
        torso_y2 = y1 + int(bh * 0.65)
        cv2.rectangle(img, (x1 + int(bw * 0.15), torso_y1), (x2 - int(bw * 0.15), torso_y2), color, -1)
        # Legs
        leg_w = int(bw * 0.25)
        cv2.rectangle(img, (x1 + int(bw * 0.18), torso_y2), (x1 + int(bw * 0.18) + leg_w, y2), (40, 40, 50), -1)
        cv2.rectangle(img, (x2 - int(bw * 0.18) - leg_w, torso_y2), (x2 - int(bw * 0.18), y2), (40, 40, 50), -1)

    elif cls_id == 1:  # chair: seat + backrest + legs
        # Backrest
        cv2.rectangle(img, (x1, y1), (x2, y1 + int(bh * 0.45)), color, -1)
        # Seat
        cv2.rectangle(img, (x1, y1 + int(bh * 0.45)), (x2, y1 + int(bh * 0.58)), (50, 50, 50), -1)
        # Legs
        cv2.rectangle(img, (x1 + 4, y1 + int(bh * 0.58)), (x1 + 10, y2), (180, 180, 180), -1)
        cv2.rectangle(img, (x2 - 10, y1 + int(bh * 0.58)), (x2 - 4, y2), (180, 180, 180), -1)

    elif cls_id == 2:  # box: cardboard box with tape
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        # Tape stripe
        cv2.line(img, (x1, (y1 + y2) // 2), (x2, (y1 + y2) // 2), (50, 90, 140), max(2, int(bh * 0.06)))

    elif cls_id == 3:  # cone: orange triangle with white stripe
        pts = np.array([[(x1 + x2) // 2, y1], [x1, y2], [x2, y2]], np.int32)
        cv2.fillPoly(img, [pts], color)
        # Reflective stripe
        stripe_y1 = y1 + int(bh * 0.45)
        stripe_y2 = y1 + int(bh * 0.65)
        cv2.rectangle(img, (x1 + int(bw * 0.2), stripe_y1), (x2 - int(bw * 0.2), stripe_y2), (240, 240, 240), -1)

    elif cls_id == 4:  # pallet: slatted wooden structure
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        # Slats
        for sy in np.linspace(y1, y2, 5):
            cv2.line(img, (x1, int(sy)), (x2, int(sy)), (20, 50, 90), 2)

    elif cls_id == 5:  # shelf: tall rack with multiple horizontal shelves
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
        for sy in np.linspace(y1 + bh * 0.2, y2 - bh * 0.2, 3):
            cv2.line(img, (x1, int(sy)), (x2, int(sy)), color, 4)

    elif cls_id == 6:  # hospital_bed: horizontal mattress + headboard
        cv2.rectangle(img, (x1, y1 + int(bh * 0.3)), (x2, y2), color, -1)
        cv2.rectangle(img, (x1, y1), (x1 + int(bw * 0.15), y2), (180, 180, 190), -1)

    elif cls_id == 7:  # cart: mobile cart on wheels
        cv2.rectangle(img, (x1, y1), (x2, y2 - int(bh * 0.2)), color, -1)
        # Wheels
        wheel_r = max(4, int(bh * 0.1))
        cv2.circle(img, (x1 + int(bw * 0.2), y2 - wheel_r), wheel_r, (20, 20, 20), -1)
        cv2.circle(img, (x2 - int(bw * 0.2), y2 - wheel_r), wheel_r, (20, 20, 20), -1)

    # Return normalized YOLO bbox: [cls_id, x_center, y_center, width, height]
    bbox_xc = ((x1 + x2) / 2.0) / W
    bbox_yc = ((y1 + y2) / 2.0) / H
    bbox_w = (x2 - x1) / W
    bbox_h = (y2 - y1) / H
    return [cls_id, bbox_xc, bbox_yc, bbox_w, bbox_h]

def generate_synthetic_dataset(base_dir, num_samples_per_split={'train': 140, 'val': 40, 'test': 20}):
    print("=" * 70)
    print("GENERATING CUSTOM SYNTHETIC DATASET FOR GAZEBO SIMULATION OBJECTS")
    print("=" * 70)

    img_w, img_h = 640, 480

    for split, count in num_samples_per_split.items():
        img_dir = os.path.join(base_dir, "images", split)
        lbl_dir = os.path.join(base_dir, "labels", split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)

        print(f"Generating {count} samples for split: '{split}'...")

        for idx in range(count):
            # 1. Background generation: simulate indoor floors and walls with varied lighting
            bg_color = random.choice([
                (190, 190, 195),  # light concrete
                (170, 175, 180),  # office grey
                (140, 150, 160),  # warehouse floor
                (210, 215, 215),  # clinic white
                (130, 135, 140)   # asphalt/industrial
            ])
            img = np.full((img_h, img_w, 3), bg_color, dtype=np.uint8)

            # Add horizontal horizon/wall divider line
            wall_y = random.randint(int(img_h * 0.25), int(img_h * 0.55))
            wall_color = [max(0, c - random.randint(20, 50)) for c in bg_color]
            img[:wall_y, :] = wall_color

            # Add subtle illumination gradient
            grad = np.linspace(0.85, 1.15, img_h)[:, None, None]
            img = np.clip(img * grad, 0, 255).astype(np.uint8)

            # 2. Place 1 to 3 objects per frame without excessive overlap
            num_objects = random.randint(1, 3)
            bboxes = []

            for _ in range(num_objects):
                cls_id = random.randint(0, len(CLASSES) - 1)

                # Scale based on simulated distance (perspective)
                dist_factor = random.uniform(0.35, 1.0)
                if cls_id in [0, 5, 6]:  # tall/large objects: person, shelf, bed
                    obj_w = int(random.uniform(70, 130) * dist_factor)
                    obj_h = int(random.uniform(160, 280) * dist_factor)
                elif cls_id in [1, 2, 7]:  # medium objects: chair, box, cart
                    obj_w = int(random.uniform(70, 140) * dist_factor)
                    obj_h = int(random.uniform(80, 150) * dist_factor)
                else:  # compact: cone, pallet
                    obj_w = int(random.uniform(50, 110) * dist_factor)
                    obj_h = int(random.uniform(60, 110) * dist_factor)

                min_x = int(obj_w / 2) + 20
                max_x = img_w - int(obj_w / 2) - 20
                cx = random.randint(min_x, max_x) if min_x < max_x else img_w // 2

                min_y = wall_y + int(obj_h / 2)
                max_y = img_h - int(obj_h / 2) - 10
                cy = random.randint(min_y, max_y) if min_y < max_y else (min_y + max_y) // 2

                bbox = draw_object(img, cls_id, cx, cy, obj_w, obj_h)
                if bbox is not None:
                    bboxes.append(bbox)

            # 3. Save image
            img_filename = f"gazebo_{split}_{idx:04d}.jpg"
            img_path = os.path.join(img_dir, img_filename)
            cv2.imwrite(img_path, img)

            # 4. Save corresponding YOLO label
            lbl_filename = f"gazebo_{split}_{idx:04d}.txt"
            lbl_path = os.path.join(lbl_dir, lbl_filename)
            with open(lbl_path, 'w') as f:
                for b in bboxes:
                    f.write(f"{b[0]} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f} {b[4]:.6f}\n")

    # 5. Save data.yaml
    data_yaml_path = os.path.join(base_dir, "data.yaml")
    yaml_data = {
        'path': os.path.abspath(base_dir),
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test',
        'nc': len(CLASSES),
        'names': {i: name for i, name in enumerate(CLASSES)}
    }
    with open(data_yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    print("-" * 70)
    print(f"Dataset generated successfully at: {base_dir}")
    print(f"Dataset YAML config: {data_yaml_path}")
    print("=" * 70)

if __name__ == '__main__':
    target_dir = "/home/soham-darade/CV_Autonomous_Navigation/datasets/gazebo_cv"
    generate_synthetic_dataset(target_dir)
