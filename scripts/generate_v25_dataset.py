#!/usr/bin/env python3
"""
CV Autonomous Navigation V2.5 — Comprehensive Dataset Generator.
Generates domain-randomized indoor facility scenes matching the Gazebo realistic_facility_world:
- Realistic 3D perspective projection with camera intrinsics (hfov=1.15 rad, 640x480)
- Full 10 classes in exact order:
    0: person
    1: cart
    2: forklift
    3: pallet
    4: box
    5: obstacle
    6: door
    7: charging_station
    8: hospital_bed
    9: directional_sign
- Multi-representation: Gazebo SDF physical shapes + realistic indoor asset variants
- Complete sign texture suite (33 PNG textures) across near/medium/far and straight/oblique angles
- 15% pure negative background images (empty corridors, wall junctions, shadows) to eliminate false positives
- Zero cross-split data leakage by disjoint scene parameterization
- Dedicated diagnostic failure-case evaluation subsets
"""

import os
import sys
import glob
import math
import random
import yaml
import numpy as np
import cv2

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
SIGNS_DIR = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs")
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "datasets/v25_cv_dataset")

IMG_W = 640
IMG_H = 480
FOV_H = 1.15  # radians (~65.9 deg)
F_LEN = (IMG_W / 2.0) / math.tan(FOV_H / 2.0)  # ~500.2 px
CX = IMG_W / 2.0
CY = IMG_H / 2.0
CAM_Z = 0.35  # mounting height in meters

CLASSES = [
    'person',
    'cart',
    'forklift',
    'pallet',
    'box',
    'obstacle',
    'door',
    'charging_station',
    'hospital_bed',
    'directional_sign'
]

# Color palettes (BGR)
PALETTES = {
    'floor_concrete': [(190, 195, 200), (170, 175, 180), (150, 155, 160)],
    'floor_tiles': [(215, 218, 222), (200, 205, 210), (180, 185, 190)],
    'floor_epoxy': [(120, 150, 160), (140, 160, 170), (110, 140, 150)],
    'wall_hospital': [(235, 238, 240), (225, 230, 235), (215, 222, 228)],
    'wall_warehouse': [(210, 220, 225), (200, 210, 218), (190, 200, 208)],
    'wall_office': [(230, 235, 238), (220, 228, 232), (210, 218, 225)],
    # Gazebo SDF colors
    'gazebo_person_blue': (204, 102, 26),      # [0.1, 0.4, 0.8] BGR
    'gazebo_forklift_yellow': (13, 191, 242),   # [0.95, 0.75, 0.05] BGR
    'gazebo_cart_orange': (26, 115, 230),       # [0.9, 0.45, 0.1] BGR
    'gazebo_trolley_cyan': (217, 178, 51),     # [0.2, 0.7, 0.85] BGR
    'gazebo_bed_light': (230, 224, 217),       # [0.85, 0.88, 0.90] BGR
    'gazebo_pallet_wood': (51, 115, 153),      # [0.60, 0.45, 0.20] BGR
    'gazebo_box_cardboard': (76, 128, 166),    # [0.65, 0.50, 0.30] BGR
    'gazebo_charging_green': (51, 178, 26),    # [0.10, 0.70, 0.20] BGR
    'obstacle_orange': (15, 120, 245),
    'door_wood': (40, 65, 110),
    'door_metal': (110, 115, 120),
}


def load_all_sign_textures():
    files = sorted(glob.glob(os.path.join(SIGNS_DIR, "*.png")))
    texs = {}
    for f in files:
        bname = os.path.basename(f)
        im = cv2.imread(f)
        if im is not None:
            texs[bname] = im
    print(f"Loaded {len(texs)} distinct sign textures.")
    return texs


SIGN_TEXTURES = load_all_sign_textures()


def project_3d_point(x, y, z):
    """Camera frame: X right, Y down, Z forward."""
    if z <= 0.20:
        return None
    u = int(CX + (x / z) * F_LEN)
    v = int(CY + (y / z) * F_LEN)
    return (u, v)


def create_3d_facility_background(bg_type="corridor", light_mult=1.0, wall_color=None, floor_type="tiles"):
    """
    Renders realistic 3D perspective facility background:
    - bg_type: "corridor", "junction", "large_room"
    """
    bg = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    if wall_color is None:
        wall_color = random.choice(PALETTES['wall_hospital'] + PALETTES['wall_warehouse'])
    
    if floor_type == "tiles":
        floor_palette = PALETTES['floor_tiles']
    elif floor_type == "epoxy":
        floor_palette = PALETTES['floor_epoxy']
    else:
        floor_palette = PALETTES['floor_concrete']

    horizon_y = int(CY)
    # Ceiling
    ceiling_color = tuple(int(min(255, c * 0.88 * light_mult)) for c in wall_color)
    bg[:horizon_y, :] = ceiling_color
    
    # Floor
    f_c1 = tuple(int(min(255, c * 0.78 * light_mult)) for c in random.choice(floor_palette))
    f_c2 = tuple(int(min(255, c * 0.68 * light_mult)) for c in random.choice(floor_palette))
    bg[horizon_y:, :] = f_c1

    # Floor grid / tile lines
    num_grid = 12 if bg_type != "large_room" else 18
    for i in range(-num_grid, num_grid + 1):
        xw = i * (0.8 if bg_type != "large_room" else 1.2)
        p_far = project_3d_point(xw, CAM_Z, 14.0)
        p_near = project_3d_point(xw, CAM_Z, 0.7)
        if p_far and p_near:
            y_start = min(IMG_H - 1, max(horizon_y, p_near[1]))
            cv2.line(bg, (p_near[0], y_start), (p_far[0], horizon_y), f_c2, 1)

    for depth in [1.2, 2.0, 3.2, 4.8, 7.0, 10.0, 14.0]:
        vd = int(CY + (CAM_Z / depth) * F_LEN)
        if horizon_y <= vd < IMG_H:
            cv2.line(bg, (0, vd), (IMG_W, vd), f_c2, 1)

    # Walls
    corridor_w = random.uniform(2.6, 4.2) if bg_type != "large_room" else random.uniform(5.5, 8.5)
    wall_l_x = -corridor_w / 2.0
    wall_r_x = corridor_w / 2.0
    wall_h = 2.8

    # Left Wall
    ptl_f = project_3d_point(wall_l_x, -(wall_h - CAM_Z), 14.0)
    pbl_f = project_3d_point(wall_l_x, CAM_Z, 14.0)
    ptl_n = project_3d_point(wall_l_x, -(wall_h - CAM_Z), 0.7)
    pbl_n = project_3d_point(wall_l_x, CAM_Z, 0.7)

    if all(p is not None for p in [ptl_f, pbl_f, ptl_n, pbl_n]):
        pts_left = np.array([
            [max(-60, ptl_n[0]), max(0, ptl_n[1])],
            [ptl_f[0], ptl_f[1]],
            [pbl_f[0], pbl_f[1]],
            [max(-60, pbl_n[0]), min(IMG_H + 60, pbl_n[1])]
        ], np.int32)
        w_shade = tuple(int(min(255, c * 0.94 * light_mult)) for c in wall_color)
        cv2.fillPoly(bg, [pts_left], w_shade)
        cv2.polylines(bg, [pts_left], isClosed=False, color=(75, 80, 85), thickness=2)

    # Right Wall
    ptr_f = project_3d_point(wall_r_x, -(wall_h - CAM_Z), 14.0)
    pbr_f = project_3d_point(wall_r_x, CAM_Z, 14.0)
    ptr_n = project_3d_point(wall_r_x, -(wall_h - CAM_Z), 0.7)
    pbr_n = project_3d_point(wall_r_x, CAM_Z, 0.7)

    if all(p is not None for p in [ptr_f, pbr_f, ptr_n, pbr_n]):
        pts_right = np.array([
            [ptr_f[0], ptr_f[1]],
            [min(IMG_W + 60, ptr_n[0]), max(0, ptr_n[1])],
            [min(IMG_W + 60, pbr_n[0]), min(IMG_H + 60, pbr_n[1])],
            [pbr_f[0], pbr_f[1]]
        ], np.int32)
        w_shade = tuple(int(min(255, c * 0.90 * light_mult)) for c in wall_color)
        cv2.fillPoly(bg, [pts_right], w_shade)
        cv2.polylines(bg, [pts_right], isClosed=False, color=(75, 80, 85), thickness=2)

    # End wall or corridor junction
    if bg_type == "junction":
        # Draw side corridor opening or cross junction
        j_depth = random.uniform(3.5, 6.5)
        pw1 = project_3d_point(wall_l_x, -(wall_h - CAM_Z), j_depth)
        pw2 = project_3d_point(wall_l_x, CAM_Z, j_depth)
        pw3 = project_3d_point(wall_l_x, CAM_Z, j_depth + 1.8)
        pw4 = project_3d_point(wall_l_x, -(wall_h - CAM_Z), j_depth + 1.8)
        if all(p is not None for p in [pw1, pw2, pw3, pw4]):
            j_opening = np.array([pw1, pw4, pw3, pw2], np.int32)
            cv2.fillPoly(bg, [j_opening], tuple(int(c * 0.75 * light_mult) for c in wall_color))
            cv2.polylines(bg, [j_opening], True, (50, 55, 60), 1)

    return bg, wall_l_x, wall_r_x


def render_sign_3d(img, tex, world_x, world_y, world_z, angle_yaw=0.0, width=0.8, height=0.4):
    """
    Renders sign texture with accurate perspective homography.
    world_x, world_y, world_z: Center of sign in camera frame.
    angle_yaw: sign yaw relative to camera axis.
    """
    if world_z <= 0.6 or world_z >= 8.0:
        return None

    half_w = (width / 2.0) * math.cos(angle_yaw)
    depth_off = (width / 2.0) * math.sin(angle_yaw)
    half_h = height / 2.0

    corners_3d = [
        (world_x - half_w, world_y - half_h, world_z - depth_off),
        (world_x + half_w, world_y - half_h, world_z + depth_off),
        (world_x + half_w, world_y + half_h, world_z + depth_off),
        (world_x - half_w, world_y + half_h, world_z - depth_off),
    ]

    pts_2d = []
    for cx_3d, cy_3d, cz_3d in corners_3d:
        p2d = project_3d_point(cx_3d, cy_3d, cz_3d)
        if p2d is None:
            return None
        pts_2d.append(p2d)

    pts_dst = np.array(pts_2d, dtype=np.float32)
    th, tw = tex.shape[:2]
    pts_src = np.array([[0, 0], [tw - 1, 0], [tw - 1, th - 1], [0, th - 1]], dtype=np.float32)

    H, _ = cv2.findHomography(pts_src, pts_dst)
    if H is None:
        return None

    warped = cv2.warpPerspective(tex, H, (IMG_W, IMG_H), flags=cv2.INTER_LINEAR)
    mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.int32(pts_dst), 255)

    cv2.copyTo(warped, mask, img)
    cv2.polylines(img, [np.int32(pts_dst)], isClosed=True, color=(30, 30, 30), thickness=2)

    min_x = max(0, int(np.min(pts_dst[:, 0])))
    max_x = min(IMG_W - 1, int(np.max(pts_dst[:, 0])))
    min_y = max(0, int(np.min(pts_dst[:, 1])))
    max_y = min(IMG_H - 1, int(np.max(pts_dst[:, 1])))

    if (max_x - min_x) < 10 or (max_y - min_y) < 8:
        return None
    return [min_x, min_y, max_x, max_y]


def render_person(img, x, z, style="gazebo", light_mult=1.0):
    """
    Renders person obstacle:
    - style='gazebo': blue cylindrical obstacle matching Gazebo dynamic_person
    - style='realistic': upright human pedestrian with torso, legs, head, and shadow
    """
    y_ground = CAM_Z
    if style == "gazebo":
        # Cylinder r=0.25, h=1.7
        h_total = 1.7
        r = 0.25
        y_top = y_ground - h_total
        p_base = project_3d_point(x, y_ground, z)
        p_top = project_3d_point(x, y_top, z)
        if p_base is None or p_top is None:
            return None
        bh = p_base[1] - p_top[1]
        bw = max(6, int((2 * r / z) * F_LEN))
        cx = p_base[0]

        x1 = max(0, cx - bw // 2)
        x2 = min(IMG_W - 1, cx + bw // 2)
        y1 = max(0, p_top[1])
        y2 = min(IMG_H - 1, p_base[1])
        if (x2 - x1) < 8 or (y2 - y1) < 16:
            return None

        color = tuple(int(min(255, c * light_mult)) for c in PALETTES['gazebo_person_blue'])
        # Draw cylindrical body
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        # Rounded top cap
        cv2.ellipse(img, (cx, y1), (bw//2, max(3, bw//4)), 0, 0, 360, tuple(int(min(255, c*1.15)) for c in color), -1)
        # Base shadow
        cv2.ellipse(img, (cx, y2), (bw//2 + 4, max(3, bw//4)), 0, 0, 360, (30, 30, 35), -1)
        return [x1, y1, x2, y2]
    else:
        # Realistic pedestrian
        h_total = 1.75
        y_head = y_ground - h_total
        p_feet = project_3d_point(x, y_ground, z)
        p_head = project_3d_point(x, y_head, z)
        if p_feet is None or p_head is None:
            return None
        bh = p_feet[1] - p_head[1]
        bw = int(bh * 0.40)
        cx = p_head[0]
        x1 = max(0, cx - bw // 2)
        x2 = min(IMG_W - 1, cx + bw // 2)
        y1 = max(0, p_head[1])
        y2 = min(IMG_H - 1, p_feet[1])
        if (x2 - x1) < 8 or (y2 - y1) < 18:
            return None

        # Shadow
        cv2.ellipse(img, (cx, y2), (bw//2 + 4, max(3, bw//5)), 0, 0, 360, (35, 35, 40), -1)
        # Head
        hr = max(2, int(bh * 0.10))
        hcy = y1 + hr
        skin = (120, 160, 200)
        cv2.circle(img, (cx, hcy), hr, skin, -1)
        # Torso
        torso_top = y1 + int(bh * 0.18)
        torso_bot = y1 + int(bh * 0.58)
        jacket = random.choice([(180, 60, 40), (40, 120, 180), (30, 160, 220), (50, 50, 50)])
        cv2.rectangle(img, (cx - int(bw*0.35), torso_top), (cx + int(bw*0.35), torso_bot), jacket, -1)
        # Legs
        leg_w = max(2, int(bw * 0.16))
        pants = (40, 45, 55)
        cv2.rectangle(img, (cx - int(bw*0.35), torso_bot), (cx - int(bw*0.35) + leg_w, y2), pants, -1)
        cv2.rectangle(img, (cx + int(bw*0.35) - leg_w, torso_bot), (cx + int(bw*0.35), y2), pants, -1)
        return [x1, y1, x2, y2]


def render_cart(img, x, z, style="warehouse", light_mult=1.0):
    """
    Renders cart obstacle:
    - style='warehouse': orange box (1.2 x 0.8 x 0.6) matching Gazebo dynamic_warehouse_cart
    - style='trolley': cyan box (0.9 x 0.6 x 0.7) matching Gazebo dynamic_hospital_trolley
    - style='mesh': warehouse handcart with handle and rails
    """
    y_ground = CAM_Z
    if style == "warehouse":
        w, h, d = 1.2, 0.6, 0.8
        color = PALETTES['gazebo_cart_orange']
    elif style == "trolley":
        w, h, d = 0.9, 0.7, 0.6
        color = PALETTES['gazebo_trolley_cyan']
    else:
        w, h, d = 0.85, 0.95, 1.1
        color = (35, 45, 210)

    y_top = y_ground - h
    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    c_base = tuple(int(min(255, c * light_mult)) for c in color)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], tuple(int(min(255, c * 1.05)) for c in c_base))
    cv2.fillPoly(img, [front], tuple(int(c * 0.85) for c in c_base))
    cv2.polylines(img, [front], True, (40, 40, 45), 2)

    # Castor wheels
    wp1 = project_3d_point(x - w*0.4, y_ground + 0.04, z)
    wp2 = project_3d_point(x + w*0.4, y_ground + 0.04, z)
    if wp1:
        cv2.circle(img, wp1, max(2, int(12 / z)), (25, 25, 25), -1)
    if wp2:
        cv2.circle(img, wp2, max(2, int(12 / z)), (25, 25, 25), -1)

    pts = np.array(proj, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 10 or (y2 - y1) < 10:
        return None
    return [x1, y1, x2, y2]


def render_forklift(img, x, z, style="gazebo", light_mult=1.0):
    """
    Renders forklift:
    - style='gazebo': yellow industrial box (1.4 x 0.9 x 0.8) matching Gazebo dynamic_forklift
    - style='mast': yellow body with vertical lift mast and overhead guard
    """
    y_ground = CAM_Z
    w, h, d = 1.4, 0.8, 0.9
    y_top = y_ground - h
    color = PALETTES['gazebo_forklift_yellow']

    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    c_base = tuple(int(min(255, c * light_mult)) for c in color)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], tuple(int(min(255, c * 1.10)) for c in c_base))
    cv2.fillPoly(img, [front], tuple(int(c * 0.90) for c in c_base))
    cv2.polylines(img, [front], True, (30, 30, 30), 2)

    all_pts = list(proj)
    if style == "mast":
        # Overhead mast
        y_mast = y_top - 0.7
        pm1 = project_3d_point(x - 0.25, y_mast, z - d/2)
        pm2 = project_3d_point(x + 0.25, y_mast, z - d/2)
        if pm1 and pm2:
            cv2.line(img, pm1, pm2, (40, 40, 45), max(2, int(8 / z)))
            all_pts.extend([pm1, pm2])
        # Amber beacon
        p_beacon = project_3d_point(x, y_mast - 0.1, z - d/2)
        if p_beacon:
            cv2.circle(img, p_beacon, max(2, int(10 / z)), (20, 160, 255), -1)
            all_pts.append(p_beacon)

    pts = np.array(all_pts, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None
    return [x1, y1, x2, y2]


def render_pallet(img, x, z, light_mult=1.0):
    """Renders wooden pallet stack matching Gazebo wh_pallets / loading_pallet_stack."""
    y_ground = CAM_Z
    w, h, d = 1.2, 0.45, 1.2
    y_top = y_ground - h
    color = PALETTES['gazebo_pallet_wood']

    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    c_base = tuple(int(min(255, c * light_mult)) for c in color)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], c_base)
    cv2.fillPoly(img, [front], tuple(int(c * 0.75) for c in c_base))

    # Pallet wood slats and fork pockets
    for k in range(1, 4):
        sx = x - w/2 + k * (w / 4.0)
        p1 = project_3d_point(sx, y_top, z - d/2)
        p2 = project_3d_point(sx, y_top, z + d/2)
        if p1 and p2:
            cv2.line(img, p1, p2, (25, 45, 75), max(1, int(4 / z)))

    pts = np.array(proj, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 10 or (y2 - y1) < 8:
        return None
    return [x1, y1, x2, y2]


def render_box(img, x, z, light_mult=1.0):
    """Renders cardboard storage box stack matching Gazebo storage_box_stack."""
    y_ground = CAM_Z
    w, h, d = 1.1, 0.9, 0.9
    y_top = y_ground - h
    color = PALETTES['gazebo_box_cardboard']

    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    c_base = tuple(int(min(255, c * light_mult)) for c in color)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], tuple(int(min(255, c * 1.05)) for c in c_base))
    cv2.fillPoly(img, [front], tuple(int(c * 0.90) for c in c_base))

    # Box tape stripe (packaging tape)
    pml = ((proj[0][0] + proj[3][0]) // 2, (proj[0][1] + proj[3][1]) // 2)
    pmr = ((proj[1][0] + proj[2][0]) // 2, (proj[1][1] + proj[2][1]) // 2)
    cv2.line(img, pml, pmr, (45, 90, 130), max(2, int(7 / z)))

    # Shipping label sticker
    lbl_w = max(4, int(20 / z))
    lbl_h = max(3, int(15 / z))
    cx_box = (proj[0][0] + proj[1][0]) // 2
    cy_box = (proj[0][1] + proj[3][1]) // 2
    cv2.rectangle(img, (cx_box - lbl_w, cy_box - lbl_h), (cx_box + lbl_w, cy_box + lbl_h), (230, 235, 240), -1)

    pts = np.array(proj, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 10 or (y2 - y1) < 10:
        return None
    return [x1, y1, x2, y2]


def render_obstacle(img, x, z, light_mult=1.0):
    """Renders traffic cone / safety barrier obstacle."""
    y_ground = CAM_Z
    w, h = 0.40, 0.70
    y_top = y_ground - h

    ptop = project_3d_point(x, y_top, z)
    pbl = project_3d_point(x - w/2, y_ground, z)
    pbr = project_3d_point(x + w/2, y_ground, z)
    if any(p is None for p in [ptop, pbl, pbr]):
        return None

    color = tuple(int(min(255, c * light_mult)) for c in PALETTES['obstacle_orange'])
    cone = np.array([ptop, pbr, pbl], np.int32)
    cv2.fillPoly(img, [cone], color)

    # Reflective white stripe
    ps1 = project_3d_point(x - w*0.30, y_ground - h*0.50, z)
    ps2 = project_3d_point(x + w*0.30, y_ground - h*0.50, z)
    ps3 = project_3d_point(x + w*0.18, y_ground - h*0.32, z)
    ps4 = project_3d_point(x - w*0.18, y_ground - h*0.32, z)
    if all(p is not None for p in [ps1, ps2, ps3, ps4]):
        cv2.fillPoly(img, [np.array([ps1, ps2, ps3, ps4], np.int32)], (240, 240, 245))

    pts = np.array([ptop, pbl, pbr], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 8 or (y2 - y1) < 10:
        return None
    return [x1, y1, x2, y2]


def render_door(img, x, z, light_mult=1.0):
    """Renders doorframe opening / facility door."""
    y_ground = CAM_Z
    w, h = 1.20, 2.15
    y_top = y_ground - h

    p1 = project_3d_point(x - w/2, y_top, z)
    p2 = project_3d_point(x + w/2, y_top, z)
    p3 = project_3d_point(x + w/2, y_ground, z)
    p4 = project_3d_point(x - w/2, y_ground, z)
    if any(p is None for p in [p1, p2, p3, p4]):
        return None

    color = tuple(int(min(255, c * light_mult)) for c in PALETTES['door_wood'])
    poly = np.array([p1, p2, p3, p4], np.int32)
    cv2.fillPoly(img, [poly], color)
    cv2.polylines(img, [poly], True, (35, 40, 45), max(2, int(6 / z)))

    # Viewing window on door
    pw1 = project_3d_point(x - w*0.25, y_top + 0.35, z)
    pw2 = project_3d_point(x + w*0.25, y_top + 0.35, z)
    pw3 = project_3d_point(x + w*0.25, y_top + 0.90, z)
    pw4 = project_3d_point(x - w*0.25, y_top + 0.90, z)
    if all(p is not None for p in [pw1, pw2, pw3, pw4]):
        cv2.fillPoly(img, [np.array([pw1, pw2, pw3, pw4], np.int32)], (210, 230, 240))

    pts = np.array([p1, p2, p3, p4], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 12 or (y2 - y1) < 18:
        return None
    return [x1, y1, x2, y2]


def render_charging_station(img, x, z, light_mult=1.0):
    """Renders AGV charging dock station matching Gazebo charging_dock_station."""
    y_ground = CAM_Z
    w, h, d = 0.60, 1.00, 1.00
    y_top = y_ground - h

    p1 = project_3d_point(x - w/2, y_top, z)
    p2 = project_3d_point(x + w/2, y_top, z)
    p3 = project_3d_point(x + w/2, y_ground, z)
    p4 = project_3d_point(x - w/2, y_ground, z)
    if any(p is None for p in [p1, p2, p3, p4]):
        return None

    color = tuple(int(min(255, c * light_mult)) for c in PALETTES['gazebo_charging_green'])
    poly = np.array([p1, p2, p3, p4], np.int32)
    cv2.fillPoly(img, [poly], color)
    cv2.polylines(img, [poly], True, (25, 45, 25), 2)

    # Green LED power indicator
    p_led = project_3d_point(x, y_top + 0.20, z)
    if p_led:
        cv2.circle(img, p_led, max(2, int(8 / z)), (60, 255, 60), -1)

    pts = np.array([p1, p2, p3, p4], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 8 or (y2 - y1) < 12:
        return None
    return [x1, y1, x2, y2]


def render_hospital_bed(img, x, z, light_mult=1.0):
    """Renders medical hospital bed matching Gazebo hospital_bed_1 / hospital_bed_2."""
    y_ground = CAM_Z
    w, h, d = 1.00, 0.70, 2.00
    y_top = y_ground - h
    color = PALETTES['gazebo_bed_light']

    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    c_base = tuple(int(min(255, c * light_mult)) for c in color)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], c_base)
    cv2.fillPoly(img, [front], tuple(int(c * 0.85) for c in c_base))
    cv2.polylines(img, [front], True, (150, 155, 160), 2)

    # Bed side rails
    pr1 = project_3d_point(x - w/2, y_top - 0.20, z + d/2)
    pr2 = project_3d_point(x + w/2, y_top - 0.20, z + d/2)
    if pr1 and pr2:
        cv2.line(img, pr1, pr2, (180, 185, 190), max(2, int(5 / z)))

    pts = np.array(proj, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 12 or (y2 - y1) < 10:
        return None
    return [x1, y1, x2, y2]


def apply_photorealistic_effects(img, motion_blur=False, exposure=1.0, noise_level=0.0):
    """Applies exposure, motion blur, and sensor noise."""
    out = img.astype(np.float32) * exposure
    out = np.clip(out, 0, 255).astype(np.uint8)

    if motion_blur:
        ksize = random.choice([3, 5])
        kernel = np.zeros((ksize, ksize))
        kernel[ksize // 2, :] = np.ones(ksize) / ksize
        out = cv2.filter2D(out, -1, kernel)

    if noise_level > 0:
        noise = np.random.normal(0, noise_level, out.shape).astype(np.float32)
        out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    return out


def generate_scene(scene_id, is_negative=False, forced_sign_type=None,
                   forced_distance=None, forced_angle=None, forced_lighting=None,
                   forced_occlusion=False, forced_motion=False):
    """
    Generates a single synthetic scene.
    If is_negative=True: generates an empty facility view with 0 labels.
    """
    random.seed(scene_id)
    np.random.seed(scene_id % (2**31 - 1))

    bg_types = ["corridor", "junction", "large_room"]
    bg_type = random.choice(bg_types)
    floor_types = ["tiles", "concrete", "epoxy"]
    floor_type = random.choice(floor_types)

    light_mult = random.uniform(0.85, 1.15)
    if forced_lighting == "bright":
        light_mult = 1.30
    elif forced_lighting == "dim":
        light_mult = 0.65

    img, wall_l_x, wall_r_x = create_3d_facility_background(
        bg_type=bg_type, light_mult=light_mult, floor_type=floor_type
    )

    labels = []
    if is_negative:
        # Negative background image: 0 objects, returns image and empty labels
        img_final = apply_photorealistic_effects(
            img,
            motion_blur=(random.random() < 0.20 or forced_motion),
            exposure=light_mult,
            noise_level=random.uniform(1.0, 3.5)
        )
        return img_final, labels

    spawn_queue = []

    # 1. Spawn directional sign (in 75% of positive scenes, or 100% if forced)
    spawn_sign = (random.random() < 0.75) or (forced_sign_type is not None)
    if spawn_sign:
        if forced_sign_type and forced_sign_type in SIGN_TEXTURES:
            tex_img = SIGN_TEXTURES[forced_sign_type]
        else:
            tex_img = random.choice(list(SIGN_TEXTURES.values()))

        # Distance
        if forced_distance == "near":
            s_depth = random.uniform(1.0, 1.9)
        elif forced_distance == "medium":
            s_depth = random.uniform(2.2, 4.0)
        elif forced_distance == "far":
            s_depth = random.uniform(4.3, 7.2)
        else:
            s_depth = random.uniform(1.1, 6.5)

        # Wall placement and angle
        wall_side = random.choice(['left', 'right', 'frontal'])
        if forced_angle == "frontal" or wall_side == "frontal":
            s_x = random.uniform(-0.6, 0.6)
            s_y = -(0.95 - CAM_Z)
            s_yaw = math.radians(random.uniform(-5, 5))
        elif forced_angle == "oblique" or wall_side in ["left", "right"]:
            if wall_side == 'left':
                s_x = wall_l_x + 0.05
                s_y = -(0.95 - CAM_Z)
                s_yaw = math.radians(random.uniform(18, 42))
            else:
                s_x = wall_r_x - 0.05
                s_y = -(0.95 - CAM_Z)
                s_yaw = math.radians(random.uniform(-42, -18))

        spawn_queue.append(('directional_sign', s_depth, (tex_img, s_x, s_y, s_yaw)))

    # 2. Spawn facility obstacles (1 to 3 objects)
    candidate_classes = [c for c in CLASSES if c != 'directional_sign']
    num_objs = random.randint(1, 3)
    if forced_occlusion:
        # Guarantee an object in front of the sign
        num_objs = max(1, num_objs)

    for i in range(num_objs):
        cls_name = random.choice(candidate_classes)
        if forced_occlusion and i == 0 and spawn_queue:
            # Place obstacle directly in front of the sign to cause partial occlusion
            sign_depth = spawn_queue[0][1]
            obj_z = max(0.9, sign_depth - random.uniform(0.6, 1.2))
            obj_x = spawn_queue[0][2][1] + random.uniform(-0.25, 0.25)
        else:
            obj_z = random.uniform(1.2, 6.5)
            obj_x = random.uniform(-1.4, 1.4)
        spawn_queue.append((cls_name, obj_z, obj_x))

    # Sort back-to-front by depth descending
    spawn_queue.sort(key=lambda it: it[1], reverse=True)

    # Render objects
    for item in spawn_queue:
        cls_name, depth, params = item
        bbox = None
        cls_id = CLASSES.index(cls_name)

        if cls_name == 'directional_sign':
            tex_img, sx, sy, syaw = params
            bbox = render_sign_3d(img, tex_img, sx, sy, depth, angle_yaw=syaw)
        elif cls_name == 'person':
            style = random.choice(['gazebo', 'realistic'])
            bbox = render_person(img, params, depth, style=style, light_mult=light_mult)
        elif cls_name == 'cart':
            style = random.choice(['warehouse', 'trolley', 'mesh'])
            bbox = render_cart(img, params, depth, style=style, light_mult=light_mult)
        elif cls_name == 'forklift':
            style = random.choice(['gazebo', 'mast'])
            bbox = render_forklift(img, params, depth, style=style, light_mult=light_mult)
        elif cls_name == 'pallet':
            bbox = render_pallet(img, params, depth, light_mult=light_mult)
        elif cls_name == 'box':
            bbox = render_box(img, params, depth, light_mult=light_mult)
        elif cls_name == 'obstacle':
            bbox = render_obstacle(img, params, depth, light_mult=light_mult)
        elif cls_name == 'door':
            bbox = render_door(img, params, depth, light_mult=light_mult)
        elif cls_name == 'charging_station':
            bbox = render_charging_station(img, params, depth, light_mult=light_mult)
        elif cls_name == 'hospital_bed':
            bbox = render_hospital_bed(img, params, depth, light_mult=light_mult)

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            xc = ((x1 + x2) / 2.0) / IMG_W
            yc = ((y1 + y2) / 2.0) / IMG_H
            bw = (x2 - x1) / IMG_W
            bh = (y2 - y1) / IMG_H
            if bw > 0.015 and bh > 0.015:
                labels.append((cls_id, xc, yc, bw, bh))

    motion_blur = (random.random() < 0.25) or forced_motion
    img_final = apply_photorealistic_effects(
        img, motion_blur=motion_blur, exposure=light_mult, noise_level=random.uniform(1.0, 3.0)
    )
    return img_final, labels


def main():
    print("================================================================")
    print("CV Autonomous Navigation V2.5 — Dataset Generator")
    print("================================================================")

    for split in ['train', 'val', 'test']:
        os.makedirs(os.path.join(OUTPUT_DIR, f"images/{split}"), exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_DIR, f"labels/{split}"), exist_ok=True)

    TOTAL_SAMPLES = 2000
    N_TRAIN = 1400  # 70%
    N_VAL = 400     # 20%
    N_TEST = 200    # 10%
    NEG_RATIO = 0.15  # 15% negative background images

    splits = [('train', N_TRAIN, 10000), ('val', N_VAL, 40000), ('test', N_TEST, 70000)]
    class_counts = {c: 0 for c in CLASSES}
    neg_counts = {}

    for split_name, count, seed_offset in splits:
        print(f"\nGenerating '{split_name}' split ({count} total images, {int(count*NEG_RATIO)} negative background)...")
        num_neg = int(count * NEG_RATIO)
        neg_counts[split_name] = num_neg

        for i in range(count):
            scene_id = seed_offset + i
            is_neg = (i < num_neg)
            img, labels = generate_scene(scene_id, is_negative=is_neg)

            # Ensure positive scenes have at least 1 object
            retry = 0
            while not is_neg and len(labels) == 0 and retry < 5:
                retry += 1
                img, labels = generate_scene(scene_id + 5000 + retry, is_negative=False)

            file_stem = f"frame_{split_name}_{i:05d}"
            img_path = os.path.join(OUTPUT_DIR, f"images/{split_name}/{file_stem}.jpg")
            lbl_path = os.path.join(OUTPUT_DIR, f"labels/{split_name}/{file_stem}.txt")

            cv2.imwrite(img_path, img, [cv2.IMWRITE_JPEG_QUALITY, 94])
            with open(lbl_path, "w") as f:
                for cls_id, xc, yc, bw, bh in labels:
                    f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")
                    class_counts[CLASSES[cls_id]] += 1

            if (i + 1) % 200 == 0 or (i + 1) == count:
                print(f"  [{split_name.upper()}] {i + 1}/{count} created...")

    # Write data.yaml
    data_yaml = {
        'path': OUTPUT_DIR,
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test',
        'nc': len(CLASSES),
        'names': {i: c for i, c in enumerate(CLASSES)}
    }
    with open(os.path.join(OUTPUT_DIR, "data.yaml"), "w") as f:
        yaml.dump(data_yaml, f, sort_keys=False)

    print("\nPrimary dataset created successfully at:", OUTPUT_DIR)
    print("Negative background count per split:", neg_counts)
    print("Class annotation totals:")
    for c, cnt in class_counts.items():
        print(f"  - {c:18s}: {cnt:5d} instances")

    # -------------------------------------------------------------
    # Diagnostic Failure-Case Evaluation Subsets (Step 7 requirement)
    # -------------------------------------------------------------
    eval_subsets = [
        ('distance_near', {'forced_distance': 'near'}),
        ('distance_medium', {'forced_distance': 'medium'}),
        ('distance_far', {'forced_distance': 'far'}),
        ('angle_frontal', {'forced_angle': 'frontal'}),
        ('angle_oblique', {'forced_angle': 'oblique'}),
        ('lighting_bright', {'forced_lighting': 'bright'}),
        ('lighting_dim', {'forced_lighting': 'dim'}),
        ('occlusion', {'forced_occlusion': True}),
        ('motion_blur', {'forced_motion': True}),
    ]

    print("\nGenerating diagnostic failure-case evaluation subsets (60 images each)...")
    for sname, kwargs in eval_subsets:
        sdir = os.path.join(OUTPUT_DIR, f"eval_subsets/{sname}")
        os.makedirs(os.path.join(sdir, "images"), exist_ok=True)
        os.makedirs(os.path.join(sdir, "labels"), exist_ok=True)

        for j in range(60):
            sid = 90000 + j * 17
            img, labels = generate_scene(sid, is_negative=False, **kwargs)
            fstem = f"eval_{sname}_{j:04d}"
            cv2.imwrite(os.path.join(sdir, f"images/{fstem}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 94])
            with open(os.path.join(sdir, f"labels/{fstem}.txt"), "w") as f:
                for cls_id, xc, yc, bw, bh in labels:
                    f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")

        # yaml for this subset
        sub_yaml = {
            'path': sdir,
            'val': 'images',
            'nc': len(CLASSES),
            'names': {i: c for i, c in enumerate(CLASSES)}
        }
        with open(os.path.join(sdir, "data.yaml"), "w") as f:
            yaml.dump(sub_yaml, f, sort_keys=False)

    print("All diagnostic evaluation subsets created successfully!")
    print("================================================================")


if __name__ == '__main__':
    main()
