#!/usr/bin/env python3
"""
CV Autonomous Navigation V2.4 — Realistic Current-Map CV Dataset Generator.
Synthesizes domain-randomized facility images with 3D perspective projection,
exact texture mapping of directional navigation signs, and detailed multi-class
indoor facility assets (hospital beds, forklifts, warehouse carts, pallets, etc.).

Classes (V2.4):
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
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "datasets/v24_cv_dataset")

IMG_W = 640
IMG_H = 480
FOV_H = 1.15  # Camera horizontal FOV in radians (~65.9 deg)
F_LEN = (IMG_W / 2.0) / math.tan(FOV_H / 2.0)  # ~500.2 px
CX = IMG_W / 2.0
CY = IMG_H / 2.0
CAM_Z = 0.35  # Camera mounting height in meters

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

# Color palettes and textures for classes
PALETTES = {
    'floor_tiles': [(210, 215, 220), (195, 200, 205), (170, 175, 180), (140, 145, 150)],
    'wall_colors': [(230, 235, 240), (220, 225, 230), (200, 210, 215), (180, 190, 195)],
    'person_skin': [(140, 180, 220), (120, 160, 200), (90, 130, 170)],
    'person_clothes': [(180, 60, 40), (40, 120, 180), (50, 150, 60), (30, 30, 40), (160, 80, 160)],
    'cart_red': (35, 45, 210),
    'cart_frame': (50, 50, 50),
    'forklift_yellow': (20, 180, 230),
    'forklift_dark': (30, 30, 35),
    'pallet_wood': (45, 95, 145),
    'box_cardboard': (65, 125, 175),
    'obstacle_orange': (15, 120, 245),
    'bed_frame': (225, 230, 235),
    'bed_mattress': (180, 210, 215),
    'charging_dock': (70, 75, 80),
    'door_wood': (40, 65, 110),
}


def load_sign_textures():
    """Load all sign PNG textures."""
    tex_files = glob.glob(os.path.join(SIGNS_DIR, "*.png"))
    textures = {}
    for f in tex_files:
        basename = os.path.basename(f)
        img = cv2.imread(f)
        if img is not None:
            textures[basename] = img
    print(f"Loaded {len(textures)} sign textures from {SIGNS_DIR}")
    return textures


SIGN_TEXTURES = load_sign_textures()


def project_3d_point(x, y, z):
    """Pinhole camera projection: X right, Y down, Z forward."""
    if z <= 0.15:
        return None
    u = int(CX + (x / z) * F_LEN)
    v = int(CY + (y / z) * F_LEN)
    return (u, v)


def create_3d_facility_background(light_mult=1.0, wall_color=None):
    """Render a 3D perspective hallway or room background with floor tiles and walls."""
    bg = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
    if wall_color is None:
        wall_color = random.choice(PALETTES['wall_colors'])
    
    # Horizon line based on camera pitch
    horizon_y = int(CY)
    
    # Ceiling
    ceiling_color = tuple(int(c * 0.85 * light_mult) for c in wall_color)
    bg[:horizon_y, :] = np.clip(ceiling_color, 0, 255)
    
    # Floor
    floor_color_1 = tuple(int(c * 0.75 * light_mult) for c in random.choice(PALETTES['floor_tiles']))
    floor_color_2 = tuple(int(c * 0.65 * light_mult) for c in random.choice(PALETTES['floor_tiles']))
    bg[horizon_y:, :] = np.clip(floor_color_1, 0, 255)
    
    # Perspective tile lines on floor
    num_grid_lines = 14
    for i in range(-num_grid_lines, num_grid_lines + 1):
        x_world = i * 0.8
        p_far = project_3d_point(x_world, CAM_Z, 12.0)
        p_near = project_3d_point(x_world, CAM_Z, 0.6)
        if p_far and p_near:
            cv2.line(bg, (p_near[0], min(IMG_H - 1, max(horizon_y, p_near[1]))),
                     (p_far[0], horizon_y), (int(floor_color_2[0]*0.9), int(floor_color_2[1]*0.9), int(floor_color_2[2]*0.9)), 1)
            
    # Horizontal depth lines
    for depth in [1.0, 1.8, 2.8, 4.2, 6.0, 8.5, 12.0]:
        v_d = int(CY + (CAM_Z / depth) * F_LEN)
        if horizon_y <= v_d < IMG_H:
            cv2.line(bg, (0, v_d), (IMG_W, v_d), floor_color_2, 1)

    # Perspective Wall: Left wall or Right wall or End corridor wall
    corridor_width = random.uniform(2.4, 4.0)
    wall_left_x = -corridor_width / 2.0
    wall_right_x = corridor_width / 2.0
    wall_height = 2.8
    
    # Left wall projection
    p_tl_far = project_3d_point(wall_left_x, -(wall_height - CAM_Z), 12.0)
    p_bl_far = project_3d_point(wall_left_x, CAM_Z, 12.0)
    p_tl_near = project_3d_point(wall_left_x, -(wall_height - CAM_Z), 0.8)
    p_bl_near = project_3d_point(wall_left_x, CAM_Z, 0.8)
    
    if p_tl_far and p_bl_far and p_tl_near and p_bl_near:
        pts_left = np.array([
            [max(-50, p_tl_near[0]), max(0, p_tl_near[1])],
            [p_tl_far[0], p_tl_far[1]],
            [p_bl_far[0], p_bl_far[1]],
            [max(-50, p_bl_near[0]), min(IMG_H + 50, p_bl_near[1])]
        ], np.int32)
        wall_shade = tuple(int(c * 0.92 * light_mult) for c in wall_color)
        cv2.fillPoly(bg, [pts_left], wall_shade)
        cv2.polylines(bg, [pts_left], isClosed=False, color=(70, 75, 80), thickness=2)

    # Right wall projection
    p_tr_far = project_3d_point(wall_right_x, -(wall_height - CAM_Z), 12.0)
    p_br_far = project_3d_point(wall_right_x, CAM_Z, 12.0)
    p_tr_near = project_3d_point(wall_right_x, -(wall_height - CAM_Z), 0.8)
    p_br_near = project_3d_point(wall_right_x, CAM_Z, 0.8)
    
    if p_tr_far and p_br_far and p_tr_near and p_br_near:
        pts_right = np.array([
            [p_tr_far[0], p_tr_far[1]],
            [min(IMG_W + 50, p_tr_near[0]), max(0, p_tr_near[1])],
            [min(IMG_W + 50, p_br_near[0]), min(IMG_H + 50, p_br_near[1])],
            [p_br_far[0], p_br_far[1]]
        ], np.int32)
        wall_shade = tuple(int(c * 0.88 * light_mult) for c in wall_color)
        cv2.fillPoly(bg, [pts_right], wall_shade)
        cv2.polylines(bg, [pts_right], isClosed=False, color=(70, 75, 80), thickness=2)

    # Baseboard trims along walls
    return bg, wall_left_x, wall_right_x


def render_sign_3d(img, tex, world_x, world_y, world_z, angle_yaw=0.0, width=0.8, height=0.4):
    """
    Render directional sign in 3D onto camera image using perspective homography.
    world_x, world_y, world_z: Center of sign in camera frame (x right, y down, z forward).
    Returns bounding box [x1, y1, x2, y2] or None.
    """
    if world_z <= 0.6 or world_z >= 7.0:
        return None
    
    half_w = (width / 2.0) * math.cos(angle_yaw)
    depth_off = (width / 2.0) * math.sin(angle_yaw)
    half_h = height / 2.0

    # 4 3D Corners of sign
    # Top-Left, Top-Right, Bottom-Right, Bottom-Left
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

    # Warp sign texture onto image
    warped = cv2.warpPerspective(tex, H, (IMG_W, IMG_H), flags=cv2.INTER_LINEAR)
    mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.int32(pts_dst), 255)

    # Blend warped sign onto image
    cv2.copyTo(warped, mask, img)
    # Add thin border
    cv2.polylines(img, [np.int32(pts_dst)], isClosed=True, color=(30, 30, 30), thickness=2)

    # Compute bounding box
    min_x = max(0, int(np.min(pts_dst[:, 0])))
    max_x = min(IMG_W - 1, int(np.max(pts_dst[:, 0])))
    min_y = max(0, int(np.min(pts_dst[:, 1])))
    max_y = min(IMG_H - 1, int(np.max(pts_dst[:, 1])))

    if (max_x - min_x) < 14 or (max_y - min_y) < 10:
        return None
    return [min_x, min_y, max_x, max_y]


def render_box_3d(img, x, z, size=(0.5, 0.5, 0.5), color=None, light_mult=1.0):
    """Render cardboard box at world coordinates (x, z) on the floor."""
    if color is None:
        color = PALETTES['box_cardboard']
    w, h, d = size
    y_ground = CAM_Z
    y_top = y_ground - h

    # 8 corners
    corners = [
        (x - w/2, y_top, z - d/2), (x + w/2, y_top, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_top, z + d/2), (x + w/2, y_top, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    # Draw front face
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    side = np.array([proj[1], proj[5], proj[6], proj[2]], np.int32) if x < 0 else np.array([proj[0], proj[4], proj[7], proj[3]], np.int32)

    c_front = tuple(int(c * 0.9 * light_mult) for c in color)
    c_top = tuple(int(c * 1.05 * light_mult) for c in color)
    c_side = tuple(int(c * 0.75 * light_mult) for c in color)

    cv2.fillPoly(img, [top], c_top)
    cv2.fillPoly(img, [side], c_side)
    cv2.fillPoly(img, [front], c_front)

    # Box tape stripe
    p_mid_l = ((proj[0][0] + proj[3][0])//2, (proj[0][1] + proj[3][1])//2)
    p_mid_r = ((proj[1][0] + proj[2][0])//2, (proj[1][1] + proj[2][1])//2)
    cv2.line(img, p_mid_l, p_mid_r, (40, 80, 120), max(2, int(8 / z)))

    pts = np.array(proj, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 12 or (y2 - y1) < 12:
        return None
    return [x1, y1, x2, y2]


def render_pallet_3d(img, x, z, light_mult=1.0):
    """Render wooden industrial pallet."""
    color = PALETTES['pallet_wood']
    w, h, d = 1.0, 0.18, 1.2
    y_ground = CAM_Z
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

    pts = np.array(proj, np.int32)
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)

    c_top = tuple(int(c * 1.0 * light_mult) for c in color)
    c_front = tuple(int(c * 0.7 * light_mult) for c in color)

    cv2.fillPoly(img, [top], c_top)
    cv2.fillPoly(img, [front], c_front)

    # Wood slat gaps
    for k in range(1, 4):
        sx = x - w/2 + k * (w / 4.0)
        p1 = project_3d_point(sx, y_top, z - d/2)
        p2 = project_3d_point(sx, y_top, z + d/2)
        if p1 and p2:
            cv2.line(img, p1, p2, (20, 40, 70), max(1, int(4 / z)))

    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 12 or (y2 - y1) < 8:
        return None
    return [x1, y1, x2, y2]


def render_cart_3d(img, x, z, light_mult=1.0):
    """Render warehouse transport cart."""
    w, h, d = 0.8, 1.0, 1.2
    y_ground = CAM_Z
    y_deck = y_ground - 0.25
    y_handle = y_ground - h

    # Deck base
    corners = [
        (x - w/2, y_deck, z - d/2), (x + w/2, y_deck, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_deck, z + d/2), (x + w/2, y_deck, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    # Red body
    body_color = tuple(int(c * light_mult) for c in PALETTES['cart_red'])
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], body_color)
    cv2.fillPoly(img, [front], tuple(int(c*0.8) for c in body_color))

    # Metal mesh side rails & handle
    p_h_tl = project_3d_point(x - w/2, y_handle, z + d/2)
    p_h_tr = project_3d_point(x + w/2, y_handle, z + d/2)
    p_h_bl = project_3d_point(x - w/2, y_deck, z + d/2)
    p_h_br = project_3d_point(x + w/2, y_deck, z + d/2)
    if all(p is not None for p in [p_h_tl, p_h_tr, p_h_bl, p_h_br]):
        cv2.line(img, p_h_tl, p_h_tr, (70, 70, 75), max(2, int(6 / z)))
        cv2.line(img, p_h_tl, p_h_bl, (70, 70, 75), max(2, int(5 / z)))
        cv2.line(img, p_h_tr, p_h_br, (70, 70, 75), max(2, int(5 / z)))

    # Wheels
    w_pts = [
        project_3d_point(x - w/2 + 0.1, y_ground + 0.05, z - d/2 + 0.1),
        project_3d_point(x + w/2 - 0.1, y_ground + 0.05, z - d/2 + 0.1)
    ]
    for wp in w_pts:
        if wp:
            cv2.circle(img, wp, max(2, int(15 / z)), (20, 20, 20), -1)

    all_pts = [p for p in proj + [p_h_tl, p_h_tr] if p is not None]
    pts = np.array(all_pts, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 14 or (y2 - y1) < 14:
        return None
    return [x1, y1, x2, y2]


def render_person_3d(img, x, z, light_mult=1.0):
    """Render upright person model in corridor."""
    h_total = 1.75
    y_ground = CAM_Z
    y_head_top = y_ground - h_total

    p_feet = project_3d_point(x, y_ground, z)
    p_head = project_3d_point(x, y_head_top, z)
    if p_feet is None or p_head is None:
        return None

    bh = p_feet[1] - p_head[1]
    bw = int(bh * 0.42)
    cx = p_head[0]
    cy = (p_head[1] + p_feet[1]) // 2

    x1 = max(0, cx - bw // 2)
    x2 = min(IMG_W - 1, cx + bw // 2)
    y1 = max(0, p_head[1])
    y2 = min(IMG_H - 1, p_feet[1])

    if (x2 - x1) < 10 or (y2 - y1) < 20:
        return None

    # Head
    head_r = int(bh * 0.10)
    head_cy = y1 + head_r
    skin = tuple(int(c * light_mult) for c in random.choice(PALETTES['person_skin']))
    cv2.circle(img, (cx, head_cy), max(2, head_r), skin, -1)

    # Torso (jacket/shirt)
    clothes = tuple(int(c * light_mult) for c in random.choice(PALETTES['person_clothes']))
    torso_top = y1 + int(bh * 0.18)
    torso_bottom = y1 + int(bh * 0.60)
    torso_w = int(bw * 0.75)
    cv2.rectangle(img, (cx - torso_w//2, torso_top), (cx + torso_w//2, torso_bottom), clothes, -1)

    # Legs (trousers)
    leg_w = max(2, int(torso_w * 0.42))
    pants_color = tuple(int(c * light_mult) for c in (35, 40, 50))
    cv2.rectangle(img, (cx - torso_w//2, torso_bottom), (cx - torso_w//2 + leg_w, y2), pants_color, -1)
    cv2.rectangle(img, (cx + torso_w//2 - leg_w, torso_bottom), (cx + torso_w//2, y2), pants_color, -1)

    return [x1, y1, x2, y2]


def render_hospital_bed_3d(img, x, z, light_mult=1.0):
    """Render medical hospital bed with mattress and frame."""
    w, h, d = 0.9, 0.95, 2.0
    y_ground = CAM_Z
    y_deck = y_ground - 0.55
    y_rail = y_ground - h

    corners = [
        (x - w/2, y_deck, z - d/2), (x + w/2, y_deck, z - d/2),
        (x + w/2, y_ground, z - d/2), (x - w/2, y_ground, z - d/2),
        (x - w/2, y_deck, z + d/2), (x + w/2, y_deck, z + d/2),
        (x + w/2, y_ground, z + d/2), (x - w/2, y_ground, z + d/2),
    ]
    proj = [project_3d_point(*c) for c in corners]
    if any(p is None for p in proj):
        return None

    # Mattress
    mat_color = tuple(int(c * light_mult) for c in PALETTES['bed_mattress'])
    top = np.array([proj[0], proj[1], proj[5], proj[4]], np.int32)
    front = np.array([proj[0], proj[1], proj[2], proj[3]], np.int32)
    cv2.fillPoly(img, [top], mat_color)
    cv2.fillPoly(img, [front], tuple(int(c*0.85) for c in mat_color))

    # Bed headboard & footboard
    frame_color = tuple(int(c * light_mult) for c in PALETTES['bed_frame'])
    p_head_l = project_3d_point(x - w/2, y_rail, z + d/2)
    p_head_r = project_3d_point(x + w/2, y_rail, z + d/2)
    if p_head_l and p_head_r:
        headboard = np.array([p_head_l, p_head_r, proj[5], proj[4]], np.int32)
        cv2.fillPoly(img, [headboard], frame_color)

    # Bed legs
    cv2.line(img, proj[0], proj[3], frame_color, max(2, int(6 / z)))
    cv2.line(img, proj[1], proj[2], frame_color, max(2, int(6 / z)))

    all_pts = [p for p in proj + [p_head_l, p_head_r] if p is not None]
    pts = np.array(all_pts, np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 14 or (y2 - y1) < 14:
        return None
    return [x1, y1, x2, y2]


def render_forklift_3d(img, x, z, light_mult=1.0):
    """Render industrial forklift vehicle."""
    w, h, d = 1.1, 1.8, 1.8
    y_ground = CAM_Z
    y_chassis = y_ground - 0.7
    y_mast = y_ground - h

    # Chassis
    chassis_color = tuple(int(c * light_mult) for c in PALETTES['forklift_yellow'])
    dark_color = tuple(int(c * light_mult) for c in PALETTES['forklift_dark'])

    p_fl = project_3d_point(x - w/2, y_chassis, z - d/2)
    p_fr = project_3d_point(x + w/2, y_chassis, z - d/2)
    p_bl = project_3d_point(x - w/2, y_ground, z - d/2)
    p_br = project_3d_point(x + w/2, y_ground, z - d/2)
    p_tl = project_3d_point(x - w/2, y_chassis, z + d/2)
    p_tr = project_3d_point(x + w/2, y_chassis, z + d/2)

    if any(p is None for p in [p_fl, p_fr, p_bl, p_br, p_tl, p_tr]):
        return None

    # Draw body
    body_poly = np.array([p_fl, p_fr, p_br, p_bl], np.int32)
    cv2.fillPoly(img, [body_poly], chassis_color)

    # Mast & roll cage
    p_m1 = project_3d_point(x - w/3, y_mast, z - d/2)
    p_m2 = project_3d_point(x + w/3, y_mast, z - d/2)
    if p_m1 and p_m2:
        cv2.line(img, p_fl, p_m1, dark_color, max(2, int(8 / z)))
        cv2.line(img, p_fr, p_m2, dark_color, max(2, int(8 / z)))
        cv2.line(img, p_m1, p_m2, dark_color, max(2, int(8 / z)))

    # Amber strobe light
    p_strobe = project_3d_point(x, y_mast - 0.1, z - d/2)
    if p_strobe:
        cv2.circle(img, p_strobe, max(2, int(8 / z)), (20, 140, 255), -1)

    pts = np.array([p for p in [p_fl, p_fr, p_bl, p_br, p_m1, p_m2, p_strobe] if p is not None], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 14 or (y2 - y1) < 14:
        return None
    return [x1, y1, x2, y2]


def render_charging_station_3d(img, x, z, light_mult=1.0):
    """Render AGV charging pillar and dock."""
    w, h, d = 0.5, 1.2, 0.4
    y_ground = CAM_Z
    y_top = y_ground - h

    p1 = project_3d_point(x - w/2, y_top, z)
    p2 = project_3d_point(x + w/2, y_top, z)
    p3 = project_3d_point(x + w/2, y_ground, z)
    p4 = project_3d_point(x - w/2, y_ground, z)
    if any(p is None for p in [p1, p2, p3, p4]):
        return None

    color = tuple(int(c * light_mult) for c in PALETTES['charging_dock'])
    poly = np.array([p1, p2, p3, p4], np.int32)
    cv2.fillPoly(img, [poly], color)

    # Green LED charging halo
    p_led = project_3d_point(x, y_top + 0.2, z)
    if p_led:
        cv2.circle(img, p_led, max(2, int(8 / z)), (50, 255, 50), -1)

    pts = np.array([p1, p2, p3, p4], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 10 or (y2 - y1) < 14:
        return None
    return [x1, y1, x2, y2]


def render_obstacle_cone_3d(img, x, z, light_mult=1.0):
    """Render orange safety cone or barrier."""
    w, h = 0.35, 0.65
    y_ground = CAM_Z
    y_top = y_ground - h

    p_top = project_3d_point(x, y_top, z)
    p_bl = project_3d_point(x - w/2, y_ground, z)
    p_br = project_3d_point(x + w/2, y_ground, z)
    if any(p is None for p in [p_top, p_bl, p_br]):
        return None

    color = tuple(int(c * light_mult) for c in PALETTES['obstacle_orange'])
    cone_poly = np.array([p_top, p_br, p_bl], np.int32)
    cv2.fillPoly(img, [cone_poly], color)

    # Reflective white stripe
    p_s1 = project_3d_point(x - w*0.3, y_ground - h*0.5, z)
    p_s2 = project_3d_point(x + w*0.3, y_ground - h*0.5, z)
    p_s3 = project_3d_point(x + w*0.2, y_ground - h*0.35, z)
    p_s4 = project_3d_point(x - w*0.2, y_ground - h*0.35, z)
    if all(p is not None for p in [p_s1, p_s2, p_s3, p_s4]):
        cv2.fillPoly(img, [np.array([p_s1, p_s2, p_s3, p_s4], np.int32)], (240, 240, 240))

    pts = np.array([p_top, p_bl, p_br], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 8 or (y2 - y1) < 10:
        return None
    return [x1, y1, x2, y2]


def render_door_3d(img, x, z, light_mult=1.0):
    """Render corridor door and frame."""
    w, h = 1.1, 2.1
    y_ground = CAM_Z
    y_top = y_ground - h

    p1 = project_3d_point(x - w/2, y_top, z)
    p2 = project_3d_point(x + w/2, y_top, z)
    p3 = project_3d_point(x + w/2, y_ground, z)
    p4 = project_3d_point(x - w/2, y_ground, z)
    if any(p is None for p in [p1, p2, p3, p4]):
        return None

    color = tuple(int(c * light_mult) for c in PALETTES['door_wood'])
    poly = np.array([p1, p2, p3, p4], np.int32)
    cv2.fillPoly(img, [poly], color)
    # Frame border
    cv2.polylines(img, [poly], isClosed=True, color=(30, 30, 35), thickness=max(2, int(6 / z)))

    # Window / kick plate
    p_w1 = project_3d_point(x - w*0.25, y_top + 0.3, z)
    p_w2 = project_3d_point(x + w*0.25, y_top + 0.3, z)
    p_w3 = project_3d_point(x + w*0.25, y_top + 0.8, z)
    p_w4 = project_3d_point(x - w*0.25, y_top + 0.8, z)
    if all(p is not None for p in [p_w1, p_w2, p_w3, p_w4]):
        cv2.fillPoly(img, [np.array([p_w1, p_w2, p_w3, p_w4], np.int32)], (200, 220, 230))

    pts = np.array([p1, p2, p3, p4], np.int32)
    x1, y1 = max(0, int(np.min(pts[:, 0]))), max(0, int(np.min(pts[:, 1])))
    x2, y2 = min(IMG_W - 1, int(np.max(pts[:, 0]))), min(IMG_H - 1, int(np.max(pts[:, 1])))
    if (x2 - x1) < 14 or (y2 - y1) < 20:
        return None
    return [x1, y1, x2, y2]


def apply_domain_randomization(img):
    """Apply motion blur, exposure shift, noise, and shadow artifacts."""
    # Exposure / brightness
    gain = random.uniform(0.75, 1.25)
    bias = random.randint(-15, 15)
    img_mod = np.clip(img.astype(np.float32) * gain + bias, 0, 255).astype(np.uint8)

    # Motion blur (robot moving)
    if random.random() < 0.30:
        ksize = random.choice([3, 5])
        kernel = np.zeros((ksize, ksize))
        kernel[int((ksize - 1)/2), :] = np.ones(ksize)
        kernel /= ksize
        img_mod = cv2.filter2D(img_mod, -1, kernel)

    # Gaussian noise
    if random.random() < 0.25:
        noise = np.random.normal(0, random.uniform(2, 6), img_mod.shape).astype(np.float32)
        img_mod = np.clip(img_mod.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    # Random overhead lighting gradient / shadow
    if random.random() < 0.40:
        grad = np.tile(np.linspace(random.uniform(0.8, 1.1), random.uniform(0.7, 1.0), IMG_H)[:, None, None], (1, IMG_W, 3))
        img_mod = np.clip(img_mod.astype(np.float32) * grad, 0, 255).astype(np.uint8)

    return img_mod


def generate_single_scene(scene_id):
    """Generate one synthetic scene with 3D projection, objects, signs, and labels."""
    light_mult = random.uniform(0.8, 1.2)
    img, wall_l_x, wall_r_x = create_3d_facility_background(light_mult=light_mult)
    labels = []  # (class_id, x_c, y_c, w, h)

    # Determine objects to spawn, sorted back-to-front (largest Z first)
    spawn_objects = []

    # 1. Sign (50% chance per frame)
    if random.random() < 0.65 and len(SIGN_TEXTURES) > 0:
        tex_name, tex_img = random.choice(list(SIGN_TEXTURES.items()))
        # Place sign on left wall, right wall, or end corridor wall
        sign_depth = random.uniform(1.2, 5.0)
        wall_side = random.choice(['left', 'right', 'end'])
        
        if wall_side == 'left':
            s_x = wall_l_x + 0.05
            s_y = -(1.10 - CAM_Z)  # sign height 1.10m above floor
            s_yaw = math.radians(random.uniform(15, 45))
        elif wall_side == 'right':
            s_x = wall_r_x - 0.05
            s_y = -(1.10 - CAM_Z)
            s_yaw = math.radians(random.uniform(-45, -15))
        else:
            s_x = random.uniform(-0.8, 0.8)
            s_y = -(1.10 - CAM_Z)
            s_yaw = 0.0
            
        spawn_objects.append(('directional_sign', sign_depth, (tex_img, s_x, s_y, s_yaw)))

    # 2. Add other objects in corridor/room (1 to 4 objects)
    num_objs = random.randint(1, 4)
    candidate_classes = [c for c in CLASSES if c != 'directional_sign']
    for _ in range(num_objs):
        cls_name = random.choice(candidate_classes)
        obj_z = random.uniform(1.2, 5.8)
        obj_x = random.uniform(-1.2, 1.2)
        spawn_objects.append((cls_name, obj_z, obj_x))

    # Sort objects back-to-front by depth Z descending
    spawn_objects.sort(key=lambda item: item[1], reverse=True)

    # Render objects
    for item in spawn_objects:
        cls_name, depth, params = item
        bbox = None
        cls_id = CLASSES.index(cls_name)

        if cls_name == 'directional_sign':
            tex_img, s_x, s_y, s_yaw = params
            bbox = render_sign_3d(img, tex_img, s_x, s_y, depth, angle_yaw=s_yaw)
        elif cls_name == 'box':
            bbox = render_box_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'pallet':
            bbox = render_pallet_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'cart':
            bbox = render_cart_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'person':
            bbox = render_person_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'hospital_bed':
            bbox = render_hospital_bed_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'forklift':
            bbox = render_forklift_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'charging_station':
            bbox = render_charging_station_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'obstacle':
            bbox = render_obstacle_cone_3d(img, params, depth, light_mult=light_mult)
        elif cls_name == 'door':
            bbox = render_door_3d(img, params, depth, light_mult=light_mult)

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            # Normalize to YOLO format
            xc = ((x1 + x2) / 2.0) / IMG_W
            yc = ((y1 + y2) / 2.0) / IMG_H
            bw = (x2 - x1) / IMG_W
            bh = (y2 - y1) / IMG_H
            if bw > 0.02 and bh > 0.02:
                labels.append((cls_id, xc, yc, bw, bh))

    # Apply domain randomization (blur, exposure, noise)
    img_final = apply_domain_randomization(img)
    return img_final, labels


def main():
    print("================================================================")
    print("CV Autonomous Navigation V2.4 — Current-Map Dataset Generator")
    print("================================================================")
    os.makedirs(os.path.join(OUTPUT_DIR, "images/train"), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "images/val"), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "images/test"), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "labels/train"), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "labels/val"), exist_ok=True)
    os.makedirs(os.path.join(OUTPUT_DIR, "labels/test"), exist_ok=True)

    TOTAL_SAMPLES = 1200
    N_TRAIN = int(TOTAL_SAMPLES * 0.70)  # 840
    N_VAL = int(TOTAL_SAMPLES * 0.20)    # 240
    N_TEST = TOTAL_SAMPLES - N_TRAIN - N_VAL  # 120

    print(f"Generating {TOTAL_SAMPLES} samples: {N_TRAIN} Train | {N_VAL} Val | {N_TEST} Test")

    splits = [('train', N_TRAIN), ('val', N_VAL), ('test', N_TEST)]
    class_counts = {c: 0 for c in CLASSES}

    img_counter = 0
    for split_name, count in splits:
        print(f"Generating split '{split_name}' ({count} images)...")
        for i in range(count):
            img_counter += 1
            img, labels = generate_single_scene(img_counter)
            
            # If no labels (rare), regenerate with guaranteed sign
            while len(labels) == 0:
                img, labels = generate_single_scene(img_counter)

            file_stem = f"frame_{split_name}_{i:05d}"
            img_path = os.path.join(OUTPUT_DIR, f"images/{split_name}/{file_stem}.jpg")
            lbl_path = os.path.join(OUTPUT_DIR, f"labels/{split_name}/{file_stem}.txt")

            cv2.imwrite(img_path, img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            with open(lbl_path, "w") as f:
                for cls_id, xc, yc, bw, bh in labels:
                    f.write(f"{cls_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n")
                    class_counts[CLASSES[cls_id]] += 1

            if (i + 1) % 100 == 0 or (i + 1) == count:
                print(f"  [{split_name.upper()}] {i + 1}/{count} generated...")

    # Write data.yaml
    yaml_content = {
        'path': OUTPUT_DIR,
        'train': 'images/train',
        'val': 'images/val',
        'test': 'images/test',
        'nc': len(CLASSES),
        'names': {i: name for i, name in enumerate(CLASSES)}
    }
    yaml_path = os.path.join(OUTPUT_DIR, "data.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(yaml_content, f, sort_keys=False)

    print("\nDataset generation complete!")
    print(f"YAML config written to: {yaml_path}")
    print("\nPer-Class Annotation Distribution:")
    for cls_name, cnt in class_counts.items():
        print(f"  - {cls_name:18s}: {cnt:4d} instances")
    print("================================================================")


if __name__ == '__main__':
    main()
