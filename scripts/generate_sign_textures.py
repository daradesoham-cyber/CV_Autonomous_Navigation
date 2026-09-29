#!/usr/bin/env python3
"""
Generate high-contrast visual sign textures for Gazebo Sim environment.
Produces crisp PNG images with text and directional arrows.
"""
import os
import cv2
import numpy as np

SIGNS_DIR = "/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs"
os.makedirs(SIGNS_DIR, exist_ok=True)

# List of signs: (filename, text, direction, bg_color, text_color, border_color)
# Colors in BGR format
SIGNS = [
    ("hospital_right.png", "HOSPITAL", "RIGHT", (245, 245, 245), (180, 50, 20), (180, 50, 20)),
    ("hospital_straight.png", "HOSPITAL", "STRAIGHT", (245, 245, 245), (180, 50, 20), (180, 50, 20)),
    ("warehouse_straight.png", "WAREHOUSE", "STRAIGHT", (240, 240, 240), (20, 100, 200), (20, 100, 200)),
    ("warehouse_right.png", "WAREHOUSE", "RIGHT", (240, 240, 240), (20, 100, 200), (20, 100, 200)),
    ("office_straight.png", "OFFICE", "STRAIGHT", (245, 245, 245), (30, 30, 30), (30, 30, 30)),
    ("office_right.png", "OFFICE", "RIGHT", (245, 245, 245), (30, 30, 30), (30, 30, 30)),
    ("office_left.png", "OFFICE", "LEFT", (245, 245, 245), (30, 30, 30), (30, 30, 30)),
    ("lab_left.png", "LAB", "LEFT", (245, 245, 245), (150, 40, 140), (150, 40, 140)),
    ("lab_straight.png", "LAB", "STRAIGHT", (245, 245, 245), (150, 40, 140), (150, 40, 140)),
    ("storage_left.png", "STORAGE", "LEFT", (240, 240, 240), (40, 120, 180), (40, 120, 180)),
    ("storage_right.png", "STORAGE", "RIGHT", (240, 240, 240), (40, 120, 180), (40, 120, 180)),
    ("storage_straight.png", "STORAGE", "STRAIGHT", (240, 240, 240), (40, 120, 180), (40, 120, 180)),
    ("cafeteria_right.png", "CAFETERIA", "RIGHT", (245, 245, 245), (40, 140, 40), (40, 140, 40)),
    ("cafeteria_left.png", "CAFETERIA", "LEFT", (245, 245, 245), (40, 140, 40), (40, 140, 40)),
    ("exit_straight.png", "EXIT", "STRAIGHT", (240, 245, 240), (30, 140, 30), (30, 140, 30)),
    ("emergency_exit.png", "EMERGENCY EXIT", "STRAIGHT", (240, 240, 250), (20, 20, 200), (20, 20, 200)),
    ("room_a_straight.png", "ROOM A", "STRAIGHT", (250, 245, 230), (160, 80, 20), (160, 80, 20)),
    ("room_a_left.png", "ROOM A", "LEFT", (250, 245, 230), (160, 80, 20), (160, 80, 20)),
    ("room_a_right.png", "ROOM A", "RIGHT", (250, 245, 230), (160, 80, 20), (160, 80, 20)),
    ("room_b_straight.png", "ROOM B", "STRAIGHT", (240, 245, 250), (20, 100, 160), (20, 100, 160)),
    ("room_b_left.png", "ROOM B", "LEFT", (240, 245, 250), (20, 100, 160), (20, 100, 160)),
    ("room_b_right.png", "ROOM B", "RIGHT", (240, 245, 250), (20, 100, 160), (20, 100, 160)),
    ("charging_straight.png", "CHARGING", "STRAIGHT", (230, 250, 230), (20, 160, 40), (20, 160, 40)),
    ("charging_left.png", "CHARGING", "LEFT", (230, 250, 230), (20, 160, 40), (20, 160, 40)),
    ("charging_right.png", "CHARGING", "RIGHT", (230, 250, 230), (20, 160, 40), (20, 160, 40)),
    ("loading_straight.png", "LOADING", "STRAIGHT", (230, 240, 250), (20, 70, 200), (20, 70, 200)),
    ("loading_right.png", "LOADING", "RIGHT", (230, 240, 250), (20, 70, 200), (20, 70, 200)),
    ("restricted_straight.png", "RESTRICTED", "STRAIGHT", (230, 230, 250), (10, 10, 220), (10, 10, 220)),
    ("keep_left.png", "KEEP LEFT", "LEFT", (245, 245, 245), (180, 100, 20), (180, 100, 20)),
    ("keep_right.png", "KEEP RIGHT", "RIGHT", (245, 245, 245), (180, 100, 20), (180, 100, 20)),
]

def draw_arrow(img, direction, x, y, size, color):
    """Draw a bold directional arrow."""
    if direction == "RIGHT":
        # Shaft
        cv2.line(img, (x - size, y), (x + size, y), color, 14, cv2.LINE_AA)
        # Arrowhead
        pts = np.array([
            [x + size + 8, y],
            [x + size - 24, y - 28],
            [x + size - 16, y],
            [x + size - 24, y + 28]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
    elif direction == "LEFT":
        # Shaft
        cv2.line(img, (x + size, y), (x - size, y), color, 14, cv2.LINE_AA)
        # Arrowhead
        pts = np.array([
            [x - size - 8, y],
            [x - size + 24, y - 28],
            [x - size + 16, y],
            [x - size + 24, y + 28]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
    elif direction == "STRAIGHT":
        # Shaft
        cv2.line(img, (x, y + size), (x, y - size), color, 14, cv2.LINE_AA)
        # Arrowhead
        pts = np.array([
            [x, y - size - 8],
            [x - 28, y - size + 24],
            [x, y - size + 16],
            [x + 28, y - size + 24]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)

def create_sign(filename, text, direction, bg_color, text_color, border_color):
    width, height = 640, 320
    img = np.full((height, width, 3), bg_color, dtype=np.uint8)

    # Outer border
    cv2.rectangle(img, (12, 12), (width - 12, height - 12), border_color, 12)
    # Inner border
    cv2.rectangle(img, (24, 24), (width - 24, height - 24), (200, 200, 200), 2)

    # Text rendering
    font = cv2.FONT_HERSHEY_DUPLEX
    scale = 1.6 if len(text) <= 10 else 1.2
    thickness = 4
    (text_w, text_h), baseline = cv2.getTextSize(text, font, scale, thickness)

    # Layout: Text on one side / top, Arrow beside or below
    if direction == "RIGHT":
        # Text on left, Arrow on right
        tx = 50
        ty = height // 2 + text_h // 2
        cv2.putText(img, text, (tx, ty), font, scale, text_color, thickness, cv2.LINE_AA)
        ax = width - 110
        ay = height // 2
        draw_arrow(img, "RIGHT", ax, ay, 45, text_color)
    elif direction == "LEFT":
        # Arrow on left, Text on right
        ax = 110
        ay = height // 2
        draw_arrow(img, "LEFT", ax, ay, 45, text_color)
        tx = 190
        ty = height // 2 + text_h // 2
        cv2.putText(img, text, (tx, ty), font, scale, text_color, thickness, cv2.LINE_AA)
    elif direction == "STRAIGHT":
        # Text centered, Arrow centered above/beside
        tx = (width - text_w) // 2
        ty = height // 2 + text_h // 2 - 25
        cv2.putText(img, text, (tx, ty), font, scale, text_color, thickness, cv2.LINE_AA)
        ax = width // 2
        ay = height - 70
        draw_arrow(img, "STRAIGHT", ax, ay, 35, text_color)

    filepath = os.path.join(SIGNS_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"Generated sign: {filepath}")

def main():
    for sign in SIGNS:
        create_sign(*sign)
    print(f"\nAll {len(SIGNS)} sign textures successfully generated in {SIGNS_DIR}")

if __name__ == "__main__":
    main()
