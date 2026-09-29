#!/usr/bin/env python3
"""
Generate crisp visual navigation sign textures from config/semantic_map.yaml.
Supports:
  - Text + Arrow Direction (LEFT, RIGHT, STRAIGHT)
  - Color styling per department (Medical: Red, Industrial: Blue, Emergency: Green, Service: Dark Green, Lab: Purple)
  - High-visibility border & clear typography
  - Dynamic / Mutation signs (e.g. STORAGE CLOSED)
"""

import os
import sys
import yaml
import cv2
import numpy as np

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
MAP_YAML = os.path.join(PROJECT_ROOT, "config/semantic_map.yaml")
SIGNS_DIR = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs")

os.makedirs(SIGNS_DIR, exist_ok=True)

# Color palettes (BGR format)
PALETTES = {
    "medical": {"bg": (250, 250, 250), "text": (180, 50, 20), "border": (180, 50, 20)},
    "industrial": {"bg": (245, 245, 245), "text": (20, 100, 200), "border": (20, 100, 200)},
    "emergency": {"bg": (245, 250, 245), "text": (20, 140, 30), "border": (20, 140, 30)},
    "service": {"bg": (240, 250, 240), "text": (20, 150, 40), "border": (20, 150, 40)},
    "lab": {"bg": (250, 245, 250), "text": (140, 40, 150), "border": (140, 40, 150)},
    "administrative": {"bg": (250, 250, 250), "text": (40, 40, 40), "border": (40, 40, 40)},
    "storage": {"bg": (245, 245, 245), "text": (40, 120, 180), "border": (40, 120, 180)},
    "public": {"bg": (250, 250, 250), "text": (50, 50, 50), "border": (50, 50, 50)},
    "blocked": {"bg": (230, 230, 250), "text": (20, 20, 220), "border": (20, 20, 220)},
    "default": {"bg": (245, 245, 245), "text": (40, 40, 40), "border": (40, 40, 40)}
}

def draw_arrow(img, direction, x, y, size, color):
    if direction == "RIGHT":
        cv2.line(img, (x - size, y), (x + size, y), color, 14, cv2.LINE_AA)
        pts = np.array([
            [x + size + 8, y],
            [x + size - 24, y - 28],
            [x + size - 16, y],
            [x + size - 24, y + 28]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
    elif direction == "LEFT":
        cv2.line(img, (x - size, y), (x + size, y), color, 14, cv2.LINE_AA)
        pts = np.array([
            [x - size - 8, y],
            [x - size + 24, y - 28],
            [x - size + 16, y],
            [x - size + 24, y + 28]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
    elif direction == "STRAIGHT":
        cv2.line(img, (x, y + size), (x, y - size), color, 14, cv2.LINE_AA)
        pts = np.array([
            [x, y - size - 8],
            [x - 28, y - size + 24],
            [x, y - size + 16],
            [x + 28, y - size + 24]
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
    elif direction == "BLOCKED":
        # Draw crossed circle / no-entry symbol
        cv2.circle(img, (x, y), size, color, 12, cv2.LINE_AA)
        cv2.line(img, (x - int(size*0.7), y - int(size*0.7)), (x + int(size*0.7), y + int(size*0.7)), color, 12, cv2.LINE_AA)

def create_sign(filename, text, direction, palette):
    width, height = 800, 400
    img = np.full((height, width, 3), palette["bg"], dtype=np.uint8)

    # Outer border
    cv2.rectangle(img, (12, 12), (width - 12, height - 12), palette["border"], 12)
    cv2.rectangle(img, (22, 22), (width - 22, height - 22), (255, 255, 255), 4)

    # Determine layout based on direction
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 1.9 if len(text) <= 9 else 1.5
    thickness = 4

    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)

    if direction == "RIGHT":
        text_x = 50
        text_y = (height + text_h) // 2
        arrow_x = width - 130
        arrow_y = height // 2
    elif direction == "LEFT":
        text_x = width - text_w - 50
        text_y = (height + text_h) // 2
        arrow_x = 130
        arrow_y = height // 2
    elif direction == "STRAIGHT":
        text_x = (width - text_w) // 2
        text_y = height - 90
        arrow_x = width // 2
        arrow_y = 130
    elif direction == "BLOCKED":
        text_x = (width - text_w) // 2
        text_y = height - 70
        arrow_x = width // 2
        arrow_y = 140
    else:
        text_x = (width - text_w) // 2
        text_y = (height + text_h) // 2
        arrow_x = None
        arrow_y = None

    # Draw text
    cv2.putText(img, text, (text_x, text_y), font, font_scale, palette["text"], thickness, cv2.LINE_AA)

    # Draw arrow if applicable
    if arrow_x is not None:
        draw_arrow(img, direction, arrow_x, arrow_y, 50, palette["border"])

    out_path = os.path.join(SIGNS_DIR, filename)
    cv2.imwrite(out_path, img)
    print(f"  [OK] Generated texture: {filename} -> '{text}' ({direction})")

def main():
    print("=" * 80)
    print("   AUTOMATIC NAVIGATION SIGN TEXTURE GENERATOR (FROM SEMANTIC MAP)")
    print("=" * 80)

    with open(MAP_YAML, 'r') as f:
        data = yaml.safe_load(f)

    destinations = data.get('destinations', {})
    signs = data.get('signs', {})

    # Generate standard signs
    for s_id, s_info in signs.items():
        fname = s_info.get('texture_file')
        text = s_info.get('text', '')
        direction = s_info.get('direction', 'STRAIGHT')
        dest = s_info.get('destination', '')
        dest_cat = destinations.get(dest, {}).get('category', 'default')
        palette = PALETTES.get(dest_cat, PALETTES['default'])
        create_sign(fname, text, direction, palette)

    # Generate dynamic / scenario signs
    dynamic_scenarios = data.get('dynamic_sign_scenarios', {})
    for scn_id, scn_info in dynamic_scenarios.items():
        fname = scn_info.get('new_texture_file')
        text = scn_info.get('new_text', '')
        direction = scn_info.get('new_direction', 'STRAIGHT')
        palette = PALETTES['blocked'] if direction == "BLOCKED" else PALETTES['storage']
        create_sign(fname, text, direction, palette)

    print("\n[SUCCESS] All physical and dynamic sign textures generated successfully.")

if __name__ == '__main__':
    main()
