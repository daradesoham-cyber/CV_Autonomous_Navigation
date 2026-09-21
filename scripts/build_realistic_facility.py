#!/usr/bin/env python3
"""
Build Realistic Facility World and Matching 2D Occupancy Map.
Creates:
1. worlds/realistic_facility_world.sdf (32m x 26m indoor hospital/warehouse/office/lab facility)
2. maps/realistic_facility_map.pgm (0.05m/cell high-resolution occupancy grid)
3. maps/realistic_facility_map.yaml (Nav2 map server configuration)
"""

import os
import math
import numpy as np
import cv2

PROJECT_ROOT = "/home/soham-darade/CV_Autonomous_Navigation"
WORLDS_DIR = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_gazebo/worlds")
MAPS_DIR = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_navigation/maps")
SIGNS_DIR = os.path.join(PROJECT_ROOT, "ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs")

os.makedirs(WORLDS_DIR, exist_ok=True)
os.makedirs(MAPS_DIR, exist_ok=True)

# ==============================================================================
# 1. GEOMETRY DEFINITION (32m x 26m Facility)
# ==============================================================================
# Bounds: X in [-16.0, 16.0], Y in [-13.0, 13.0]
# Wall height: 2.8m, thickness: 0.2m

WALLS = [
    # --- Outer Perimeter ---
    # South Wall (with entrance opening at X in [-1.5, 1.5], Y = -13.0)
    ("wall_south_left", -8.75, -13.0, 14.5, 0.2),
    ("wall_south_right", 8.75, -13.0, 14.5, 0.2),
    # North Wall
    ("wall_north", 0.0, 13.0, 32.0, 0.2),
    # West Wall (with emergency exit door at Y = -9.0)
    ("wall_west_south", -16.0, -11.0, 0.2, 4.0),
    ("wall_west_mid_north", -16.0, 2.0, 0.2, 22.0),
    # East Wall
    ("wall_east", 16.0, 0.0, 0.2, 26.0),

    # --- Interior Room Partitions ---
    # Entrance / Reception Dividers (Y = -7.0, opening in [-1.8, 1.8] for main corridor)
    ("wall_rec_div_w", -8.5, -7.0, 13.4, 0.2),
    ("wall_rec_div_e", 8.5, -7.0, 13.4, 0.2),

    # Main Central Spine Walls (X = -1.8 and X = +1.8, running Y: -7.0 to +1.0)
    # Opening at Junction 1 (Y = -5.0) for East/West corridors: gaps in Y [-5.8, -4.2]
    ("wall_spine_w_south", -1.8, -6.4, 0.2, 1.2),
    ("wall_spine_w_mid", -1.8, -1.6, 0.2, 5.2),
    ("wall_spine_e_south", 1.8, -6.4, 0.2, 1.2),
    ("wall_spine_e_mid", 1.8, -1.6, 0.2, 5.2),

    # --- Hospital / Medical Wing (South-East: X in [3.5, 15.5], Y in [-12.5, -2.0]) ---
    # Hospital Corridor runs along Y = -5.0 from X = 1.8 to 15.0
    # Hospital Ward (South of corridor: Y in [-12.5, -6.0], X in [3.5, 9.5])
    ("wall_hosp_ward_w", 3.5, -9.5, 0.2, 5.0),
    ("wall_hosp_ward_div", 9.5, -9.5, 0.2, 5.0),
    # Triage Room (East: X in [10.5, 15.5], Y in [-6.0, -2.0])
    ("wall_triage_n", 13.0, -2.0, 5.0, 0.2),
    # Dead End 1: Hospital Equipment Alcove (X in [10.5, 15.5], Y in [-12.5, -7.0])
    ("wall_hosp_alcove_n", 13.5, -7.0, 4.0, 0.2),
    ("wall_hosp_alcove_s", 13.0, -11.5, 5.0, 0.2),

    # --- Cafeteria & Dining Hall (South-West: X in [-15.5, -3.5], Y in [-12.5, -4.0]) ---
    # Cafeteria corridor runs Y = -5.0 from X = -1.8 to -15.0
    # Dining Hall (South: X in [-11.0, -3.5], Y in [-12.5, -6.0])
    ("wall_cafe_e", -3.5, -9.5, 0.2, 5.0),
    ("wall_cafe_w", -11.0, -9.5, 0.2, 5.0),
    # Dead End 2: Utility Service Room (X in [-15.5, -12.5], Y in [-6.0, -3.0])
    ("wall_utility_n", -14.0, -3.0, 3.0, 0.2),
    ("wall_utility_e", -12.5, -4.5, 0.2, 3.0),

    # --- Central Crossway & North Spine (Y = +1.0 to +7.0) ---
    # Cross connector at Y = 1.0 (opening X in [-1.8, 1.8])
    ("wall_cross_w", -7.0, 1.0, 10.4, 0.2),
    ("wall_cross_e", 7.0, 1.0, 10.4, 0.2),

    # Office Wing (North-Central: X in [-3.5, 3.5], Y in [4.0, 12.5])
    ("wall_office_w", -3.5, 8.5, 0.2, 7.0),
    ("wall_office_e", 3.5, 8.5, 0.2, 7.0),
    # Dead End 3: Office File Archive (X in [0.5, 3.5], Y in [9.5, 12.5])
    ("wall_archive_s", 2.0, 9.5, 3.0, 0.2),

    # --- Industrial Warehouse Wing (North-East: X in [4.0, 15.5], Y in [1.5, 12.5]) ---
    # East Medical-Warehouse Connector Corridor: X = 5.0 to 7.0, Y = -4.0 to +7.0
    ("wall_wh_corridor_e", 7.0, -1.0, 0.2, 4.0),
    # Warehouse internal dividing racks / walls
    ("wall_wh_div_1", 11.5, 4.0, 7.0, 0.2),
    ("wall_wh_div_2", 11.5, 9.0, 7.0, 0.2),

    # --- Research Lab & Storage (North-West: X in [-15.5, -4.0], Y in [1.5, 12.5]) ---
    # Storage section divider (Y = 5.0, X in [-12.0, -4.0])
    ("wall_storage_div", -8.0, 5.0, 8.0, 0.2),
    # Dead End 4: Chemical Storage Vault (X in [-15.5, -11.0], Y in [6.0, 12.5])
    ("wall_vault_e", -11.0, 9.5, 0.2, 5.0),
    ("wall_vault_s", -13.5, 6.0, 4.0, 0.2),

    # --- West Bypass Corridor (Genuine Shortcut/Alternate around storage) ---
    # Running along X = -14.0 from Y = -2.0 to Y = 5.0 (opening into Lab & Cafeteria corridor)
    ("wall_west_bypass_inner", -13.0, 1.5, 0.2, 7.0),
]

# Architectural pillars (0.5m x 0.5m)
PILLARS = [
    ("pillar_rec_1", -1.8, -9.0, 0.5, 0.5),
    ("pillar_rec_2", 1.8, -9.0, 0.5, 0.5),
    ("pillar_center_1", -1.8, 3.5, 0.5, 0.5),
    ("pillar_center_2", 1.8, 3.5, 0.5, 0.5),
    ("pillar_wh_1", 9.0, 6.5, 0.6, 0.6),
    ("pillar_lab_1", -7.0, 8.5, 0.5, 0.5),
]

# Physical Signs: (name, x, y, z, roll, pitch, yaw, texture_filename, width, height)
SIGNS = [
    # 1. Entrance / Junction 1 (x=0.0, y=-5.0 facing South): Hospital ->
    ("sign_j1_hospital", 0.0, -4.8, 1.4, 0, 0, 3.14159, "hospital_right.png", 0.8, 0.4),
    # 2. Junction 1 facing East: Hospital Straight
    ("sign_j1_east", 2.0, -5.0, 1.4, 0, 0, -1.5708, "hospital_straight.png", 0.8, 0.4),
    # 3. Junction 1 facing North: Office / Warehouse Straight
    ("sign_j1_north", 0.0, -3.5, 1.4, 0, 0, 3.14159, "office_straight.png", 0.8, 0.4),
    # 4. Junction 1 facing West: Cafeteria ->
    ("sign_j1_west", -2.0, -5.0, 1.4, 0, 0, 1.5708, "cafeteria_right.png", 0.8, 0.4),
    # 5. Junction 2 (x=6.0, y=-5.0 facing West): Warehouse ^ / Hospital ->
    ("sign_j2_hosp_ward", 6.0, -5.2, 1.4, 0, 0, 0.0, "hospital_right.png", 0.8, 0.4),
    ("sign_j2_warehouse", 6.2, -4.8, 1.4, 0, 0, 3.14159, "warehouse_straight.png", 0.8, 0.4),
    # 6. Junction 3 (x=-6.0, y=-5.0 facing East): Cafeteria -> / Exit <-
    ("sign_j3_cafeteria", -6.0, -5.2, 1.4, 0, 0, 0.0, "cafeteria_right.png", 0.8, 0.4),
    ("sign_j3_exit", -6.2, -4.8, 1.4, 0, 0, 3.14159, "exit_straight.png", 0.8, 0.4),
    # 7. Junction 4 (x=0.0, y=1.0 facing South): Office ^ / Warehouse -> / Lab <-
    ("sign_j4_office", 0.0, 1.2, 1.4, 0, 0, 3.14159, "office_straight.png", 0.8, 0.4),
    ("sign_j4_warehouse", 1.6, 1.0, 1.4, 0, 0, -1.5708, "warehouse_right.png", 0.8, 0.4),
    ("sign_j4_lab", -1.6, 1.0, 1.4, 0, 0, 1.5708, "lab_left.png", 0.8, 0.4),
    # 8. Junction 5 (x=0.0, y=7.0 facing South): Office ^
    ("sign_j5_office", 0.0, 7.2, 1.4, 0, 0, 3.14159, "office_straight.png", 0.8, 0.4),
    # 9. Emergency Exit (x=-15.8, y=-9.0 facing East): Emergency Exit
    ("sign_emerg_exit", -15.8, -9.0, 1.5, 0, 0, 1.5708, "emergency_exit.png", 0.8, 0.4),
]

# Furniture and Objects (Reception, Hospital, Warehouse, Office, Lab, Cafeteria)
FURNITURE = [
    # Reception Desk at (0, -8.0)
    ("reception_desk", 0.0, -8.0, 0.45, 0, 0, 0, 2.4, 0.8, 0.9, (0.2, 0.2, 0.25)),
    # Waiting Benches
    ("waiting_bench_w", -2.5, -9.5, 0.25, 0, 0, 0, 1.6, 0.6, 0.5, (0.15, 0.15, 0.15)),
    ("waiting_bench_e", 2.5, -9.5, 0.25, 0, 0, 0, 1.6, 0.6, 0.5, (0.15, 0.15, 0.15)),

    # Hospital Ward Beds (x=6.5, y=-8.0 and -10.5)
    ("hospital_bed_1", 6.5, -8.0, 0.35, 0, 0, 0, 2.0, 1.0, 0.7, (0.85, 0.88, 0.9)),
    ("hospital_bed_2", 6.5, -10.5, 0.35, 0, 0, 0, 2.0, 1.0, 0.7, (0.85, 0.88, 0.9)),
    # Bedside Tables
    ("bedside_table_1", 8.0, -8.0, 0.35, 0, 0, 0, 0.5, 0.5, 0.7, (0.9, 0.9, 0.9)),
    ("bedside_table_2", 8.0, -10.5, 0.35, 0, 0, 0, 0.5, 0.5, 0.7, (0.9, 0.9, 0.9)),

    # Warehouse Shelves / Racks (Rows at x=10.0, 12.5, 14.5)
    ("wh_rack_1", 10.0, 2.5, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_2", 10.0, 7.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_3", 10.0, 11.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_4", 13.5, 2.5, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_5", 13.5, 7.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_6", 13.5, 11.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    # Warehouse Pallet Stack
    ("wh_pallets", 8.0, 11.5, 0.4, 0, 0, 0, 1.2, 1.2, 0.8, (0.6, 0.45, 0.2)),

    # Executive Office Desks & Chairs (x=-1.5 and +1.5, y=11.0)
    ("office_desk_1", -1.5, 11.0, 0.38, 0, 0, 0, 1.5, 0.8, 0.75, (0.35, 0.25, 0.15)),
    ("office_desk_2", 1.5, 11.0, 0.38, 0, 0, 0, 1.5, 0.8, 0.75, (0.35, 0.25, 0.15)),
    ("office_chair_1", -1.5, 11.8, 0.45, 0, 0, 0, 0.6, 0.6, 0.9, (0.1, 0.1, 0.1)),
    ("office_chair_2", 1.5, 11.8, 0.45, 0, 0, 0, 0.6, 0.6, 0.9, (0.1, 0.1, 0.1)),

    # Research Lab Benches (x=-7.0, y=7.0 and 10.0)
    ("lab_bench_1", -7.0, 7.0, 0.45, 0, 0, 0, 2.5, 0.9, 0.9, (0.75, 0.75, 0.78)),
    ("lab_bench_2", -7.0, 10.0, 0.45, 0, 0, 0, 2.5, 0.9, 0.9, (0.75, 0.75, 0.78)),

    # Cafeteria Dining Tables & Chairs (x=-7.0, y=-8.0 and -10.5)
    ("cafe_table_1", -7.0, -8.0, 0.38, 0, 0, 0, 1.4, 0.9, 0.75, (0.8, 0.6, 0.4)),
    ("cafe_table_2", -7.0, -10.5, 0.38, 0, 0, 0, 1.4, 0.9, 0.75, (0.8, 0.6, 0.4)),
    ("cafe_chair_1", -7.0, -7.2, 0.4, 0, 0, 0, 0.5, 0.5, 0.8, (0.9, 0.3, 0.2)),
    ("cafe_chair_2", -7.0, -8.8, 0.4, 0, 0, 0, 0.5, 0.5, 0.8, (0.9, 0.3, 0.2)),

    # Storage Boxes (x=-9.0, y=3.0)
    ("storage_box_stack", -9.0, 3.0, 0.5, 0, 0, 0, 1.2, 1.0, 1.0, (0.65, 0.5, 0.3)),
]

# Lights (Differentiated intensity and color temperature)
LIGHTS = [
    # Main Entrance & Reception: Warm Neutral (3800K, 0.8 intensity)
    ("light_reception", 0.0, -9.0, 2.6, 0.9, 0.85, 0.75, 15.0),
    # Central Crossway: Neutral White (4500K)
    ("light_center_1", 0.0, -5.0, 2.6, 0.95, 0.95, 0.95, 12.0),
    ("light_center_2", 0.0, 1.0, 2.6, 0.95, 0.95, 0.95, 12.0),
    # Hospital Wing: Cool Clinical White (5500K, bright)
    ("light_hosp_ward", 6.5, -9.0, 2.6, 0.95, 0.98, 1.0, 18.0),
    ("light_hosp_corridor", 6.0, -5.0, 2.6, 0.95, 0.98, 1.0, 14.0),
    # Warehouse: Warm Industrial Sodium/LED (3200K, slightly dimmer)
    ("light_warehouse_1", 10.0, 4.0, 2.6, 0.95, 0.85, 0.65, 14.0),
    ("light_warehouse_2", 10.0, 9.0, 2.6, 0.95, 0.85, 0.65, 14.0),
    # Research Lab: Cool White (5000K)
    ("light_lab", -7.0, 8.5, 2.6, 0.92, 0.96, 1.0, 16.0),
    # Cafeteria: Warm Ambient (3000K)
    ("light_cafeteria", -7.0, -9.0, 2.6, 1.0, 0.85, 0.65, 15.0),
    # Emergency Exit / South-West: Dimmer green/white
    ("light_emerg_exit", -13.0, -7.0, 2.6, 0.8, 0.9, 0.8, 10.0),
]


def generate_sdf():
    sdf = []
    sdf.append("<?xml version='1.0' encoding='utf-8'?>")
    sdf.append("<sdf version='1.9'>")
    sdf.append("  <world name='realistic_facility_world'>")
    sdf.append("    <gravity>0 0 -9.8</gravity>")
    sdf.append("    <physics name='default_physics' default='true' type='ode'>")
    sdf.append("      <max_step_size>0.005</max_step_size>")
    sdf.append("      <real_time_factor>1.0</real_time_factor>")
    sdf.append("      <real_time_update_rate>200</real_time_update_rate>")
    sdf.append("    </physics>")
    sdf.append("")
    sdf.append("    <!-- Core Plugins -->")
    sdf.append("    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>")
    sdf.append("    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>")
    sdf.append("    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>")
    sdf.append("    <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'>")
    sdf.append("      <render_engine>ogre2</render_engine>")
    sdf.append("    </plugin>")
    sdf.append("    <plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'/>")
    sdf.append("")
    sdf.append("    <!-- Scene & Ambient -->")
    sdf.append("    <scene>")
    sdf.append("      <ambient>0.6 0.6 0.65 1.0</ambient>")
    sdf.append("      <background>0.85 0.88 0.92 1.0</background>")
    sdf.append("      <shadows>true</shadows>")
    sdf.append("      <grid>false</grid>")
    sdf.append("    </scene>")
    sdf.append("")
    sdf.append("    <!-- Directional Overhead Sunlight -->")
    sdf.append("    <light type='directional' name='sun'>")
    sdf.append("      <cast_shadows>true</cast_shadows>")
    sdf.append("      <pose>0 0 15 0 0 0</pose>")
    sdf.append("      <diffuse>0.7 0.7 0.75 1</diffuse>")
    sdf.append("      <specular>0.3 0.3 0.3 1</specular>")
    sdf.append("      <direction>0.2 0.2 -1</direction>")
    sdf.append("    </light>")
    sdf.append("")

    # Add Point Lights
    for name, lx, ly, lz, lr, lg, lb, dist in LIGHTS:
        sdf.append(f"    <light type='point' name='{name}'>")
        sdf.append(f"      <pose>{lx} {ly} {lz} 0 0 0</pose>")
        sdf.append(f"      <diffuse>{lr} {lg} {lb} 1</diffuse>")
        sdf.append(f"      <specular>0.2 0.2 0.2 1</specular>")
        sdf.append(f"      <attenuation><range>{dist}</range><constant>0.3</constant><linear>0.08</linear><quadratic>0.015</quadratic></attenuation>")
        sdf.append(f"      <cast_shadows>false</cast_shadows>")
        sdf.append(f"    </light>")

    # Ground Plane with Floor Texture
    sdf.append("")
    sdf.append("    <!-- Realistic Facility Ground Plane (36m x 30m) -->")
    sdf.append("    <model name='ground_plane'>")
    sdf.append("      <static>true</static>")
    sdf.append("      <link name='link'>")
    sdf.append("        <collision name='collision'>")
    sdf.append("          <geometry><plane><normal>0 0 1</normal><size>36 30</size></plane></geometry>")
    sdf.append("        </collision>")
    sdf.append("        <visual name='visual'>")
    sdf.append("          <geometry><plane><normal>0 0 1</normal><size>36 30</size></plane></geometry>")
    sdf.append("          <material>")
    sdf.append("            <ambient>0.82 0.82 0.85 1</ambient>")
    sdf.append("            <diffuse>0.82 0.82 0.85 1</diffuse>")
    sdf.append("            <specular>0.15 0.15 0.15 1</specular>")
    sdf.append("          </material>")
    sdf.append("        </visual>")
    sdf.append("      </link>")
    sdf.append("    </model>")
    sdf.append("")

    # Build Walls
    sdf.append("    <!-- Facility Walls (2.8m height) -->")
    for name, wx, wy, sx, sy in WALLS:
        sdf.append(f"    <model name='{name}'>")
        sdf.append("      <static>true</static>")
        sdf.append(f"      <pose>{wx:.3f} {wy:.3f} 1.4 0 0 0</pose>")
        sdf.append("      <link name='link'>")
        sdf.append("        <collision name='collision'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} 2.8</size></box></geometry>")
        sdf.append("        </collision>")
        sdf.append("        <visual name='visual'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} 2.8</size></box></geometry>")
        sdf.append("          <material>")
        sdf.append("            <ambient>0.88 0.88 0.85 1</ambient>")
        sdf.append("            <diffuse>0.88 0.88 0.85 1</diffuse>")
        sdf.append("            <specular>0.1 0.1 0.1 1</specular>")
        sdf.append("          </material>")
        sdf.append("        </visual>")
        sdf.append("      </link>")
        sdf.append("    </model>")

    # Build Pillars
    sdf.append("")
    sdf.append("    <!-- Architectural Support Pillars -->")
    for name, px, py, sx, sy in PILLARS:
        sdf.append(f"    <model name='{name}'>")
        sdf.append("      <static>true</static>")
        sdf.append(f"      <pose>{px:.3f} {py:.3f} 1.4 0 0 0</pose>")
        sdf.append("      <link name='link'>")
        sdf.append("        <collision name='collision'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} 2.8</size></box></geometry>")
        sdf.append("        </collision>")
        sdf.append("        <visual name='visual'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} 2.8</size></box></geometry>")
        sdf.append("          <material>")
        sdf.append("            <ambient>0.7 0.72 0.75 1</ambient>")
        sdf.append("            <diffuse>0.7 0.72 0.75 1</diffuse>")
        sdf.append("          </material>")
        sdf.append("        </visual>")
        sdf.append("      </link>")
        sdf.append("    </model>")

    # Build Furniture
    sdf.append("")
    sdf.append("    <!-- Facility Furniture and Semantic Objects -->")
    for name, fx, fy, fz, fr, fp, fyaw, sx, sy, sz, color in FURNITURE:
        cr, cg, cb = color
        sdf.append(f"    <model name='{name}'>")
        sdf.append("      <static>true</static>")
        sdf.append(f"      <pose>{fx:.3f} {fy:.3f} {fz:.3f} {fr} {fp} {fyaw}</pose>")
        sdf.append("      <link name='link'>")
        sdf.append("        <collision name='collision'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>")
        sdf.append("        </collision>")
        sdf.append("        <visual name='visual'>")
        sdf.append(f"          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>")
        sdf.append("          <material>")
        sdf.append(f"            <ambient>{cr:.2f} {cg:.2f} {cb:.2f} 1</ambient>")
        sdf.append(f"            <diffuse>{cr:.2f} {cg:.2f} {cb:.2f} 1</diffuse>")
        sdf.append("          </material>")
        sdf.append("        </visual>")
        sdf.append("      </link>")
        sdf.append("    </model>")

    # Build Directional Signs with PBR Albedo Textures
    sdf.append("")
    sdf.append("    <!-- Directional Semantic Signposts with PBR Textures -->")
    for name, sx, sy, sz, sroll, spitch, syaw, tex_file, sw, sh in SIGNS:
        tex_path = os.path.join(SIGNS_DIR, tex_file)
        sdf.append(f"    <model name='{name}'>")
        sdf.append("      <static>true</static>")
        sdf.append(f"      <pose>{sx:.3f} {sy:.3f} {sz:.3f} {sroll} {spitch} {syaw:.4f}</pose>")
        sdf.append("      <link name='link'>")
        sdf.append("        <visual name='board_visual'>")
        sdf.append(f"          <geometry><box><size>{sw} 0.03 {sh}</size></box></geometry>")
        sdf.append("          <material>")
        sdf.append("            <pbr>")
        sdf.append("              <metal>")
        sdf.append(f"                <albedo_map>{tex_path}</albedo_map>")
        sdf.append("                <roughness>0.2</roughness>")
        sdf.append("                <metalness>0.0</metalness>")
        sdf.append("              </metal>")
        sdf.append("            </pbr>")
        sdf.append("          </material>")
        sdf.append("        </visual>")
        sdf.append("        <collision name='board_collision'>")
        sdf.append(f"          <geometry><box><size>{sw} 0.03 {sh}</size></box></geometry>")
        sdf.append("        </collision>")
        sdf.append("      </link>")
        sdf.append("    </model>")

    # Dynamic Moving Warehouse Cart (Patrols Y in [3.0, 9.0] at X = 8.0)
    sdf.append("")
    sdf.append("    <!-- Dynamic Moving Obstacle: Autonomous Warehouse Cart -->")
    sdf.append("    <model name='dynamic_warehouse_cart'>")
    sdf.append("      <pose>8.0 3.5 0.3 0 0 1.5708</pose>")
    sdf.append("      <link name='cart_link'>")
    sdf.append("        <inertial>")
    sdf.append("          <mass>25.0</mass>")
    sdf.append("          <inertia><ixx>1.5</ixx><iyy>1.5</iyy><izz>1.5</izz></inertia>")
    sdf.append("        </inertial>")
    sdf.append("        <collision name='cart_collision'>")
    sdf.append("          <geometry><box><size>1.2 0.8 0.6</size></box></geometry>")
    sdf.append("        </collision>")
    sdf.append("        <visual name='cart_visual'>")
    sdf.append("          <geometry><box><size>1.2 0.8 0.6</size></box></geometry>")
    sdf.append("          <material>")
    sdf.append("            <ambient>0.9 0.45 0.1 1</ambient>")
    sdf.append("            <diffuse>0.9 0.45 0.1 1</diffuse>")
    sdf.append("          </material>")
    sdf.append("        </visual>")
    sdf.append("      </link>")
    sdf.append("    </model>")

    # Dynamic Moving Hospital Trolley (Patrols Y in [-8.0, -5.0] at X = 6.0)
    sdf.append("")
    sdf.append("    <!-- Dynamic Moving Obstacle: Hospital Trolley -->")
    sdf.append("    <model name='dynamic_hospital_trolley'>")
    sdf.append("      <pose>6.0 -6.5 0.35 0 0 0</pose>")
    sdf.append("      <link name='trolley_link'>")
    sdf.append("        <inertial>")
    sdf.append("          <mass>18.0</mass>")
    sdf.append("          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>")
    sdf.append("        </inertial>")
    sdf.append("        <collision name='trolley_collision'>")
    sdf.append("          <geometry><box><size>0.9 0.6 0.7</size></box></geometry>")
    sdf.append("        </collision>")
    sdf.append("        <visual name='trolley_visual'>")
    sdf.append("          <geometry><box><size>0.9 0.6 0.7</size></box></geometry>")
    sdf.append("          <material>")
    sdf.append("            <ambient>0.2 0.7 0.85 1</ambient>")
    sdf.append("            <diffuse>0.2 0.7 0.85 1</diffuse>")
    sdf.append("          </material>")
    sdf.append("        </visual>")
    sdf.append("      </link>")
    sdf.append("    </model>")

    sdf.append("  </world>")
    sdf.append("</sdf>")
    return "\n".join(sdf)


# ==============================================================================
# 2. OCCUPANCY MAP GENERATION (0.05m / cell, 700x600 pixels)
# ==============================================================================
# Map origin: [-17.5, -15.0]
# Width: 700 px * 0.05 = 35.0m (X: -17.5 to +17.5)
# Height: 600 px * 0.05 = 30.0m (Y: -15.0 to +15.0)

RESOLUTION = 0.05
ORIGIN_X = -17.5
ORIGIN_Y = -15.0
MAP_W = 700
MAP_H = 600

def world_to_map(wx, wy):
    mx = int(round((wx - ORIGIN_X) / RESOLUTION))
    my = int(round((wy - ORIGIN_Y) / RESOLUTION))
    # In PGM, row 0 is top (Y_max), so invert Y
    my_inv = MAP_H - 1 - my
    return mx, my_inv

def generate_occupancy_map():
    # 254: Free space, 0: Occupied wall/obstacle, 205: Unknown
    # Start with free space inside bounds, unknown outside
    grid = np.full((MAP_H, MAP_W), 254, dtype=np.uint8)

    # Mark outside perimeter as occupied / border
    bx_min, by_min = world_to_map(-16.1, -13.1)
    bx_max, by_max = world_to_map(16.1, 13.1)
    # Fill border
    grid[0:by_max, :] = 205
    grid[by_min:MAP_H, :] = 205
    grid[:, 0:bx_min] = 205
    grid[:, bx_max:MAP_W] = 205

    # Draw Walls
    for name, wx, wy, sx, sy in WALLS:
        w_half = sx / 2.0
        h_half = sy / 2.0
        x1, y1 = world_to_map(wx - w_half, wy - h_half)
        x2, y2 = world_to_map(wx + w_half, wy + h_half)
        # Bounding box
        xmin, xmax = min(x1, x2), max(x1, x2)
        ymin, ymax = min(y1, y2), max(y1, y2)
        cv2.rectangle(grid, (xmin, ymin), (xmax, ymax), 0, -1)

    # Draw Pillars
    for name, px, py, sx, sy in PILLARS:
        x1, y1 = world_to_map(px - sx/2.0, py - sy/2.0)
        x2, y2 = world_to_map(px + sx/2.0, py + sy/2.0)
        cv2.rectangle(grid, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), 0, -1)

    # Draw Major Furniture / Racks
    for name, fx, fy, fz, fr, fp, fyaw, sx, sy, sz, color in FURNITURE:
        x1, y1 = world_to_map(fx - sx/2.0, fy - sy/2.0)
        x2, y2 = world_to_map(fx + sx/2.0, fy + sy/2.0)
        cv2.rectangle(grid, (min(x1, x2), min(y1, y2)), (max(x1, x2), max(y1, y2)), 0, -1)

    # Ensure doorways / corridor passages are clear
    corridors = [
        # Entrance to Reception: X in [-1.5, 1.5], Y in [-13.0, -7.0]
        (-1.5, -13.0, 1.5, -7.0),
        # Junction 1 East corridor: X in [1.5, 6.5], Y in [-5.6, -4.4]
        (1.5, -5.6, 6.5, -4.4),
        # Junction 1 West corridor: X in [-6.5, -1.5], Y in [-5.6, -4.4]
        (-6.5, -5.6, -1.5, -4.4),
        # Central Spine: X in [-1.5, 1.5], Y in [-5.0, 1.0]
        (-1.5, -5.0, 1.5, 1.0),
        # Junction 4 to Warehouse: X in [1.5, 8.0], Y in [0.4, 1.6]
        (1.5, 0.4, 8.0, 1.6),
        # Junction 4 to Lab: X in [-8.0, -1.5], Y in [0.4, 1.6]
        (-8.0, 0.4, -1.5, 1.6),
        # Office Corridor: X in [-1.5, 1.5], Y in [1.0, 8.0]
        (-1.5, 1.0, 1.5, 8.0),
        # East Medical-Warehouse Bypass: X in [5.4, 6.6], Y in [-5.0, 2.0]
        (5.4, -5.0, 6.6, 2.0),
        # West Bypass Corridor: X in [-14.6, -13.4], Y in [-5.0, 5.0]
        (-14.6, -5.0, -13.4, 5.0),
    ]

    for cx1, cy1, cx2, cy2 in corridors:
        mx1, my1 = world_to_map(cx1, cy1)
        mx2, my2 = world_to_map(cx2, cy2)
        cv2.rectangle(grid, (min(mx1, mx2), min(my1, my2)), (max(mx1, mx2), max(my1, my2)), 254, -1)

    return grid


def main():
    print("==================================================")
    print("Building Realistic Facility World & Occupancy Map")
    print("==================================================")

    # 1. Write SDF World
    world_sdf_path = os.path.join(WORLDS_DIR, "realistic_facility_world.sdf")
    sdf_content = generate_sdf()
    with open(world_sdf_path, "w") as f:
        f.write(sdf_content)
    print(f"[OK] Wrote realistic facility SDF to: {world_sdf_path}")

    # Also sync to install share if it exists
    install_world_path = os.path.join(
        PROJECT_ROOT,
        "ros2_ws/install/autonomous_robot_gazebo/share/autonomous_robot_gazebo/worlds/realistic_facility_world.sdf"
    )
    if os.path.exists(os.path.dirname(install_world_path)):
        with open(install_world_path, "w") as f:
            f.write(sdf_content)
        print(f"[OK] Synced SDF to install directory: {install_world_path}")

    # 2. Write PGM Occupancy Map
    map_pgm_path = os.path.join(MAPS_DIR, "realistic_facility_map.pgm")
    grid = generate_occupancy_map()
    cv2.imwrite(map_pgm_path, grid)
    print(f"[OK] Wrote 700x600 PGM map to: {map_pgm_path}")

    # Also sync PGM to install share
    install_map_pgm = os.path.join(
        PROJECT_ROOT,
        "ros2_ws/install/autonomous_robot_navigation/share/autonomous_robot_navigation/maps/realistic_facility_map.pgm"
    )
    if os.path.exists(os.path.dirname(install_map_pgm)):
        cv2.imwrite(install_map_pgm, grid)
        print(f"[OK] Synced PGM to install directory: {install_map_pgm}")

    # 3. Write YAML Map Metadata
    map_yaml_path = os.path.join(MAPS_DIR, "realistic_facility_map.yaml")
    yaml_content = f"""image: realistic_facility_map.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(map_yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"[OK] Wrote map YAML metadata to: {map_yaml_path}")

    install_map_yaml = os.path.join(
        PROJECT_ROOT,
        "ros2_ws/install/autonomous_robot_navigation/share/autonomous_robot_navigation/maps/realistic_facility_map.yaml"
    )
    if os.path.exists(os.path.dirname(install_map_yaml)):
        with open(install_map_yaml, "w") as f:
            f.write(yaml_content)
        print(f"[OK] Synced YAML to install directory: {install_map_yaml}")

    print("==================================================")
    print("Facility Generation Complete.")
    print("==================================================")


if __name__ == "__main__":
    main()
