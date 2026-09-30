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
    # Entrance / Reception Dividers (Y = -7.0, opening in [-1.8, 1.8] for main corridor, [-7.0, -5.4] for Cafeteria, and [-13.25, -11.75] for Emergency Exit)
    ("wall_rec_div_w1_a", -14.014, -7.0, 1.95, 0.2),
    ("wall_rec_div_w1_b", -9.375, -7.0, 4.75, 0.2),
    ("wall_rec_div_w2", -3.6, -7.0, 3.6, 0.2),
    # Hospital Wing North Wall with realistic 1.50m wide doorway at X in [5.25, 6.75]
    ("wall_rec_div_e1", 3.525, -7.0, 3.45, 0.2),
    ("wall_rec_div_e2", 9.025, -7.0, 4.55, 0.2),

    # Main Central Spine Walls (X = -1.8 and X = +1.8, running Y: -7.0 to +1.0)
    # Opening at Junction 1 (Y = -5.0) for East/West corridors: gaps in Y [-5.8, -4.2]
    ("wall_spine_w_south", -1.8, -6.4, 0.2, 1.2),
    ("wall_spine_w_mid", -1.8, -1.6, 0.2, 5.2),
    ("wall_spine_e_south", 1.8, -6.4, 0.2, 1.2),
    ("wall_spine_e_mid", 1.8, -1.6, 0.2, 5.2),

    # --- Hospital / Medical Wing (South-East: X in [3.5, 15.5], Y in [-12.5, -2.0]) ---
    # Hospital Corridor runs along Y = -5.0 from X = 1.8 to 15.0
    # Hospital Ward (South of corridor: Y in [-12.5, -7.0], X in [3.5, 9.5], doorway at Y=-7.0, X in [5.25, 6.75])
    ("wall_hosp_ward_w", 3.5, -9.5, 0.2, 5.0),
    ("wall_hosp_ward_div", 9.5, -9.5, 0.2, 5.0),
    # Triage Room (East: X in [10.5, 15.5], Y in [-6.0, -2.0])
    ("wall_triage_n", 13.0, -2.0, 5.0, 0.2),
    # Dead End 1: Hospital Equipment Alcove (X in [10.5, 15.5], Y in [-12.5, -7.0], doorway in [11.3, 12.8])
    ("wall_hosp_alcove_n", 14.1, -7.0, 2.6, 0.2),
    ("wall_hosp_alcove_s", 13.0, -11.5, 5.0, 0.2),

    # --- Cafeteria & Dining Hall (South-West: X in [-15.5, -3.5], Y in [-12.5, -4.0]) ---
    # Cafeteria corridor runs Y = -5.0 from X = -1.8 to -15.0
    # Dining Hall (South: X in [-11.0, -3.5], Y in [-12.5, -6.0])
    ("wall_cafe_e", -3.5, -9.5, 0.2, 5.0),
    ("wall_cafe_w", -11.0, -9.5, 0.2, 5.0),
    # Dead End 2: Utility Service Room (X in [-16.0, -14.4], Y in [-6.0, -3.0], 1.4m South Access corridor)
    ("wall_utility_n", -15.2, -3.0, 1.6, 0.2),
    ("wall_utility_e", -14.4, -2.937, 0.2, 3.0),

    # --- Central Crossway & North Spine (Y = +1.0 to +7.0) ---
    # Cross connector dividing walls with open corridors connecting Junction 4 to Warehouse and Storage
    # West corridor into Storage open X in [-6.5, -1.8]; wall covers X in [-12.2, -6.5]
    ("wall_cross_w", -9.35, 1.0, 5.7, 0.2),
    # East corridor into Warehouse: crossway connects X in [1.8, 7.0]; wall covers corner X in [7.0, 8.5]
    ("wall_cross_e", 7.75, 1.0, 1.5, 0.2),

    # Office Wing (North-Central: X in [-3.5, 3.5], Y in [4.0, 12.5])
    ("wall_office_w", -3.5, 8.5, 0.2, 7.0),
    ("wall_office_e_south", 3.5, 5.6, 0.2, 1.2),
    ("wall_office_e_north", 3.5, 9.9, 0.2, 4.2),
    # Dead End 3: Office File Archive (X in [0.5, 3.5], Y in [9.5, 12.5], doorway in [0.5, 1.8])
    ("wall_archive_s", 2.65, 9.5, 1.7, 0.2),

    # --- Industrial Warehouse Wing (North-East: X in [4.0, 15.5], Y in [1.5, 12.5]) ---
    # East Medical-Warehouse Connector Corridor: X = 5.0 to 7.0, Y = -4.0 to +7.0
    ("wall_wh_corridor_e", 7.0, -1.0, 0.2, 4.0),
    # Warehouse internal dividing racks / walls with open central aisle along X in [6.5, 9.5]
    ("wall_wh_div_1", 12.75, 4.0, 5.5, 0.2),
    ("wall_wh_div_2", 12.75, 9.0, 5.5, 0.2),

    # --- Research Lab & Storage (North-West: X in [-15.5, -4.0], Y in [1.5, 12.5]) ---
    # Storage section divider with clear 1.6m corridor doorway at X in [-7.0, -5.4]
    ("wall_storage_div_w", -9.25, 5.0, 4.5, 0.2),
    ("wall_storage_div_e", -4.357, 5.0, 1.4, 0.2),
    # Dead End 4: Chemical Storage Vault (X in [-15.5, -12.5], Y in [6.0, 12.5])
    ("wall_vault_e", -11.0, 9.5, 0.2, 5.0),
    ("wall_vault_s", -14.0, 6.0, 3.0, 0.2),

    # --- West Bypass Corridor (Genuine Shortcut/Alternate around storage with 1.4m clearance) ---
    # Running along X = -12.9 from Y = -1.9 to Y = 4.6 (opening into Lab & Cafeteria corridor)
    ("wall_west_bypass_inner", -12.9, 1.35, 0.2, 6.5),
]

# Architectural pillars (0.5m x 0.5m)
PILLARS = [
    ("pillar_rec_1", -1.8, -9.0, 0.5, 0.5),
    ("pillar_rec_2", 1.8, -9.0, 0.5, 0.5),
    ("pillar_center_1", -1.8, 3.5, 0.5, 0.5),
    ("pillar_center_2", 1.8, 3.5, 0.5, 0.5),
    ("pillar_wh_1", 9.0, 6.5, 0.6, 0.6),
    ("pillar_lab_1", -7.5, 8.5, 0.5, 0.5),
    ("pillar_j3", -6.8, -4.0, 0.4, 0.4),
    ("pillar_j2", 6.8, -4.0, 0.4, 0.4),
]

import yaml

# Physical Signs: Loaded dynamically from config/semantic_map.yaml as single source of truth
SEMANTIC_MAP_PATH = os.path.join(PROJECT_ROOT, "config/semantic_map.yaml")

def load_signs_from_semantic_map(yaml_path=SEMANTIC_MAP_PATH):
    if not os.path.exists(yaml_path):
        print(f"Warning: {yaml_path} not found!")
        return []
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    signs = []
    for sign_key, sdata in data.get("signs", {}).items():
        name = sdata.get("id", sign_key)
        wall = sdata.get("wall_attachment", {})
        sx = float(wall.get("x", 0.0))
        sy = float(wall.get("y", 0.0))
        sz = float(wall.get("z", 1.10))
        sroll = float(wall.get("roll", 0.0))
        spitch = float(wall.get("pitch", 0.0))
        syaw = float(wall.get("yaw", 0.0))
        tex_file = sdata.get("texture_file", "exit_straight.png")
        sw = float(sdata.get("width", 0.8))
        sh = float(sdata.get("height", 0.4))
        signs.append((name, sx, sy, sz, sroll, spitch, syaw, tex_file, sw, sh))
    return signs

SIGNS = load_signs_from_semantic_map()

# Furniture and Objects (Reception, Hospital, Warehouse, Office, Lab, Cafeteria, Loading, Charging)
FURNITURE = [
    # Reception Desk shifted to west foyer wall (x=-2.2, y=-8.0) to open central promenade sightline
    ("reception_desk", -2.2, -8.0, 0.45, 0, 0, 0, 1.8, 0.6, 0.9, (0.2, 0.2, 0.25)),
    # Waiting Benches
    ("waiting_bench_w", -3.2, -10.0, 0.25, 0, 0, 0, 1.6, 0.6, 0.5, (0.15, 0.15, 0.15)),
    ("waiting_bench_e", 2.5, -9.5, 0.25, 0, 0, 0, 1.6, 0.6, 0.5, (0.15, 0.15, 0.15)),

    # Hospital Ward Beds (x=7.5, y=-8.8 and -11.0) — cleared north doorway (Y=-7.0, X in [5.25, 6.75])
    ("hospital_bed_1", 7.5, -8.8, 0.35, 0, 0, 0, 2.0, 1.0, 0.7, (0.85, 0.88, 0.9)),
    ("hospital_bed_2", 7.5, -11.0, 0.35, 0, 0, 0, 2.0, 1.0, 0.7, (0.85, 0.88, 0.9)),
    # Bedside Tables along east dividing wall
    ("bedside_table_1", 8.9, -8.8, 0.35, 0, 0, 0, 0.5, 0.5, 0.7, (0.9, 0.9, 0.9)),
    ("bedside_table_2", 8.9, -11.0, 0.35, 0, 0, 0, 0.5, 0.5, 0.7, (0.9, 0.9, 0.9)),

    # Warehouse Shelves / Racks (Rows at x=11.5, 13.5; wh_rack_1 sized 1.6m centered at y=3.1 to open south loading access lane)
    ("wh_rack_1", 11.5, 3.1, 1.0, 0, 0, 0, 0.8, 1.6, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_2", 11.5, 7.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_3", 11.5, 11.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_4", 14.0, 2.5, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_5", 14.0, 7.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    ("wh_rack_6", 14.0, 11.0, 1.0, 0, 0, 0, 0.8, 2.4, 2.0, (0.2, 0.4, 0.7)),
    # Warehouse Pallet Stack
    ("wh_pallets", 8.0, 11.5, 0.4, 0, 0, 0, 1.2, 1.2, 0.8, (0.6, 0.45, 0.2)),
    # Loading Zone Pallet Stack
    ("loading_pallet_stack", 13.5, 3.5, 0.4, 0, 0, 0, 1.2, 1.2, 0.8, (0.65, 0.5, 0.25)),
    # Workstation in Warehouse along office partition wall
    ("wh_workstation_1", 5.0, 3.5, 0.45, 0, 0, 0, 2.0, 0.9, 0.9, (0.3, 0.35, 0.4)),

    # Executive Office Desks & Chairs (x=-1.5 and +1.5, y=11.0)
    ("office_desk_1", -1.5, 11.0, 0.38, 0, 0, 0, 1.5, 0.8, 0.75, (0.35, 0.25, 0.15)),
    ("office_desk_2", 1.5, 11.0, 0.38, 0, 0, 0, 1.5, 0.8, 0.75, (0.35, 0.25, 0.15)),
    ("office_chair_1", -1.5, 11.8, 0.45, 0, 0, 0, 0.6, 0.6, 0.9, (0.1, 0.1, 0.1)),
    ("office_chair_2", 1.5, 11.8, 0.45, 0, 0, 0, 0.6, 0.6, 0.9, (0.1, 0.1, 0.1)),

    # Research Lab Benches (shifted to x=-7.6 to open north-south transit corridor X: [-6.5, -3.5])
    ("lab_bench_1", -7.6, 7.0, 0.45, 0, 0, 0, 2.2, 0.8, 0.9, (0.75, 0.75, 0.78)),
    ("lab_bench_2", -7.6, 11.0, 0.45, 0, 0, 0, 2.2, 0.8, 0.9, (0.75, 0.75, 0.78)),

    # Room A Assembly Bench along west lab wall
    ("room_a_assembly_bench", -9.8, 9.5, 0.45, 0, 0, 0, 1.0, 2.4, 0.9, (0.4, 0.5, 0.6)),

    # Charging Station Pylon & Dock against outer west wall
    ("charging_dock_station", -15.2, 2.0, 0.5, 0, 0, 0, 0.6, 1.0, 1.0, (0.1, 0.7, 0.2)),

    # Cafeteria Dining Tables & Chairs along west dining wall
    ("cafe_table_1", -8.5, -9.0, 0.38, 0, 0, 0, 1.4, 0.9, 0.75, (0.8, 0.6, 0.4)),
    ("cafe_table_2", -8.5, -11.0, 0.38, 0, 0, 0, 1.4, 0.9, 0.75, (0.8, 0.6, 0.4)),
    ("cafe_chair_1", -8.5, -8.2, 0.4, 0, 0, 0, 0.5, 0.5, 0.8, (0.9, 0.3, 0.2)),
    ("cafe_chair_2", -8.5, -9.8, 0.4, 0, 0, 0, 0.5, 0.5, 0.8, (0.9, 0.3, 0.2)),

    # Storage Boxes against west wall
    ("storage_box_stack", -10.0, 3.0, 0.5, 0, 0, 0, 1.2, 1.0, 1.0, (0.65, 0.5, 0.3)),
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
        sdf.append("            <ambient>1.0 1.0 1.0 1.0</ambient>")
        sdf.append("            <diffuse>1.0 1.0 1.0 1.0</diffuse>")
        sdf.append("            <specular>0.2 0.2 0.2 1.0</specular>")
        sdf.append("            <emissive>0.08 0.08 0.08 1.0</emissive>")
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
    sdf.append("    <!-- Dynamic Moving Obstacle 1: Autonomous Warehouse Cart -->")
    sdf.append("    <model name='dynamic_warehouse_cart'>")
    sdf.append("      <pose>8.0 3.2035 0.3 0 0 1.5708</pose>")
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
    sdf.append("      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'>")
    sdf.append("        <link_name>cart_link</link_name>")
    sdf.append("        <topic>/cart/cmd_vel</topic>")
    sdf.append("      </plugin>")
    sdf.append("    </model>")
    sdf.append("")
    sdf.append("    <!-- Dynamic Moving Obstacle 2: Hospital Trolley (Patrols medical corridor along Y=-5.0) -->")
    sdf.append("    <model name='dynamic_hospital_trolley'>")
    sdf.append("      <pose>7.7935 -5.0 0.35 0 0 0</pose>")
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
    sdf.append("      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'>")
    sdf.append("        <link_name>trolley_link</link_name>")
    sdf.append("        <topic>/trolley/cmd_vel</topic>")
    sdf.append("      </plugin>")
    sdf.append("    </model>")
    sdf.append("")
    sdf.append("    <!-- Dynamic Moving Obstacle 3: Autonomous Forklift -->")
    sdf.append("    <model name='dynamic_forklift'>")
    sdf.append("      <pose>11.5 4.8 0.4 0 0 1.5708</pose>")
    sdf.append("      <link name='forklift_link'>")
    sdf.append("        <inertial>")
    sdf.append("          <mass>40.0</mass>")
    sdf.append("          <inertia><ixx>2.5</ixx><iyy>2.5</iyy><izz>2.5</izz></inertia>")
    sdf.append("        </inertial>")
    sdf.append("        <collision name='forklift_collision'>")
    sdf.append("          <geometry><box><size>1.4 0.9 0.8</size></box></geometry>")
    sdf.append("        </collision>")
    sdf.append("        <visual name='forklift_visual'>")
    sdf.append("          <geometry><box><size>1.4 0.9 0.8</size></box></geometry>")
    sdf.append("          <material>")
    sdf.append("            <ambient>0.95 0.75 0.05 1</ambient>")
    sdf.append("            <diffuse>0.95 0.75 0.05 1</diffuse>")
    sdf.append("          </material>")
    sdf.append("        </visual>")
    sdf.append("      </link>")
    sdf.append("      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'>")
    sdf.append("        <link_name>forklift_link</link_name>")
    sdf.append("        <topic>/forklift/cmd_vel</topic>")
    sdf.append("      </plugin>")
    sdf.append("    </model>")
    sdf.append("")
    sdf.append("    <!-- Dynamic Moving Obstacle 4: Walking Personnel -->")
    sdf.append("    <model name='dynamic_person'>")
    sdf.append("      <pose>-6.0 -6.5 0.85 0 0 1.5708</pose>")
    sdf.append("      <link name='person_link'>")
    sdf.append("        <inertial>")
    sdf.append("          <mass>70.0</mass>")
    sdf.append("          <inertia><ixx>5.0</ixx><iyy>5.0</iyy><izz>1.5</izz></inertia>")
    sdf.append("        </inertial>")
    sdf.append("        <collision name='person_collision'>")
    sdf.append("          <geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry>")
    sdf.append("        </collision>")
    sdf.append("        <visual name='person_visual'>")
    sdf.append("          <geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry>")
    sdf.append("          <material>")
    sdf.append("            <ambient>0.1 0.4 0.8 1</ambient>")
    sdf.append("            <diffuse>0.1 0.4 0.8 1</diffuse>")
    sdf.append("          </material>")
    sdf.append("        </visual>")
    sdf.append("      </link>")
    sdf.append("      <plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'>")
    sdf.append("        <link_name>person_link</link_name>")
    sdf.append("        <topic>/person/cmd_vel</topic>")
    sdf.append("      </plugin>")
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
        # West Bypass Corridor & South Access: X in [-14.6, -12.8], Y in [-5.0, 5.0]
        (-14.6, -5.0, -12.8, 5.0),
        # West Bypass North Connection into Lab: X in [-14.6, -11.4], Y in [4.4, 6.0]
        (-14.6, 4.4, -11.4, 6.0),
        # Loading Bay Area: X in [8.0, 14.0], Y in [0.5, 3.5]
        (8.0, 0.5, 14.0, 3.5),
        # Charging Area: X in [-14.5, -11.0], Y in [0.5, 3.5]
        (-14.5, 0.5, -11.0, 3.5),
        # Room A & Lab Passage: X in [-9.0, -5.0], Y in [6.0, 11.5]
        (-9.0, 6.0, -5.0, 11.5),
        # Room B & Hospital Ward: X in [4.5, 8.5], Y in [-11.5, -6.5]
        (4.5, -11.5, 8.5, -6.5),
        # Cafeteria Doorway: X in [-7.0, -5.2], Y in [-8.0, -6.0]
        (-7.0, -8.0, -5.2, -6.0),
        # Storage-Lab North-South Corridor Doorway: X in [-7.0, -5.0], Y in [4.0, 6.0]
        (-7.0, 4.0, -5.0, 6.0),
        # Office-Warehouse Doorway: X in [2.5, 4.5], Y in [6.2, 7.8]
        (2.5, 6.2, 4.5, 7.8),
        # Warehouse Main Aisle: X in [6.8, 9.8], Y in [1.0, 11.5]
        (6.8, 1.0, 9.8, 11.5),
        # Hospital Ward Entry (1.5m wide): X in [5.2, 6.8], Y in [-7.6, -6.4]
        (5.2, -7.6, 6.8, -6.4),
        # Hospital Alcove & Triage Doorway (1.5m wide): X in [11.2, 12.9], Y in [-7.6, -6.4]
        (11.2, -7.6, 12.9, -6.4),
        # Emergency Exit Corridor South: X in [-13.6, -11.2], Y in [-9.5, -4.5]
        (-13.6, -9.5, -11.2, -4.5),
        # Chemical Vault Entry: X in [-12.8, -10.5], Y in [5.5, 7.5]
        (-12.8, 5.5, -10.5, 7.5),
        # Office Archive Doorway: X in [0.4, 2.0], Y in [8.5, 10.5]
        (0.4, 8.5, 2.0, 10.5),
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
