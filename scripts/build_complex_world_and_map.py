#!/usr/bin/env python3
"""
Script: build_complex_world_and_map.py
Generates the extended Gazebo Sim 10.5 SDF world (complex_world.sdf) with:
- Maze sections, multiple alternative routes, loops, and 4 distinct dead ends.
- High-contrast visual signposts with PBR textures at all key junctions.
- Generates the matching 600x600 complex_map.pgm for AMCL and Nav2.
"""

import os
import cv2
import numpy as np

PROJ_DIR = "/home/soham-darade/CV_Autonomous_Navigation"
SDF_PATH = os.path.join(PROJ_DIR, "ros2_ws/src/autonomous_robot_gazebo/worlds/complex_world.sdf")
MAP_PGM_PATH = os.path.join(PROJ_DIR, "ros2_ws/src/autonomous_robot_navigation/maps/complex_map.pgm")
SIGNS_DIR = os.path.join(PROJ_DIR, "ros2_ws/src/autonomous_robot_gazebo/materials/textures/signs")

# Generate the SDF content
def generate_sdf():
    sdf = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="complex_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Essential Gazebo Sim Systems -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <!-- Ground Plane (40m x 40m) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>40 40</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>40 40</size></plane>
          </geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ==================== OUTER BOUNDARY WALLS (28m x 28m) ==================== -->
    <model name="boundary_walls">
      <static>true</static>
      <!-- North Wall -->
      <link name="north_wall">
        <pose>0 14 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>28.4 0.4 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>28.4 0.4 2.5</size></box></geometry><material><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>
      </link>
      <!-- South Wall -->
      <link name="south_wall">
        <pose>0 -14 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>28.4 0.4 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>28.4 0.4 2.5</size></box></geometry><material><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>
      </link>
      <!-- East Wall -->
      <link name="east_wall">
        <pose>14 0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.4 28.4 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.4 28.4 2.5</size></box></geometry><material><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>
      </link>
      <!-- West Wall -->
      <link name="west_wall">
        <pose>-14 0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.4 28.4 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.4 28.4 2.5</size></box></geometry><material><diffuse>0.7 0.72 0.75 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== INTERIOR DIVIDERS & CORRIDORS ==================== -->
    <model name="interior_dividers">
      <static>true</static>
      <!-- Divider between South and North: West Segment with Bypass Opening at x=-8 -->
      <link name="div_sn_west_a">
        <pose>-11.5 0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>5.0 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>5.0 0.3 2.5</size></box></geometry><material><diffuse>0.6 0.65 0.7 1</diffuse></material></visual>
      </link>
      <!-- West Bypass Opening is between x=-9.0 and x=-7.0 -->
      <link name="div_sn_west_b">
        <pose>-4.0 0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>6.0 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6.0 0.3 2.5</size></box></geometry><material><diffuse>0.6 0.65 0.7 1</diffuse></material></visual>
      </link>

      <!-- Central Archway is between x=-1.0 and x=+2.0 -->
      <link name="div_sn_east">
        <pose>8.0 0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>12.0 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>12.0 0.3 2.5</size></box></geometry><material><diffuse>0.6 0.65 0.7 1</diffuse></material></visual>
      </link>

      <!-- Divider between Zone A (Open) and Zone B (Warehouse) with corridor gap at y=-10 -->
      <link name="divider_ab_south">
        <pose>0 -9 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 10 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 10 2.5</size></box></geometry><material><diffuse>0.6 0.65 0.7 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== DEAD END 1: ZONE A STORAGE ALCOVE ==================== -->
    <model name="dead_end_1_storage_alcove">
      <static>true</static>
      <!-- Wall enclosing storage alcove -->
      <link name="alcove_wall_north">
        <pose>-11.0 -6.0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>6.0 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6.0 0.3 2.5</size></box></geometry><material><diffuse>0.65 0.6 0.6 1</diffuse></material></visual>
      </link>
      <link name="alcove_wall_east">
        <pose>-8.0 -4.5 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 3.0 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 3.0 2.5</size></box></geometry><material><diffuse>0.65 0.6 0.6 1</diffuse></material></visual>
      </link>
      <!-- Dead end obstruction (piles of crates/boxes) at (-12, -4.5) -->
      <link name="alcove_boxes">
        <pose>-12.0 -4.5 0.6 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.6 2.0 1.2</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.6 2.0 1.2</size></box></geometry><material><diffuse>0.6 0.45 0.25 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== DEAD END 2: WAREHOUSE CUL-DE-SAC ==================== -->
    <model name="dead_end_2_warehouse_culdesac">
      <static>true</static>
      <!-- Partition wall at x=11.0 separating outer aisle from dead end -->
      <link name="culdesac_wall">
        <pose>10.5 -12.5 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 3.0 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 3.0 2.5</size></box></geometry><material><diffuse>0.6 0.6 0.65 1</diffuse></material></visual>
      </link>
      <!-- Obstruction pallets blocking the end at (12.5, -12.5) -->
      <link name="culdesac_pallets">
        <pose>12.5 -12.5 0.7 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.5 1.5 1.4</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.5 1.5 1.4</size></box></geometry><material><diffuse>0.5 0.35 0.2 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== DEAD END 3: OFFICE UTILITY CLOSET ==================== -->
    <model name="dead_end_3_utility_closet">
      <static>true</static>
      <!-- Corridor walls forming narrow utility dead end at (-12, 12) -->
      <link name="closet_wall_south">
        <pose>-11.0 10.5 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>6.0 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>6.0 0.3 2.5</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
      <link name="closet_wall_east">
        <pose>-8.0 12.0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 3.0 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 3.0 2.5</size></box></geometry><material><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== DEAD END 4: HOSPITAL EAST ALCOVE ==================== -->
    <model name="dead_end_4_hospital_alcove">
      <static>true</static>
      <!-- Wall blocking the far east hospital corridor at (12, 8) -->
      <link name="hospital_alcove_block">
        <pose>12.0 8.0 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>3.6 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>3.6 0.3 2.5</size></box></geometry><material><diffuse>0.7 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== ZONE A: OPEN PLAZA & PILLARS ==================== -->
    <model name="zone_a_pillars">
      <static>true</static>
      <link name="p1">
        <pose>-10 -5 1.25 0 0 0</pose>
        <collision name="col"><geometry><cylinder><radius>0.4</radius><length>2.5</length></cylinder></geometry></collision>
        <visual name="vis"><geometry><cylinder><radius>0.4</radius><length>2.5</length></cylinder></geometry><material><diffuse>0.8 0.4 0.1 1</diffuse></material></visual>
      </link>
      <link name="p2">
        <pose>-6 -8 1.25 0 0 0</pose>
        <collision name="col"><geometry><cylinder><radius>0.4</radius><length>2.5</length></cylinder></geometry></collision>
        <visual name="vis"><geometry><cylinder><radius>0.4</radius><length>2.5</length></cylinder></geometry><material><diffuse>0.8 0.4 0.1 1</diffuse></material></visual>
      </link>
      <link name="p3">
        <pose>-9 -11 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.8 0.8 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.8 0.8 2.5</size></box></geometry><material><diffuse>0.3 0.5 0.8 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== ZONE B: WAREHOUSE AISLES & SHELVES ==================== -->
    <model name="warehouse_shelves">
      <static>true</static>
      <!-- Rack Row 1 (Main Aisle Border) -->
      <link name="shelf1">
        <pose>4 -8 1.2 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.0 8.0 2.4</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.0 8.0 2.4</size></box></geometry><material><diffuse>0.2 0.3 0.6 1</diffuse></material></visual>
      </link>
      <!-- Rack Row 2 -->
      <link name="shelf2">
        <pose>8 -8 1.2 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.0 8.0 2.4</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.0 8.0 2.4</size></box></geometry><material><diffuse>0.2 0.3 0.6 1</diffuse></material></visual>
      </link>
      <!-- Rack Row 3 -->
      <link name="shelf3">
        <pose>12 -7 1.2 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.0 6.0 2.4</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.0 6.0 2.4</size></box></geometry><material><diffuse>0.2 0.3 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Warehouse Boxes and Pallets -->
    <model name="warehouse_boxes">
      <static>true</static>
      <link name="box1">
        <pose>6 -4 0.5 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.8 0.8 1.0</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.8 0.8 1.0</size></box></geometry><material><diffuse>0.7 0.5 0.3 1</diffuse></material></visual>
      </link>
      <link name="box2">
        <pose>10 -7 0.4 0 0 0.3</pose>
        <collision name="col"><geometry><box><size>0.7 0.7 0.8</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.7 0.7 0.8</size></box></geometry><material><diffuse>0.65 0.45 0.25 1</diffuse></material></visual>
      </link>
      <link name="pallet_stack">
        <pose>6 -11 0.6 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.2 1.2 1.2</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.2 1.2 1.2</size></box></geometry><material><diffuse>0.5 0.35 0.2 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== ZONE C: OFFICE & HOSPITAL ROOMS ==================== -->
    <model name="zone_c_rooms">
      <static>true</static>
      <!-- Room 1 Wall (West Office) -->
      <link name="r1_east">
        <pose>-6 8 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 8 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 8 2.5</size></box></geometry><material><diffuse>0.75 0.75 0.7 1</diffuse></material></visual>
      </link>
      <!-- Central Corridor Wall -->
      <link name="corridor_wall_east">
        <pose>3 7 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.3 10 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.3 10 2.5</size></box></geometry><material><diffuse>0.75 0.75 0.7 1</diffuse></material></visual>
      </link>
      <!-- Hospital Room Divider -->
      <link name="hospital_room_wall">
        <pose>8.5 7 1.25 0 0 0</pose>
        <collision name="col"><geometry><box><size>11 0.3 2.5</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>11 0.3 2.5</size></box></geometry><material><diffuse>0.7 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>

    <!-- Office & Hospital Furniture (Semantic Objects for CV) -->
    <model name="office_furniture">
      <static>true</static>
      <!-- Meeting Table -->
      <link name="table1">
        <pose>-10 8 0.4 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.6 2.4 0.8</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.6 2.4 0.8</size></box></geometry><material><diffuse>0.4 0.25 0.15 1</diffuse></material></visual>
      </link>
      <!-- Office Chairs -->
      <link name="chair1">
        <pose>-11.2 8 0.45 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.5 0.9</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.5 0.5 0.9</size></box></geometry><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      </link>
      <link name="chair2">
        <pose>-8.8 8 0.45 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.5 0.9</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.5 0.5 0.9</size></box></geometry><material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>
      </link>
      <!-- Hospital Bed / Stretcher -->
      <link name="hospital_bed">
        <pose>8 11 0.45 0 0 0</pose>
        <collision name="col"><geometry><box><size>1.2 2.2 0.9</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>1.2 2.2 0.9</size></box></geometry><material><diffuse>0.9 0.9 0.95 1</diffuse></material></visual>
      </link>
      <link name="hospital_chair">
        <pose>6.5 11 0.45 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.5 0.9</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.5 0.5 0.9</size></box></geometry><material><diffuse>0.2 0.4 0.7 1</diffuse></material></visual>
      </link>
      <!-- Person Model -->
      <link name="person_standing">
        <pose>-1 5 0.85 0 0 0</pose>
        <collision name="col"><geometry><cylinder><radius>0.25</radius><length>1.7</length></cylinder></geometry></collision>
        <visual name="vis_torso"><geometry><cylinder><radius>0.25</radius><length>1.3</length></cylinder></geometry><material><diffuse>0.2 0.3 0.7 1</diffuse></material></visual>
        <visual name="vis_head"><pose>0 0 0.75 0 0 0</pose><geometry><sphere><radius>0.15</radius></sphere></geometry><material><diffuse>0.85 0.7 0.6 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== DYNAMIC OBSTACLES ==================== -->
    <model name="dynamic_obstacle_cart">
      <pose>0 2 0.3 0 0 0</pose>
      <link name="cart_link">
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.5</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.5</iyy><iyz>0</iyz><izz>0.5</izz>
          </inertia>
        </inertial>
        <collision name="col"><geometry><box><size>0.8 0.6 0.6</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.8 0.6 0.6</size></box></geometry><material><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>
      </link>
    </model>

    <!-- ==================== VISUAL SIGNPOSTS AT JUNCTIONS ==================== -->
    <!-- Signpost 1: Plaza Junction (at -5.5, -9.2 facing West/South) -->
    <model name="signpost_1_plaza">
      <static>true</static>
      <pose>-5.5 -9.2 0 0 0 0</pose>
      <!-- Post -->
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: WAREHOUSE -> -->
        <visual name="vis_sign_warehouse">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "warehouse_right.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: OFFICE ^ -->
        <visual name="vis_sign_office">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "office_straight.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Signpost 2: Warehouse Entrance (at 2.5, -9.2 facing South) -->
    <model name="signpost_2_warehouse_entrance">
      <static>true</static>
      <pose>2.5 -9.2 0 0 0 1.57</pose>
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: WAREHOUSE ^ -->
        <visual name="vis_sign_warehouse">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "warehouse_straight.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: HOSPITAL ^ -->
        <visual name="vis_sign_hospital">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "hospital_straight.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Signpost 3: Warehouse Mid Junction (at 5.5, -2.8 facing South) -->
    <model name="signpost_3_warehouse_mid">
      <static>true</static>
      <pose>5.5 -2.8 0 0 0 1.57</pose>
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: HOSPITAL ^ -->
        <visual name="vis_sign_hospital">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "hospital_straight.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: LAB <- -->
        <visual name="vis_sign_lab">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "lab_left.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Signpost 4: Central Archway Junction (at 0.8, 1.2 facing South) -->
    <model name="signpost_4_central_archway">
      <static>true</static>
      <pose>0.8 1.2 0 0 0 1.57</pose>
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: HOSPITAL -> -->
        <visual name="vis_sign_hospital">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "hospital_right.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: OFFICE <- -->
        <visual name="vis_sign_office">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "office_left.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Signpost 5: Office Junction (at -4.2, 3.8 facing East) -->
    <model name="signpost_5_office">
      <static>true</static>
      <pose>-4.2 3.8 0 0 0 3.14</pose>
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: OFFICE ^ -->
        <visual name="vis_sign_office">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "office_straight.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: LAB <- -->
        <visual name="vis_sign_lab">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "lab_left.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Signpost 6: Hospital Junction (at 4.5, 5.2 facing West) -->
    <model name="signpost_6_hospital">
      <static>true</static>
      <pose>4.5 5.2 0 0 0 0</pose>
      <link name="post">
        <collision name="col"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="vis_post"><pose>0 0 0.75 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>
        <!-- Sign Board: HOSPITAL -> -->
        <visual name="vis_sign_hospital">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "hospital_right.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
        <!-- Sign Board: CAFETERIA <- -->
        <visual name="vis_sign_cafeteria">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>0.8 0.04 0.4</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>""" + os.path.join(SIGNS_DIR, "cafeteria_left.png") + """</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
"""
    return sdf

def generate_map_pgm():
    # 600 x 600 grid @ 0.05m resolution, origin = (-15.0, -15.0)
    # 254 = free (white), 0 = occupied (black)
    grid = np.full((600, 600), 254, dtype=np.uint8)

    def to_pixel(x, y):
        # x: [-15, 15] -> col: [0, 599]
        # y: [-15, 15] -> row: [599, 0]
        col = int(round((x + 15.0) / 0.05))
        row = int(round((15.0 - y) / 0.05))
        return np.clip(col, 0, 599), np.clip(row, 0, 599)

    def draw_box(x, y, sx, sy):
        c1, r1 = to_pixel(x - sx/2, y + sy/2)
        c2, r2 = to_pixel(x + sx/2, y - sy/2)
        cv2.rectangle(grid, (min(c1, c2), min(r1, r2)), (max(c1, c2), max(r1, r2)), 0, -1)

    def draw_cylinder(x, y, radius):
        c, r = to_pixel(x, y)
        rad_px = int(round(radius / 0.05))
        cv2.circle(grid, (c, r), rad_px, 0, -1)

    # 1. Boundary Walls (28m x 28m)
    draw_box(0, 14, 28.4, 0.4)   # North
    draw_box(0, -14, 28.4, 0.4)  # South
    draw_box(14, 0, 0.4, 28.4)   # East
    draw_box(-14, 0, 0.4, 28.4)  # West

    # 2. Interior Dividers
    draw_box(-11.5, 0, 5.0, 0.3) # West A
    draw_box(-4.0, 0, 6.0, 0.3)  # West B
    draw_box(8.0, 0, 12.0, 0.3)  # East
    draw_box(0, -9.0, 0.3, 10.0) # Divider AB South

    # 3. Dead End 1: Storage Alcove
    draw_box(-11.0, -6.0, 6.0, 0.3)
    draw_box(-8.0, -4.5, 0.3, 3.0)
    draw_box(-12.0, -4.5, 1.6, 2.0)

    # 4. Dead End 2: Warehouse Cul-de-sac
    draw_box(10.5, -12.5, 0.3, 3.0)
    draw_box(12.5, -12.5, 1.5, 1.5)

    # 5. Dead End 3: Utility Closet
    draw_box(-11.0, 10.5, 6.0, 0.3)
    draw_box(-8.0, 12.0, 0.3, 3.0)

    # 6. Dead End 4: Hospital East Alcove
    draw_box(12.0, 8.0, 3.6, 0.3)

    # 7. Zone A Pillars
    draw_cylinder(-10, -5, 0.4)
    draw_cylinder(-6, -8, 0.4)
    draw_box(-9, -11, 0.8, 0.8)

    # 8. Warehouse Shelves & Obstacles
    draw_box(4, -8, 1.0, 8.0)
    draw_box(8, -8, 1.0, 8.0)
    draw_box(12, -7, 1.0, 6.0)
    draw_box(6, -4, 0.8, 0.8)
    draw_box(10, -7, 0.7, 0.7)
    draw_box(6, -11, 1.2, 1.2)

    # 9. Zone C Rooms
    draw_box(-6, 8, 0.3, 8.0)
    draw_box(3, 7, 0.3, 10.0)
    draw_box(8.5, 7, 11.0, 0.3)

    # 10. Furniture
    draw_box(-10, 8, 1.6, 2.4)
    draw_box(-11.2, 8, 0.5, 0.5)
    draw_box(-8.8, 8, 0.5, 0.5)
    draw_box(8, 11, 1.2, 2.2)
    draw_box(6.5, 11, 0.5, 0.5)
    draw_cylinder(-1, 5, 0.25)

    # 11. Signposts (small 0.1m post)
    signposts = [(-5.5, -9.2), (2.5, -9.2), (5.5, -2.8), (0.8, 1.2), (-4.2, 3.8), (4.5, 5.2)]
    for sx, sy in signposts:
        draw_cylinder(sx, sy, 0.1)

    return grid

def main():
    # 1. Write SDF
    sdf_content = generate_sdf()
    with open(SDF_PATH, "w") as f:
        f.write(sdf_content)
    print(f"[OK] Generated extended SDF world at: {SDF_PATH}")

    # Also update install path if exists
    install_sdf = os.path.join(PROJ_DIR, "ros2_ws/install/autonomous_robot_gazebo/share/autonomous_robot_gazebo/worlds/complex_world.sdf")
    if os.path.exists(os.path.dirname(install_sdf)):
        with open(install_sdf, "w") as f:
            f.write(sdf_content)
        print(f"[OK] Synced SDF world to install share: {install_sdf}")

    # 2. Write Map PGM
    map_grid = generate_map_pgm()
    cv2.imwrite(MAP_PGM_PATH, map_grid)
    print(f"[OK] Generated matching 600x600 occupancy map at: {MAP_PGM_PATH}")

    install_map = os.path.join(PROJ_DIR, "ros2_ws/install/autonomous_robot_navigation/share/autonomous_robot_navigation/maps/complex_map.pgm")
    if os.path.exists(os.path.dirname(install_map)):
        cv2.imwrite(install_map, map_grid)
        print(f"[OK] Synced map PGM to install share: {install_map}")

if __name__ == "__main__":
    main()
