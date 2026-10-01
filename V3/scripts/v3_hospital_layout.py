"""
Single source of truth for the V3 NEW hospital (AWS RoboMaker Hospital World floor plan) navigation layout.
Coordinates are Gazebo world = map frame (metres). Used by:
  build_aws_hospital_world.py  (signs, plaques, payload in the world)
  build_v3_hospital_topology.py (navigation memory DB, semantic map, logistics locations)

Floor plan (x east, y north; building 25 x 56 m):
  north  y 12..20   main entrance / elevator lobby, storage (NW), laboratory (NE)
         y  4..12   main lobby with reception desk, west + east waiting areas, pharmacy (NE small room)
         y -6..4    nursing station block (centre), patient rooms west / east
  corridors          west corridor x=-5, east corridor x=+5 (y -32..4); cross corridors y=-8.5, -14.5, -24.5
         y -10..-24 west ward (collection point), east ward, restrooms + diagnostics/imaging block (centre)
  south  y -26..-32 recovery bay (SE), staff lounge (SW)
"""
import math

SPAWN = (0.0, 13.0, -math.pi / 2)  # main entrance lobby, facing south into the hospital

# id, name, x, y, theta, type, semantic label
NODES = [
    ("entrance", "Main Entrance / Elevator Lobby", 0.0, 15.5, -1.5708, "destination", "ENTRANCE"),
    ("lobby", "Main Lobby", 0.0, 11.0, -1.5708, "junction", ""),
    ("reception", "Reception Desk", 0.0, 8.7, -1.5708, "destination", "RECEPTION"),
    ("lobby_west", "Lobby West", -4.6, 10.6, 3.1416, "junction", ""),
    ("lobby_east", "Lobby East", 4.6, 10.6, 0.0, "junction", ""),
    ("waiting_west", "West Waiting Area", -6.5, 10.4, 3.1416, "destination", "WAITING"),
    ("storage_door", "Storage Door", -6.7, 11.4, 1.5708, "waypoint", ""),
    ("storage", "Medical Storage", -8.6, 14.4, 1.5708, "destination", "STORAGE"),
    ("lab_door", "Laboratory Door", 6.7, 11.4, 1.5708, "waypoint", ""),
    ("laboratory", "Laboratory / Test Point", 8.6, 14.6, 1.5708, "destination", "LABORATORY"),
    ("pharmacy", "Pharmacy", 7.6, 5.2, 0.0, "destination", "PHARMACY"),
    ("corridor_w_north", "West Corridor North", -5.0, 4.6, -1.5708, "junction", ""),
    ("corridor_e_north", "East Corridor North", 5.0, 4.6, -1.5708, "junction", ""),
    ("nursing_station", "Nursing Station", -2.6, 1.6, 0.0, "destination", "NURSING STATION"),
    ("corridor_w_mid", "West Corridor Mid", -5.0, -3.4, -1.5708, "waypoint", ""),
    ("corridor_e_mid", "East Corridor Mid", 5.0, -3.4, -1.5708, "waypoint", ""),
    ("x_west_north", "Intersection West / North Cross", -5.0, -8.6, -1.5708, "junction", ""),
    ("x_east_north", "Intersection East / North Cross", 5.0, -8.6, -1.5708, "junction", ""),
    ("cross_north_mid", "North Cross Corridor", -1.0, -8.4, 0.0, "waypoint", ""),
    ("patient_room_west", "Patient Rooms West", -8.6, -5.0, 1.5708, "destination", "PATIENT ROOM"),
    ("patient_room_east", "Patient Rooms East", 8.6, -5.0, 1.5708, "destination", "PATIENT ROOM EAST"),
    ("x_west_mid", "Intersection West / Mid Cross", -5.0, -14.5, -1.5708, "junction", ""),
    ("x_east_mid", "Intersection East / Mid Cross", 5.0, -14.5, -1.5708, "junction", ""),
    ("diagnostics", "Diagnostics / Imaging", 0.0, -14.5, -1.5708, "destination", "DIAGNOSTICS"),
    ("x_west_south", "Intersection West / South Cross", -5.0, -24.5, 0.0, "junction", ""),
    ("x_east_south", "Intersection East / South Cross", 5.0, -24.5, 0.0, "junction", ""),
    ("cross_south_mid", "South Cross Corridor", 0.0, -24.5, 0.0, "waypoint", ""),
    ("ward_west_door", "West Ward Door", -7.6, -23.0, 1.5708, "waypoint", ""),
    ("collection_point", "Collection Point (West Ward)", -8.5, -15.6, 1.5708, "destination", "WARD"),
    ("ward_east_door", "East Ward Door", 7.6, -23.0, 1.5708, "waypoint", ""),
    ("ward_east", "East Ward", 8.5, -16.0, 1.5708, "destination", "WARD EAST"),
    ("recovery", "Recovery Bay", 5.4, -28.0, -1.5708, "destination", "RECOVERY"),
    ("staff_lounge", "Staff Lounge", -8.6, -26.6, 3.1416, "destination", "STAFF LOUNGE"),
]

# undirected edges (u, v)
EDGES = [
    ("entrance", "lobby"),
    ("lobby", "reception"), ("lobby", "lobby_west"), ("lobby", "lobby_east"),
    ("lobby_west", "waiting_west"), ("lobby_west", "storage_door"), ("storage_door", "storage"),
    ("lobby_west", "corridor_w_north"),
    ("lobby_east", "lab_door"), ("lab_door", "laboratory"),
    ("lobby_east", "corridor_e_north"), ("corridor_e_north", "pharmacy"),
    ("reception", "corridor_w_north"), ("reception", "corridor_e_north"),
    ("corridor_w_north", "corridor_w_mid"), ("corridor_w_north", "nursing_station"),
    ("corridor_w_mid", "x_west_north"),
    ("corridor_e_north", "corridor_e_mid"), ("corridor_e_mid", "x_east_north"),
    ("x_west_north", "patient_room_west"), ("x_east_north", "patient_room_east"),
    ("x_west_north", "cross_north_mid"), ("cross_north_mid", "x_east_north"),
    ("x_west_north", "x_west_mid"), ("x_east_north", "x_east_mid"),
    ("x_west_mid", "diagnostics"), ("diagnostics", "x_east_mid"),
    ("x_west_mid", "x_west_south"), ("x_east_mid", "x_east_south"),
    ("x_west_south", "cross_south_mid"), ("cross_south_mid", "x_east_south"),
    ("x_west_south", "ward_west_door"), ("ward_west_door", "collection_point"),
    ("x_west_south", "staff_lounge"),
    ("x_east_south", "ward_east_door"), ("ward_east_door", "ward_east"),
    ("cross_south_mid", "recovery"),
]

LOGISTICS = {
    "collection_point": {"node": "collection_point", "label": "Collection Point (West Ward)",
                         "marker_class": "collection_point_marker"},
    "laboratory": {"node": "laboratory", "label": "Laboratory / Test Point", "marker_class": "lab_test_point_marker"},
    "reception": {"node": "reception", "label": "Reception"},
    "nursing_station": {"node": "nursing_station", "label": "Nursing Station"},
    "storage": {"node": "storage", "label": "Medical Storage"},
    "pharmacy": {"node": "pharmacy", "label": "Pharmacy"},
    "diagnostics": {"node": "diagnostics", "label": "Diagnostics / Imaging"},
    "ward_east": {"node": "ward_east", "label": "East Ward"},
    "recovery": {"node": "recovery", "label": "Recovery Bay"},
    "entrance": {"node": "entrance", "label": "Main Entrance"},
}

# Logistics landmark plaques: name, x, y, z, yaw (readable face normal = (-sin yaw, cos yaw)), texture key
PLAQUES = [
    # standing on the collection table (bedside table at the ward aisle), facing south toward the approaching robot
    ("v3_sign_collection_point", -7.15, -16.3, 1.12, math.pi, "collection_point"),
    # on the laboratory bench back (bench along the north wall), facing south toward the test point
    ("v3_sign_laboratory_test_point", 8.6, 16.62, 1.12, math.pi, "laboratory_test_point"),
]

# Directional signs: id, junction node, destination node, text, direction, x, y, z, yaw, approaching_from.
# Board centre pose; readable face normal = (-sin yaw, cos yaw). Corridor signs are projecting post signs
# 0.45 m off the corridor wall (board visual only, thin collision pole at the wall) so a robot driving along
# the corridor faces them; lobby signs are flush on walls or free-standing posts.
SIGNS = [
    # west corridor, northbound (ward -> lobby): face south
    ("sign_xws_laboratory", "x_west_south", "laboratory", "LABORATORY", "STRAIGHT", -4.05, -21.0, 0.85, math.pi, ["ward_west_door", "x_west_south"]),
    ("sign_xwm_diagnostics", "x_west_mid", "diagnostics", "DIAGNOSTICS", "RIGHT", -4.05, -16.3, 0.85, math.pi, ["x_west_south"]),
    ("sign_xwn_nursing", "x_west_north", "nursing_station", "NURSING STATION", "STRAIGHT", -4.05, -10.4, 0.85, math.pi, ["x_west_mid"]),
    ("sign_cwm_reception", "corridor_w_mid", "reception", "RECEPTION", "STRAIGHT", -4.05, -1.0, 0.85, math.pi, ["x_west_north"]),
    # west corridor, southbound (lobby -> ward): face north
    ("sign_xwn_ward", "x_west_north", "collection_point", "WARD", "STRAIGHT", -4.05, -6.6, 0.85, 0.0, ["corridor_w_mid"]),
    # east corridor, southbound: face north
    ("sign_xen_testing", "x_east_north", "diagnostics", "TESTING", "STRAIGHT", 4.05, -6.6, 0.85, 0.0, ["corridor_e_mid"]),
    # east corridor, northbound: face south
    ("sign_cen_laboratory", "corridor_e_north", "laboratory", "LABORATORY", "STRAIGHT", 4.05, -1.0, 0.85, math.pi, ["corridor_e_mid"]),
    # main lobby, eastbound toward the laboratory: flush on the laboratory west wall, face west
    ("sign_le_laboratory", "lobby_east", "laboratory", "LABORATORY", "LEFT", 5.82, 13.4, 0.85, math.pi / 2, ["lobby", "corridor_e_north"]),
    # main lobby, southbound from the entrance: free-standing posts, face north
    ("sign_lobby_reception", "lobby", "reception", "RECEPTION", "STRAIGHT", -1.7, 12.3, 0.85, 0.0, ["entrance"]),
    ("sign_lobby_pharmacy", "lobby", "pharmacy", "PHARMACY", "LEFT", 1.7, 12.3, 0.85, 0.0, ["entrance"]),
]

# Dynamic traffic: model name, visual (fuel model, mesh, z offset), collision, crossing line (x0,y0,x1,y1), park, speed.
# All lines CROSS north-south corridor segments that carry no east-west topology edge (kinematic set_pose motion along a robot lane can shove the robot and
# corrupt its wheel odometry: measured 14 m AMCL error with a trolley moving along the west corridor).
TRAFFIC = [
    ("dynamic_person", ("Scrubs", "meshes/scrubs.obj", -0.85), "person",
     [-6.0, -18.5, -3.9, -18.5], [-5.9, -20.5], 0.4),        # nurse crossing the west corridor (mission route)
    ("dynamic_person_2", ("FemaleVisitor", "meshes/FemaleVisitor.obj", -0.85), "person",
     [-2.5, 6.0, -2.5, 10.8], [-2.6, 15.8], 0.35),           # visitor walking across the west lobby (crosses lobby <-> lobby_west, reception -> west corridor)
    ("dynamic_hospital_trolley", ("SurgicalTrolley", "meshes/trolley.obj", -0.85), "trolley",
     [4.1, -1.0, 5.9, -1.0], [3.0, -3.4], 0.3),             # trolley pushed across the east corridor (beside the nursing block)
    ("dynamic_person_3", ("MaleVisitorOnPhone", "meshes/MaleVisitorStatic.obj", -0.85), "person",
     [3.9, -19.5, 6.1, -19.5], [6.1, -21.0], 0.35),          # visitor crossing the east corridor south of the mid cross
]
