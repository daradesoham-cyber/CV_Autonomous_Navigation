"""
Navigation Memory System for Semantic Memory-Based Autonomous Navigation.
Persists topological nodes, edges, sign observations, traversal history,
and obstacle blockages in an SQLite database.
"""

import os
import sqlite3
import time
import json
from contextlib import contextmanager
from typing import List, Dict, Optional, Any, Tuple
from autonomous_robot_navigation.topological_graph import TopologicalGraph, Node, Edge


class NavigationMemory:
    def __init__(
        self,
        db_path: str = "/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db",
        layout: str = "realistic"
    ):
        self.db_path = os.path.realpath(os.path.abspath(db_path))
        self.layout = layout
        self._last_sign_time = {}
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

    @contextmanager
    def _get_connection(self):
        conn = sqlite3.connect(self.db_path, timeout=15.0)
        conn.row_factory = sqlite3.Row
        try:
            conn.execute("PRAGMA journal_mode=WAL;")
            conn.execute("PRAGMA synchronous=NORMAL;")
        except Exception:
            pass
        try:
            yield conn
        finally:
            conn.close()

    def _init_db(self):
        """Create database tables if they do not exist."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                CREATE TABLE IF NOT EXISTS nodes (
                    id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    theta REAL DEFAULT 0.0,
                    node_type TEXT DEFAULT 'junction',
                    semantic_label TEXT DEFAULT ''
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS edges (
                    from_node TEXT NOT NULL,
                    to_node TEXT NOT NULL,
                    distance REAL NOT NULL,
                    cost REAL NOT NULL,
                    traversal_count INTEGER DEFAULT 0,
                    is_blocked INTEGER DEFAULT 0,
                    is_dead_end INTEGER DEFAULT 0,
                    PRIMARY KEY (from_node, to_node),
                    FOREIGN KEY (from_node) REFERENCES nodes(id),
                    FOREIGN KEY (to_node) REFERENCES nodes(id)
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS signs (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    node_id TEXT NOT NULL,
                    text TEXT NOT NULL,
                    direction TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    timestamp REAL NOT NULL
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS traversals (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    from_node TEXT NOT NULL,
                    to_node TEXT NOT NULL,
                    success INTEGER NOT NULL,
                    duration REAL,
                    timestamp REAL NOT NULL
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS obstacles (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    edge_from TEXT NOT NULL,
                    edge_to TEXT NOT NULL,
                    obstacle_class TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    timestamp REAL NOT NULL
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS journeys (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    journey_uuid TEXT UNIQUE,
                    created_at TEXT NOT NULL,
                    start_label TEXT NOT NULL,
                    goal_label TEXT NOT NULL,
                    start_x REAL NOT NULL,
                    start_y REAL NOT NULL,
                    goal_x REAL NOT NULL,
                    goal_y REAL NOT NULL,
                    path_nodes TEXT,
                    path_coordinates TEXT,
                    distance_travelled REAL DEFAULT 0.0,
                    travel_time REAL DEFAULT 0.0,
                    average_speed REAL DEFAULT 0.0,
                    min_lidar_clearance REAL DEFAULT 0.5,
                    obstacles_encountered INTEGER DEFAULT 0,
                    replans_count INTEGER DEFAULT 0,
                    recovery_events_count INTEGER DEFAULT 0,
                    dead_ends_count INTEGER DEFAULT 0,
                    route_selected TEXT,
                    route_alternatives TEXT,
                    success INTEGER DEFAULT 1,
                    failure_reason TEXT DEFAULT '',
                    avg_obstacle_density REAL DEFAULT 0.0
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS route_segments (
                    from_node TEXT NOT NULL,
                    to_node TEXT NOT NULL,
                    distance REAL NOT NULL,
                    traversal_count INTEGER DEFAULT 0,
                    success_count INTEGER DEFAULT 0,
                    failure_count INTEGER DEFAULT 0,
                    avg_travel_time REAL DEFAULT 0.0,
                    avg_clearance REAL DEFAULT 0.5,
                    obstacle_encounter_count INTEGER DEFAULT 0,
                    replan_count INTEGER DEFAULT 0,
                    is_narrow INTEGER DEFAULT 0,
                    is_dead_end INTEGER DEFAULT 0,
                    reliability_score REAL DEFAULT 1.0,
                    last_traversed REAL DEFAULT 0.0,
                    PRIMARY KEY (from_node, to_node)
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS dead_ends (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    node_id TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    detected_count INTEGER DEFAULT 1,
                    last_detected REAL NOT NULL,
                    is_active INTEGER DEFAULT 1
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS recovery_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    journey_uuid TEXT,
                    timestamp REAL NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    recovery_type TEXT NOT NULL,
                    actions_taken TEXT,
                    success INTEGER DEFAULT 1,
                    message TEXT
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS oscillation_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    journey_uuid TEXT,
                    timestamp REAL NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    heading_changes INTEGER DEFAULT 0,
                    velocity_reversals INTEGER DEFAULT 0,
                    time_window REAL DEFAULT 0.0,
                    message TEXT
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS replan_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    journey_uuid TEXT,
                    timestamp REAL NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    from_node TEXT NOT NULL,
                    to_node TEXT NOT NULL,
                    replan_reason TEXT NOT NULL,
                    trigger_details TEXT
                )
            """)
            conn.commit()

            # Dynamic schema migrations for existing journeys table
            cur.execute("PRAGMA table_info(journeys)")
            existing_cols = {row['name'] for row in cur.fetchall()}
            new_cols = [
                ("planned_distance", "REAL DEFAULT 0.0"),
                ("actual_distance", "REAL DEFAULT 0.0"),
                ("path_efficiency", "REAL DEFAULT 100.0"),
                ("oscillation_events_count", "INTEGER DEFAULT 0"),
                ("backtracking_distance", "REAL DEFAULT 0.0"),
                ("recovery_distance", "REAL DEFAULT 0.0"),
                ("replan_reasons", "TEXT DEFAULT ''"),
                ("average_path_deviation", "REAL DEFAULT 0.0"),
                ("arrival_x", "REAL DEFAULT 0.0"),
                ("arrival_y", "REAL DEFAULT 0.0")
            ]
            for col_name, col_type in new_cols:
                if col_name not in existing_cols:
                    cur.execute(f"ALTER TABLE journeys ADD COLUMN {col_name} {col_type}")
            conn.commit()

            # If nodes table is empty, populate initial default topology
            cur.execute("SELECT COUNT(*) FROM nodes")
            count = cur.fetchone()[0]
            if count == 0:
                if self.layout == "realistic":
                    self._populate_realistic_topology(cur)
                else:
                    self._populate_default_topology(cur)
                conn.commit()

    def _populate_realistic_topology(self, cur: sqlite3.Cursor):
        """Initializes realistic facility topological map matching realistic_facility_world.sdf."""
        nodes = [
            ('start', 'Main Entrance Spawn', 0.0, -11.0, 1.57, 'start', ''),
            ('reception', 'Reception Desk & Waiting Area', 0.0, -8.6, 1.57, 'destination', 'RECEPTION'),
            ('junction_1', 'Central South Crossway', 0.0, -5.0, 1.57, 'junction', ''),
            ('corridor_east_1', 'East Medical Corridor Entry', 3.0, -5.0, 0.0, 'waypoint', ''),
            ('junction_2', 'Medical Wing Junction 2', 6.0, -5.0, 0.0, 'junction', ''),
            ('hospital_ward_entry', 'Hospital Ward Entry', 6.0, -7.0, -1.57, 'waypoint', ''),
            ('hospital', 'Hospital Medical Ward', 6.5, -9.5, -1.57, 'destination', 'HOSPITAL'),
            ('room_b', 'Room B Inspection Ward', 6.5, -9.5, -1.57, 'destination', 'ROOM B'),
            ('triage_entry', 'Triage Corridor Entry', 9.0, -5.0, 0.0, 'waypoint', ''),
            ('triage', 'Triage Room', 12.0, -4.0, 0.0, 'destination', 'TRIAGE'),
            ('dead_end_1', 'Hospital Equipment Alcove', 12.0, -9.0, -1.57, 'dead_end', ''),
            ('restricted', 'Restricted Vault Zone', 12.5, -9.0, -1.57, 'destination', 'RESTRICTED'),
            ('east_bypass_1', 'East Warehouse Bypass South', 6.0, -2.0, 1.57, 'waypoint', ''),
            ('east_bypass_2', 'East Warehouse Bypass Mid', 6.0, 1.0, 1.57, 'waypoint', ''),
            ('corridor_west_1', 'West Cafeteria Corridor Entry', -3.0, -5.0, 3.14, 'waypoint', ''),
            ('junction_3', 'West Cafeteria Junction 3', -6.0, -5.0, 3.14, 'junction', ''),
            ('cafeteria_entry', 'Cafeteria Dining Entry', -6.0, -7.5, -1.57, 'waypoint', ''),
            ('cafeteria', 'Cafeteria Dining Hall', -6.0, -9.5, -1.57, 'destination', 'CAFETERIA'),
            ('exit_corridor', 'Emergency Exit Corridor Turn', -12.5, -5.0, 3.14, 'waypoint', ''),
            ('exit', 'Emergency Exit Door', -12.5, -9.0, -1.57, 'destination', 'EXIT'),
            ('dead_end_2', 'Utility Service Room', -15.0, -4.5, 3.14, 'dead_end', ''),
            ('central_spine_1', 'Central Spine South', 0.0, -2.0, 1.57, 'waypoint', ''),
            ('junction_4', 'Central Crossway', 0.0, 1.0, 1.57, 'junction', ''),
            ('office_corridor', 'Office Wing Corridor', 0.0, 4.0, 1.57, 'waypoint', ''),
            ('junction_5', 'Office Wing Junction 5', 0.0, 7.0, 1.57, 'junction', ''),
            ('office', 'Executive Office Suite', 0.0, 10.0, 1.57, 'destination', 'OFFICE'),
            ('dead_end_3', 'Office File Archive', 2.5, 10.5, 0.0, 'dead_end', ''),
            ('office_wh_bypass', 'Office-Warehouse North Bypass', 3.0, 7.0, 0.0, 'waypoint', ''),
            ('wh_corridor_entry', 'Warehouse West Entry', 3.0, 1.0, 0.0, 'waypoint', ''),
            ('wh_junction_6', 'Warehouse Mid Junction 6', 6.0, 1.0, 0.0, 'junction', ''),
            ('warehouse', 'Industrial Warehouse Hub', 8.0, 7.0, 1.57, 'destination', 'WAREHOUSE'),
            ('wh_aisle_south', 'Warehouse South Aisle', 8.0, 2.0, 1.57, 'waypoint', ''),
            ('wh_aisle_north', 'Warehouse North Aisle', 8.0, 10.5, 1.57, 'waypoint', ''),
            ('loading', 'Loading Bay Area', 12.0, 2.0, 0.0, 'destination', 'LOADING'),
            ('lab_corridor_entry', 'Lab East Entry', -3.0, 1.0, 3.14, 'waypoint', ''),
            ('junction_7', 'Research & Storage Junction 7', -6.0, 1.0, 3.14, 'junction', ''),
            ('storage', 'Storage Depot', -6.5, 3.0, 1.57, 'destination', 'STORAGE'),
            ('storage_approach', 'Storage Approach Waypoint', -6.0, 2.5, 1.57, 'waypoint', ''),
            ('junction_8', 'Research Lab Junction 8', -6.0, 7.0, 1.57, 'junction', ''),
            ('lab_bypass_entry', 'Lab Bypass Entry', -6.0, 5.5, 1.57, 'waypoint', ''),
            ('lab', 'Research Laboratory', -7.0, 9.5, 1.57, 'destination', 'LAB'),
            ('room_a', 'Room A Research Suite', -7.0, 9.5, 1.57, 'destination', 'ROOM A'),
            ('dead_end_4', 'Chemical Storage Vault', -13.0, 8.0, 3.14, 'dead_end', ''),
            ('vault_entry', 'Chemical Vault Entry', -11.5, 7.0, 3.14, 'waypoint', ''),
            ('west_bypass_south', 'West Bypass South', -14.0, -2.0, 1.57, 'waypoint', ''),
            ('west_bypass_north', 'West Bypass North', -14.0, 2.0, 1.57, 'waypoint', ''),
            ('west_bypass_top', 'West Bypass North Corner', -14.0, 5.5, 0.0, 'waypoint', ''),
            ('charging', 'Charging Station Dock', -13.5, 2.0, 1.57, 'destination', 'CHARGING'),
        ]

        cur.executemany("""
            INSERT OR REPLACE INTO nodes (id, name, x, y, theta, node_type, semantic_label)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, nodes)

        pos = {n[0]: (n[2], n[3]) for n in nodes}
        def dist(u, v):
            import math
            return math.hypot(pos[u][0] - pos[v][0], pos[u][1] - pos[v][1])

        edges = [
            # Main Entrance Spine
            ('start', 'reception', False, False),
            ('reception', 'junction_1', False, False),
            ('junction_1', 'central_spine_1', False, False),
            ('central_spine_1', 'junction_4', False, False),
            ('junction_4', 'office_corridor', False, False),
            ('office_corridor', 'junction_5', False, False),
            ('junction_5', 'office', False, False),
            ('junction_5', 'dead_end_3', False, True),
            ('junction_5', 'office_wh_bypass', False, False),
            # Medical / Hospital Wing
            ('junction_1', 'corridor_east_1', False, False),
            ('corridor_east_1', 'junction_2', False, False),
            ('junction_2', 'hospital_ward_entry', False, False),
            ('hospital_ward_entry', 'hospital', False, False),
            ('hospital_ward_entry', 'room_b', False, False),
            ('junction_2', 'triage_entry', False, False),
            ('triage_entry', 'triage', False, False),
            ('triage', 'dead_end_1', False, True),
            ('triage', 'restricted', False, False),
            ('junction_2', 'east_bypass_1', False, False),
            ('east_bypass_1', 'east_bypass_2', False, False),
            ('east_bypass_2', 'wh_junction_6', False, False),
            # Cafeteria & Exit Wing
            ('junction_1', 'corridor_west_1', False, False),
            ('corridor_west_1', 'junction_3', False, False),
            ('junction_3', 'cafeteria_entry', False, False),
            ('cafeteria_entry', 'cafeteria', False, False),
            ('junction_3', 'exit_corridor', False, False),
            ('exit_corridor', 'exit', False, False),
            ('exit_corridor', 'dead_end_2', False, True),
            ('junction_3', 'west_bypass_south', False, False),
            ('west_bypass_south', 'west_bypass_north', False, False),
            ('west_bypass_north', 'charging', False, False),
            ('west_bypass_north', 'west_bypass_top', False, False),
            ('west_bypass_top', 'lab_bypass_entry', False, False),
            ('lab_bypass_entry', 'junction_8', False, False),
            # Central Crossway & Warehouse
            ('junction_4', 'wh_corridor_entry', False, False),
            ('wh_corridor_entry', 'wh_junction_6', False, False),
            ('wh_junction_6', 'wh_aisle_south', False, False),
            ('wh_aisle_south', 'warehouse', False, False),
            ('wh_aisle_south', 'loading', False, False),
            ('warehouse', 'wh_aisle_north', False, False),
            ('office_wh_bypass', 'warehouse', False, False),
            # Research Lab & Storage
            ('junction_4', 'lab_corridor_entry', False, False),
            ('lab_corridor_entry', 'junction_7', False, False),
            ('junction_7', 'storage_approach', False, False),
            ('storage_approach', 'storage', False, False),
            ('storage_approach', 'lab_bypass_entry', False, False),
            ('junction_8', 'lab', False, False),
            ('junction_8', 'room_a', False, False),
            ('junction_8', 'vault_entry', False, False),
            ('vault_entry', 'dead_end_4', False, True),
        ]

        edge_rows = []
        segment_rows = []
        for u, v, blocked, dead_end in edges:
            d = dist(u, v)
            edge_rows.append((u, v, d, d, 0, int(blocked), int(dead_end)))
            edge_rows.append((v, u, d, d, 0, int(blocked), int(dead_end)))
            # Route segments with initial reliability 1.0
            is_narrow = int('bypass' in u or 'bypass' in v or 'aisle' in u or 'aisle' in v)
            segment_rows.append((u, v, d, 0, 0, 0, d / 0.45, 0.5, 0, 0, is_narrow, int(dead_end), 1.0, 0.0))
            segment_rows.append((v, u, d, 0, 0, 0, d / 0.45, 0.5, 0, 0, is_narrow, int(dead_end), 1.0, 0.0))

        cur.executemany("""
            INSERT OR REPLACE INTO edges (from_node, to_node, distance, cost, traversal_count, is_blocked, is_dead_end)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, edge_rows)

        cur.executemany("""
            INSERT OR REPLACE INTO route_segments (
                from_node, to_node, distance, traversal_count, success_count, failure_count,
                avg_travel_time, avg_clearance, obstacle_encounter_count, replan_count,
                is_narrow, is_dead_end, reliability_score, last_traversed
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, segment_rows)

    def _populate_default_topology(self, cur: sqlite3.Cursor):
        """Initializes default topological map matching complex_world.sdf."""
        default_nodes = [
            ('start', 'Start Position', 0.0, -8.0, 1.57, 'start', ''),
            ('junction_1', 'Central Junction 1', 0.0, 0.0, 0.0, 'junction', ''),
            ('junction_2', 'East Junction 2', 3.0, 0.0, 0.0, 'junction', ''),
            ('hospital_corridor', 'Hospital Corridor', 3.0, -4.0, -1.57, 'waypoint', ''),
            ('hospital', 'Hospital Wing', 5.0, -4.0, 0.0, 'destination', 'HOSPITAL'),
            ('dead_end_1', 'East Cul-de-sac', 5.5, 0.0, 0.0, 'dead_end', ''),
            ('junction_3', 'West Junction 3', -3.0, 0.0, 3.14, 'junction', ''),
            ('dead_end_2', 'South Cul-de-sac', -3.0, -4.0, -1.57, 'dead_end', ''),
            ('junction_4', 'West Junction 4', -6.0, 0.0, 3.14, 'junction', ''),
            ('cafeteria', 'Cafeteria Dining', -3.0, -5.0, 0.0, 'destination', 'CAFETERIA'),
            ('junction_5', 'North East Junction 5', 3.0, 6.0, 1.57, 'junction', ''),
            ('office', 'Executive Office', 3.0, 8.0, 1.57, 'destination', 'OFFICE'),
            ('warehouse', 'Warehouse Hub', 6.0, 6.0, 0.0, 'destination', 'WAREHOUSE'),
            ('warehouse_alt', 'Warehouse Long Bypass', 6.0, 2.0, 1.57, 'waypoint', ''),
            ('storage', 'Storage Depot', -6.0, 4.0, 1.57, 'destination', 'STORAGE'),
            ('lab', 'Research Lab', -3.0, 4.0, 0.0, 'destination', 'LAB'),
            ('junction_6', 'South West Junction 6', -6.0, -5.0, -1.57, 'junction', ''),
            ('dead_end_3', 'West Cul-de-sac', -8.0, -5.0, 3.14, 'dead_end', ''),
            ('exit', 'Emergency Exit', -6.0, -8.0, -1.57, 'destination', 'EXIT'),
            ('west_bypass_mid', 'West Bypass Mid', -8.0, 0.0, 1.57, 'waypoint', ''),
            ('west_bypass_north', 'West Bypass North', -8.0, 4.0, 0.0, 'waypoint', ''),
            ('central_hall', 'Central North Corridor', 0.0, 4.5, 1.57, 'waypoint', ''),
        ]

        cur.executemany("""
            INSERT INTO nodes (id, name, x, y, theta, node_type, semantic_label)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, default_nodes)

        # Helper to compute euclidean distance
        pos = {n[0]: (n[2], n[3]) for n in default_nodes}
        def dist(u, v):
            import math
            return math.hypot(pos[u][0] - pos[v][0], pos[u][1] - pos[v][1])

        default_edges = [
            ('start', 'junction_1', False, False),
            ('junction_1', 'junction_2', False, False),
            ('junction_1', 'junction_3', False, False),
            ('junction_1', 'central_hall', False, False),
            ('central_hall', 'junction_5', False, False),
            ('junction_2', 'hospital_corridor', False, False),
            ('hospital_corridor', 'hospital', False, False),
            ('junction_2', 'dead_end_1', False, True),  # is_dead_end = True
            ('junction_2', 'junction_5', False, False),
            ('junction_2', 'warehouse_alt', False, False),
            ('warehouse_alt', 'warehouse', False, False),
            ('junction_3', 'dead_end_2', False, True),  # is_dead_end = True
            ('junction_3', 'junction_4', False, False),
            ('junction_4', 'storage', False, False),
            ('junction_4', 'junction_6', False, False),
            ('junction_4', 'west_bypass_mid', False, False),
            ('west_bypass_mid', 'west_bypass_north', False, False),
            ('west_bypass_north', 'storage', False, False),
            ('storage', 'lab', False, False),
            ('junction_5', 'warehouse', False, False),
            ('junction_5', 'office', False, False),
            ('junction_6', 'cafeteria', False, False),
            ('junction_6', 'dead_end_3', False, True),  # is_dead_end = True
            ('junction_6', 'exit', False, False),
        ]

        edge_rows = []
        for u, v, blocked, dead_end in default_edges:
            d = dist(u, v)
            edge_rows.append((u, v, d, d, 0, int(blocked), int(dead_end)))
            edge_rows.append((v, u, d, d, 0, int(blocked), int(dead_end)))

        cur.executemany("""
            INSERT OR REPLACE INTO edges (from_node, to_node, distance, cost, traversal_count, is_blocked, is_dead_end)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, edge_rows)

    def load_graph(self) -> TopologicalGraph:
        """Construct TopologicalGraph instance from database state, loading edge analytics."""
        graph = TopologicalGraph()
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT id, name, x, y, theta, node_type, semantic_label FROM nodes")
            for row in cur.fetchall():
                graph.add_node(
                    node_id=row['id'],
                    name=row['name'],
                    x=row['x'],
                    y=row['y'],
                    theta=row['theta'],
                    node_type=row['node_type'],
                    semantic_label=row['semantic_label']
                )

            # Check if route_segments table has records
            cur.execute("SELECT COUNT(*) FROM route_segments")
            has_segments = cur.fetchone()[0] > 0

            if has_segments:
                cur.execute("""
                    SELECT e.from_node, e.to_node, e.distance, e.cost, e.traversal_count, e.is_blocked, e.is_dead_end,
                           COALESCE(rs.obstacle_encounter_count, 0) as obs_count,
                           COALESCE(rs.failure_count, 0) as fail_count,
                           COALESCE(rs.is_narrow, 0) as is_narrow,
                           COALESCE(rs.avg_travel_time, 0.0) as avg_time,
                           COALESCE(rs.reliability_score, 1.0) as reliability
                    FROM edges e
                    LEFT JOIN route_segments rs ON (e.from_node = rs.from_node AND e.to_node = rs.to_node)
                """)
                for row in cur.fetchall():
                    obs_density = float(row['obs_count']) / max(1, row['traversal_count']) if row['traversal_count'] > 0 else 0.0
                    edge = Edge(
                        from_node=row['from_node'],
                        to_node=row['to_node'],
                        distance=row['distance'],
                        cost=row['cost'],
                        traversal_count=row['traversal_count'],
                        is_blocked=bool(row['is_blocked']),
                        is_dead_end=bool(row['is_dead_end']),
                        obstacle_density=obs_density,
                        failure_count=row['fail_count'],
                        congestion_cost=0.0,
                        is_narrow=bool(row['is_narrow']),
                        turns_count=0,
                        avg_travel_time=row['avg_time'],
                        reliability_score=row['reliability']
                    )
                    graph.edges[(row['from_node'], row['to_node'])] = edge
                    if row['to_node'] not in graph.adjacency.get(row['from_node'], []):
                        graph.adjacency.setdefault(row['from_node'], []).append(row['to_node'])
            else:
                cur.execute("SELECT from_node, to_node, distance, cost, traversal_count, is_blocked, is_dead_end FROM edges")
                for row in cur.fetchall():
                    graph.add_edge(
                        u=row['from_node'],
                        v=row['to_node'],
                        cost=row['cost'],
                        bidirectional=False,
                        is_blocked=bool(row['is_blocked']),
                        is_dead_end=bool(row['is_dead_end'])
                    )
                    edge = graph.get_edge(row['from_node'], row['to_node'])
                    if edge:
                        edge.traversal_count = row['traversal_count']

        return graph

    def save_graph(self, graph: TopologicalGraph):
        """Persist entire graph state to database."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            for node in graph.nodes.values():
                cur.execute("""
                    INSERT OR REPLACE INTO nodes (id, name, x, y, theta, node_type, semantic_label)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                """, (node.id, node.name, node.x, node.y, node.theta, node.node_type, node.semantic_label))

            for (u, v), edge in graph.edges.items():
                cur.execute("""
                    INSERT OR REPLACE INTO edges (from_node, to_node, distance, cost, traversal_count, is_blocked, is_dead_end)
                    VALUES (?, ?, ?, ?, ?, ?, ?)
                """, (u, v, edge.distance, edge.cost, edge.traversal_count, int(edge.is_blocked), int(edge.is_dead_end)))
            conn.commit()

    def record_sign(self, node_id: str, text: str, direction: str, confidence: float):
        """Record observed semantic sign in database (throttled to avoid redundant spam)."""
        now = time.time()
        key = (node_id, text, direction)
        if hasattr(self, '_last_sign_time') and (now - self._last_sign_time.get(key, 0.0)) < 2.0:
            return
        if not hasattr(self, '_last_sign_time'):
            self._last_sign_time = {}
        self._last_sign_time[key] = now

        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO signs (node_id, text, direction, confidence, timestamp)
                VALUES (?, ?, ?, ?, ?)
            """, (node_id, text, direction, confidence, now))
            conn.commit()

    def record_traversal(self, from_node: str, to_node: str, success: bool, duration: Optional[float] = None):
        """Record corridor traversal outcome and update edge count/state."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO traversals (from_node, to_node, success, duration, timestamp)
                VALUES (?, ?, ?, ?, ?)
            """, (from_node, to_node, int(success), duration, time.time()))

            if success:
                cur.execute("""
                    UPDATE edges SET traversal_count = traversal_count + 1, is_blocked = 0
                    WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
                """, (from_node, to_node, to_node, from_node))
            else:
                cur.execute("""
                    UPDATE edges SET is_blocked = 1
                    WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
                """, (from_node, to_node, to_node, from_node))
            conn.commit()

    def record_obstacle(self, edge_from: str, edge_to: str, obstacle_class: str, x: float, y: float):
        """Record detected obstacle causing corridor blockage."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO obstacles (edge_from, edge_to, obstacle_class, x, y, timestamp)
                VALUES (?, ?, ?, ?, ?, ?)
            """, (edge_from, edge_to, obstacle_class, x, y, time.time()))
            cur.execute("""
                UPDATE edges SET is_blocked = 1
                WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
            """, (edge_from, edge_to, edge_to, edge_from))
            conn.commit()

    def mark_blocked(self, from_node: str, to_node: str, blocked: bool = True):
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                UPDATE edges SET is_blocked = ?
                WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
            """, (int(blocked), from_node, to_node, to_node, from_node))
            conn.commit()

    def mark_dead_end(self, from_node: str, to_node: str, is_dead_end: bool = True):
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                UPDATE edges SET is_dead_end = ?
                WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
            """, (int(is_dead_end), from_node, to_node, to_node, from_node))
            conn.commit()

    def get_known_signs(self) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT node_id, text, direction, confidence, timestamp FROM signs ORDER BY timestamp DESC")
            return [dict(row) for row in cur.fetchall()]

    def record_journey(self, journey_data: Optional[Dict[str, Any]] = None, **kwargs) -> int:
        """
        Record completed journey with comprehensive travel metrics (V2.2 & V2.3).
        Supports either a dictionary or keyword arguments.
        Returns the inserted journey ID.
        """
        data = {}
        if journey_data and isinstance(journey_data, dict):
            data.update(journey_data)
        data.update(kwargs)

        # Aliases for convenience
        if 'journey_id' in data and 'journey_uuid' not in data:
            data['journey_uuid'] = data['journey_id']
        if 'start_node' in data and 'start_label' not in data:
            data['start_label'] = data['start_node']
        if 'goal_node' in data and 'goal_label' not in data:
            data['goal_label'] = data['goal_node']
        if 'path_coords' in data and 'path_coordinates' not in data:
            data['path_coordinates'] = data['path_coords']
        if 'recovery_count' in data and 'recovery_events_count' not in data:
            data['recovery_events_count'] = data['recovery_count']
        if 'route_alternatives_considered' in data and 'route_alternatives' not in data:
            data['route_alternatives'] = data['route_alternatives_considered']

        # Synchronize distance_travelled and actual_distance
        actual_dist = float(data.get('actual_distance', data.get('distance_travelled', 0.0)))
        planned_dist = float(data.get('planned_distance', actual_dist))
        if 'path_efficiency' in data:
            path_eff = float(data['path_efficiency'])
        else:
            path_eff = (planned_dist / actual_dist * 100.0) if actual_dist > 0.05 else 100.0
            path_eff = min(100.0, max(0.0, path_eff))

        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO journeys (
                    journey_uuid, created_at, start_label, goal_label,
                    start_x, start_y, goal_x, goal_y,
                    path_nodes, path_coordinates, distance_travelled,
                    travel_time, average_speed, min_lidar_clearance,
                    obstacles_encountered, replans_count, recovery_events_count,
                    dead_ends_count, route_selected, route_alternatives,
                    success, failure_reason, avg_obstacle_density,
                    planned_distance, actual_distance, path_efficiency,
                    oscillation_events_count, backtracking_distance, recovery_distance,
                    replan_reasons, average_path_deviation, arrival_x, arrival_y
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                data.get('journey_uuid', f"J_{int(time.time()*1000)}"),
                data.get('created_at', time.strftime("%Y-%m-%d %H:%M:%S")),
                data.get('start_label', 'UNKNOWN'),
                data.get('goal_label', 'UNKNOWN'),
                float(data.get('start_x', 0.0)),
                float(data.get('start_y', 0.0)),
                float(data.get('goal_x', 0.0)),
                float(data.get('goal_y', 0.0)),
                data.get('path_nodes', ''),
                data.get('path_coordinates', '[]') if isinstance(data.get('path_coordinates'), str) else json.dumps(data.get('path_coordinates', [])),
                actual_dist,
                float(data.get('travel_time', 0.0)),
                float(data.get('average_speed', 0.0)),
                float(data.get('min_lidar_clearance', 0.5)),
                int(data.get('obstacles_encountered', 0)),
                int(data.get('replans_count', 0)),
                int(data.get('recovery_events_count', 0)),
                int(data.get('dead_ends_count', 0)),
                data.get('route_selected', 'Primary'),
                data.get('route_alternatives', '[]') if isinstance(data.get('route_alternatives'), str) else json.dumps(data.get('route_alternatives', [])),
                int(data.get('success', 1)),
                data.get('failure_reason', ''),
                float(data.get('avg_obstacle_density', 0.0)),
                planned_dist,
                actual_dist,
                path_eff,
                int(data.get('oscillation_events_count', 0)),
                float(data.get('backtracking_distance', 0.0)),
                float(data.get('recovery_distance', 0.0)),
                str(data.get('replan_reasons', '')),
                float(data.get('average_path_deviation', 0.0)),
                float(data.get('arrival_x', data.get('goal_x', 0.0))),
                float(data.get('arrival_y', data.get('goal_y', 0.0)))
            ))
            conn.commit()
            return cur.lastrowid

    def get_journeys(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Retrieve recent journeys for Travel History table and dashboard."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                SELECT id, journey_uuid, created_at, start_label, goal_label,
                       start_x, start_y, goal_x, goal_y, path_nodes, path_coordinates,
                       distance_travelled, travel_time, average_speed, min_lidar_clearance,
                       obstacles_encountered, replans_count, recovery_events_count,
                       dead_ends_count, route_selected, route_alternatives,
                       success, failure_reason, avg_obstacle_density,
                       planned_distance, actual_distance, path_efficiency,
                       oscillation_events_count, backtracking_distance, recovery_distance,
                       replan_reasons, average_path_deviation, arrival_x, arrival_y
                FROM journeys
                ORDER BY id DESC
                LIMIT ?
            """, (limit,))
            rows = [dict(r) for r in cur.fetchall()]
            for r in rows:
                r['journey_id'] = r.get('journey_uuid', f"J-{r.get('id', '')}")
                r['start_node'] = r.get('start_label', '')
                r['goal_node'] = r.get('goal_label', '')
                if isinstance(r.get('path_coordinates'), str):
                    try:
                        r['path_coordinates'] = json.loads(r['path_coordinates'])
                    except Exception:
                        r['path_coordinates'] = []
                if isinstance(r.get('route_alternatives'), str):
                    try:
                        r['route_alternatives'] = json.loads(r['route_alternatives'])
                    except Exception:
                        r['route_alternatives'] = []
            return rows

    def get_journey_by_id(self, journey_id: Any) -> Optional[Dict[str, Any]]:
        """Retrieve full details of a specific journey."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            if isinstance(journey_id, int) or str(journey_id).isdigit():
                cur.execute("SELECT * FROM journeys WHERE id = ? OR journey_uuid = ?", (int(journey_id), str(journey_id)))
            else:
                cur.execute("SELECT * FROM journeys WHERE journey_uuid = ?", (str(journey_id),))
            row = cur.fetchone()
            if not row:
                return None
            res = dict(row)
            res['journey_id'] = res.get('journey_uuid', f"J-{res.get('id', '')}")
            res['start_node'] = res.get('start_label', '')
            res['goal_node'] = res.get('goal_label', '')
            if isinstance(res.get('path_coordinates'), str):
                try:
                    res['path_coordinates'] = json.loads(res['path_coordinates'])
                except Exception:
                    res['path_coordinates'] = []
            return res

    def update_segment_metrics(
        self,
        from_node: str,
        to_node: str,
        success: bool,
        duration: float = 0.0,
        clearance: float = 0.5,
        obstacle_encountered: bool = False,
        replan_triggered: bool = False,
        prior_alpha: float = 5.0,
        prior_beta: float = 1.0
    ):
        """
        Update segment reliability score with Bayesian prior smoothing (Section 6 & V2.2).
        A single failure will not permanently blacklist a frequently successful route.
        """
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                SELECT traversal_count, success_count, failure_count,
                       avg_travel_time, avg_clearance, obstacle_encounter_count, replan_count
                FROM route_segments
                WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
            """, (from_node, to_node, to_node, from_node))
            row = cur.fetchone()

            now = time.time()
            if row:
                t_count = row['traversal_count'] + 1
                s_count = row['success_count'] + (1 if success else 0)
                f_count = row['failure_count'] + (0 if success else 1)
                obs_count = row['obstacle_encounter_count'] + (1 if obstacle_encountered else 0)
                rep_count = row['replan_count'] + (1 if replan_triggered else 0)

                prev_time = row['avg_travel_time']
                new_time = duration if prev_time <= 0.0 else (0.7 * prev_time + 0.3 * duration)

                prev_clear = row['avg_clearance']
                new_clear = 0.7 * prev_clear + 0.3 * clearance

                # Bayesian smoothed reliability: (S + alpha) / (S + F + 0.2*R + alpha + beta)
                smoothed_rel = float(s_count + prior_alpha) / float(s_count + f_count + 0.2 * rep_count + prior_alpha + prior_beta)
                rel_score = max(0.10, min(1.0, smoothed_rel))

                cur.execute("""
                    UPDATE route_segments SET
                        traversal_count = ?, success_count = ?, failure_count = ?,
                        avg_travel_time = ?, avg_clearance = ?, obstacle_encounter_count = ?,
                        replan_count = ?, reliability_score = ?, last_traversed = ?
                    WHERE (from_node = ? AND to_node = ?) OR (from_node = ? AND to_node = ?)
                """, (t_count, s_count, f_count, new_time, new_clear, obs_count, rep_count, rel_score, now,
                      from_node, to_node, to_node, from_node))
            conn.commit()

    def record_dead_end(self, node_id: str, x: float, y: float):
        """Record identified dead-end node and coordinates (Section 7)."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT id, detected_count FROM dead_ends WHERE node_id = ?", (node_id,))
            row = cur.fetchone()
            now = time.time()
            if row:
                cur.execute("""
                    UPDATE dead_ends SET detected_count = detected_count + 1, last_detected = ?, is_active = 1
                    WHERE id = ?
                """, (now, row['id']))
            else:
                cur.execute("""
                    INSERT INTO dead_ends (node_id, x, y, detected_count, last_detected, is_active)
                    VALUES (?, ?, ?, 1, ?, 1)
                """, (node_id, x, y, now))
            conn.commit()

    def get_dead_ends(self) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT node_id, x, y, detected_count, last_detected, is_active FROM dead_ends WHERE is_active = 1")
            return [dict(r) for r in cur.fetchall()]

    def record_recovery_event(
        self,
        journey_uuid: str,
        x: float,
        y: float,
        recovery_type: str,
        actions_taken: str,
        success: bool = True,
        message: str = ""
    ):
        """Log recovery event for audit and analytics (Section 8)."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO recovery_events (journey_uuid, timestamp, x, y, recovery_type, actions_taken, success, message)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (journey_uuid, time.time(), x, y, recovery_type, actions_taken, int(success), message))
            conn.commit()

    def get_recovery_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT * FROM recovery_events ORDER BY id DESC LIMIT ?", (limit,))
            return [dict(r) for r in cur.fetchall()]

    def record_oscillation_event(
        self,
        journey_uuid: str,
        x: float,
        y: float,
        heading_changes: int = 0,
        velocity_reversals: int = 0,
        time_window: float = 0.0,
        message: str = ""
    ):
        """Log oscillation event (V2.2 Section 5)."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO oscillation_events (journey_uuid, timestamp, x, y, heading_changes, velocity_reversals, time_window, message)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (journey_uuid, time.time(), x, y, heading_changes, velocity_reversals, time_window, message))
            conn.commit()

    def get_oscillation_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT * FROM oscillation_events ORDER BY id DESC LIMIT ?", (limit,))
            return [dict(r) for r in cur.fetchall()]

    def record_replan_event(
        self,
        journey_uuid: str,
        x: float,
        y: float,
        from_node: str,
        to_node: str,
        replan_reason: str,
        trigger_details: str = ""
    ):
        """Log replan event with categorized reason (V2.2 Section 6)."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO replan_events (journey_uuid, timestamp, x, y, from_node, to_node, replan_reason, trigger_details)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            """, (journey_uuid, time.time(), x, y, from_node, to_node, replan_reason, trigger_details))
            conn.commit()

    def get_replan_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT * FROM replan_events ORDER BY id DESC LIMIT ?", (limit,))
            return [dict(r) for r in cur.fetchall()]

    def get_route_heatmap_data(self) -> List[Dict[str, Any]]:
        """Returns edge traversal frequencies, blockages, and measurable stats for heatmap overlay."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                SELECT n1.x as x1, n1.y as y1, n2.x as x2, n2.y as y2,
                       e.from_node, e.to_node, e.distance, e.is_blocked, e.is_dead_end,
                       COALESCE(rs.traversal_count, e.traversal_count, 0) as traversal_count,
                       COALESCE(rs.failure_count, 0) as failure_count,
                       COALESCE(rs.replan_count, 0) as recovery_count,
                       COALESCE(rs.avg_travel_time, 0.0) as avg_travel_time,
                       COALESCE(rs.avg_clearance, 0.5) as avg_clearance,
                       COALESCE(rs.obstacle_encounter_count, 0) as obstacle_count,
                       COALESCE(rs.reliability_score, 1.0) as reliability_score,
                       COALESCE(rs.is_narrow, 0) as is_narrow
                FROM edges e
                JOIN nodes n1 ON e.from_node = n1.id
                JOIN nodes n2 ON e.to_node = n2.id
                LEFT JOIN route_segments rs ON (e.from_node = rs.from_node AND e.to_node = rs.to_node)
            """)
            rows = [dict(r) for r in cur.fetchall()]
            for r in rows:
                if r['is_blocked']:
                    r['category'] = 'Blocked'
                elif r['is_dead_end']:
                    r['category'] = 'Dead End'
                elif r['failure_count'] > 0 or r['reliability_score'] < 0.60:
                    r['category'] = 'High Failure'
                elif r['obstacle_count'] >= 2:
                    r['category'] = 'Obstacle Heavy'
                elif r['traversal_count'] >= 3:
                    r['category'] = 'Frequently Used'
                else:
                    r['category'] = 'Normal'
            return rows

    def get_analytics_summary(self) -> Dict[str, Any]:
        """Aggregate performance metrics for analytics dashboard graphs."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("SELECT COUNT(*) as total, SUM(success) as successes FROM journeys")
            j_res = cur.fetchone()
            total_journeys = j_res['total'] if j_res else 0
            successes = j_res['successes'] if (j_res and j_res['successes']) else 0
            success_rate = round((float(successes) / max(1, total_journeys)) * 100.0, 1)

            cur.execute("SELECT AVG(distance_travelled) as avg_d, AVG(travel_time) as avg_t, AVG(average_speed) as avg_s, SUM(replans_count) as total_replans, MIN(min_lidar_clearance) as min_clear FROM journeys")
            stats = cur.fetchone()

            cur.execute("SELECT COUNT(*) as rec_count FROM recovery_events")
            rec_row = cur.fetchone()
            total_recoveries = rec_row['rec_count'] if rec_row else 0

            cur.execute("SELECT COUNT(*) as dead_end_count FROM dead_ends")
            de_row = cur.fetchone()
            total_dead_ends = de_row['dead_end_count'] if de_row else 0

            return {
                'total_journeys': total_journeys,
                'successful_journeys': successes,
                'success_rate_pct': success_rate,
                'avg_distance_m': round(stats['avg_d'] or 0.0, 2) if (stats and stats['avg_d']) else 0.0,
                'avg_travel_time_s': round(stats['avg_t'] or 0.0, 1) if (stats and stats['avg_t']) else 0.0,
                'avg_speed_mps': round(stats['avg_s'] or 0.0, 2) if (stats and stats['avg_s']) else 0.0,
                'total_replans': int(stats['total_replans'] or 0) if (stats and stats['total_replans']) else 0,
                'min_clearance_m': round(stats['min_clear'] or 0.0, 2) if (stats and stats['min_clear']) else 0.0,
                'total_recoveries': total_recoveries,
                'total_dead_ends': total_dead_ends
            }

    def reset_transient_blockages(self) -> int:
        """
        V2.6 Phase 1: Clears all transient edge blockages in SQLite navigation memory.
        Differentiates permanent topological structure (nodes, distance, dead-ends)
        from temporary runtime obstacle blockages. Preserves traversal history,
        obstacle logs, reliability scores, and route segment metrics.
        Returns the number of unblocked edges.
        """
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("UPDATE edges SET is_blocked = 0 WHERE is_blocked = 1")
            unblocked_count = cur.rowcount
            conn.commit()
            return unblocked_count

    def reset_memory(self):
        """Reset dynamic memory while preserving topological structure."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM signs")
            cur.execute("DELETE FROM traversals")
            cur.execute("DELETE FROM obstacles")
            cur.execute("UPDATE edges SET is_blocked = 0, traversal_count = 0")
            cur.execute("UPDATE route_segments SET traversal_count = 0, success_count = 0, failure_count = 0, obstacle_encounter_count = 0, replan_count = 0, reliability_score = 1.0")
            cur.execute("UPDATE dead_ends SET is_active = 0")
            conn.commit()

    def hard_reset(self):
        """Delete entire database and re-initialize default topology."""
        if os.path.exists(self.db_path):
            os.remove(self.db_path)
        self._init_db()

