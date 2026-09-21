"""
Navigation Memory System for Semantic Memory-Based Autonomous Navigation.
Persists topological nodes, edges, sign observations, traversal history,
and obstacle blockages in an SQLite database.
"""

import os
import sqlite3
import time
from typing import List, Dict, Optional, Any, Tuple
from autonomous_robot_navigation.topological_graph import TopologicalGraph, Node, Edge


class NavigationMemory:
    def __init__(
        self,
        db_path: str = "/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db",
        layout: str = "realistic"
    ):
        self.db_path = os.path.abspath(db_path)
        self.layout = layout
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

    def _get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

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
            ('start', 'Main Entrance', 0.0, -11.0, 1.57, 'start', ''),
            ('reception', 'Reception Desk & Waiting Area', 0.0, -8.0, 1.57, 'destination', 'RECEPTION'),
            ('junction_1', 'Central South Crossway', 0.0, -5.0, 1.57, 'junction', ''),
            ('corridor_east_1', 'East Medical Corridor Entry', 3.0, -5.0, 0.0, 'waypoint', ''),
            ('junction_2', 'Medical Wing Junction 2', 6.0, -5.0, 0.0, 'junction', ''),
            ('hospital_ward_entry', 'Hospital Ward Entry', 6.0, -7.5, -1.57, 'waypoint', ''),
            ('hospital', 'Hospital Medical Ward', 6.5, -9.5, -1.57, 'destination', 'HOSPITAL'),
            ('triage_entry', 'Triage Corridor', 10.0, -5.0, 0.0, 'waypoint', ''),
            ('triage', 'Triage Room', 12.0, -4.0, 0.0, 'destination', 'TRIAGE'),
            ('dead_end_1', 'Hospital Equipment Alcove', 12.0, -9.0, -1.57, 'dead_end', ''),
            ('east_bypass_1', 'East Warehouse Bypass South', 6.0, -2.0, 1.57, 'waypoint', ''),
            ('east_bypass_2', 'East Warehouse Bypass Mid', 6.0, 1.0, 1.57, 'waypoint', ''),
            ('corridor_west_1', 'West Cafeteria Corridor Entry', -3.0, -5.0, 3.14, 'waypoint', ''),
            ('junction_3', 'West Cafeteria Junction 3', -6.0, -5.0, 3.14, 'junction', ''),
            ('cafeteria_entry', 'Cafeteria Dining Entry', -6.0, -7.5, -1.57, 'waypoint', ''),
            ('cafeteria', 'Cafeteria Dining Hall', -7.0, -9.5, -1.57, 'destination', 'CAFETERIA'),
            ('exit_corridor', 'Emergency Exit Corridor', -10.0, -5.0, 3.14, 'waypoint', ''),
            ('exit', 'Emergency Exit Door', -13.0, -9.0, -1.57, 'destination', 'EXIT'),
            ('dead_end_2', 'Utility Service Room', -14.0, -5.0, 3.14, 'dead_end', ''),
            ('central_spine_1', 'Central Spine South', 0.0, -2.0, 1.57, 'waypoint', ''),
            ('junction_4', 'Central Crossway', 0.0, 1.0, 1.57, 'junction', ''),
            ('office_corridor', 'Office Wing Corridor', 0.0, 4.0, 1.57, 'waypoint', ''),
            ('junction_5', 'Office Wing Junction 5', 0.0, 7.0, 1.57, 'junction', ''),
            ('office', 'Executive Office Suite', 0.0, 10.5, 1.57, 'destination', 'OFFICE'),
            ('dead_end_3', 'Office File Archive', 2.5, 10.5, 0.0, 'dead_end', ''),
            ('office_wh_bypass', 'Office-Warehouse North Bypass', 3.0, 7.0, 0.0, 'waypoint', ''),
            ('wh_corridor_entry', 'Warehouse West Entry', 3.0, 1.0, 0.0, 'waypoint', ''),
            ('wh_junction_6', 'Warehouse Mid Junction 6', 6.0, 1.0, 0.0, 'junction', ''),
            ('warehouse', 'Industrial Warehouse Hub', 8.0, 7.0, 1.57, 'destination', 'WAREHOUSE'),
            ('wh_aisle_north', 'Warehouse North Racks Aisle', 8.0, 10.5, 1.57, 'waypoint', ''),
            ('lab_corridor_entry', 'Lab East Entry', -3.0, 1.0, 3.14, 'waypoint', ''),
            ('junction_7', 'Research & Storage Junction 7', -6.0, 1.0, 3.14, 'junction', ''),
            ('storage', 'Storage Depot', -7.0, 3.0, 1.57, 'destination', 'STORAGE'),
            ('junction_8', 'Research Lab Junction 8', -6.0, 7.0, 1.57, 'junction', ''),
            ('lab', 'Research Laboratory', -7.5, 9.5, 1.57, 'destination', 'LAB'),
            ('dead_end_4', 'Chemical Storage Vault', -13.0, 8.0, 3.14, 'dead_end', ''),
            ('west_bypass_south', 'West Bypass South', -14.0, -2.0, 1.57, 'waypoint', ''),
            ('west_bypass_north', 'West Bypass North', -14.0, 3.5, 1.57, 'waypoint', ''),
        ]

        cur.executemany("""
            INSERT INTO nodes (id, name, x, y, theta, node_type, semantic_label)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, nodes)

        pos = {n[0]: (n[2], n[3]) for n in nodes}
        def dist(u, v):
            import math
            return math.hypot(pos[u][0] - pos[v][0], pos[u][1] - pos[v][1])

        edges = [
            ('start', 'reception', False, False),
            ('reception', 'junction_1', False, False),
            ('junction_1', 'corridor_east_1', False, False),
            ('corridor_east_1', 'junction_2', False, False),
            ('junction_2', 'hospital_ward_entry', False, False),
            ('hospital_ward_entry', 'hospital', False, False),
            ('junction_2', 'triage_entry', False, False),
            ('triage_entry', 'triage', False, False),
            ('junction_2', 'dead_end_1', False, True),
            ('junction_2', 'east_bypass_1', False, False),
            ('east_bypass_1', 'east_bypass_2', False, False),
            ('east_bypass_2', 'wh_junction_6', False, False),
            ('junction_1', 'corridor_west_1', False, False),
            ('corridor_west_1', 'junction_3', False, False),
            ('junction_3', 'cafeteria_entry', False, False),
            ('cafeteria_entry', 'cafeteria', False, False),
            ('junction_3', 'exit_corridor', False, False),
            ('exit_corridor', 'exit', False, False),
            ('junction_3', 'dead_end_2', False, True),
            ('junction_3', 'west_bypass_south', False, False),
            ('west_bypass_south', 'west_bypass_north', False, False),
            ('west_bypass_north', 'junction_8', False, False),
            ('junction_1', 'central_spine_1', False, False),
            ('central_spine_1', 'junction_4', False, False),
            ('junction_4', 'office_corridor', False, False),
            ('office_corridor', 'junction_5', False, False),
            ('junction_5', 'office', False, False),
            ('junction_5', 'dead_end_3', False, True),
            ('junction_5', 'office_wh_bypass', False, False),
            ('office_wh_bypass', 'warehouse', False, False),
            ('junction_4', 'wh_corridor_entry', False, False),
            ('wh_corridor_entry', 'wh_junction_6', False, False),
            ('wh_junction_6', 'warehouse', False, False),
            ('warehouse', 'wh_aisle_north', False, False),
            ('junction_4', 'lab_corridor_entry', False, False),
            ('lab_corridor_entry', 'junction_7', False, False),
            ('junction_7', 'storage', False, False),
            ('junction_7', 'junction_8', False, False),
            ('junction_8', 'lab', False, False),
            ('junction_8', 'dead_end_4', False, True),
        ]

        edge_rows = []
        for u, v, blocked, dead_end in edges:
            d = dist(u, v)
            edge_rows.append((u, v, d, d, 0, int(blocked), int(dead_end)))
            edge_rows.append((v, u, d, d, 0, int(blocked), int(dead_end)))

        cur.executemany("""
            INSERT OR REPLACE INTO edges (from_node, to_node, distance, cost, traversal_count, is_blocked, is_dead_end)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, edge_rows)

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
        """Construct TopologicalGraph instance from database state."""
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
                # Set traversal count on edge
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
        """Record observed semantic sign in database."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO signs (node_id, text, direction, confidence, timestamp)
                VALUES (?, ?, ?, ?, ?)
            """, (node_id, text, direction, confidence, time.time()))
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

    def reset_memory(self):
        """Reset dynamic memory (signs, traversals, obstacles, blockages) while preserving topological structure."""
        with self._get_connection() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM signs")
            cur.execute("DELETE FROM traversals")
            cur.execute("DELETE FROM obstacles")
            cur.execute("UPDATE edges SET is_blocked = 0, traversal_count = 0")
            conn.commit()

    def hard_reset(self):
        """Delete entire database and re-initialize default topology."""
        if os.path.exists(self.db_path):
            os.remove(self.db_path)
        self._init_db()
