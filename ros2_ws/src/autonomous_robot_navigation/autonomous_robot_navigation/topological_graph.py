"""
Topological Navigation Graph for Semantic Memory-Based Autonomous Navigation.
Represents junctions, destinations, corridors, and dead ends with Dijkstra/A* path planning,
dead-end avoidance, and dynamic obstacle edge re-weighting.
"""

import math
import heapq
from typing import Dict, List, Optional, Tuple, Any


class Node:
    def __init__(
        self,
        node_id: str,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
        node_type: str = "junction",
        semantic_label: str = ""
    ):
        self.id = node_id
        self.name = name
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.node_type = node_type  # 'start', 'junction', 'destination', 'dead_end', 'waypoint'
        self.semantic_label = semantic_label  # e.g. 'HOSPITAL', 'WAREHOUSE', 'OFFICE', 'LAB', 'STORAGE', 'CAFETERIA', 'EXIT'

    def distance_to(self, other: 'Node') -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'node_type': self.node_type,
            'semantic_label': self.semantic_label
        }


class Edge:
    def __init__(
        self,
        from_node: str,
        to_node: str,
        distance: float,
        cost: Optional[float] = None,
        traversal_count: int = 0,
        is_blocked: bool = False,
        is_dead_end: bool = False
    ):
        self.from_node = from_node
        self.to_node = to_node
        self.distance = float(distance)
        self.base_cost = float(distance)
        self.cost = float(cost if cost is not None else distance)
        self.traversal_count = int(traversal_count)
        self.is_blocked = bool(is_blocked)
        self.is_dead_end = bool(is_dead_end)

    def get_effective_cost(self) -> float:
        """Calculate dynamic cost considering blockages, dead ends, and familiarity."""
        if self.is_blocked:
            return 1e6  # Impassable / extremely high penalty
        if self.is_dead_end:
            return 1e5  # Avoid dead ends unless specifically targeted

        # Slight discount for well-traversed corridors (familiarity benefit)
        familiarity_bonus = min(0.15 * self.traversal_count, 0.3) * self.distance
        return max(0.1, self.cost - familiarity_bonus)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'from_node': self.from_node,
            'to_node': self.to_node,
            'distance': self.distance,
            'cost': self.cost,
            'traversal_count': self.traversal_count,
            'is_blocked': self.is_blocked,
            'is_dead_end': self.is_dead_end
        }


class TopologicalGraph:
    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.edges: Dict[Tuple[str, str], Edge] = {}
        self.adjacency: Dict[str, List[str]] = {}

    def add_node(
        self,
        node_id: str,
        name: str,
        x: float,
        y: float,
        theta: float = 0.0,
        node_type: str = "junction",
        semantic_label: str = ""
    ) -> Node:
        node = Node(node_id, name, x, y, theta, node_type, semantic_label)
        self.nodes[node_id] = node
        if node_id not in self.adjacency:
            self.adjacency[node_id] = []
        return node

    def add_edge(
        self,
        u: str,
        v: str,
        cost: Optional[float] = None,
        bidirectional: bool = True,
        is_blocked: bool = False,
        is_dead_end: bool = False
    ):
        if u not in self.nodes or v not in self.nodes:
            raise KeyError(f"Both nodes '{u}' and '{v}' must exist before adding edge.")

        dist = self.nodes[u].distance_to(self.nodes[v])
        edge_uv = Edge(u, v, dist, cost, 0, is_blocked, is_dead_end)
        self.edges[(u, v)] = edge_uv
        if v not in self.adjacency[u]:
            self.adjacency[u].append(v)

        if bidirectional:
            edge_vu = Edge(v, u, dist, cost, 0, is_blocked, is_dead_end)
            self.edges[(v, u)] = edge_vu
            if u not in self.adjacency[v]:
                self.adjacency[v].append(u)

    def get_edge(self, u: str, v: str) -> Optional[Edge]:
        return self.edges.get((u, v))

    def mark_edge_blocked(self, u: str, v: str, blocked: bool = True, bidirectional: bool = True):
        if (u, v) in self.edges:
            self.edges[(u, v)].is_blocked = blocked
        if bidirectional and (v, u) in self.edges:
            self.edges[(v, u)].is_blocked = blocked

    def mark_edge_dead_end(self, u: str, v: str, dead_end: bool = True, bidirectional: bool = True):
        if (u, v) in self.edges:
            self.edges[(u, v)].is_dead_end = dead_end
        if bidirectional and (v, u) in self.edges:
            self.edges[(v, u)].is_dead_end = dead_end

    def record_traversal(self, u: str, v: str, success: bool = True, duration: Optional[float] = None):
        if (u, v) in self.edges:
            edge = self.edges[(u, v)]
            if success:
                edge.traversal_count += 1
                edge.is_blocked = False
            else:
                edge.is_blocked = True

    def find_nearest_node(self, x: float, y: float) -> Optional[str]:
        """Find the ID of the node closest to metric position (x, y)."""
        best_id = None
        best_dist = float('inf')
        for nid, node in self.nodes.items():
            d = math.hypot(node.x - x, node.y - y)
            if d < best_dist:
                best_dist = d
                best_id = nid
        return best_id

    def find_node_by_semantic_label(self, label: str) -> Optional[str]:
        """Find node matching target semantic label (case-insensitive)."""
        clean_label = label.strip().upper()
        for nid, node in self.nodes.items():
            if node.semantic_label.strip().upper() == clean_label:
                return nid
        return None

    def get_shortest_path(
        self,
        start_id: str,
        goal_id: str,
        ignore_blocked: bool = False
    ) -> Optional[List[str]]:
        """
        Dijkstra/A* algorithm to find optimal path through topological graph.
        Respects blocked edges and dead-end penalties.
        """
        if start_id not in self.nodes or goal_id not in self.nodes:
            return None

        if start_id == goal_id:
            return [start_id]

        goal_node = self.nodes[goal_id]

        # Priority queue: (f_score, g_score, current_node, path)
        open_set = []
        start_h = self.nodes[start_id].distance_to(goal_node)
        heapq.heappush(open_set, (start_h, 0.0, start_id, [start_id]))

        g_scores: Dict[str, float] = {start_id: 0.0}

        while open_set:
            f, g, current, path = heapq.heappop(open_set)

            if current == goal_id:
                return path

            if g > g_scores.get(current, float('inf')):
                continue

            for neighbor in self.adjacency.get(current, []):
                edge = self.get_edge(current, neighbor)
                if not edge:
                    continue

                if edge.is_blocked and not ignore_blocked:
                    continue

                # Don't route through dead ends unless neighbor IS the goal
                if edge.is_dead_end and neighbor != goal_id and not ignore_blocked:
                    continue

                eff_cost = edge.get_effective_cost()
                tentative_g = g + eff_cost

                if tentative_g < g_scores.get(neighbor, float('inf')):
                    g_scores[neighbor] = tentative_g
                    h = self.nodes[neighbor].distance_to(goal_node)
                    heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor, path + [neighbor]))

        return None  # No valid path found

    def to_dict(self) -> Dict[str, Any]:
        return {
            'nodes': [n.to_dict() for n in self.nodes.values()],
            'edges': [e.to_dict() for e in self.edges.values()]
        }
