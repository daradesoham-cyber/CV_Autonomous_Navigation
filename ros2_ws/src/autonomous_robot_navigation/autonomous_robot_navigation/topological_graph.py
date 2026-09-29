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
        is_dead_end: bool = False,
        obstacle_density: float = 0.0,
        failure_count: int = 0,
        congestion_cost: float = 0.0,
        is_narrow: bool = False,
        turns_count: int = 0,
        avg_travel_time: float = 0.0,
        reliability_score: float = 1.0
    ):
        self.from_node = from_node
        self.to_node = to_node
        self.distance = float(distance)
        self.base_cost = float(distance)
        self.cost = float(cost if cost is not None else distance)
        self.traversal_count = int(traversal_count)
        self.is_blocked = bool(is_blocked)
        self.is_dead_end = bool(is_dead_end)

        # Advanced V2 Route Cost Attributes
        self.obstacle_density = float(obstacle_density)
        self.failure_count = int(failure_count)
        self.congestion_cost = float(congestion_cost)
        self.is_narrow = bool(is_narrow)
        self.turns_count = int(turns_count)
        self.avg_travel_time = float(avg_travel_time)
        self.reliability_score = float(reliability_score)

    def calculate_cost(self, weights: Optional[Dict[str, float]] = None) -> float:
        """
        Multi-criteria Route Cost System:
        Cost = distance * w_dist + obstacle_density * w_obs + failure_penalty * w_hist
               + congestion * w_cong + turns * w_turn + narrow_penalty * w_narrow
               + travel_time * w_time - familiarity_bonus
        """
        if self.is_blocked:
            return float(weights.get('blocked_edge_penalty', 1e6) if weights else 1e6)
        if self.is_dead_end:
            return float(weights.get('dead_end_penalty', 1e5) if weights else 1e5)

        w = weights or {
            'distance_weight': 1.0,
            'obstacle_weight': 3.5,
            'history_failure_weight': 5.0,
            'congestion_weight': 2.0,
            'turn_penalty_weight': 0.8,
            'narrow_corridor_penalty': 2.5,
            'travel_time_weight': 0.5
        }

        dist_cost = w.get('distance_weight', 1.0) * self.distance
        obs_cost = w.get('obstacle_weight', 3.5) * self.obstacle_density
        hist_fail = w.get('history_failure_weight', 5.0) * (self.failure_count + (1.0 - self.reliability_score) * 2.0)
        cong_cost = w.get('congestion_weight', 2.0) * self.congestion_cost
        turn_cost = w.get('turn_penalty_weight', 0.8) * self.turns_count
        narrow_cost = w.get('narrow_corridor_penalty', 2.5) * (1.5 if self.is_narrow else 0.0)
        estimated_time = self.avg_travel_time if self.avg_travel_time > 0 else (self.distance / 0.4)
        time_cost = w.get('travel_time_weight', 0.5) * estimated_time

        # Familiarity discount for frequently and successfully traversed routes
        familiarity_bonus = min(0.12 * self.traversal_count, 0.25) * self.distance

        total_cost = dist_cost + obs_cost + hist_fail + cong_cost + turn_cost + narrow_cost + time_cost - familiarity_bonus
        return max(0.1, total_cost)

    def get_effective_cost(self, weights: Optional[Dict[str, float]] = None) -> float:
        """Calculate dynamic cost considering blockages, dead ends, and familiarity."""
        return self.calculate_cost(weights)

    def to_dict(self) -> Dict[str, Any]:
        return {
            'from_node': self.from_node,
            'to_node': self.to_node,
            'distance': round(self.distance, 2),
            'cost': round(self.cost, 2),
            'traversal_count': self.traversal_count,
            'is_blocked': self.is_blocked,
            'is_dead_end': self.is_dead_end,
            'obstacle_density': round(self.obstacle_density, 2),
            'failure_count': self.failure_count,
            'congestion_cost': round(self.congestion_cost, 2),
            'is_narrow': self.is_narrow,
            'turns_count': self.turns_count,
            'avg_travel_time': round(self.avg_travel_time, 2),
            'reliability_score': round(self.reliability_score, 2)
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
        weights: Optional[Dict[str, float]] = None,
        ignore_blocked: bool = False,
        edge_penalties: Optional[Dict[Tuple[str, str], float]] = None
    ) -> Optional[List[str]]:
        """
        Multi-criteria A* algorithm to find optimal path through topological graph.
        Respects blocked edges, dead-end penalties, and route cost weights.
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

                eff_cost = edge.calculate_cost(weights)
                if edge_penalties and (current, neighbor) in edge_penalties:
                    eff_cost += edge_penalties[(current, neighbor)]

                tentative_g = g + eff_cost

                if tentative_g < g_scores.get(neighbor, float('inf')):
                    g_scores[neighbor] = tentative_g
                    h = self.nodes[neighbor].distance_to(goal_node)
                    heapq.heappush(open_set, (tentative_g + h, tentative_g, neighbor, path + [neighbor]))

        return None  # No valid path found

    def calculate_route_metrics(
        self,
        path: List[str],
        weights: Optional[Dict[str, float]] = None
    ) -> Dict[str, Any]:
        """Calculates quantitative metric breakdown for a path."""
        if not path or len(path) < 2:
            return {
                'distance': 0.0,
                'total_cost': 0.0,
                'estimated_time': 0.0,
                'obstacle_density': 0.0,
                'reliability_pct': 100.0,
                'turns': 0,
                'has_narrow': False,
                'cost_breakdown': {}
            }

        total_dist = 0.0
        total_cost = 0.0
        obs_densities = []
        reliabilities = []
        turns = 0
        has_narrow = False

        w = weights or {
            'distance_weight': 1.0,
            'obstacle_weight': 3.5,
            'history_failure_weight': 5.0,
            'congestion_weight': 2.0,
            'turn_penalty_weight': 0.8,
            'narrow_corridor_penalty': 2.5,
            'travel_time_weight': 0.5
        }

        total_dist_cost = 0.0
        total_obs_cost = 0.0
        total_hist_cost = 0.0
        total_cong_cost = 0.0
        total_turn_cost = 0.0
        total_narrow_cost = 0.0
        total_time_cost = 0.0
        total_fam_bonus = 0.0

        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            edge = self.get_edge(u, v)
            if edge:
                total_dist += edge.distance
                total_cost += edge.calculate_cost(weights)
                obs_densities.append(edge.obstacle_density)
                reliabilities.append(edge.reliability_score)
                turns += edge.turns_count
                if edge.is_narrow:
                    has_narrow = True

                total_dist_cost += w.get('distance_weight', 1.0) * edge.distance
                total_obs_cost += w.get('obstacle_weight', 3.5) * edge.obstacle_density
                total_hist_cost += w.get('history_failure_weight', 5.0) * (edge.failure_count + (1.0 - edge.reliability_score) * 2.0)
                total_cong_cost += w.get('congestion_weight', 2.0) * edge.congestion_cost
                total_turn_cost += w.get('turn_penalty_weight', 0.8) * edge.turns_count
                total_narrow_cost += w.get('narrow_corridor_penalty', 2.5) * (1.5 if edge.is_narrow else 0.0)
                est_edge_time = edge.avg_travel_time if edge.avg_travel_time > 0 else (edge.distance / 0.4)
                total_time_cost += w.get('travel_time_weight', 0.5) * est_edge_time
                total_fam_bonus += min(0.12 * edge.traversal_count, 0.25) * edge.distance

        avg_obs = float(sum(obs_densities) / len(obs_densities)) if obs_densities else 0.0
        avg_rel = float(sum(reliabilities) / len(reliabilities)) if reliabilities else 1.0
        est_time = round(total_dist / 0.45, 1)

        return {
            'distance': round(total_dist, 2),
            'total_cost': round(total_cost, 2),
            'estimated_time': est_time,
            'obstacle_density': round(avg_obs, 2),
            'reliability_pct': round(avg_rel * 100.0, 1),
            'turns': turns,
            'has_narrow': has_narrow,
            'cost_breakdown': {
                'distance_cost': round(total_dist_cost, 2),
                'obstacle_cost': round(total_obs_cost, 2),
                'history_cost': round(total_hist_cost, 2),
                'congestion_cost': round(total_cong_cost, 2),
                'turn_cost': round(total_turn_cost, 2),
                'narrow_cost': round(total_narrow_cost, 2),
                'time_cost': round(total_time_cost, 2),
                'familiarity_bonus': round(total_fam_bonus, 2),
                'reliability_factor': round(avg_rel, 2)
            }
        }

    def find_alternative_routes(
        self,
        start_id: str,
        goal_id: str,
        k: int = 3,
        weights: Optional[Dict[str, float]] = None
    ) -> List[Dict[str, Any]]:
        """
        Finds k candidate alternative routes between start_id and goal_id,
        evaluates their multi-criteria costs, and returns them sorted by total cost.
        """
        candidate_paths = []
        edge_penalties: Dict[Tuple[str, str], float] = {}

        for _ in range(k):
            path = self.get_shortest_path(start_id, goal_id, weights=weights, edge_penalties=edge_penalties)
            if not path or path in candidate_paths:
                break
            candidate_paths.append(path)

            # Penalize edges in the discovered path to encourage finding alternative corridors
            for j in range(len(path) - 1):
                u, v = path[j], path[j + 1]
                edge_penalties[(u, v)] = edge_penalties.get((u, v), 0.0) + 15.0
                edge_penalties[(v, u)] = edge_penalties.get((v, u), 0.0) + 15.0

        if not candidate_paths:
            # Fallback: allow traversing blocked edges with heavy penalty rather than failing completely
            for _ in range(k):
                path = self.get_shortest_path(start_id, goal_id, weights=weights, ignore_blocked=True, edge_penalties=edge_penalties)
                if not path or path in candidate_paths:
                    break
                candidate_paths.append(path)
                for j in range(len(path) - 1):
                    u, v = path[j], path[j + 1]
                    edge_penalties[(u, v)] = edge_penalties.get((u, v), 0.0) + 15.0
                    edge_penalties[(v, u)] = edge_penalties.get((v, u), 0.0) + 15.0

        routes = []
        for idx, p in enumerate(candidate_paths):
            metrics = self.calculate_route_metrics(p, weights=weights)
            routes.append({
                'route_id': f"Route {chr(65 + idx)}",
                'name': f"{self.nodes[p[0]].name} to {self.nodes[p[-1]].name} (Option {chr(65 + idx)})",
                'path': p,
                'path_str': ' -> '.join(p),
                'distance': metrics['distance'],
                'estimated_time': metrics['estimated_time'],
                'total_cost': metrics['total_cost'],
                'obstacle_density': metrics['obstacle_density'],
                'reliability_pct': metrics['reliability_pct'],
                'turns': metrics['turns'],
                'has_narrow': metrics['has_narrow'],
                'cost_breakdown': metrics['cost_breakdown'],
                'is_selected': (idx == 0)
            })

        # Sort routes by total cost ascending
        routes.sort(key=lambda r: r['total_cost'])
        if routes:
            for i, r in enumerate(routes):
                r['is_selected'] = (i == 0)
        return routes

    def to_dict(self) -> Dict[str, Any]:
        return {
            'nodes': [n.to_dict() for n in self.nodes.values()],
            'edges': [e.to_dict() for e in self.edges.values()]
        }

