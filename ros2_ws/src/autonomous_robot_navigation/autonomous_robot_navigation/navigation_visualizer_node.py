#!/usr/bin/env python3
"""
Navigation Visualizer Node for RViz Display of Topological Graph and Memory.
Publishes MarkerArray on /navigation/topological_graph_markers showing:
- Nodes (Junctions, Destinations, Dead Ends, Waypoints)
- Edges (Open, Traversed, Blocked, Dead-End)
- Active Path / Planned Route
- Sign Annotations
"""

import os
import sys

# Ensure Python AI virtual environment site-packages are accessible
venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

nav_pkg_dir = '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation'
if nav_pkg_dir not in sys.path:
    sys.path.insert(0, nav_pkg_dir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from autonomous_robot_navigation.navigation_memory import NavigationMemory
from autonomous_robot_navigation.topological_graph import TopologicalGraph


class NavigationVisualizerNode(Node):
    def __init__(self):
        super().__init__('navigation_visualizer_node')

        self.declare_parameter(
            'db_path',
            '/home/soham-darade/CV_Autonomous_Navigation/ros2_ws/src/autonomous_robot_navigation/config/navigation_memory.db'
        )
        db_path = self.get_parameter('db_path').get_parameter_value().string_value

        self.memory = NavigationMemory(db_path)
        self.active_path = []

        self.sub_active_path = self.create_subscription(
            String,
            '/navigation/active_path',
            self._path_callback,
            10
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/navigation/topological_graph_markers',
            10
        )

        # Publish markers at 2 Hz
        self.timer = self.create_timer(0.5, self._publish_markers)
        self.get_logger().info("NavigationVisualizerNode initialized.")

    def _path_callback(self, msg: String):
        if msg.data:
            self.active_path = [nid.strip() for nid in msg.data.split(',') if nid.strip()]
        else:
            self.active_path = []

    def _publish_markers(self):
        graph: TopologicalGraph = self.memory.load_graph()
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: Nodes (Spheres)
        m_nodes = Marker()
        m_nodes.header.frame_id = 'map'
        m_nodes.header.stamp = now
        m_nodes.ns = 'nodes'
        m_nodes.id = 0
        m_nodes.type = Marker.SPHERE_LIST
        m_nodes.action = Marker.ADD
        m_nodes.scale.x = 0.5
        m_nodes.scale.y = 0.5
        m_nodes.scale.z = 0.2

        # Marker 2: Node Labels (3D text)
        # Marker 3: Edges (Lines)
        m_edges_open = Marker()
        m_edges_open.header.frame_id = 'map'
        m_edges_open.header.stamp = now
        m_edges_open.ns = 'edges_open'
        m_edges_open.id = 1
        m_edges_open.type = Marker.LINE_LIST
        m_edges_open.action = Marker.ADD
        m_edges_open.scale.x = 0.08
        m_edges_open.color.r = 0.2
        m_edges_open.color.g = 0.8
        m_edges_open.color.b = 0.3
        m_edges_open.color.a = 0.8

        m_edges_blocked = Marker()
        m_edges_blocked.header.frame_id = 'map'
        m_edges_blocked.header.stamp = now
        m_edges_blocked.ns = 'edges_blocked'
        m_edges_blocked.id = 2
        m_edges_blocked.type = Marker.LINE_LIST
        m_edges_blocked.action = Marker.ADD
        m_edges_blocked.scale.x = 0.15
        m_edges_blocked.color.r = 1.0
        m_edges_blocked.color.g = 0.0
        m_edges_blocked.color.b = 0.0
        m_edges_blocked.color.a = 1.0

        label_id = 100
        for nid, node in graph.nodes.items():
            p = Point(x=node.x, y=node.y, z=0.1)
            m_nodes.points.append(p)

            # Node color
            color = Marker().color
            color.a = 0.9
            if node.node_type == 'destination':
                color.r, color.g, color.b = 0.1, 0.9, 0.2
            elif node.node_type == 'dead_end':
                color.r, color.g, color.b = 0.9, 0.1, 0.1
            elif node.node_type == 'start':
                color.r, color.g, color.b = 0.1, 0.4, 0.9
            else:
                color.r, color.g, color.b = 0.0, 0.8, 0.9
            m_nodes.colors.append(color)

            # Text label
            lbl = Marker()
            lbl.header.frame_id = 'map'
            lbl.header.stamp = now
            lbl.ns = 'labels'
            lbl.id = label_id
            label_id += 1
            lbl.type = Marker.TEXT_VIEW_FACING
            lbl.action = Marker.ADD
            lbl.pose.position.x = node.x
            lbl.pose.position.y = node.y
            lbl.pose.position.z = 0.6
            lbl.scale.z = 0.3
            lbl.color.r, lbl.color.g, lbl.color.b, lbl.color.a = 1.0, 1.0, 1.0, 1.0
            lbl.text = f"{node.semantic_label if node.semantic_label else node.name}"
            marker_array.markers.append(lbl)

        marker_array.markers.append(m_nodes)

        # Build edge lines
        seen_edges = set()
        for (u, v), edge in graph.edges.items():
            edge_key = tuple(sorted([u, v]))
            if edge_key in seen_edges:
                continue
            seen_edges.add(edge_key)

            nu = graph.nodes.get(u)
            nv = graph.nodes.get(v)
            if not nu or not nv:
                continue

            pu = Point(x=nu.x, y=nu.y, z=0.05)
            pv = Point(x=nv.x, y=nv.y, z=0.05)

            if edge.is_blocked:
                m_edges_blocked.points.extend([pu, pv])
            else:
                m_edges_open.points.extend([pu, pv])

        marker_array.markers.append(m_edges_open)
        marker_array.markers.append(m_edges_blocked)

        # Active Path Line Strip
        if len(self.active_path) > 1:
            m_path = Marker()
            m_path.header.frame_id = 'map'
            m_path.header.stamp = now
            m_path.ns = 'active_path'
            m_path.id = 50
            m_path.type = Marker.LINE_STRIP
            m_path.action = Marker.ADD
            m_path.scale.x = 0.18
            m_path.color.r = 1.0
            m_path.color.g = 0.1
            m_path.color.b = 0.9
            m_path.color.a = 0.95

            for nid in self.active_path:
                if nid in graph.nodes:
                    node = graph.nodes[nid]
                    m_path.points.append(Point(x=node.x, y=node.y, z=0.15))

            marker_array.markers.append(m_path)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
