#!/usr/bin/env python3
"""
V3 Hospital Logistics web UI server (ROS 2 node + Flask), following the V2.6 dashboard_backend
architecture (Flask thread inside an rclpy node).

All data shown comes from runtime ROS topics, the V3 mission DB and the V3 navigation memory DB:
  /v3/mission/state, /v3/spatial_objects, /vision/detections, /camera/image_raw, /scan, /odom,
  /amcl_pose, /navigation/mission_status, /navigation/current_state, /vision/ttc,
  /vision/costmap_obstacles, /clock, /v3/perception/detection_timing.
Controls go ONLY through a fixed command whitelist published on /v3/mission/command (start, pause,
resume, cancel, reset, estop) and /simulation/enable_dynamic_obstacles (traffic on/off). No shell
execution, no arbitrary topic publishing.

Network: binds 0.0.0.0:<port> so the UI is reachable on localhost and the laptop's LAN address, but
every request from a non-loopback, non-private (RFC 1918 / link-local) client address is refused
(HTTP 403): the UI is not meant to be reachable from the public internet.
"""
import ipaddress
import json
import math
import os
import sqlite3
import sys
import threading
import time
from collections import deque

VENV_SITE = os.environ.get("V3_VENV_SITE", os.path.expanduser("~/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages"))
if VENV_SITE not in sys.path:
    sys.path.insert(0, VENV_SITE)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from flask import Flask, Response, abort, jsonify, request, send_from_directory  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, LaserScan, PointCloud2  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

from autonomous_robot_interfaces.msg import Detection2DArray  # noqa: E402

V3_ROOT = os.environ.get("V3_ROOT", os.path.expanduser("~/CV_Autonomous_Navigation/V3"))
UI_DIR = os.path.join(V3_ROOT, "ui")
ALLOWED_CMDS = {"start", "pause", "resume", "cancel", "reset", "estop"}
AMCL_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class Freshness:
    def __init__(self):
        self.t = deque(maxlen=600)  # >= 5 s at the fastest topic (camera 30 Hz, clock ~100 Hz)

    def tick(self):
        self.t.append(time.time())

    def hz(self):
        now = time.time()
        recent = [x for x in self.t if now - x < 5.0]
        return round(len(recent) / 5.0, 1) if recent else 0.0

    def age(self):
        return round(time.time() - self.t[-1], 2) if self.t else None


class V3WebServer(Node):
    def __init__(self):
        super().__init__("v3_web_server")
        self.declare_parameter("port", 8080)
        self.declare_parameter("stream_fps", 8.0)
        self.declare_parameter("jpeg_quality", 70)
        self.port = int(self.get_parameter("port").value)
        self.stream_period = 1.0 / float(self.get_parameter("stream_fps").value)
        self.jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self.lock = threading.Lock()
        self.fresh = {k: Freshness() for k in ("clock", "camera", "scan", "odom", "amcl", "detections", "fusion",
                                               "mission", "decision_engine", "costmap", "ttc")}
        self.latest = {"mission": None, "spatial": None, "de_status": None, "de_state": None, "ttc": None,
                       "pose": None, "twist": None, "costmap_points": 0, "det_timing": None, "scan_min": None}
        self.frame = None           # latest raw BGR frame
        self.frame_stamp = None
        self.stream_clients = 0
        self.jpeg = None
        self.jpeg_time = 0.0
        self.encode_ms = deque(maxlen=50)
        self.nav_events = deque(maxlen=60)   # decision engine /navigation/events (replans, TTC, obstacles, signs)
        self.scan = None                     # latest /scan, downsampled for the UI LiDAR view
        self.node_names = []                 # live ROS graph node names (refreshed every 2 s)
        self.node_names_t = 0.0
        self.started = time.time()

        self.create_subscription(Clock, "/clock", lambda m: self.fresh["clock"].tick(), 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, AMCL_QOS)
        self.create_subscription(Detection2DArray, "/vision/detections", lambda m: self.fresh["detections"].tick(), 10)
        self.create_subscription(String, "/v3/spatial_objects", self.on_spatial, 10)
        self.create_subscription(String, "/v3/mission/state", self.on_mission, 10)
        self.create_subscription(String, "/navigation/mission_status", self.on_de_status, 10)
        self.create_subscription(String, "/navigation/current_state", self.on_de_state, 10)
        self.create_subscription(Float32, "/vision/ttc", self.on_ttc, 10)
        self.create_subscription(PointCloud2, "/vision/costmap_obstacles", self.on_costmap, 10)
        self.create_subscription(String, "/v3/perception/detection_timing", self.on_timing, 10)
        self.create_subscription(String, "/navigation/events", self.on_nav_event, 50)
        self.pub_cmd = self.create_publisher(String, "/v3/mission/command", 10)
        self.pub_traffic = self.create_publisher(Bool, "/simulation/enable_dynamic_obstacles", 10)
        self.traffic_enabled = None
        self.map_info = self.load_map()
        self.topology = self.load_topology()
        self.get_logger().info(f"V3 web UI on 0.0.0.0:{self.port} (private/loopback clients only)")

    # ---------------- ROS callbacks
    def on_image(self, m):
        self.fresh["camera"].tick()
        if self.stream_clients <= 0:
            return  # only keep frames while someone is watching
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            img = img[:, :, ::-1]
        with self.lock:
            self.frame = img.copy()
            self.frame_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_scan(self, m):
        self.fresh["scan"].tick()
        v = [r for r in m.ranges if math.isfinite(r) and r >= m.range_min]
        self.latest["scan_min"] = round(min(v), 3) if v else None
        step = max(1, len(m.ranges) // 240)
        self.scan = {"angle_min": m.angle_min, "angle_inc": m.angle_increment * step, "range_max": m.range_max,
                     "ranges": [round(r, 2) if math.isfinite(r) and m.range_min <= r <= m.range_max else None
                                for r in m.ranges[::step]]}

    def on_nav_event(self, m):
        try:
            e = json.loads(m.data)
        except Exception:
            return
        e["t"] = time.time()
        self.nav_events.append(e)

    def ros_nodes(self):
        if time.time() - self.node_names_t > 2.0:
            try:
                self.node_names = sorted(n for n, _ns in self.get_node_names_and_namespaces())
            except Exception:
                pass
            self.node_names_t = time.time()
        return self.node_names

    def on_odom(self, m):
        self.fresh["odom"].tick()
        self.latest["twist"] = (round(m.twist.twist.linear.x, 3), round(m.twist.twist.angular.z, 3))

    def on_amcl(self, m):
        self.fresh["amcl"].tick()
        p = m.pose.pose
        q = p.orientation
        self.latest["pose"] = (round(p.position.x, 3), round(p.position.y, 3),
                               round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 3))

    def on_spatial(self, m):
        self.fresh["fusion"].tick()
        self.latest["spatial"] = json.loads(m.data)

    def on_mission(self, m):
        self.fresh["mission"].tick()
        self.latest["mission"] = json.loads(m.data)

    def on_de_status(self, m):
        self.fresh["decision_engine"].tick()
        try:
            self.latest["de_status"] = json.loads(m.data)
        except Exception:
            pass

    def on_de_state(self, m):
        self.latest["de_state"] = m.data

    def on_ttc(self, m):
        self.fresh["ttc"].tick()
        self.latest["ttc"] = round(float(m.data), 2)

    def on_costmap(self, m):
        self.fresh["costmap"].tick()
        self.latest["costmap_points"] = m.width * m.height

    def on_timing(self, m):
        self.latest["det_timing"] = json.loads(m.data)

    # ---------------- static data
    def load_map(self):
        ypath = os.path.join(V3_ROOT, "maps", "v3_hospital_map.yaml")
        y = yaml.safe_load(open(ypath))
        pgm = cv2.imread(os.path.join(os.path.dirname(ypath), y["image"]), cv2.IMREAD_GRAYSCALE)
        os.makedirs(os.path.join(UI_DIR, "generated"), exist_ok=True)
        # The NEW hospital is tall (27 x 59 m): the UI shows it rotated 90 deg (north to the left, east up)
        rotated = pgm.shape[0] > 1.3 * pgm.shape[1]
        cv2.imwrite(os.path.join(UI_DIR, "generated", "map.png"),
                    cv2.rotate(pgm, cv2.ROTATE_90_COUNTERCLOCKWISE) if rotated else pgm)
        return {"image": "/generated/map.png", "resolution": y["resolution"], "origin": y["origin"][:2],
                "width": int(pgm.shape[1]), "height": int(pgm.shape[0]), "rotated": rotated}

    def load_topology(self):
        db = os.path.join(V3_ROOT, "data", "v3_hospital_navigation_memory.db")
        if not os.path.exists(db):
            return {"nodes": {}, "edges": []}
        with sqlite3.connect(db) as c:
            nodes = {r[0]: {"name": r[1], "x": r[2], "y": r[3], "type": r[4]} for r in c.execute("SELECT id, name, x, y, node_type FROM nodes")}
            edges = [list(r) for r in c.execute("SELECT from_node, to_node FROM edges")]
        return {"nodes": nodes, "edges": edges}

    # ---------------- camera stream
    def annotated_jpeg(self):
        now = time.time()
        if self.jpeg is not None and now - self.jpeg_time < self.stream_period:
            return self.jpeg
        with self.lock:
            img = None if self.frame is None else self.frame.copy()
        if img is None:
            return None
        t0 = time.perf_counter()
        sp = self.latest["spatial"] or {}
        for o in sp.get("objects", []):
            x0, y0, x1, y1 = map(int, o["box"])
            col = (0, 200, 0) if o["range_source"] == "lidar" else (0, 200, 255) if o["range_source"] == "ground_plane" else (255, 160, 0)
            cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
            label = f"{o['class']} {o['confidence']:.2f}"
            if o.get("range") is not None:
                label += f" {o['range']:.2f}m ({o['x']:.2f},{o['y']:.2f})"
            elif o["range_source"] == "visual_only":
                label += " visual-only"
            cv2.putText(img, label, (x0 + 2, max(14, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        self.encode_ms.append((time.perf_counter() - t0) * 1000)
        if ok:
            self.jpeg, self.jpeg_time = buf.tobytes(), now
        return self.jpeg

    # ---------------- aggregated state
    def status(self):
        f = {k: {"hz": v.hz(), "age_s": v.age()} for k, v in self.fresh.items()}
        ok = lambda k, hz_min=0.5: f[k]["hz"] >= hz_min
        mission = self.latest["mission"] or {}
        de = self.latest["de_status"] or {}
        return {
            "server_time": time.time(),
            "system": {"ros": True, "gazebo": ok("clock", 5), "camera": ok("camera", 5), "lidar": ok("scan", 5),
                       "localization": self.latest["pose"] is not None, "yolo": ok("detections", 1), "fusion": ok("fusion", 1),
                       "navigation": ok("decision_engine", 0.5), "mission_manager": ok("mission", 0.5),
                       "costmap_obstacles": ok("costmap", 1), "topics": f},
            "robot": {"pose": self.latest["pose"], "twist": self.latest["twist"], "scan_min_m": self.latest["scan_min"]},
            "mission": mission,
            "navigation": {"decision_engine_state": self.latest["de_state"], "active_path": de.get("active_path"),
                           "route_cost": de.get("route_cost"), "reliability_pct": de.get("reliability_pct"),
                           "cost_breakdown": de.get("cost_breakdown"), "replans": de.get("replans_count"),
                           "last_replan_reason": de.get("last_replan_reason"), "ttc": self.latest["ttc"],
                           "ttc_events": de.get("ttc_events_count"), "clearances": de.get("clearances"),
                           "progress_pct": de.get("progress_pct"), "costmap_obstacle_points": self.latest["costmap_points"]},
            "perception": {"spatial": self.latest["spatial"], "detection_timing": self.latest["det_timing"],
                           "stream_encode_ms": round(float(np.mean(self.encode_ms)), 2) if self.encode_ms else None,
                           "stream_clients": self.stream_clients},
            "traffic_enabled": self.traffic_enabled,
            "nav_events": list(self.nav_events)[-30:],
            "ros_nodes": self.ros_nodes(),
            "ui_backend": {"uptime_s": round(time.time() - self.started, 1), "port": self.port,
                           "stream_fps": round(1.0 / self.stream_period, 1)},
        }


def create_app(node):
    app = Flask(__name__, static_folder=None)

    @app.before_request
    def lan_only():
        try:
            ip = ipaddress.ip_address(request.remote_addr)
        except ValueError:
            abort(403)
        if not (ip.is_loopback or ip.is_private or ip.is_link_local):
            abort(403)

    @app.route("/")
    def index():
        return send_from_directory(UI_DIR, "index.html")

    @app.route("/generated/<path:f>")
    def generated(f):
        return send_from_directory(os.path.join(UI_DIR, "generated"), f)

    @app.route("/api/state")
    def api_state():
        return jsonify(node.status())

    @app.route("/api/map")
    def api_map():
        return jsonify({"map": node.map_info, "topology": node.topology})

    @app.route("/api/history")
    def api_history():
        db = os.path.join(V3_ROOT, "data", "v3_hospital_missions.db")
        if not os.path.exists(db):
            return jsonify([])
        with sqlite3.connect(db) as c:
            c.row_factory = sqlite3.Row
            rows = c.execute("SELECT mission_id, start_time, source, destination, result, failure_reason, duration_s, "
                             "distance_m, min_lidar_range_m, replans, ttc_events, obstacle_encounters, sign_events, pauses, legs_json "
                             "FROM missions ORDER BY start_time DESC LIMIT 100").fetchall()
        out = []
        for r in rows:
            d = dict(r)
            legs = json.loads(d.pop("legs_json") or "[]")
            d["routes"] = [{"target": leg.get("target"), "result": leg.get("result"), "duration_s": leg.get("duration_s"),
                            "path": (leg.get("route") or {}).get("active_path")} for leg in legs]
            out.append(d)
        return jsonify(out)

    @app.route("/api/scan")
    def api_scan():
        return jsonify(node.scan or {})

    @app.route("/api/learning")
    def api_learning():
        """Route learning exactly as the decision engine sees it: its NavigationMemory.load_graph() edges and
        their calculate_cost() terms, plus the stored replan / obstacle records per segment (read only)."""
        db = os.path.join(V3_ROOT, "data", "v3_hospital_navigation_memory.db")
        if not os.path.exists(db):
            return jsonify({"segments": [], "replans": [], "obstacles": {}})
        with sqlite3.connect(db) as c:
            c.row_factory = sqlite3.Row
            segs = [dict(r) for r in c.execute(
                "SELECT rs.from_node, rs.to_node, rs.distance, rs.traversal_count, rs.success_count, rs.failure_count, "
                "rs.avg_travel_time, rs.obstacle_encounter_count, rs.replan_count, rs.is_narrow, rs.reliability_score, "
                "rs.last_traversed, e.is_blocked, e.is_dead_end FROM route_segments rs "
                "LEFT JOIN edges e ON e.from_node = rs.from_node AND e.to_node = rs.to_node")]
            replans = [dict(r) for r in c.execute(
                "SELECT timestamp, from_node, to_node, replan_reason, trigger_details FROM replan_events "
                "ORDER BY timestamp DESC LIMIT 100")]
            obstacles = {}
            for r in c.execute("SELECT edge_from, edge_to, obstacle_class, count(*) n FROM obstacles GROUP BY edge_from, edge_to, obstacle_class"):
                obstacles.setdefault(f"{r[0]}|{r[1]}", {})[r[2]] = r[3]
            traversals = c.execute("SELECT count(*), sum(success) FROM traversals").fetchone()
        try:
            from autonomous_robot_navigation.navigation_memory import NavigationMemory
            g = NavigationMemory(db).load_graph()
            cost = {(u, v): e for (u, v), e in g.edges.items()}
        except Exception:
            cost = {}
        for s in segs:
            e = cost.get((s["from_node"], s["to_node"]))
            if e is not None:
                s["route_cost"] = round(e.calculate_cost(), 2)
                s["obstacle_density"] = round(e.obstacle_density, 3)
        return jsonify({"segments": segs, "replans": replans, "obstacles": obstacles,
                        "traversals": {"total": traversals[0], "successful": traversals[1] or 0},
                        "cost_model": "distance + 3.5*obstacle_density + 5*(failures + 2*(1-reliability)) + 2.5*1.5[narrow] "
                                      "+ 0.5*travel_time - min(0.12*traversals, 0.25)*distance"})

    @app.route("/api/history/<mission_id>")
    def api_history_one(mission_id):
        db = os.path.join(V3_ROOT, "data", "v3_hospital_missions.db")
        with sqlite3.connect(db) as c:
            c.row_factory = sqlite3.Row
            r = c.execute("SELECT * FROM missions WHERE mission_id = ?", (mission_id,)).fetchone()
        if r is None:
            abort(404)
        d = dict(r)
        for k in ("legs_json", "path_json", "events_json"):
            d[k[:-5]] = json.loads(d.pop(k) or "[]")
        return jsonify(d)

    @app.route("/api/mission", methods=["POST"])
    def api_mission():
        body = request.get_json(silent=True) or {}
        cmd = str(body.get("cmd", "")).lower()
        if cmd not in ALLOWED_CMDS:
            return jsonify({"ok": False, "error": f"command not allowed: {cmd}"}), 400
        msg = {"cmd": cmd}
        if cmd == "start":
            locs = ((node.latest["mission"] or {}).get("locations") or {})
            for k in ("source", "destination"):
                if k in body:
                    if locs and body[k] not in locs:
                        return jsonify({"ok": False, "error": f"unknown {k}: {body[k]}"}), 400
                    msg[k] = body[k]
            if msg.get("source") is not None and msg.get("source") == msg.get("destination"):
                return jsonify({"ok": False, "error": "source and destination must differ"}), 400
        node.pub_cmd.publish(String(data=json.dumps(msg)))
        return jsonify({"ok": True, "sent": msg})

    @app.route("/api/traffic", methods=["POST"])
    def api_traffic():
        body = request.get_json(silent=True) or {}
        en = bool(body.get("enable", False))
        node.pub_traffic.publish(Bool(data=en))
        node.traffic_enabled = en
        return jsonify({"ok": True, "enabled": en})

    @app.route("/api/camera.mjpg")
    def camera():
        def gen():
            node.stream_clients += 1
            try:
                while True:
                    jpg = node.annotated_jpeg()
                    if jpg is not None:
                        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                    time.sleep(node.stream_period)
            finally:
                node.stream_clients -= 1
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/camera.jpg")
    def camera_single():
        node.stream_clients += 1
        try:
            end = time.time() + 2.0
            jpg = node.annotated_jpeg()
            while jpg is None and time.time() < end:
                time.sleep(0.1)
                jpg = node.annotated_jpeg()
        finally:
            node.stream_clients -= 1
        if jpg is None:
            abort(503)
        return Response(jpg, mimetype="image/jpeg")

    return app


def main(args=None):
    rclpy.init(args=args)
    node = V3WebServer()
    app = create_app(node)
    th = threading.Thread(target=lambda: app.run(host="0.0.0.0", port=node.port, threaded=True, use_reloader=False), daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
