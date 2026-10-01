#!/usr/bin/env python3
"""
V3 hospital logistics mission manager.

Mission: SOURCE (default collection_point) -> collect payload -> DESTINATION (default laboratory)
-> deliver -> complete. Navigation is executed by the V2.6 decision engine (topological routing with
learned route costs, destination-aware sign gating, replanning, TTC yield) through its existing
interfaces: /navigation/goal_label (topology node id), /navigation/control (PAUSE/RESUME/ABORT),
/navigation/current_state, /navigation/mission_status, /navigation/alternative_paths.

States: IDLE, READY, GO_TO_COLLECTION, AT_COLLECTION, COLLECTING, PAYLOAD_SECURED, GO_TO_LAB,
AT_LAB, DELIVERING, MISSION_COMPLETE, PAUSED, CANCELLED, FAILED, EMERGENCY_STOP.
All transitions are driven by runtime events (decision-engine state, timers, operator commands).

Commands (std_msgs/String JSON on /v3/mission/command):
  {"cmd": "start", "source": "collection_point", "destination": "laboratory"}
  {"cmd": "pause"} {"cmd": "resume"} {"cmd": "cancel"} {"cmd": "reset"} {"cmd": "estop"}
State: /v3/mission/state (std_msgs/String JSON, 2 Hz + on change).
History: every finished mission (complete / failed / cancelled / e-stopped) is stored in the V3 mission
database (SQLite, V3/data/v3_hospital_missions.db) with the route legs and measured metrics.
"""
import json
import math
import os
import sqlite3
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

# Nav2 AMCL publishes /amcl_pose transient-local: late subscribers still get the last pose
AMCL_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

from autonomous_robot_interfaces.msg import Detection2DArray, SemanticObstacleArray

V3_ROOT = os.environ.get('V3_ROOT', os.path.expanduser('~/CV_Autonomous_Navigation/V3'))
NAV_ACTIVE = {'PLANNING', 'NAVIGATING', 'OBSTACLE_DETECTED', 'REPLANNING', 'RECOVERY', 'BACKTRACKING', 'DEAD_END'}

SCHEMA = """
CREATE TABLE IF NOT EXISTS missions (
  mission_id TEXT PRIMARY KEY, start_time REAL, end_time REAL, source TEXT, destination TEXT,
  payload TEXT, result TEXT, failure_reason TEXT, duration_s REAL, distance_m REAL,
  min_lidar_range_m REAL, replans INTEGER, ttc_events INTEGER, obstacle_encounters INTEGER,
  sign_events INTEGER, pauses INTEGER, legs_json TEXT, path_json TEXT, events_json TEXT,
  collection_marker_seen INTEGER, lab_marker_seen INTEGER, result_note TEXT
);
"""


def default_db_path():
    return os.path.join(V3_ROOT, 'data', 'v3_hospital_missions.db')


class MissionDB:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        with sqlite3.connect(path) as c:
            c.executescript(SCHEMA)

    def insert(self, m):
        cols = ['mission_id', 'start_time', 'end_time', 'source', 'destination', 'payload', 'result', 'failure_reason',
                'duration_s', 'distance_m', 'min_lidar_range_m', 'replans', 'ttc_events', 'obstacle_encounters',
                'sign_events', 'pauses', 'legs_json', 'path_json', 'events_json', 'collection_marker_seen', 'lab_marker_seen',
                'result_note']
        with sqlite3.connect(self.path) as c:
            c.execute(f"INSERT OR REPLACE INTO missions ({','.join(cols)}) VALUES ({','.join('?' * len(cols))})",
                      [m.get(k) for k in cols])


class V3MissionManager(Node):
    def __init__(self):
        super().__init__('v3_mission_manager')
        self.declare_parameter('locations_file', os.path.join(V3_ROOT, 'config', 'logistics_locations.yaml'))
        self.declare_parameter('db_path', default_db_path())
        cfg = yaml.safe_load(open(self.get_parameter('locations_file').value))
        self.locations = cfg['locations']
        self.mcfg = cfg['mission']
        self.db = MissionDB(self.get_parameter('db_path').value)

        self.state = 'IDLE'
        self.prev_state_before_pause = None
        self.de_state = None           # decision engine state
        self.de_status = {}            # last /navigation/mission_status
        self.alt_routes = []
        self.pose = None               # map pose from AMCL
        self.last_odom = None
        self.mission = None
        self.dwell_until = None
        self.dwell_remaining = None
        self.leg_started = None
        self.estop_active = False
        self.last_marker_seen = {}     # class -> wall time
        self.events = []

        # used ONLY to verify that the Nav2 navigate_to_pose action server is up before declaring READY
        # (the decision engine sends the goals); measured failure: READY at 7 s, Nav2 not yet active
        self.nav2_probe = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub_goal = self.create_publisher(String, '/navigation/goal_label', 10)
        self.pub_ctrl = self.create_publisher(String, '/navigation/control', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # V2.6 decision_engine PAUSE only sends one zero Twist and keeps the Nav2 goal alive, so the robot
        # kept driving to the current waypoint (measured: ~3.3 s / ~1 m after PAUSE). V3 cancels the active
        # NavigateToPose goal itself (zero goal id + zero stamp = cancel all); the paused decision engine
        # ignores the CANCELED result and re-dispatches the same waypoint on RESUME.
        self.nav2_cancel = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.pub_state = self.create_publisher(String, '/v3/mission/state', 10)
        self.create_subscription(String, '/v3/mission/command', self.on_command, 10)
        self.create_subscription(String, '/navigation/current_state', self.on_de_state, 10)
        self.create_subscription(String, '/navigation/mission_status', self.on_de_status, 10)
        self.create_subscription(String, '/navigation/alternative_paths', self.on_alt, 10)
        self.create_subscription(String, '/navigation/events', self.on_de_event, 50)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, AMCL_QOS)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32, '/vision/ttc', self.on_ttc, 10)
        self.create_subscription(Detection2DArray, '/vision/detections', self.on_dets, 10)
        self.create_subscription(SemanticObstacleArray, '/vision/semantic_obstacles', self.on_sem, 10)
        self.last_dets_t = 0.0
        self.last_fusion_t = 0.0
        self.create_subscription(String, '/v3/spatial_objects', lambda m: setattr(self, 'last_fusion_t', time.time()), 10)
        self.create_timer(0.5, self.tick)
        self.create_timer(0.05, self.estop_tick)
        self.get_logger().info(f'V3 mission manager ready; mission DB {self.db.path}')

    # ------------------------------------------------------------------ helpers
    def log(self, msg, level='INFO'):
        e = {'t': round(time.time(), 2), 'level': level, 'msg': msg, 'state': self.state}
        self.events.append(e)
        self.events = self.events[-200:]
        if self.mission is not None:
            self.mission['events'].append(e)
        self.get_logger().info(f'[{self.state}] {msg}')

    def set_state(self, s, msg=''):
        if s != self.state:
            self.log(f'{self.state} -> {s}' + (f': {msg}' if msg else ''))
            self.state = s
            self.publish_state()

    def node_of(self, loc):
        return self.locations[loc]['node']

    def dist_to(self, loc):
        if self.pose is None:
            return None
        L = self.locations[loc]
        return math.hypot(self.pose[0] - L['x'], self.pose[1] - L['y'])

    def ready_conditions(self):
        return {'amcl_pose': self.pose is not None,
                'decision_engine': self.count_subscribers('/navigation/goal_label') > 0,
                'nav2_navigate_to_pose': self.nav2_probe.server_is_ready(),
                # perception must be producing output (YOLO model loaded, fusion running) before READY
                'yolo_detections': time.time() - self.last_dets_t < 2.0,
                'spatial_fusion': time.time() - self.last_fusion_t < 2.0}

    # ------------------------------------------------------------------ subscriptions
    def on_pose(self, m):
        p = m.pose.pose
        q = p.orientation
        self.pose = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        if self.mission is not None and self.state not in ('MISSION_COMPLETE', 'CANCELLED', 'FAILED', 'EMERGENCY_STOP'):
            path = self.mission['path']
            if not path or math.hypot(path[-1][0] - self.pose[0], path[-1][1] - self.pose[1]) > 0.1:
                path.append([round(self.pose[0], 2), round(self.pose[1], 2)])

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.last_odom is not None and self.mission is not None and self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
            d = math.hypot(p.x - self.last_odom[0], p.y - self.last_odom[1])
            if d < 0.5:  # ignore teleports / resets
                self.mission['distance_m'] += d
        self.last_odom = (p.x, p.y)
        self.twist = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def on_scan(self, m):
        if self.mission is None or self.state not in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
            return
        v = [r for r in m.ranges if math.isfinite(r) and r >= m.range_min]
        if v:
            self.mission['min_lidar_range_m'] = min(self.mission['min_lidar_range_m'], min(v))

    def on_ttc(self, m):
        if self.mission is not None and 0.0 < m.data < 1.8 and self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
            now = time.time()
            if now - self.mission.get('_last_ttc', 0) > 2.0:  # count distinct TTC alerts, not samples
                self.mission['ttc_events'] += 1
                self.mission['_last_ttc'] = now

    def on_dets(self, m):
        now = time.time()
        self.last_dets_t = now
        for d in m.detections:
            if d.class_name in ('collection_point_marker', 'lab_test_point_marker'):
                self.last_marker_seen[d.class_name] = now

    def on_sem(self, m):
        if self.mission is None or self.state not in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
            return
        for o in m.obstacles:
            tid = o.direction.split('ID:')[1].split()[0] if 'ID:' in o.direction else None
            if tid:
                self.mission['_obstacle_ids'].add(f'{o.class_name}:{tid}')

    def on_alt(self, m):
        try:
            self.alt_routes = json.loads(m.data)
        except Exception:
            pass

    def on_de_status(self, m):
        try:
            self.de_status = json.loads(m.data)
        except Exception:
            pass

    def on_de_event(self, m):
        try:
            e = json.loads(m.data)
        except Exception:
            return
        if self.mission is None:
            return
        txt = e.get('message', '')
        # sign decisions of the decision engine, de-duplicated per (category, sign): the engine re-emits
        # the same decision for every confirmed frame
        cat = ('CONFIRMS_ROUTE' if txt.startswith('Sign CONFIRMS route') else
               'IRRELEVANT_PRESERVED_ROUTE' if txt.startswith('Sign observed:') and 'preserving global route' in txt else
               'CLOSED_PENALIZED' if txt.startswith("Sign '") and 'Penalized' in txt else
               'DETECTED' if txt.startswith('Sign detected:') else None)
        if cat:
            sign = txt.split("'")[1] if "'" in txt else txt
            key = f'{cat}|{sign}'
            if key not in self.mission['_sign_keys']:
                self.mission['_sign_keys'].add(key)
                self.mission['sign_events'].append({'category': cat, 'sign': sign, 'event': txt, 't': round(time.time(), 2)})
        if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB') and self.mission['legs']:
            self.mission['legs'][-1]['decision_engine_events'].append(txt)

    def on_de_state(self, m):
        prev, self.de_state = self.de_state, m.data
        if self.mission is None:
            return
        if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB') and self.mission['legs']:
            self.mission['legs'][-1]['de_states'].append(m.data)
        if self.state == 'GO_TO_COLLECTION' and m.data == 'GOAL_REACHED':
            self.finish_leg('arrived')
            self.set_state('AT_COLLECTION', 'decision engine GOAL_REACHED at collection point')
            self.begin_dwell('COLLECTING', self.mcfg['collect_dwell_s'])
        elif self.state == 'GO_TO_LAB' and m.data == 'GOAL_REACHED':
            self.finish_leg('arrived')
            self.set_state('AT_LAB', 'decision engine GOAL_REACHED at laboratory')
            self.begin_dwell('DELIVERING', self.mcfg['deliver_dwell_s'])
        elif self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB') and m.data == 'FAILED':
            self.finish_leg('failed')
            self.end_mission('FAILED', f'decision engine FAILED ({self.de_status.get("last_replan_reason", "unknown")})')

    # ------------------------------------------------------------------ mission flow
    def start_leg(self, target_loc, state):
        leg = {'target': target_loc, 'node': self.node_of(target_loc), 'start': time.time(), 'end': None,
               'result': None, 'route': None, 'de_states': [], 'decision_engine_events': []}
        self.mission['legs'].append(leg)
        self.leg_started = time.time()
        self.alt_routes = []
        self.set_state(state, f"goal '{leg['node']}' sent to decision engine")
        self.pub_goal.publish(String(data=leg['node']))

    def finish_leg(self, result):
        leg = self.mission['legs'][-1]
        leg['end'] = time.time()
        leg['result'] = result
        leg['paused_s'] = round(sum(leg.pop('_paused', [])), 1)
        leg.pop('_pause_start', None)
        leg['duration_s'] = round(leg['end'] - leg['start'] - leg['paused_s'], 1)
        st = self.de_status or {}
        leg['route'] = {'active_path': st.get('active_path'), 'route_cost': st.get('route_cost'),
                        'cost_breakdown': st.get('cost_breakdown'), 'reliability_pct': st.get('reliability_pct'),
                        'replans': st.get('replans_count', 0), 'replan_reasons': st.get('replan_reasons'),
                        'ttc_events_de': st.get('ttc_events_count', 0), 'de_min_clearance': (st.get('clearances') or {}).get('min'),
                        'alternatives': [{k: r.get(k) for k in ('route_id', 'path', 'distance', 'total_cost', 'reliability_pct',
                                                                  'estimated_time', 'cost_breakdown', 'is_selected')}
                                         for r in self.alt_routes]}
        self.mission['replans'] += int(st.get('replans_count', 0) or 0)

    def begin_dwell(self, state, seconds):
        self.set_state(state, f'simulated hand-over {seconds:.0f} s')
        self.dwell_until = time.time() + seconds

    def start_mission(self, source, dest):
        if source not in self.locations or dest not in self.locations or source == dest:
            self.log(f'invalid mission {source} -> {dest}', 'ERROR')
            return
        self.mission = {'mission_id': time.strftime('M%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:4], 'start_time': time.time(),
                        'source': source, 'destination': dest, 'payload': self.mcfg.get('payload', 'sample'),
                        'payload_state': 'NONE', 'legs': [], 'path': [], 'events': [], 'distance_m': 0.0,
                        'min_lidar_range_m': float('inf'), 'replans': 0, 'ttc_events': 0, 'sign_events': [],
                        'pauses': 0, '_obstacle_ids': set(), '_sign_keys': set(), 'collection_marker_seen': 0, 'lab_marker_seen': 0}
        self.log(f"mission {self.mission['mission_id']} start: {source} -> {dest}")
        d = self.dist_to(source)
        if d is not None and d <= self.mcfg['arrival_tolerance_m']:
            self.set_state('AT_COLLECTION', f'already at {source} ({d:.2f} m)')
            self.begin_dwell('COLLECTING', self.mcfg['collect_dwell_s'])
        else:
            self.start_leg(source, 'GO_TO_COLLECTION')

    def end_mission(self, result, reason=''):
        m = self.mission
        if m is None:
            return
        state = {'COMPLETE': 'MISSION_COMPLETE', 'FAILED': 'FAILED', 'CANCELLED': 'CANCELLED', 'ESTOP': 'EMERGENCY_STOP'}[result]
        self.set_state(state, reason)
        m['end_time'] = time.time()
        rec = {k: v for k, v in m.items() if not k.startswith('_')}
        rec.update(result=result, failure_reason=None if result == 'COMPLETE' else (reason or None), result_note=reason, duration_s=round(m['end_time'] - m['start_time'], 1),
                   distance_m=round(m['distance_m'], 2),
                   min_lidar_range_m=None if m['min_lidar_range_m'] == float('inf') else round(m['min_lidar_range_m'], 3),
                   obstacle_encounters=len(m['_obstacle_ids']), sign_events=len(m['sign_events']),
                   legs_json=json.dumps(m['legs']), path_json=json.dumps(m['path']),
                   events_json=json.dumps({'mission_events': m['events'][-100:], 'sign_decisions': m['sign_events']}))
        try:
            self.db.insert(rec)
            self.log(f"mission {m['mission_id']} stored: {result} {reason}")
        except Exception as e:
            self.log(f'could not store mission: {e}', 'ERROR')
        self.last_result = rec

    # ------------------------------------------------------------------ commands
    def on_command(self, msg):
        try:
            c = json.loads(msg.data)
        except Exception:
            c = {'cmd': msg.data.strip().lower()}
        cmd = c.get('cmd', '').lower()
        self.log(f'command: {c}')
        if cmd == 'start':
            if self.state not in ('READY', 'MISSION_COMPLETE'):
                self.log(f'start refused in state {self.state}', 'WARN')
                return
            self.start_mission(c.get('source', self.mcfg['default_source']), c.get('destination', self.mcfg['default_destination']))
        elif cmd == 'pause':
            if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB', 'COLLECTING', 'DELIVERING'):
                self.prev_state_before_pause = self.state
                if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
                    self.pub_ctrl.publish(String(data='PAUSE'))
                    self.cancel_nav2_goals()
                    self.mission['legs'][-1]['_pause_start'] = time.time()
                if self.dwell_until:
                    self.dwell_remaining = max(0.0, self.dwell_until - time.time())
                    self.dwell_until = None
                self.mission['pauses'] += 1
                self.set_state('PAUSED', f'paused during {self.prev_state_before_pause}')
        elif cmd == 'resume':
            if self.state == 'PAUSED' and self.prev_state_before_pause:
                s = self.prev_state_before_pause
                self.prev_state_before_pause = None
                if s in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
                    self.pub_ctrl.publish(String(data='RESUME'))
                    leg = self.mission['legs'][-1]
                    if leg.get('_pause_start'):
                        leg.setdefault('_paused', []).append(time.time() - leg.pop('_pause_start'))
                if self.dwell_remaining is not None:
                    self.dwell_until = time.time() + self.dwell_remaining
                    self.dwell_remaining = None
                self.set_state(s, 'resumed')
        elif cmd == 'cancel':
            if self.mission is not None and self.state not in ('MISSION_COMPLETE', 'CANCELLED', 'FAILED', 'EMERGENCY_STOP'):
                self.pub_ctrl.publish(String(data='ABORT'))
                self.cancel_nav2_goals()
                if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB') or self.prev_state_before_pause in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
                    self.finish_leg('cancelled')
                self.dwell_until = None
                self.end_mission('CANCELLED', 'cancelled by operator')
        elif cmd == 'estop':
            self.estop_active = True
            self.pub_ctrl.publish(String(data='ABORT'))
            self.cancel_nav2_goals()
            self.pub_cmd_vel.publish(Twist())
            self.dwell_until = None
            if self.mission is not None and self.state not in ('MISSION_COMPLETE', 'CANCELLED', 'FAILED', 'EMERGENCY_STOP'):
                if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB'):
                    self.finish_leg('emergency_stop')
                self.end_mission('ESTOP', 'emergency stop by operator')
            else:
                self.set_state('EMERGENCY_STOP', 'emergency stop by operator')
        elif cmd == 'reset':
            if self.state in ('MISSION_COMPLETE', 'CANCELLED', 'FAILED', 'EMERGENCY_STOP', 'IDLE'):
                self.estop_active = False
                self.mission = None
                self.dwell_until = None
                self.prev_state_before_pause = None
                self.set_state('READY' if all(self.ready_conditions().values()) else 'IDLE', 'reset')

    def cancel_nav2_goals(self):
        if self.nav2_cancel.service_is_ready():
            self.nav2_cancel.call_async(CancelGoal.Request())
        else:
            self.log('Nav2 cancel service not available; pause relies on the decision engine only', 'WARN')
        self.pub_cmd_vel.publish(Twist())

    def estop_tick(self):
        if self.estop_active:  # hold the robot still while the emergency stop is latched
            self.pub_cmd_vel.publish(Twist())
            # a goal accepted by Nav2 just after the e-stop (in flight when ABORT arrived) is cancelled too
            # (new hospital, measured T09: a recovery goal re-sent by the decision engine kept the robot at
            # 0.5 m/s against the zero-twist hold) -> cancel every 0.2 s and repeat ABORT every second
            now = time.time()
            if now - getattr(self, '_last_estop_cancel', 0.0) >= 0.2:
                self._last_estop_cancel = now
                if self.nav2_cancel.service_is_ready():
                    self.nav2_cancel.call_async(CancelGoal.Request())
            if now - getattr(self, '_last_estop_abort', 0.0) >= 1.0:
                self._last_estop_abort = now
                self.pub_ctrl.publish(String(data='ABORT'))

    # ------------------------------------------------------------------ periodic
    def tick(self):
        now = time.time()
        if self.state == 'IDLE' and all(self.ready_conditions().values()):
            self.set_state('READY', 'localization + decision engine + navigation available')
        if self.dwell_until and now >= self.dwell_until:
            self.dwell_until = None
            if self.state == 'COLLECTING':
                w = self.mcfg['marker_window_s']
                self.mission['collection_marker_seen'] = int(now - self.last_marker_seen.get('collection_point_marker', 0) <= w + self.mcfg['collect_dwell_s'])
                self.mission['payload_state'] = 'SECURED'
                self.set_state('PAYLOAD_SECURED', f"{self.mission['payload']} secured (collection marker seen: {bool(self.mission['collection_marker_seen'])})")
                self.start_leg(self.mission['destination'], 'GO_TO_LAB')
            elif self.state == 'DELIVERING':
                w = self.mcfg['marker_window_s']
                self.mission['lab_marker_seen'] = int(now - self.last_marker_seen.get('lab_test_point_marker', 0) <= w + self.mcfg['deliver_dwell_s'])
                self.mission['payload_state'] = 'DELIVERED'
                self.end_mission('COMPLETE', f"{self.mission['payload']} delivered")
        if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB') and self.mission and self.mission['legs']:
            leg = self.mission['legs'][-1]
            active = now - leg['start'] - sum(p for p in leg.get('_paused', []))
            if active > self.mcfg['leg_timeout_s']:
                self.pub_ctrl.publish(String(data='ABORT'))
                self.finish_leg('timeout')
                self.end_mission('FAILED', f"timeout: leg to {leg['target']} exceeded {self.mcfg['leg_timeout_s']:.0f} s")
        self.publish_state()

    def publish_state(self):
        m = self.mission
        st = {'state': self.state, 'decision_engine_state': self.de_state, 'estop_latched': self.estop_active,
              'ready_conditions': self.ready_conditions(), 'pose': self.pose, 'locations': self.locations,
              'events': self.events[-15:]}
        if m is not None:
            legs_done = sum(1 for leg in m['legs'] if leg['result'] == 'arrived')
            st['mission'] = {'mission_id': m['mission_id'], 'source': m['source'], 'destination': m['destination'],
                             'payload': m['payload'], 'payload_state': m['payload_state'],
                             'elapsed_s': round(time.time() - m['start_time'], 1) if not m.get('end_time') else round(m['end_time'] - m['start_time'], 1),
                             'progress': {'legs_completed': legs_done, 'legs_total': 2,
                                          'route_progress_pct': (self.de_status or {}).get('progress_pct')},
                             'distance_m': round(m['distance_m'], 2), 'replans_so_far': m['replans'] + int((self.de_status or {}).get('replans_count', 0) or 0) * (self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB')),
                             'ttc_events': m['ttc_events'], 'obstacle_encounters': len(m['_obstacle_ids']),
                             'current_route': (self.de_status or {}).get('active_path') if self.state in ('GO_TO_COLLECTION', 'GO_TO_LAB', 'PAUSED') else None,
                             'alternatives': [{k: r.get(k) for k in ('route_id', 'path', 'distance', 'total_cost', 'reliability_pct', 'estimated_time', 'is_selected')}
                                              for r in self.alt_routes],
                             'path': m['path'][-400:]}
        self.pub_state.publish(String(data=json.dumps(st)))


def main(args=None):
    rclpy.init(args=args)
    node = V3MissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
