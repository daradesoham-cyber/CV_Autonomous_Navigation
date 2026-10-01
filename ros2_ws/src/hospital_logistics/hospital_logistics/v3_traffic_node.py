"""
V3 hospital traffic: a staff member (Gazebo model `dynamic_person`) repeatedly walks across the east
bypass corridor that the collection_point <-> laboratory routes use.

Why: measured in Phase 8 (V3/docs/evidence/phase8/traffic_exposure.json) the V2.6 traffic driver left
the person motionless (VelocityControl on the 70 kg cylinder produced 0.0 m travel) and its patrols
are far from the V3 mission route, so "traffic" missions met no moving person. This node moves the
person kinematically (Gazebo set_pose, 10 Hz, ~0.4 m/s walking speed) on a crossing line, pausing at
each side. Like a person, it waits while the robot is within `yield_radius_m` of its next step, so it
never teleports into the robot; after 6 s of waiting it turns back (avoids a mutual-yield deadlock). Enabled with the same toggle as the V2.6 driver
(/simulation/enable_dynamic_obstacles, std_msgs/Bool), which the web UI publishes.
The ground-truth poses it reads are used only to avoid stepping into the robot.
"""
import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

WORLD = 'realistic_facility_world'


class V3TrafficNode(Node):
    def __init__(self):
        super().__init__('v3_traffic_node')
        self.declare_parameter('enabled', False)
        self.declare_parameter('model', 'dynamic_person')
        self.declare_parameter('line', [4.6, -0.5, 6.7, -0.5])  # x0, y0, x1, y1 (map/world frame)
        self.declare_parameter('speed_mps', 0.4)
        self.declare_parameter('pause_s', 4.0)
        self.declare_parameter('yield_radius_m', 1.2)
        self.declare_parameter('park', [-6.0, -6.5])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.enabled = bool(g('enabled'))
        self.model = g('model')
        x0, y0, x1, y1 = g('line')
        self.a, self.b = (x0, y0), (x1, y1)
        self.speed, self.pause, self.yield_r = float(g('speed_mps')), float(g('pause_s')), float(g('yield_radius_m'))
        self.park = tuple(g('park'))
        self.length = math.dist(self.a, self.b)
        self.s, self.dir, self.wait = 0.0, 1, 0.0
        self.robot = None
        self.lock = threading.Lock()
        self.yields = 0
        self.yield_time = 0.0
        from gz.msgs.boolean_pb2 import Boolean
        from gz.msgs.pose_pb2 import Pose
        from gz.msgs.pose_v_pb2 import Pose_V
        from gz.transport import Node as GzNode
        self.Pose, self.Boolean = Pose, Boolean
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self.on_poses)
        self.create_subscription(Bool, '/simulation/enable_dynamic_obstacles', self.on_enable, 10)
        self.dt = 0.1
        self.create_timer(self.dt, self.step)
        self.get_logger().info(f'V3 traffic: {self.model} crossing {self.a} <-> {self.b} at {self.speed} m/s '
                               f'(enabled={self.enabled})')

    def on_poses(self, m):
        for p in m.pose:
            if p.name == 'autonomous_robot':
                with self.lock:
                    self.robot = (p.position.x, p.position.y)
                return

    def on_enable(self, msg):
        if msg.data != self.enabled:
            self.enabled = msg.data
            self.get_logger().info(f'traffic {"enabled" if self.enabled else "disabled"}')
            if not self.enabled:
                self.set_pose(*self.park, 1.5708)
            else:
                self.s, self.dir, self.wait = 0.0, 1, 0.0

    def point(self, s):
        f = s / self.length
        return self.a[0] + f * (self.b[0] - self.a[0]), self.a[1] + f * (self.b[1] - self.a[1])

    def set_pose(self, x, y, yaw):
        req = self.Pose()
        req.name = self.model
        req.position.x, req.position.y, req.position.z = x, y, 0.85
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        try:
            self.gz.request(f'/world/{WORLD}/set_pose', req, self.Pose, self.Boolean, 200)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'set_pose failed: {e}', throttle_duration_sec=5.0)

    def step(self):
        if not self.enabled:
            return
        if self.wait > 0.0:
            self.wait -= self.dt
            return
        ns = min(self.length, max(0.0, self.s + self.dir * self.speed * self.dt))
        nx, ny = self.point(ns)
        with self.lock:
            robot = self.robot
        if robot is not None and math.dist(robot, self.point(self.s)) < 1.0:
            # robot closing in on a waiting person/trolley: step back along the line, away from the robot
            back = [min(self.length, max(0.0, self.s + d * self.speed * self.dt)) for d in (1, -1)]
            sb = max(back, key=lambda s: math.dist(robot, self.point(s)))
            if math.dist(robot, self.point(sb)) > math.dist(robot, self.point(self.s)):
                self.s = sb
                bx, by = self.point(sb)
                hd = math.atan2(self.b[1] - self.a[1], self.b[0] - self.a[0])
                self.set_pose(bx, by, hd)
            return
        if robot is not None and math.dist(robot, (nx, ny)) < self.yield_r:
            self.yields += 1
            self.yield_time += self.dt
            if self.yield_time > 6.0:  # robot waiting for us too (mutual yield): turn back and clear the way
                self.dir, self.yield_time = -self.dir, 0.0
            return  # give way to the robot
        self.yield_time = 0.0
        self.s = ns
        heading = math.atan2(self.b[1] - self.a[1], self.b[0] - self.a[0]) + (0.0 if self.dir > 0 else math.pi)
        self.set_pose(nx, ny, heading)
        if self.s in (0.0, self.length):
            self.dir = -self.dir
            self.wait = self.pause


def main(args=None):
    rclpy.init(args=args)
    node = V3TrafficNode()
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
