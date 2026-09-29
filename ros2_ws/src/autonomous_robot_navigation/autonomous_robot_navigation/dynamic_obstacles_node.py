#!/usr/bin/env python3
"""
Dynamic Obstacles Controller Node.
Controls moving physical obstacles in Gazebo Sim (Autonomous Cart, Hospital Trolley,
Forklift, and Walking Personnel) along realistic patrol trajectories via cmd_vel topics.
"""

import sys
import math
import time

venv_site = '/home/soham-darade/CV_Autonomous_Navigation/.venv/lib/python3.14/site-packages'
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class DynamicObstaclesNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles_node')

        self.declare_parameter('enable_motion', True)
        self.declare_parameter('update_rate_hz', 10.0)

        # Publishers for each obstacle cmd_vel
        self.pub_cart = self.create_publisher(Twist, '/cart/cmd_vel', 10)
        self.pub_trolley = self.create_publisher(Twist, '/trolley/cmd_vel', 10)
        self.pub_forklift = self.create_publisher(Twist, '/forklift/cmd_vel', 10)
        self.pub_person = self.create_publisher(Twist, '/person/cmd_vel', 10)

        # Control subscription
        self.sub_enable = self.create_subscription(
            Bool,
            '/simulation/enable_dynamic_obstacles',
            self._enable_callback,
            10
        )

        self.enabled = self.get_parameter('enable_motion').get_parameter_value().bool_value
        rate = self.get_parameter('update_rate_hz').get_parameter_value().double_value

        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / rate, self._control_loop)
        self.get_logger().info("DynamicObstaclesNode active: controlling 4 Gazebo dynamic obstacles.")

    def _enable_callback(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f"Dynamic obstacles motion set to: {self.enabled}")

    def _control_loop(self):
        if not self.enabled:
            # Publish zero velocities
            stop_twist = Twist()
            self.pub_cart.publish(stop_twist)
            self.pub_trolley.publish(stop_twist)
            self.pub_forklift.publish(stop_twist)
            self.pub_person.publish(stop_twist)
            return

        t = time.time() - self.start_time

        # 1. Autonomous Cart: 12-second back-and-forth patrol cycle along Y axis
        # Forward for 5s, pause 1s, reverse 5s, pause 1s
        cart_cycle = t % 12.0
        cart_twist = Twist()
        if cart_cycle < 5.0:
            cart_twist.linear.x = 0.35
        elif cart_cycle < 6.0:
            cart_twist.linear.x = 0.0
        elif cart_cycle < 11.0:
            cart_twist.linear.x = -0.35
        else:
            cart_twist.linear.x = 0.0
        self.pub_cart.publish(cart_twist)

        # 2. Hospital Trolley: 10-second cycle along South-East corridor
        trolley_cycle = t % 10.0
        trolley_twist = Twist()
        if trolley_cycle < 4.0:
            trolley_twist.linear.x = 0.25
        elif trolley_cycle < 5.0:
            trolley_twist.linear.x = 0.0
        elif trolley_cycle < 9.0:
            trolley_twist.linear.x = -0.25
        else:
            trolley_twist.linear.x = 0.0
        self.pub_trolley.publish(trolley_twist)

        # 3. Autonomous Forklift: 14-second loading bay patrol
        forklift_cycle = t % 14.0
        forklift_twist = Twist()
        if forklift_cycle < 6.0:
            forklift_twist.linear.x = 0.22
        elif forklift_cycle < 7.0:
            forklift_twist.linear.x = 0.0
        elif forklift_cycle < 13.0:
            forklift_twist.linear.x = -0.22
        else:
            forklift_twist.linear.x = 0.0
        self.pub_forklift.publish(forklift_twist)

        # 4. Walking Personnel: 16-second cycle with slight sinusoidal variation
        person_cycle = t % 16.0
        person_twist = Twist()
        if person_cycle < 7.0:
            person_twist.linear.x = 0.30 + 0.05 * math.sin(t * 2.0)
        elif person_cycle < 8.0:
            person_twist.linear.x = 0.0
        elif person_cycle < 15.0:
            person_twist.linear.x = -(0.30 + 0.05 * math.sin(t * 2.0))
        else:
            person_twist.linear.x = 0.0
        self.pub_person.publish(person_twist)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
