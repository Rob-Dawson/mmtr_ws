#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleTeleop(Node):

    def __init__(self):
        super().__init__('circle_teleop')
        self.pub = self.create_publisher(Twist, '/model/mmtr/cmd_vel', 10)

        self.declare_parameter('radius', 1.0)
        self.declare_parameter('period', 20.0)
        self.declare_parameter('linear_speed', None)

        radius = self.get_parameter('radius').value
        period = self.get_parameter('period').value
        lin_override = self.get_parameter('linear_speed').value

        if lin_override is not None:
            self.v = lin_override
        else:
            self.v = 2 * math.pi * radius / period
        self.omega = self.v / radius

        timer_period = 0.05
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = Twist()
        print("Linear Vel = ", self.v)
        print("Angular Vel = ", self.omega)

        twist.linear.x = self.v
        twist.angular.z = self.omega
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CircleTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
