#!/usr/bin/env python3

## Very simple ZUPT just using commanded and actual velocity from encoders and cmd_vel

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, TwistStamped
import numpy as np

class Zupt(Node):
    def __init__(self):
        super().__init__("ZUPT")

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        self.zupt_pub = self.create_publisher(
            TwistWithCovarianceStamped, "zupt/twist", 10
        )

        self.create_timer(0.01, self.tick)

        self.actual_wheel_speed = [0]*2
        self.commanded_wheel_speed_linear = 0.0
        self.commanded_wheel_speed_angular = 0.0

        self.counter = 0

    def joint_state_cb(self, speed:JointState):
        # print(speed.velocity)
        self.actual_wheel_speed[0] = speed.velocity[0]
        self.actual_wheel_speed[1] = speed.velocity[1]


    def cmd_vel_callback(self, cmd:TwistStamped):
        self.commanded_wheel_speed_linear = cmd.twist.linear.x
        self.commanded_wheel_speed_angular = cmd.twist.angular.z

    def tick(self):
        if (self.actual_wheel_speed[0] == 0.0 and self.actual_wheel_speed[1] == 0.0) and (self.commanded_wheel_speed_linear == 0.0 and self.commanded_wheel_speed_angular == 0.0):
            self.counter = self.counter + 1
            if self.counter >= 10:
                self.publish_to_zero()
                self.get_logger().info("Stationary", once=True)
                self.counter = 0

    def publish_to_zero(self):
        msg = TwistWithCovarianceStamped()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        covariance = [0.0] * 36
        diag = [1e-3, 1e-3, 1e-2, 1e-3, 1e-3, 3e-4]
        for i, v in enumerate(diag):
            covariance[i * 6 + i] = v
        msg.twist.covariance = covariance
        self.zupt_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Zupt()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
