#!/usr/bin/env python3

## This node subscribes to the Jerk calc node and if it detects a jerk over 200 ish, mark as collision
##

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float32
from mmtr_msg.msg import CollisionEvent


class collision_detection(Node):
    def __init__(self):
        super().__init__("Collision_Detection")

        self.collision_pub = self.create_publisher(
            CollisionEvent, "/collision/event", 100
        )

        self.jerk_sub = self.create_subscription(Float32, "/dbg/jerk_corr_mag", self.jerk_buffer, 10)
        self.collision_thresh = 200.0

    def imu_cb(self, msg: Imu):
        pass

    def jerk_buffer(self, msg: Float32):
        if msg.data > self.collision_thresh:
            print("Collision")
            self.collision_detected()

    def collision_detected(self):
        # stop robot
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        # self.emergency_stop_pub.publish(t)

        # event message with proper ROS time
        evt = CollisionEvent()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.header.frame_id = "base_link"
        evt.hit = True
        evt.side = CollisionEvent.FRONT
        evt.confidence = 1.0
        self.collision_pub.publish(evt)


def main(args=None):
    rclpy.init(args=args)
    node = collision_detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
