#!/usr/bin/env python3
import math
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Vector3Stamped

@dataclass
class imu_sample:
    t_ns: int
    jerk_mag: float
    jerk_x: float
    jerk_y: float
    jerk_z: float


class imu_buffer(Node):
    def __init__(self):
        super().__init__("imu_buffer")

        self.imu_sub = self.create_subscription(
            Vector3Stamped, "/jerk", self.imu_cb, 200
        )


        self.imu_buffer = deque()

    def imu_cb(self, msg: Vector3Stamped):
        imu_time = Time.from_msg(msg.header.stamp).nanoseconds
        jerk_x = msg.vector.x
        jerk_y = msg.vector.y
        jerk_z = msg.vector.z

        jerk_mag = math.sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z)
        s = imu_sample(
            t_ns=imu_time,
            jerk_mag=jerk_mag,
            jerk_x=jerk_x,
            jerk_y=jerk_y,
            jerk_z=jerk_z,
        )
        self.imu_buffer.append(s)
        while self.imu_buffer and (imu_time - self.imu_buffer[0].t_ns) > int(2.0 * 1e9):
            self.imu_buffer.popleft()

        


def main():
    rclpy.init()
    node = imu_buffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
