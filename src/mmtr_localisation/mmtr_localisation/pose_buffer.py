#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry

@dataclass(frozen=True)
class pose_sample:
    time_ns: int
    x: float
    y: float
    yaw: float
    vel_x: float
    angular_z: float

class pose_buffer(Node):
    def __init__(self):
        super().__init__("Pose_Buffer")

        self.odom_pose_sub = self.create_subscription(Odometry, "/odometry/filtered_ukf", self.odom_pose_cb, 150)
        # self.ukf_ready = self.create_subscription(DiagnosticArray, "/diagnostics", self.diagnostic_cb, 100)
        
        self.pose_buffer = deque()
        self.ukf_ready = False

    def odom_pose_cb(self, msg):
        if not self.ukf_ready:
            if msg.header.stamp:
                self.ukf_ready = True
            return
        else:
            odom_time = Time.from_msg(msg.header.stamp).nanoseconds
            pose_x = msg.pose.pose.position.x
            pose_y = msg.pose.pose.position.y
            orient_z = msg.pose.pose.orientation.z
            vel_x = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z


            s = pose_sample(time_ns=odom_time,x=pose_x, y=pose_y, yaw=orient_z, vel_x=vel_x, angular_z=angular_vel)
            self.pose_buffer.append(s)

            while self.pose_buffer and (odom_time - self.pose_buffer[0].time_ns) >= int(2.0 * 1e9):
                self.pose_buffer.popleft()
        self.get_logger().info(f"{self.pose_buffer}")

        ##You have the odom_pose now time to build the actual queue

def main(args=None):
    rclpy.init(args=args)
    node = pose_buffer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()