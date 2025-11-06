#!/usr/bin/env python3

import math
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry

from mmtr_msg.msg import CollisionEvent

def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass
class PoseSample:
    t_ns: int
    pose_x: float
    pose_y: float
    yaw_orient: float
    vel_x: float
    angular_vel_z: float


class RewindOdom(Node):
    def __init__(self):
        super().__init__("Rewin_Odom")

        self.pose_buffer = deque()
        self.pose_sub = self.create_subscription(
            Odometry, "/odometry/filtered_ukf", self.pose_cb, 100
        )
        self.collision_detection_sub = self.create_subscription(
            CollisionEvent, "/collision/event", self.collision_cb, 10
        )

        self.ukf_ready = False
        self.collision_event_time_ns = None

    def pose_cb(self, msg: Odometry):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return

        pose_time = Time.from_msg(msg.header.stamp).nanoseconds
        yaw = quat_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        s = PoseSample(
            t_ns=pose_time,
            pose_x=msg.pose.pose.position.x,
            pose_y=msg.pose.pose.position.y,
            yaw_orient=yaw,
            vel_x=msg.twist.twist.linear.x,
            angular_vel_z=msg.twist.twist.angular.z,
        )

        #Building the pose buffer
        self.pose_buffer.append(s)
        #Pruning any pose after 2 seconds base on the pose time
        while self.pose_buffer and (pose_time - self.pose_buffer[0].t_ns >= int(2.0 * 1e9)):
            self.pose_buffer.popleft()

    def collision_cb(self, msg:CollisionEvent):
        self.collision_event_time_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.Rewind(self.collision_event_time_ns)

    def Rewind(self, collision_event_time_ns):
        before_t_event = None
        after_t_event  = None

        for s in self.pose_buffer:
            if s.t_ns <= collision_event_time_ns:
                before_t_event = s
            elif s.t_ns >= collision_event_time_ns:
                after_t_event = s
                break
        if before_t_event is not None and after_t_event is not None:
            if before_t_event.t_ns == after_t_event.t_ns:
                pass
                #ukf set_pose
            else:
                pass
                #interpolate

            
        ## Here is where the rewind will happen
        ## It takes the collision_event_time_ns and using the pose buffer,
        # interpolates where the robot should be at the given time
        pass


def main():
    rclpy.init()
    node = RewindOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()