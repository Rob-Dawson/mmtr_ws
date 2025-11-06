#!/usr/bin/env python3

#TODO
# Currently the node follows the pattern Trigger -> Stop -> Publish Collision Event
# This works for hard crashes but will fail with bumps and other things where
# the robot can actually navigate over the obstacle. Even ramps could cause issues

# An improvement would be Trigger -> Analyse data -> Decide
# Where decide will either be "This was a bump, no need to stop"
# or "This was a wall, stop and publish collision event"


from collections import deque
from dataclasses import dataclass

import math
import rclpy

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped
from mmtr_msg.msg import CollisionEvent


@dataclass
class ImuSample:
    t_ns: int
    jerk_mag: float
    jerk_x: float
    jerk_y: float
    jerk_z: float


class CollisionDetection(Node):
    def __init__(self):
        super().__init__("Collision_Detection")

        self.collision_pub = self.create_publisher(
            CollisionEvent, "/collision/event", 100
        )

        self.imu_sub = self.create_subscription(
            Vector3Stamped, "/jerk", self.jerk_buffer, 200
        )
        self.stop = self.create_publisher(Twist, "cmd_vel", 10)
        self.imu_buffer = deque()

        ##Schmitt Trigger
        self.collision_thresh_on = 200.0
        self.collision_thresh_off = 100.0
        self.trigger = False

        self.last_fire_ns = None
        # This is acting like a little compensation tag for any
        # small delay there is in the imu sampling
        # It can stay as zero for now and manually move up if necessary
        # Maybe use gazebo contact to find the precise time the robot hit
        # And then compensate for any delay
        # t_event_ns - t_contact_true
        self.delta_ns = 0
        self.refactory_ns = 200_000_000

    def imu_cb(self, msg: Imu):
        pass

    def jerk_buffer(self, msg: Vector3Stamped):
        imu_time = Time.from_msg(msg.header.stamp).nanoseconds
        jerk_x = msg.vector.x
        jerk_y = msg.vector.y
        jerk_z = msg.vector.z

        jerk_mag = math.sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z)
        s = ImuSample(
            t_ns=imu_time,
            jerk_mag=jerk_mag,
            jerk_x=jerk_x,
            jerk_y=jerk_y,
            jerk_z=jerk_z,
        )

        self.imu_buffer.append(s)
        while self.imu_buffer and (
            imu_time - self.imu_buffer[0].t_ns >= int(2.0 * 1e9)
        ):
            self.imu_buffer.popleft()

        if self.last_fire_ns is not None:
            if (imu_time - self.last_fire_ns) < self.refactory_ns:
                return

        if not self.trigger:
            if jerk_mag >= self.collision_thresh_on:
                self.trigger = True
                print("Collision")
                self.on_rising_edge(imu_time)
        else:
            if jerk_mag <= self.collision_thresh_off:
                self.trigger = False

    def on_rising_edge(self, imu_time):
        onset = self.find_onset()
        if onset is not None:
            t_event_ns = max(0, onset.t_ns - self.delta_ns)
        else:
            t_event_ns = imu_time
        self.handle_collision(t_event_ns)

    def find_onset(self):
        if not self.imu_buffer:
            return None
        idx = len(self.imu_buffer) - 1

        while idx >= 0 and self.imu_buffer[idx].jerk_mag >= self.collision_thresh_off:
            idx -= 1

        onset_idx = idx + 1
        if onset_idx < len(self.imu_buffer):
            return self.imu_buffer[onset_idx]
        return None

    def handle_collision(self, t_event_ns):
        # stop robot
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.stop.publish(t)

        self.last_fire_ns = t_event_ns

        # event message with proper ROS time
        evt = CollisionEvent()
        evt.header.stamp = Time(nanoseconds=t_event_ns).to_msg()
        evt.header.frame_id = "base_link"
        evt.hit = True
        evt.side = CollisionEvent.FRONT
        evt.confidence = 1.0
        self.collision_pub.publish(evt)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
