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

from std_msgs.msg import Float64 
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
        self.stop = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)
        self.acce_mag = self.create_publisher(Float64, "accel_mag", 10)
        self.accel_without_gravity = self.create_subscription(Vector3Stamped, "/accel_without_grav", self.accel_log, 10)

        self.imu_buffer = deque()
        self.accel_buffer = deque()
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

        self.direction_detected = False

    def accel_cb(self, accel: Vector3Stamped):
        accel_mag_float = Float64()
        x = accel.vector.x
        y = accel.vector.y
        accel_mag = math.sqrt(x * x + y * y)
        accel_mag_float.data = accel_mag
        self.acce_mag.publish(accel_mag_float)

    def imu_cb(self, msg: Imu):
        pass
    
    def accel_log(self, accel: Vector3Stamped):
        accel_time = Time.from_msg(accel.header.stamp).nanoseconds
        x = accel.vector.x
        y = accel.vector.y
        self.accel_buffer.append((accel_time,x,y))
        while self.accel_buffer and (accel_time - self.accel_buffer[0][0]) >= int(2.0*1e9):
            self.accel_buffer.popleft()
        
        if self.direction_detected:
            return 
        if self.trigger is True and self.accel_buffer[-1][0] > (self.last_fire_ns + 40_000_000):
           normal_window, crash_window = self.windows(self.last_fire_ns)
           if crash_window == []:
               return
           else:
                self.detect_direction(self.last_fire_ns, normal_window, crash_window)
                self.direction_detected = True  


        

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
                self.direction_detected = False

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
    
    def windows(self, crash_time):
        self.get_logger().info(f"Crashed at {crash_time}")
        normal_window = []
        crash_window = []
        # self.get_logger().info(f"Entire Accel Buffer: {self.accel_buffer}")


        for candidate in self.accel_buffer:
            if crash_time-40_000_000 <= candidate[0] < crash_time:
                normal_window.append(candidate)
            elif crash_time <= candidate[0] <= crash_time + 60_000_000:
                crash_window.append(candidate)

        return normal_window, crash_window

    def detect_direction(self, crash_time, normal_window, crash_window):
        self.get_logger().info(f"Detect: ")


        self.get_logger().info(f"Normal Window: {normal_window}")
        self.get_logger().info(f"Crash Window: {crash_window}")


        post_crash = 20_000_000
        stop_time = crash_time + post_crash
        
        Ix = Iy = Ex = Ey = 0
        Ix_base = Iy_base = Ex_base = Ey_base = 0

        for candidate in normal_window:
            ## Impulse Ix, Iy: net acceleration impulse
            ## Did the robot’s velocity change forward or backward during the impact?
            ## Ix + = Forward Ix - = Backward
            Ix_base += candidate[1]
            Iy_base += candidate[2]
            ## Energy Ex, Ey: how strong the motion was in each axis
            ## Was this crash mostly forward/back or mostly sideways?

        Ix_base = Ix_base / len(normal_window)
        Iy_base = Iy_base / len(normal_window)

        
        for candidate in crash_window[1:]:
            corrected_X = candidate[1] - Ix_base
            corrected_Y = candidate[2] - Iy_base

            Ix += corrected_X
            Iy += corrected_Y
            Ex += corrected_X * corrected_X
            Ey += corrected_Y * corrected_Y
            self.get_logger().info(f"Corrected_x: {corrected_X}")
            self.get_logger().info(f"Corrected_y: {corrected_Y}")


            
        self.get_logger().info(f"Ix_base: {Ix_base}")
        self.get_logger().info(f"Iy_base: {Iy_base}")


        self.get_logger().info(f"Ex: {Ex}")
        self.get_logger().info(f"Ey: {Ey}")
        self.get_logger().info(f"Ix: {Ix}")
        self.get_logger().info(f"Iy: {Iy}")





        self.get_logger().info(f"Ex = Ey * 1.5 = {1.5 * Ey}")
        
        if Ex > 1.5 * Ey:
            self.get_logger().info("Front or rear")
            if Ix > 0:
                self.get_logger().info("Contact rear")
            elif Ix < 0:
                self.get_logger().info("Contact Front")
                
        elif Ey > 1.5 * Ex:
            self.get_logger().info("Left or right")
            


    def handle_collision(self, t_event_ns):
        # stop robot
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.stop.publish(t)
        self.last_fire_ns = t_event_ns

        # event message with proper ROS time
        event = Time(nanoseconds=t_event_ns).to_msg()
        # self.get_logger().info(f"{event}")
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
