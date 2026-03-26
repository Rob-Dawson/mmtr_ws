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

##For testing
from std_msgs.msg import Float64MultiArray

from std_msgs.msg import Float64 



@dataclass
class ImuSample:
    t_ns: int
    jerk_mag: float
    jerk_x: float
    jerk_y: float
    jerk_z: float


class CollisionDetection(Node):
    def _declare_parameters(self):
        self.declare_parameter("debug", False)
        self.declare_parameter("collision_thresh_on")
        self.declare_parameter("collision_thresh_off")
        self.declare_parameter("refractory_ns")
        self.declare_parameter("delta_ns")
        self.declare_parameter("buffer_duration_ns")

    
    def _load_parameters(self):
        self.debug = self.get_parameter("debug").value
        self.collision_thresh_on = self.get_parameter("collision_thresh_on").value
        self.collision_thresh_off = self.get_parameter("collision_thresh_off").value
        self.refractory_ns = self.get_parameter("refractory_ns").value
        self.delta_ns = self.get_parameter("delta_ns").value
        self.buffer_duration_ns = self.get_parameter("buffer_duration_ns").value


    def _setup_pubs(self):
        self.collision_pub = self.create_publisher(
            CollisionEvent, "/collision/event", 100
        )
        self.stop = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)
        self.acce_mag = self.create_publisher(Float64, "accel_mag", 10)
        ##For testing
        if self.debug:
            self.inertia_snapshot_pub = self.create_publisher(Float64MultiArray, "/inertia_snapshot", 10)

    def _setup_subs(self):
        self.imu_sub = self.create_subscription(
            Vector3Stamped, "/jerk", self.jerk_buffer, 200
        )
        self.inertia_model = self.create_subscription(Imu, "/inertia_model", self.inertia_model_log, 10)

    def _init_state(self):
        self.imu_buffer = deque()
        self.inertia_buffer = deque()

        self.trigger = False

        ##The last time the IMU triggered a crash
        self.last_fire_ns = None
        self.direction_detected = False

    def __init__(self):
        super().__init__("Collision_Detection")
        self._declare_parameters()
        self._load_parameters()
        self._setup_pubs()
        self._setup_subs()
        self._init_state()

    
    def inertia_model_log(self, inertia_model: Imu):
        inertia_time = Time.from_msg(inertia_model.header.stamp).nanoseconds
        accel_x = inertia_model.linear_acceleration.x
        accel_y = inertia_model.linear_acceleration.y

        angular_vel_x = abs(inertia_model.angular_velocity.x)
        angular_vel_y = abs(inertia_model.angular_velocity.y)
        angular_vel_z = abs(inertia_model.angular_velocity.z)

        self.inertia_buffer.append((inertia_time, accel_x, accel_y, angular_vel_x, angular_vel_y, angular_vel_z))
        while self.inertia_buffer and (inertia_time - self.inertia_buffer[0][0]) >= self.buffer_duration_ns:
            self.inertia_buffer.popleft()
        
        if self.direction_detected:
            return
        
        if self.trigger is True and self.inertia_buffer[-1][0] > (self.last_fire_ns + 40_000_000):
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
            imu_time - self.imu_buffer[0].t_ns >= self.buffer_duration_ns
        ):
            self.imu_buffer.popleft()

        if self.last_fire_ns is not None:
            if (imu_time - self.last_fire_ns) < self.refractory_ns:
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
        # self.get_logger().info(f"Entire Accel Buffer: {self.inertia_buffer}")


        for candidate in self.inertia_buffer:
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
        Ix_base = Iy_base = 0

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

        peak_roll = max(crash_window, key=lambda roll: roll[3])[3]
        peak_pitch = max(crash_window, key=lambda pitch: pitch[4])[4]
        peak_yaw = max(crash_window, key=lambda yaw: yaw[5])[5]

        
        
        for candidate in crash_window[1:]:
            corrected_X = candidate[1] - Ix_base
            corrected_Y = candidate[2] - Iy_base

            Ix += corrected_X
            Iy += corrected_Y

        self.get_logger().info(f"Ix: {Ix}")
        self.get_logger().info(f"Iy: {Iy}")

        min_peak_threshold = 0.08
        min_yaw_threshold = 0.008
        min_Ix_threshold = 0.05
        min_Iy_threshold = 0.02

        if peak_pitch < min_peak_threshold and peak_yaw < min_yaw_threshold:
            return
        

        front_rear_total = 0
        left_right_total = 0

        if abs(Ix) > 0.5:
            front_rear_total += 1
            if abs(peak_yaw) < 0.2:
                front_rear_total += 2
                if abs(peak_pitch) > 0.2:
                    front_rear_total += 2
                    if abs(Ix) > abs(Iy):
                        front_rear_total += 2
        
        if abs(Iy) > 0.5:
            left_right_total +=1
            if abs(peak_yaw)>0.2:
                left_right_total +=2
                if abs(peak_pitch) < 0.2:
                    left_right_total +=1
                    if abs(Iy) > abs(Ix):
                        left_right_total+= 2


            self.get_logger().info(f"Ix: {Ix}")
            self.get_logger().info(f"Iy: {Iy}")
            self.get_logger().info(f"Peak Pitch: {peak_pitch}")
            self.get_logger().info(f"Peak Yaw: {peak_yaw}")
            self.get_logger().info(f"Peak Roll: {peak_roll}")
            self.get_logger().info(f"Left/right Total: {left_right_total}")
            self.get_logger().info(f"Front/Rear Total: {front_rear_total}")
            

            ##For testing
            inertia_snapshot = Float64MultiArray()
            inertia_snapshot.data = []
            inertia_snapshot.data.append(crash_time)
            inertia_snapshot.data.append(Ix)
            inertia_snapshot.data.append(Iy)
            inertia_snapshot.data.append(peak_roll)
            inertia_snapshot.data.append(peak_pitch)
            inertia_snapshot.data.append(peak_yaw)

            self.inertia_snapshot_pub.publish(inertia_snapshot)



        if front_rear_total > left_right_total:
            if Ix >= 0:
                return CollisionEvent.FRONT
            return CollisionEvent.REAR
        else:
            if Iy >= 0:
                return CollisionEvent.RIGHT
            return CollisionEvent.LEFT




        # self.get_logger().info(f"Ex = Ey * 1.5 = {1.5 * Ey}")
        
        # if Ex > 1.5 * Ey:
        #     self.get_logger().info("Front or rear")
        #     if Ix > 0:
        #         self.get_logger().info("Contact rear")
        #     elif Ix < 0:
        #         self.get_logger().info("Contact Front")
                
        # elif Ey > 1.5 * Ex:
        #     self.get_logger().info("Left or right")
            


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
