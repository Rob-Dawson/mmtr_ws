#!/usr/bin/env python3

#TODO
# Currently the node follows the pattern Trigger -> Stop -> Publish Collision Event
# This works for hard crashes but will fail with bumps and other things where
# the robot can actually navigate over the obstacle. Even ramps could cause issues

# An improvement would be Trigger -> Analyse data -> Decide
# Where decide will either be "This was a bump, no need to stop"
# or "This was a wall, stop and publish collision event"

from collections import deque
from enum import Enum, auto
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3Stamped
from mmtr_msg.msg import CollisionEvent
from mmtr_collision_detection.config import collision_config
##For testing
from std_msgs.msg import Float64MultiArray

from std_msgs.msg import Float64 


class State(Enum):
    IDLE = auto()
    CANDIDATE = auto()

class CollisionDetection(Node):
    
    def _declare_parameters(self):
        self.declare_parameter("debug", False)
        self.declare_parameter("collision_thresh_on")
        self.declare_parameter("collision_thresh_off")
        self.declare_parameter("refractory_ns")
        self.declare_parameter("delta_ns")
        self.declare_parameter("buffer_duration_ns")
        self.declare_parameter("min_peak_threshold")
        self.declare_parameter("yaw_val")
        self.declare_parameter("pitch_val")
        self.declare_parameter("Ix_val")
        self.declare_parameter("Iy_val")
        self.cfg = collision_config.Config

        self.declare_parameter("front_rear_yaw_weight")
        self.declare_parameter("front_rear_pitch_weight")
        self.declare_parameter("front_rear_Ix_weight")
        self.declare_parameter("front_rear_Ix_greater_weight")

        self.declare_parameter("left_right_yaw_weight")
        self.declare_parameter("left_right_pitch_weight")
        self.declare_parameter("left_right_Iy_weight")

    def _load_parameters(self):
        self.debug = self.get_parameter("debug").value
        self.collision_thresh_on = self.get_parameter("collision_thresh_on").value
        self.collision_thresh_off = self.get_parameter("collision_thresh_off").value
        self.refractory_ns = self.get_parameter("refractory_ns").value
        self.delta_ns = self.get_parameter("delta_ns").value
        self.buffer_duration_ns = self.get_parameter("buffer_duration_ns").value
        
        self.min_peak_threshold = self.get_parameter("min_peak_threshold").value
        self.min_yaw_threshold = self.get_parameter("min_yaw_threshold").value
        
        self.cfg.thresholds.yaw    = self.get_parameter("yaw_val").value
        self.cfg.thresholds.pitch   = self.get_parameter("pitch_val").value
        self.cfg.thresholds.ix     = self.get_parameter("Ix_val").value
        self.cfg.thresholds.iy     = self.get_parameter("Iy_val").value

        self.cfg.weights.front_rear.yaw = self.get_parameter("front_rear_yaw_weight").value
        self.cfg.weights.front_rear.pitch = self.get_parameter("front_rear_pitch_weight").value
        self.cfg.weights.front_rear.axis = self.get_parameter("front_rear_axis").value
        self.cfg.weights.front_rear.dominant_axis = self.get_parameter("front_rear_dominant_axis").value
        
        self.cfg.weights.left_right.axis = self.get_parameter("left_right_axis_weight").value
        self.cfg.weights.left_right.yaw = self.get_parameter("left_right_yaw_weight").value
        self.cfg.weights.left_right.pitch = self.get_parameter("left_right_pitch_weight").value
        self.cfg.weights.left_right.dominant_axis = self.get_parameter("left_right_dominant_axis").value

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
        self.last_trigger_time_ns = None
        self.reset_candidate()

    def reset_candidate(self):
        self.state = State.IDLE
        self.candidate_event_time = None

    def __init__(self):
        super().__init__("Collision_Detection")
        self._declare_parameters()
        self._load_parameters()
        self._setup_pubs()
        self._setup_subs()
        self._init_state()


    def jerk_buffer(self, msg: Vector3Stamped):
        imu_time = Time.from_msg(msg.header.stamp).nanoseconds
        jerk_x = msg.vector.x
        jerk_y = msg.vector.y
        jerk_z = msg.vector.z
        jerk_mag = math.sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z)
        s = collision_config.ImuSample(
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
        
        if self.last_trigger_time_ns is not None:
            if imu_time < self.last_trigger_time_ns + self.refractory_ns:
                return

        if self.state == State.IDLE and jerk_mag >= self.collision_thresh_on:
            self.state = State.CANDIDATE
            self.last_trigger_time_ns = imu_time
            self.on_rising_edge(imu_time)

    def on_rising_edge(self, imu_time):
        onset = self.find_onset()
        if onset is not None:
            self.candidate_event_time = max(0, onset.t_ns - self.delta_ns)
        else:
            self.candidate_event_time = imu_time


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

        if self.state is not State.CANDIDATE:
            return
        
        if self.inertia_buffer[-1][0] > (self.candidate_event_time + 40_000_000):
           normal_window, crash_window = self.windows(self.candidate_event_time)
           if not crash_window:
                self.reset_candidate()
                return
           else:
                direction = self.detect_direction(self.candidate_event_time, normal_window, crash_window)
                if direction is CollisionEvent.UNKNOWN:
                    self.reset_candidate()
                    return None
                else:
                    self.handle_collision(direction)
                    self.reset_candidate()

    
    def windows(self, crash_time):
        self.get_logger().debug(f"Crashed at {crash_time}")
        normal_window = []
        crash_window = []

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

        
        Ix = Iy = 0
        Ix_base = Iy_base = 0

        for candidate in normal_window:
            ## Impulse Ix, Iy: net acceleration impulse
            ## Did the robot’s velocity change forward or backward during the impact?
            ## Ix + = Forward Ix - = Backward
            Ix_base += candidate[1]
            Iy_base += candidate[2]

        Ix_base = Ix_base / len(normal_window)
        Iy_base = Iy_base / len(normal_window)

        peak_roll = max(crash_window, key=lambda roll: roll[3])[3]
        peak_pitch = max(crash_window, key=lambda pitch: pitch[4])[4]
        peak_yaw = max(crash_window, key=lambda yaw: yaw[5])[5]
       
        for candidate in crash_window:
            corrected_X = candidate[1] - Ix_base
            corrected_Y = candidate[2] - Iy_base

            Ix += corrected_X
            Iy += corrected_Y

        if peak_pitch < self.min_peak_threshold and peak_yaw < self.min_yaw_threshold:
            self.state = State.IDLE
            return None
        

        front_rear_total = 0
        left_right_total = 0

        if abs(Ix) > self.cfg.thresholds.ix:
            front_rear_total += self.cfg.weights.front_rear.axis
            
            if abs(peak_yaw) < self.cfg.thresholds.yaw:
                front_rear_total += self.cfg.weights.front_rear.yaw
                
                if abs(peak_pitch) > self.cfg.thresholds.pitch:
                    front_rear_total += self.cfg.weights.front_rear.pitch
                    
                    if abs(Ix) > abs(Iy):
                        front_rear_total += self.cfg.weights.front_rear.dominant_axis
        
        if abs(Iy) > self.cfg.thresholds.iy:
            left_right_total += self.cfg.weights.left_right.axis
            
            if abs(peak_yaw) > self.cfg.thresholds.yaw:
                left_right_total += self.cfg.weights.left_right.yaw
            
                if abs(peak_pitch) < self.cfg.thresholds.pitch:
                    left_right_total += self.cfg.weights.left_right.pitch
            
                    if abs(Iy) > abs(Ix):
                        left_right_total += self.cfg.weights.left_right.dominant_axis


        if self.debug:
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
        elif left_right_total > front_rear_total:
            if Iy >= 0:
                return CollisionEvent.RIGHT
            return CollisionEvent.LEFT
        else:
            return CollisionEvent.UNKNOWN
        
    def handle_collision(self, direction):
        # stop robot
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.stop.publish(t)

        # event message with proper ROS time
        # self.get_logger().info(f"{event}")
        evt = CollisionEvent()
        evt.header.stamp = Time(nanoseconds=self.candidate_event_time).to_msg()
        evt.header.frame_id = "base_link"
        evt.hit = True
        evt.side = direction
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
