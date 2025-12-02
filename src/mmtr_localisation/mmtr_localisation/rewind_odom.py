#!/usr/bin/env python3

import math
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from robot_localization.srv import SetPose, SetPose_Request, SetPose_Response
from mmtr_msg.msg import CollisionEvent
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

def wrap_to_pi(angle):
    wrapped = math.fmod(angle, 2 * math.pi)
    if wrapped > math.pi:
        return wrapped - 2 * math.pi
    elif wrapped <= -math.pi:
        return wrapped + 2 * math.pi
    else:
        return wrapped

def covariance_from_diagonal(diag):
    cov = [0.0] * 36
    for i, val in enumerate(diag):
        cov[i * 6 + i] = float(val)
    return cov

def quat_to_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def yaw_to_quat(yaw):
    scalar = math.cos(yaw/2)
    x = 0
    y = 0
    z = math.sin(yaw/2)
    return (x, y, z, scalar)


@dataclass
class PoseSample:
    t_ns: int
    pose_x: float
    pose_y: float
    pose_z: float
    yaw_orient: float
    vel_x: float
    angular_vel_z: float


class RewindOdom(Node):
    def __init__(self):
        super().__init__("Rewind_Odom")

        self.pose_buffer = deque()
        self.pose_sub = self.create_subscription(
            Odometry, "/odometry/filtered_ukf", self.pose_cb, 100
        )
        self.collision_detection_sub = self.create_subscription(
            CollisionEvent, "/collision/event", self.collision_cb, 10
        )
        self.zero_twist_pub = self.create_publisher(TwistWithCovarianceStamped, "rewind/zero_twist", 100)

        self.pending_buffer = False
        self.ukf_ready = False
        self.collision_event_time_ns = None

        self.ukf_set = self.create_client(SetPose, '/set_pose')
        self.pose_cov_diag  = [0.001, 0.001, 1e6, 1e6, 1e6, 0.001]
        self.twist_cov_diag = [1e-12, 1e6, 1e6, 1e6, 1e6, 1e-12]

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
            pose_z=msg.pose.pose.position.z,
            yaw_orient=yaw,
            vel_x=msg.twist.twist.linear.x,
            angular_vel_z=msg.twist.twist.angular.z,
        )

        #Building the pose buffer
        self.pose_buffer.append(s)
        # self.get_logger().info(f"{self.pose_buffer}")
        #Pruning any pose after 2 seconds base on the pose time
        while self.pose_buffer and (pose_time - self.pose_buffer[0].t_ns >= int(2.0 * 1e9)):
            self.pose_buffer.popleft()

        if self.pending_buffer and self.pose_buffer[-1].t_ns >= self.collision_event_time_ns:
            self.get_logger().info("Pending buffer")
            self.pending_buffer = False
            self.rewind(self.collision_event_time_ns)         


    def collision_cb(self, msg:CollisionEvent):
        self.get_logger().info("Collision")
        self.collision_event_time_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.pending_buffer = True
        # if not self.pending_buffer:
        #     self.rewind(self.collision_event_time_ns)

    def set_ukf_pose(self, candidate_event: PoseSample):
        new_twist_stamped = TwistWithCovarianceStamped()
        new_twist_stamped.header.frame_id = "base_footprint"
        new_twist_stamped.header.stamp = self.get_clock().now().to_msg()
        new_twist_stamped.twist.twist.linear.x = 0.0
        new_twist_stamped.twist.twist.linear.y = 0.0
        new_twist_stamped.twist.twist.linear.z = 0.0
        new_twist_stamped.twist.twist.angular.x = 0.0
        new_twist_stamped.twist.twist.angular.y = 0.0
        new_twist_stamped.twist.twist.angular.z = 0.0
        new_twist_stamped.twist.covariance = covariance_from_diagonal(self.twist_cov_diag)
        self.zero_twist_pub.publish(new_twist_stamped)

        new_pose_stamped = PoseWithCovarianceStamped()
        quat = yaw_to_quat(candidate_event.yaw_orient)
        new_pose_stamped.header.frame_id = "mmtr/odom"
        new_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        new_pose_stamped.pose.pose.position.x = float(candidate_event.pose_x)
        new_pose_stamped.pose.pose.position.y = float(candidate_event.pose_y)
        new_pose_stamped.pose.pose.position.z = float(candidate_event.pose_z)

        new_pose_stamped.pose.pose.orientation.x = float(quat[0])
        new_pose_stamped.pose.pose.orientation.y = float(quat[1])
        new_pose_stamped.pose.pose.orientation.z = float(quat[2])
        new_pose_stamped.pose.pose.orientation.w = float(quat[3])

        new_pose_stamped.pose.covariance = covariance_from_diagonal(self.pose_cov_diag)
        
        request = SetPose.Request()
        request.pose.header.frame_id="mmtr/odom"
        request.pose.header.stamp = new_twist_stamped.header.stamp
        request.pose.pose.pose.position.x = new_pose_stamped.pose.pose.position.x
        request.pose.pose.pose.position.y = new_pose_stamped.pose.pose.position.y
        request.pose.pose.pose.position.z = new_pose_stamped.pose.pose.position.z
        request.pose.pose.pose.orientation.x = new_pose_stamped.pose.pose.orientation.x
        request.pose.pose.pose.orientation.y = new_pose_stamped.pose.pose.orientation.y
        request.pose.pose.pose.orientation.z = new_pose_stamped.pose.pose.orientation.z
        request.pose.pose.pose.orientation.w = new_pose_stamped.pose.pose.orientation.w
        request.pose.pose.covariance = new_pose_stamped.pose.covariance
        


        future = self.ukf_set.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info(
    f"SetPose result: {getattr(f.result(), 'success', 'no_success_field')}"))
        self.get_logger().info(f"{new_pose_stamped.header.frame_id}")
        self.get_logger().info(
            f"[REWIND] SetPose -> frame=mmtr/odom | x={candidate_event.pose_x}m, y={candidate_event.pose_y} m, z={candidate_event.pose_z} m | yaw={candidate_event.yaw_orient}"
        )


    
    def interpolate_pose(self, before: PoseSample, after: PoseSample, collision_event_time_ns):
        self.get_logger().info("Interpolating")
        interpolation_factor = (collision_event_time_ns - before.t_ns) / (after.t_ns - before.t_ns)
        pose_x = (1-interpolation_factor) * before.pose_x + interpolation_factor * after.pose_x
        pose_y = (1-interpolation_factor) * before.pose_y + interpolation_factor * after.pose_y
        pose_z = before.pose_z
        yaw_difference_raw = after.yaw_orient - before.yaw_orient
        yaw_difference_to_pi = wrap_to_pi(yaw_difference_raw)
        time = collision_event_time_ns
        new_yaw = before.yaw_orient + interpolation_factor * yaw_difference_to_pi
        new_yaw = wrap_to_pi(new_yaw)
        self.get_logger().info(f"{pose_x}")
        return PoseSample(time, pose_x, pose_y, pose_z,new_yaw, before.vel_x, before.angular_vel_z)
    
    def rewind(self, collision_event_time_ns):
        before_t_event = None
        after_t_event  = None
        self.get_logger().info(f"Collision Event Time: {collision_event_time_ns}")
        for s in self.pose_buffer:
            if s.t_ns <= collision_event_time_ns:
                before_t_event = s
            if s.t_ns >= collision_event_time_ns and after_t_event is None:
                after_t_event = s
                break
            self.get_logger().info(f"Pose Buffer: {s}")

        self.get_logger().info(f"Before: {before_t_event}")
        self.get_logger().info(f"After: {after_t_event}")

        if before_t_event is not None and after_t_event is not None:
            self.get_logger().info(f"{before_t_event.t_ns}")
            self.get_logger().info(f"{after_t_event.t_ns}")

            if before_t_event.t_ns == after_t_event.t_ns:
                self.set_ukf_pose(before_t_event)
            else:
                interpolated_pose = self.interpolate_pose(before_t_event, after_t_event, collision_event_time_ns)    
                self.set_ukf_pose(interpolated_pose)
        ## Here is where the rewind will happen
        ## It takes the collision_event_time_ns and using the pose buffer,
        # interpolates where the robot should be at the given time


def main():
    rclpy.init()
    node = RewindOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()