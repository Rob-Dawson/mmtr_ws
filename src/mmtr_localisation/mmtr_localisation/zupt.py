#!/usr/bin/env python3

import math, statistics
import collections
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist

GRAVITY = 9.80665


def vec_mag(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


class Zupt(Node):
    def __init__(self):
        super().__init__("ZUPT")
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/model/mmtr/cmd_vel", self.cmd_vel_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "model/mmtr/odometry", self.odom_callback, 10
        )

        self.zupt_pub = self.create_publisher(TwistWithCovarianceStamped, "zupt/twist", 10)

        ## Parameters
        self.timer_rate_hz = 30.0
        self.window_size_sec = 0.4
        ## Thresholds

        # odom
        self.velocity_threshold = 1e-3  # 0.001 m/s
        self.angular_threshold = 1e-3  # 0.001 rad/s

        # IMU
        self.imu_buffer = collections.deque()
        self.buffer_size = int(self.timer_rate_hz * self.window_size_sec)
        self.last_imu_time = None
        self.angular_vel_mean_thresh = 0.01
        self.angular_vel_std_thresh = 0.003
        self.linear_accel_mean_thresh = 0.3
        self.linear_accel_std_thresh = 0.05

        # This is based on the cmd_vel. If it's greater than 0, this means I want the robot to move
        # meaning a ZUPT should not happen

        ##States
        self.allow_zupt = False  # True when commands are zero
        self.odom_is_zero = False
        self.is_zupt = False

        self.imu_quiet_since = None
        self.timer = self.create_timer(1.0 / self.timer_rate_hz, self.tick)

    def cmd_vel_callback(self, cmd):
        print("HIIIII")
        linear_magnitude = vec_mag(cmd.linear.x, cmd.linear.y, cmd.linear.z)
        angular_magnitude = vec_mag(cmd.angular.x, cmd.angular.y, cmd.angular.z)
        if linear_magnitude > 0.0 or angular_magnitude > 0.0:
            self.allow_zupt = False
        else:
            self.allow_zupt = True
        print("cmd_vel_callback")

    def odom_callback(self, odom):
        linear_magnitude = vec_mag(
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
        )
        angular_magnitude = vec_mag(
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
        )

        if (
            linear_magnitude < self.velocity_threshold
            and angular_magnitude < self.angular_threshold
        ):
            self.odom_is_zero = True
        else:
            self.odom_is_zero = False

    def imu_callback(self, imu):
        angular_magnitude = vec_mag(
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z
        )
        linear_magnitude = vec_mag(
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        )
        agap = abs(linear_magnitude - GRAVITY)

        if self.last_imu_time is not None:
            dt = (imu.header.stamp.sec - self.last_imu_time.sec) + 1e-9 * (
                imu.header.stamp.nanosec - self.last_imu_time.nanosec
            )
            if dt > 1e-6:
                imu_rate = 1.0 / dt
                self.buffer_size = max(5, int(self.window_size_sec * imu_rate))
                while len(self.imu_buffer) > self.buffer_size:
                    self.imu_buffer.popleft()
        self.last_imu_time = imu.header.stamp
        self.imu_buffer.append((angular_magnitude, agap, imu.header.stamp))
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.popleft()

    def imu_stationary_check(self):
        if len(self.imu_buffer) < max(5, int(0.1 * self.buffer_size)):
            return False
        max_angular_velocity = [b[0] for b in self.imu_buffer]
        max_linear_acceleration = [b[1] for b in self.imu_buffer]

        angular_velocity_mean = statistics.fmean(max_angular_velocity)
        angular_velocity_std = statistics.pstdev(max_angular_velocity)
        linear_acceleration_mean = statistics.fmean(max_linear_acceleration)
        linear_acceleration_std = statistics.pstdev(max_linear_acceleration)
        is_stationary = (
            angular_velocity_mean < self.angular_vel_mean_thresh
            and angular_velocity_std < self.angular_vel_std_thresh
            and linear_acceleration_mean < self.linear_accel_mean_thresh
            and linear_acceleration_std < self.linear_accel_std_thresh
        )
        return bool(is_stationary)

    def tick(self):
        imu_stationary = self.imu_stationary_check()
        if not (self.allow_zupt and self.odom_is_zero):
            self.is_zupt = False
            self.imu_quiet_since = None
            return
        # How long the IMU should be quiet for
        hold_time = min(0.5 * self.window_size_sec, 0.25)
        if imu_stationary:
            # Use wall time via timer rate (simple) — or use ROS time if you prefer
            if self.imu_quiet_since is None:
                self.imu_quiet_since = self.get_clock().now().nanoseconds * 1e-9
            elapsed = self.get_clock().now().nanoseconds * 1e-9 - self.imu_quiet_since
            self.is_zupt = elapsed >= hold_time
            self.publish_to_zero()
            print("STATIONARY")
        else:
            self.is_zupt = False
            self.imu_quiet_since = None
            print("not Stationary")

    def publish_to_zero(self):

        msg = TwistWithCovarianceStamped()
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        covariance = [0.0] * 36
        diag = [1e-3, 1e-3, 1e-2, 1e-3, 1e-3, 3e-4]
        for i, v in enumerate(diag):
            covariance[i*6 + i] = v
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
