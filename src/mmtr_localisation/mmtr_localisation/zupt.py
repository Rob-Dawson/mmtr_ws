#!/usr/bin/env python3

import math, statistics
import collections
import rclpy
from rclpy.time import Time
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

        ##Pubs and Subs
        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_callback, 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/model/mmtr/cmd_vel", self.cmd_vel_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "model/mmtr/odometry", self.odom_callback, 10
        )

        self.zupt_pub = self.create_publisher(
            TwistWithCovarianceStamped, "zupt/twist", 10
        )

        ## Parameters
        self.timer_rate_hz = 30.0
        self.window_size_sec = 0.4
        ## Thresholds

        # odom
        self.velocity_threshold = 1e-3  # 0.001 m/s
        self.angular_threshold = 1e-3  # 0.001 rad/s

        # IMU
        self.imu_buffer = collections.deque()
        self.imu_buffer_window = int(0.5 * 1e9)
        self.angular_vel_mean_thresh = 0.01
        self.angular_vel_std_thresh = 0.003
        self.linear_accel_mean_thresh = 0.3
        self.linear_accel_std_thresh = 0.05

        ##States
        self.allow_zupt = False  # True when commands are zero
        self.odom_is_zero = False
        self.is_zupt = False

        self.imu_quiet_since = None
        self.timer = self.create_timer(1.0 / self.timer_rate_hz, self.tick)


    def cmd_vel_callback(self, cmd):
        linear_magnitude = vec_mag(cmd.linear.x, cmd.linear.y, cmd.linear.z)
        angular_magnitude = vec_mag(cmd.angular.x, cmd.angular.y, cmd.angular.z)
        if linear_magnitude > 0.0 or angular_magnitude > 0.0:
            self.allow_zupt = False
        else:
            self.allow_zupt = True

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

    def imu_callback(self, imu: Imu):
        imu_time = Time.from_msg(imu.header.stamp).nanoseconds
        angular_magnitude = vec_mag(
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z
        )
        linear_magnitude = vec_mag(
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        )
        agap = abs(linear_magnitude - GRAVITY)

        self.imu_buffer.append((imu_time, angular_magnitude, agap))
        # self.get_logger().info(f"Len{len(self.imu_buffer)}")
        while self.imu_buffer and (imu_time - self.imu_buffer[0][0] >= self.imu_buffer_window):
            self.imu_buffer.popleft()

    def imu_stationary_check(self):
        if len(self.imu_buffer) < 2:
            return False    
        span_ns = self.imu_buffer[-1][0] - self.imu_buffer[0][0]
        if span_ns < 0.5 * self.imu_buffer_window:
            return False
        max_angular_velocity = [b[1] for b in self.imu_buffer]
        max_linear_acceleration = [b[2] for b in self.imu_buffer]

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
            if elapsed >= hold_time:
                self.publish_to_zero()
                self.get_logger().info("Stationary")
        else:
            self.is_zupt = False
            self.imu_quiet_since = None
            self.get_logger().info("Not Stationary")

    def publish_to_zero(self):

        msg = TwistWithCovarianceStamped()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0

        covariance = [0.0] * 36
        diag = [1e-3, 1e-3, 1e-2, 1e-3, 1e-3, 3e-4]
        for i, v in enumerate(diag):
            covariance[i * 6 + i] = v
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
