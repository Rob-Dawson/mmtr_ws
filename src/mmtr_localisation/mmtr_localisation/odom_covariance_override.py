#!/usr/bin/env python3

"""
Some notes for me
Hard Turn is basically how quickly the robot rotates or how tight the curves are.
Generally important as slips are more noticable during hard turns and so 

curvature_min_speed:
    This is a divide by zero threshold which is useful when the linear_velocity is near 0 (Stopped)
    Only used in curvature mode. The number is chosen as a minumum speed the robot will go
    Normal vel is around 0.5 or 1.0 depending on what it's doing and curvature_min_speed is about 5% or 10% of that

max_gain:
    This is the maximum the velocity_x is allowed to be above cmd_vel so no overshoots. A clamp.
    When the true velocity_x is being estimated, it may be perfect or it may be slightly higher.

    
turn_metric:
    The turn metric is just how much the robot is turning. There are numerious methods for calculating this
    and are discribed in the turn_model

turn_model:
    There are two turn_models which can be used for calculating the turn_metric:
        yaw_rate
        curvature
    Yaw_rate is based entirly on the angular_velocity. The faster the angular_velocity, the greater the turn_metric.
    The Curvature is the yaw rate from the IMU and the speed. At the moment, the plugin only provides open loop control
    I.e. i set 1.0 and it always thinks it's 1.0. 
    Curvature must remain off until there is a better metric (trajectory prior ("i know i have to move in a 2m arc at 5m/s 
    so i should be here. Slip correction determines it's actually going 4.8 and changes the odom to see this.
    2))

    


"""



import math
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def covariance_from_diagonal(diag):
    cov = [0.0] * 36
    for i, val in enumerate(diag):
        cov[i * 6 + i] = float(val)
    return cov

class CovarianceOverride(Node):
    def __init__(self):
        super().__init__("covariance_override")

        # turn correction (yaw-rate model by default)
        self.declare_parameter("turn_model", "yaw_rate")   # or "curvature"
        self.declare_parameter("turn_a", 0.14)             # linear coef
        # self.declare_parameter("turn_a", 0.0)             # linear coef

        self.declare_parameter("turn_b", 0.0)              # quadratic (optional)
        self.declare_parameter("max_gain", 1.15)           # clamp
        self.declare_parameter("curvature_eps", 0.05)      # m/s

        # inflate vx variance in hard turns
        self.declare_parameter("turn_inflate_threshold", 0.3)  # rad/s
        self.declare_parameter("turn_inflate_factor", 3.0)     # × variance


        self.pose_cov_diag  = [1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2]       
        self.twist_cov_diag = [1e-3, 6.4e-3, 1e6, 1e6, 1e6, 1e-2]

        self.turn_model = self.get_parameter("turn_model").value.lower()
        self.turn_a     = float(self.get_parameter("turn_a").value)
        self.turn_b     = float(self.get_parameter("turn_b").value)
        self.min_gain = 0.8
        self.max_gain   = float(self.get_parameter("max_gain").value)
        self.curvature_min_speed  = 0.05

        self.turn_th     = float(self.get_parameter("turn_inflate_threshold").value)
        self.turn_factor = float(self.get_parameter("turn_inflate_factor").value)

        # ---------- I/O ----------
        self.sub = self.create_subscription(Odometry, "/model/mmtr/odometry", self.odom_cb, 50)
        self.pub_fix = self.create_publisher(Odometry, "/model/mmtr/odom_cov_fixed", 10)

        self.last_log_time = self.get_clock().now()
        self.log_period = 0.5  # seconds

    def odom_cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = "base_footprint"
        out.pose = msg.pose
        out.twist = msg.twist  

        ## The linear velocity X comes from the plugin's odom
        linear_velocity_x = float(msg.twist.twist.linear.x) 

        ## This angular velocity Z comes from the robot's IMU as this is more accurate than the raw odom
        angular_velocity_z = float(msg.twist.twist.angular.z) # Yaw Rate (Turning Speed)


        # The turn metric is either the turning_rate (Yaw Mode)
        # If the model is "curvature" it's the path curvature

        if self.turn_model == "curvature":
            turn_metric = abs(angular_velocity_z) / max(abs(linear_velocity_x), self.curvature_min_speed)
        else:
            turn_metric = abs(angular_velocity_z)

        # --- gain and correction ---
        # adjusts the velocity x 
        #odom says turning at speed X
        #imu says turning at speed X

        gain = 1.0 + self.turn_a * turn_metric + self.turn_b * (turn_metric**2)
        gain = max(self.min_gain, min(gain, self.max_gain))
        vx_corr = linear_velocity_x * gain

        # --- write corrected twist ---
        out.twist.twist.linear.x  = vx_corr
        out.twist.twist.linear.y = 0.0
        out.twist.twist.angular.z = angular_velocity_z

        # --- covariances ---
        out.pose.covariance = covariance_from_diagonal(self.pose_cov_diag)
        twist_cov_diag = list(self.twist_cov_diag)
        if abs(angular_velocity_z) > self.turn_th:
            twist_cov_diag[0] *= self.turn_factor
        out.twist.covariance = covariance_from_diagonal(twist_cov_diag)

        # --- publish corrected ---
        self.pub_fix.publish(out)

        # --- log corrected values (rate-limited) ---
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 >= self.log_period:
            self.get_logger().info(
                f"wz={angular_velocity_z:+.3f} rad/s  m={turn_metric:.3f}  gain={gain:.3f}  "
                f"vx_raw={linear_velocity_x:+.3f} → vx_corr={vx_corr:+.3f}"
            )
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceOverride()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


