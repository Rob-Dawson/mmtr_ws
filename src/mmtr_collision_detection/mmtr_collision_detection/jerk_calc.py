#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import Float32
from mmtr_msg.msg import CollisionEvent


def quat_to_Rwb(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array(
        [
            [1 - 2*(yy + zz),   2*(xy - wz),     2*(xz + wy)],
            [  2*(xy + wz),   1 - 2*(xx + zz),   2*(yz - wx)],
            [  2*(xz - wy),     2*(yz + wx),   1 - 2*(xx + yy)],
        ],
        dtype=float,
    )


class jerk_calc(Node):
    def __init__(self):
        super().__init__("jerk_calc")

        # Debug pubs
        self.pub_jraw  = self.create_publisher(Vector3Stamped, "/dbg/jerk_raw", 10)
        self.pub_oxgb  = self.create_publisher(Vector3Stamped, "/dbg/omega_cross_gb", 10)
        self.pub_jcorr = self.create_publisher(Vector3Stamped, "/dbg/jerk_corr", 10)
        self.pub_jmag  = self.create_publisher(Float32, "/dbg/jerk_corr_mag", 10)


        self.pub_jerk = self.create_publisher(Vector3Stamped, "/jerk", 10)


        # IO
        self.imu_sub  = self.create_subscription(Imu, "/imu/data", self.imu_cb, 100)
        self.odom_sub = self.create_subscription(Odometry, "/model/mmtr/odometry", self.odom_cb, 10)
        self.collision_pub = self.create_publisher(CollisionEvent, "/collision/event", 100)
        self.emergency_stop_pub = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)

        self.speed = 0.0

        # ENU (z up): gravity is negative Z
        self.g_world = np.array([0.0, 0.0, -9.80665], dtype=float)

        # State
        self.prev_t_ns   = None
        self.prev_flp    = None
        self.prev_omega  = None
        self.prev_g_b    = None
        self.omega_lp    = None

        # Params
        self.dt_min = 1e-3
        self.fc_w   = 20.0  # Hz (gyro LPF)
        self.fc_f   = 20.0  # Hz (specific force LPF)
        self.r      = np.array([0.0, 0.0, 0.015], dtype=float)  # lever arm
        self.collision_thresh = 200.0

        self.turn_gate_until = 0.0

    def odom_cb(self, msg: Odometry):
        self.speed = abs(msg.twist.twist.linear.x)

    def imu_cb(self, msg: Imu):
        # --- time (integer ns from header) ---
        t_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if self.prev_t_ns is None:
            self.prev_t_ns = t_ns
            return
        dt = max(self.dt_min, (t_ns - self.prev_t_ns) * 1e-9)

        # --- sensors ---
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z], dtype=float)
        omega_raw = np.array([msg.angular_velocity.x,
                              msg.angular_velocity.y,
                              msg.angular_velocity.z], dtype=float)

        q = np.array([msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w], dtype=float)
        
        q /= max(1e-12, np.linalg.norm(q))
        Rwb = quat_to_Rwb(*q)
        g_b = Rwb.T @ self.g_world  # world -> body

        # --- small LPF on gyro ---
        aw = 1.0 - math.exp(-2.0 * math.pi * self.fc_w * dt)
        aw = min(max(aw, 1e-3), 0.9)
        if self.omega_lp is None:
            self.omega_lp = omega_raw.copy()
        omega = aw * omega_raw + (1.0 - aw) * self.omega_lp
        self.omega_lp = omega

        # angular acceleration (filtered)
        ang_acc = np.zeros(3) if self.prev_omega is None else (omega - self.prev_omega) / dt

        # --- lever-arm correction (centripetal only by default) ---
        a_rot = np.cross(omega, np.cross(omega, self.r))
        a_corr = accel - a_rot

        # --- gravity removal ---
        f = a_corr - g_b

        # --- LPF on specific force ---
        af = 1.0 - math.exp(-2.0 * math.pi * self.fc_f * dt)
        af = min(max(af, 1e-4), 0.9)

        if self.prev_flp is None:
            self.prev_flp   = f.copy()
            self.prev_omega = omega.copy()
            self.prev_g_b   = g_b.copy()
            self.prev_t_ns  = t_ns
            return

        flp   = af * f + (1.0 - af) * self.prev_flp
        j_raw = (flp - self.prev_flp) / dt

        # --- subtract rotation-of-gravity term (midpoint) ---
        omega_mid = 0.5 * (omega + self.prev_omega)
        g_b_mid   = 0.5 * (g_b   + self.prev_g_b)
        oxgb      = np.cross(omega_mid, g_b_mid)
        j_corr    = j_raw - oxgb
        J         = float(np.linalg.norm(j_corr))

        # optional turn-start gate (suppress triggers briefly on big alpha_z)
        if abs(ang_acc[2]) > 3.0:  # rad/s^2, tune
            self.turn_gate_until = (t_ns * 1e-9) + 0.08
        in_turn_start = (t_ns * 1e-9) < self.turn_gate_until  # (use later in detector)

        # --- publish debug (stamped) ---
        self._pub_vec(self.pub_jraw,  msg.header.stamp, *j_raw)
        self._pub_vec(self.pub_oxgb,  msg.header.stamp, *oxgb)
        self._pub_vec(self.pub_jcorr, msg.header.stamp, *j_corr)
        self.pub_jmag.publish(Float32(data=J))


        jerk = Vector3Stamped()
        jerk.header.stamp = msg.header.stamp
        jerk.vector.x = float(j_corr[0])
        jerk.vector.y = float(j_corr[1])
        jerk.vector.z = float(j_corr[2])
        self.pub_jerk.publish(jerk)


        # --- update state (IMPORTANT) ---
        self.prev_flp   = flp
        self.prev_omega = omega
        self.prev_g_b   = g_b
        self.prev_t_ns  = t_ns


    def _pub_vec(self, pub, stamp, x, y, z):
        m = Vector3Stamped()
        m.header.stamp = stamp
        m.vector.x, m.vector.y, m.vector.z = float(x), float(y), float(z)
        pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = jerk_calc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
