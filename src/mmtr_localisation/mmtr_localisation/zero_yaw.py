#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist
from std_msgs.msg import Float32MultiArray

GRAVITY = 9.80665  # m/s^2

# ----- helpers -----
def quat_to_yaw(q):
    qw, qx, qy, qz = q
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

def quat_from_yaw(yaw):
    h = 0.5*yaw
    return (math.cos(h), 0.0, 0.0, math.sin(h))

def q_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

def norm_quat(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)

def cov_diag(rx, ry, rz):
    return [rx, 0.0, 0.0, 0.0, ry, 0.0, 0.0, 0.0, rz]


class AdaptiveIMUNode(Node):
    """Cmd-dominant hybrid yaw gate (no ZUPT inside), with IMU-extend, clamp & fade."""

    def __init__(self):
        super().__init__('adaptive_imu_gating')

        # latches & timing
        self.declare_parameter("post_settle_s",   2.5)    # fade-in after gate
        self.declare_parameter('min_cruise_s', 1.5)    # keep gate during cruise

        # covariances
        self.declare_parameter("RP_VAR", 1e6)     # de-weight roll/pitch for 2D
        self.declare_parameter("YAW_VAR_LO", 3e-3)
        self.declare_parameter("YAW_VAR_HI", 1e8) # very high: ignore IMU yaw while moving
        self.declare_parameter("GYR_VAR", 1e-4)
        self.declare_parameter("ACC_VAR", 1e-3)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("samples", 30)
        self.declare_parameter("init_duration", 5)

        # fetch

        self.init_duration  = float(self.get_parameter('init_duration').value)
        self.samples        = int(self.get_parameter('samples').value)

        self.MIN_CRUISE_S   = float(self.get_parameter('min_cruise_s').value)


        self.GYR_VAR_HI   = 1e-2 
        self.GYR_VAR_LOW  = 1e-4

        self.RP_VAR         = float(self.get_parameter('RP_VAR').value)
        self.YAW_VAR_LO     = float(self.get_parameter('YAW_VAR_LO').value)
        self.YAW_VAR_HI     = float(self.get_parameter('YAW_VAR_HI').value)
        self.GYR_VAR        = float(self.get_parameter('GYR_VAR').value)
        self.ACC_VAR        = float(self.get_parameter('ACC_VAR').value)
        self.publish_debug  = bool(self.get_parameter('publish_debug').value)
        # IO
        self.sub     = self.create_subscription(Imu,   'imu/data', self.cb_imu, 100)
        self.pub     = self.create_publisher(Imu,      'imu/data_zero_yaw', 10)
        self.pub_dbg = self.create_publisher(Float32MultiArray, 'imu/dynamics_diag', 10) if self.publish_debug else None
        self.cmd_sub = self.create_subscription(Twist, 'model/mmtr/cmd_vel', self.cb_cmd, 10)

        # state
        self.start_time = None
        self.y_sin_sum = 0.0
        self.y_cos_sum = 0.0
        self.n = 0
        self.yaw0 = None
        self.q_offset = None

        self.start_gate = 3.0
        self.stop_hold_s = 3.0
        self.min_cruise = 0.0
        self.last_cmd = Twist()

        self.cmd_non_zero_since = None
        self.motion_since = 0.0
        self.gate_until = 0.0
        self.cmd_vel_deadband = 0.01
        self.TURN_THR = 0.2

    def cb_cmd(self, msg: Twist):
        now = time.monotonic()
        moving_now = abs(msg.linear.x) > self.cmd_vel_deadband
        moved_prev = (abs(self.last_cmd.linear.x) > self.cmd_vel_deadband)

        # Rising edge
        if moving_now and not moved_prev:
            self.gate_until = max(self.gate_until, now + self.start_gate)

        # While moving
        if moving_now:
            self.gate_until = max(self.gate_until, now, self.MIN_CRUISE_S)
        
        # Falling Edge
        if (not moving_now) and moved_prev:
            print("Stopping")
            self.gate_until = max(self.gate_until, now + self.stop_hold_s)

        self.last_cmd = msg

    def cb_imu(self, msg: Imu):
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        q_f = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

        # lock initial yaw
        if self.yaw0 is None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            yaw = quat_to_yaw(q_f)
            self.y_sin_sum += math.sin(yaw)
            self.y_cos_sum += math.cos(yaw)
            self.n += 1

            if (elapsed >= self.init_duration) and (self.n >= self.samples):
                self.yaw0 = math.atan2(self.y_sin_sum / self.samples, self.y_cos_sum / self.samples)
                self.q_offset = quat_from_yaw(-self.yaw0)
                self.get_logger().info(f"Locked yaw at {self.yaw0:.3f} rad")
            else:
                self.pub.publish(msg)  # passthrough until locked
                return

        # Apply yaw-zeroing
        q_corr = norm_quat(q_mult(self.q_offset, q_f))

        ## Decide gating from cmd + turning immunity
        now = time.monotonic()
        angular_vel = msg.angular_velocity.z
        turning = False
        if (abs(angular_vel) > self.TURN_THR) or (abs(self.last_cmd.angular.z) > self.TURN_THR):
            turning = True
        
        gate_yaw = (now < self.gate_until) and (not turning)
        yaw_var = self.YAW_VAR_HI if gate_yaw else self.YAW_VAR_LO
        self.GYR_VAR = self.GYR_VAR_HI if gate_yaw else self.GYR_VAR_LOW 

        # ---- publish ----
        out = Imu()
        out.header = msg.header
        out.orientation = Quaternion(x=q_corr[1], y=q_corr[2], z=q_corr[3], w=q_corr[0])
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        out.orientation_covariance         = cov_diag(self.RP_VAR, self.RP_VAR, yaw_var)
        out.angular_velocity_covariance    = cov_diag(self.GYR_VAR, self.GYR_VAR, self.GYR_VAR)
        out.linear_acceleration_covariance = cov_diag(self.ACC_VAR, self.ACC_VAR, self.ACC_VAR)

        self.pub.publish(out)

        if self.pub_dbg:
            v = Float32MultiArray()
            v.data = [
                float(1.0 if gate_yaw else 0.0),   # 0
                float(1.0 if turning else 0.0),    # 1
                float(yaw_var),                    # 2
                float(self.gate_until - now)       # 3 remaining gate seconds (can be <0)
            ]
            self.pub_dbg.publish(v)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveIMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
