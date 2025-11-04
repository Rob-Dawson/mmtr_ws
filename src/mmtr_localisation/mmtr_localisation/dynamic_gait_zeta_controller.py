#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

G = 9.80665
W_MOVING = 0.03    # rad/s  (enter moving)
W_REST   = 0.02    # rad/s  (enter rest)
A_MOV    = 0.08    # m/s^2  (enter moving)
A_REST   = 0.03    # m/s^2  (enter rest)
TAU      = 0.25    # s      (LPF time constant for omega)
CMD_TIMEOUT = 0.3  # s      (cmd_vel implies imminent motion)
MOVE_DWELL = 0.3   # s
REST_DWELL = 1.0   # s

class MotionScheduler(Node):
    def __init__(self):
        super().__init__('motion_scheduler')
        self.state = 'REST'
        self.omega_lp = 0.0
        self.last_update = time.monotonic()
        self.last_cmd_time = 0.0
        self.edge_t = None
        self._gain = None
        self._zeta = None

        # Param client to the Madgwick node (adjust name/namespace if needed)
        self.param_cli = self.create_client(SetParameters, '/madgwick_filter/set_parameters')

        # Subs
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.create_subscription(Twist, 'model/mmtr/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, sensor_qos)

        # Start with REST params
        self.apply_params(0.10, 0.015, force=True)

    def cmd_cb(self, msg: Twist):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_cmd_time = time.monotonic()

    def imu_cb(self, msg: Imu):
        now = time.monotonic()
        dt = max(1e-3, now - self.last_update)
        self.last_update = now

        wx, wy, wz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        omega = math.hypot(wx, wy, wz)
        a_dev = abs(math.hypot(ax, ay, az) - G)

        # LPF on omega
        alpha = math.exp(-dt/TAU)
        self.omega_lp = alpha * self.omega_lp + (1.0 - alpha) * omega

        imu_moving  = (self.omega_lp > W_MOVING) or (a_dev > A_MOV)
        imu_resting = (self.omega_lp < W_REST)   and (a_dev < A_REST)
        recent_cmd  = (now - self.last_cmd_time) < CMD_TIMEOUT

        want_moving = imu_moving or recent_cmd
        want_rest   = imu_resting and not recent_cmd

        if self.state == 'REST':
            if want_moving:
                if self.edge_t is None:
                    self.edge_t = now
                elif (now - self.edge_t) >= MOVE_DWELL:
                    self.state = 'MOVING'
                    self.edge_t = None
                    self.apply_params(0.06, 0.010)  # moving
            else:
                self.edge_t = None
        else:  # MOVING
            if want_rest:
                if self.edge_t is None:
                    self.edge_t = now
                elif (now - self.edge_t) >= REST_DWELL:
                    self.state = 'REST'
                    self.edge_t = None
                    self.apply_params(0.10, 0.015)  # rest
            else:
                self.edge_t = None

    def apply_params(self, gain: float, zeta: float, force=False):
        if not force and self._gain == gain and self._zeta == zeta:
            return
        self._gain, self._zeta = gain, zeta

        if not self.param_cli.wait_for_service(timeout_sec=0.25):
            self.get_logger().warn('madgwick set_parameters service not available yet')
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='gain',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=gain)
            ),
            Parameter(
                name='zeta',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=zeta)
            ),
        ]
        self.param_cli.call_async(req)
        self.get_logger().info(f'state={self.state} → gain={gain:.3f}, zeta={zeta:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotionScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
