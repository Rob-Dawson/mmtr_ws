#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu

class CalibrateMag(Node):
    def __init__(self):
        super().__init__("Calibrate_Mag")
        self.mag_sub = self.create_subscription(
            MagneticField, "/magnetometer", self.magnet_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data_raw", self.imu_callback, 10
        )
        self.mag_calibration_pub = self.create_publisher(
            MagneticField, "/imu/mag", 10
        )
        self.hard_iron_offset_x = 0
        self.hard_iron_offset_y = 0
        self.hard_iron_offset_z = 0
        self.imu_stamp = None

        self.flip_horizontal = False

        self.declination_deg = self.declare_parameter('declination_deg', 0.0).value
        self.align_window_s  = self.declare_parameter('align_window_s', 0.8).value
        self.still_thresh    = self.declare_parameter('still_thresh', 0.01).value  # rad/s
        self.gate_until_aligned = self.declare_parameter('gate_until_aligned', True).value

        self._collecting = False
        self._t0 = 0.0
        self._yaw_offset = 0.0
        self._have_offset = False
        self._acc_last = (0.0, 0.0, 0.0)
        self._mag_sum = [0.0, 0.0, 0.0]
        self._n_mag = 0
        self._acc_sum = [0.0, 0.0, 0.0]
        self._n_acc = 0


    def imu_callback(self, imu):
        self.imu_stamp = imu.header.stamp
        # keep latest accel + gyro to test "still"
        self._acc_last = (imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z)
        g = (imu.angular_velocity.x**2 + imu.angular_velocity.y**2 + imu.angular_velocity.z**2)**0.5
        if self._have_offset:
                return

        if not self._collecting and g < self.still_thresh:
            self._collecting = True
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            self._mag_sum = [0.0, 0.0, 0.0]
            self._n_mag = 0
        elif self._collecting and g >= self.still_thresh:
            self._collecting = False  # moved; abort window

    def magnet_callback(self, mag):
        # The mag uses NED frame but everything else world, imu etc use EMU "I don't know why, it's weird"
        # So this needs to be converted from NED to ENU
        # I have made this extremly explicit to understand what is going on
        mag_X_NED_N = mag.magnetic_field.x
        mag_Y_NED_E = mag.magnetic_field.y
        mag_Z_NED_D = mag.magnetic_field.z
        
        #Scale from gauss to tesla
        mag_X_NED_N *= 1e-4
        mag_Y_NED_E *= 1e-4
        mag_Z_NED_D *= 1e-4

        mag_x_ENU_E = -mag_Y_NED_E
        mag_y_ENU_N = mag_X_NED_N
        mag_z_ENU_U = -mag_Z_NED_D

        #Now use ENU for scaling etc

        mag_x_ENU_E -= self.hard_iron_offset_x
        mag_y_ENU_N -= self.hard_iron_offset_y
        mag_z_ENU_U -= self.hard_iron_offset_z

      
        out = MagneticField()
        out.header = mag.header
        out.header.frame_id = "imu_link"
        out.header.stamp = self.imu_stamp or mag.header.stamp
        out.magnetic_field.x = mag_x_ENU_E
        out.magnetic_field.y = mag_y_ENU_N
        out.magnetic_field.z = mag_z_ENU_U
        out.magnetic_field_covariance = mag.magnetic_field_covariance
        self.mag_calibration_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateMag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()