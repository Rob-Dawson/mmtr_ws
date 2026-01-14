#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.time import Time 
class IMUCalibration(Node):
    def __init__(self):
        super().__init__("IMU_Calibration")
        self.imu_sub = self.create_subscription(Imu, "/imu/out", self.imu_callback, 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)

        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []

        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.gyro_x_error = None
        self.gyro_y_error = None
        self.gyro_z_error = None

        self.accel_x_error = None
        self.accel_y_error = None
        self.accel_z_error = None

        self.calibration_time = 5 #seconds
        self.imu_calibrated = False
        self.elapsed_time = 0
        self.start_time = None

    def mean_error(self, gyro_x, gyro_y, gyro_z,
                   accel_x,accel_y,accel_z):
        self.gyro_x_error = sum(gyro_x) / len(gyro_x)
        self.gyro_y_error = sum(gyro_y) / len(gyro_y)
        self.gyro_z_error = sum(gyro_z) / len(gyro_z)

        self.accel_x_error = sum(accel_x) / len(accel_x)
        self.accel_y_error = sum(accel_y) / len(accel_y)
        self.accel_z_error = (sum(accel_z) / len(accel_z)) - 9.80665

    def imu_callback(self, imu:Imu):
        if self.start_time is None:
            self.start_time = Time.from_msg(imu.header.stamp).nanoseconds
        imu_linear_x = imu.linear_acceleration.x
        imu_linear_y = imu.linear_acceleration.y
        imu_linear_z = imu.linear_acceleration.z

        imu_angular_x = imu.angular_velocity.x
        imu_angular_y = imu.angular_velocity.y
        imu_angular_z = imu.angular_velocity.z
        
        if self.imu_calibrated:
            imu_linear_x -= self.accel_x_error
            imu_linear_y -= self.accel_y_error
            imu_linear_z -= self.accel_z_error

            imu_angular_x -= self.gyro_x_error
            imu_angular_y -= self.gyro_y_error
            imu_angular_z -= self.gyro_z_error

            new_imu_msg = Imu()
            new_imu_msg.header.stamp = imu.header.stamp
            new_imu_msg.header.frame_id = "imu_link"
            
            new_imu_msg.orientation_covariance = [-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0]

            new_imu_msg.linear_acceleration.x = imu_linear_x
            new_imu_msg.linear_acceleration.y = imu_linear_y
            new_imu_msg.linear_acceleration.z = imu_linear_z
            new_imu_msg.linear_acceleration_covariance = imu.linear_acceleration_covariance


            new_imu_msg.angular_velocity.x = imu_angular_x
            new_imu_msg.angular_velocity.y = imu_angular_y
            new_imu_msg.angular_velocity.z = imu_angular_z
            new_imu_msg.angular_velocity_covariance = imu.angular_velocity_covariance


            self.imu_pub.publish(new_imu_msg)
            return
        self.gyro_x.append(imu_angular_x)
        self.gyro_y.append(imu_angular_y)
        self.gyro_z.append(imu_angular_z)

        self.accel_x.append(imu_linear_x)
        self.accel_y.append(imu_linear_y)
        self.accel_z.append(imu_linear_z)

        current_time = Time.from_msg(imu.header.stamp).nanoseconds
        if current_time - self.start_time > self.calibration_time * 1e9:
            self.imu_calibrated = True
            self.mean_error(self.gyro_x, self.gyro_y, self.gyro_z,
            self.accel_x,self.accel_y,self.accel_z)
        
def main(args=None):
    rclpy.init(args=args)
    node = IMUCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == "__main__":
    main()