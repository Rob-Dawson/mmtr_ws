#!/usr/bin/env python3

##Many IMU's arrive uncalibrated which may result in strange values
# including gravity being too high or low. 
# This is a simple test to ensure if this is a scaling issue 
# 
# 
# 
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUScalingTest(Node):
    def __init__(self):
        super().__init__("Gravity_Test")
        self.create_subscription(Imu, "/imu/filtered", self.imu_cb, 10)

    def _get_magnitude(self, acceleration):
        return math.sqrt(acceleration[0]*acceleration[0] + acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2])
    
    def imu_cb(self, imu:Imu):
        accel = [imu.linear_acceleration.x, imu.linear_acceleration.y,imu.linear_acceleration.z]
        accel_mag = self._get_magnitude(accel)
        self.get_logger().info(f"Accleration Magnitude: {accel_mag}")

def main():
    rclpy.init()
    node = IMUScalingTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()