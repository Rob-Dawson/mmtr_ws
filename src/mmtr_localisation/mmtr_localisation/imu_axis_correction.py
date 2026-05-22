#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from geometry_msgs.msg import Quaternion
def quaternion_mul(q_correction, q_imu):
    x1 = q_correction[0]
    y1 = q_correction[1]
    z1 = q_correction[2]
    w1 = q_correction[3]

    x2 = q_imu.x
    y2 = q_imu.y
    z2 = q_imu.z
    w2 = q_imu.w
    
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    corrected_q = Quaternion()
    corrected_q.x = x
    corrected_q.y = y
    corrected_q.z = z
    corrected_q.w = w 
    return corrected_q

class IMUAxisCorrection(Node):
    def __init__(self):
        super().__init__("Imu_Axis_correction")
        
        self.create_subscription(Imu, "/imu", self.imu_cb, 10)
        self.imu_pub = self.create_publisher(Imu, "imu_corrected_axis", 10)

    def imu_cb(self, imu:Imu):
        
        orientation = imu.orientation
        linear_accel = imu.linear_acceleration
        angular_vel = imu.angular_velocity
        q_corr = [1.0,0.0,0.0,0.0]
        corrected_q = quaternion_mul(q_corr, orientation)
        normalise = math.sqrt(corrected_q.x * corrected_q.x +
                              corrected_q.y * corrected_q.y +
                              corrected_q.z * corrected_q.z +
                              corrected_q.w * corrected_q.w)
        if normalise > 0:
            corrected_q.x /= normalise
            corrected_q.y /= normalise
            corrected_q.z /= normalise
            corrected_q.w /= normalise

        
        corrected_imu = Imu()
        corrected_imu.header = imu.header
        corrected_imu.orientation = corrected_q
        corrected_imu.angular_velocity.x = angular_vel.x
        corrected_imu.angular_velocity.y = -angular_vel.y
        corrected_imu.angular_velocity.z = -angular_vel.z
        corrected_imu.linear_acceleration.x = linear_accel.x
        corrected_imu.linear_acceleration.y = -linear_accel.y
        corrected_imu.linear_acceleration.z = -linear_accel.z

        self.imu_pub.publish(corrected_imu)



def main():
    rclpy.init()
    node = IMUAxisCorrection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()