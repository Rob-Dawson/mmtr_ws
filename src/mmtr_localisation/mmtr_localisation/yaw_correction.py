#!/usr/bin/env python3
##Used for correcting yaw based error under acceleration due to gravity

##A better ap

import math 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
GRAVITY = 9.80665

def rpy_to_quat(r,p,y):
    c_roll = math.cos(r/2)
    s_roll = math.sin(r/2)
    
    c_pitch = math.cos(p/2)
    s_pitch = math.sin(p/2)
    
    c_yaw = math.cos(y/2)
    s_yaw = math.sin(y/2)

    qx = s_roll * c_pitch * c_yaw - c_roll * s_pitch * s_yaw
    qy = c_roll * s_pitch * c_yaw + s_roll * c_pitch * s_yaw
    qz = c_roll * c_pitch * s_yaw - s_roll * s_pitch * c_pitch
    qw = c_roll * c_pitch * c_yaw + s_roll * s_pitch * s_yaw
    return qx,qy,qz,qw

def quat_to_rpy(q):

    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w
      
    roll = math.atan2(2.0 * (qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy))
    pitch = math.asin(2.0 * (qw*qy - qz*qx))
    yaw =  math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy+qz*qz))
    return roll, pitch, yaw



def quat_from_yaw(yaw):
    h = 0.5*yaw
    return (math.cos(h), 0.0, 0.0, math.sin(h))

def wrap_pi(accel: float):
    return (accel + math.pi) % (2.0 * math.pi) - math.pi

class Yaw_Correction(Node):
    def __init__(self):
        super().__init__("Yaw_Correction_Node")

        self.imu_sub = self.create_subscription(Imu, "imu/data", self.imu_sub_cb, 10)
        self.yaw_correction_pub = self.create_publisher(Imu, "imu/data_yaw_correction", 10)

        self.yaw_estimate = None
        
        ##Essentially a Schmitt trigger m/s2
        self.gate_hi = 1.2
        self.gate_low = 0.6

        ##Used to 
        self.hold = False
        self.last_stamp = None
        
    def imu_sub_cb(self, imu:Imu):
        orientation = imu.orientation
        _,_,yaw = quat_to_rpy(orientation)

        timestamp = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = timestamp
            self.yaw_estimate = yaw
            self.hold = False
            return 
        
        current_time = timestamp - self.last_stamp
        self.last_stamp = timestamp
        if current_time <= 0.0 or current_time > 0.2:
            current_time = 0.0
        
        acceleration_mag = math.sqrt(imu.linear_acceleration.x *
                                     imu.linear_acceleration.x +
                                     imu.linear_acceleration.y *
                                     imu.linear_acceleration.y +
                                     imu.linear_acceleration.z *
                                     imu.linear_acceleration.z)
        accel_without_gravity = abs(acceleration_mag - GRAVITY)
        
        if accel_without_gravity > self.gate_hi:
            self.hold = True
        if accel_without_gravity < self.gate_low:
            self.hold = False

        if self.hold:
            self.yaw_estimate = wrap_pi(self.yaw_estimate + imu.angular_velocity.z * current_time)
        else:
            error = wrap_pi(yaw - self.yaw_estimate)
            k = 1.0 - math.exp(-current_time / 2.2) if current_time > 0.0 else 0.0
            self.yaw_estimate = wrap_pi(self.yaw_estimate+ k * error)


        roll, pitch, _ = quat_to_rpy(imu.orientation)
        qx, qy, qz, qw = rpy_to_quat(roll, pitch, self.yaw_estimate)
        imu_out = imu
        imu_out.orientation = Quaternion(x=float(qx),y=float(qy),z=float(qz),w=float(qw))
        self.yaw_correction_pub.publish(imu_out)

def main():
    rclpy.init()
    node = Yaw_Correction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()