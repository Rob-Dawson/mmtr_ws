#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import math
class mag_check(Node):
    def __init__(self):
        super().__init__("mag_check")

        self.mag_sub = self.create_subscription(MagneticField, "/imu/mag", self.mag_cb, 10)

    def mag_cb(self, msg:MagneticField):
        yaw_mag = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        print(yaw_mag)

def main(args=None):
    rclpy.init(args=args)
    node = mag_check()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()