#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class flipper_default_pose(Node):
    def __init__(self):
        super().__init__("Flipper_Pose")
        self.flipper_pub = self.create_publisher(Float64MultiArray, "/flipper_controller/commands", 10)
        self.timer = self.create_timer(1.0, self.flipper_pub_cb)

    def flipper_pub_cb(self):
        pose = Float64MultiArray()
        pose.data = [0.001,0.001,-0.001,-0.001]
        self.flipper_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = flipper_default_pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()