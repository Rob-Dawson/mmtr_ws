#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Subscriber to Odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/model/mmtr/odometry',  # Adjust if needed
            self.odom_callback,
            10)
        
        self.subscription  # Prevent unused variable warning
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        # Create a TransformStamped message
        tf_msg = TransformStamped()
        
        # Set header information
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "odom"  # Parent frame (adjust if needed)
        tf_msg.child_frame_id = "base_link"  # Child frame (adjust if needed)

        # Set translation (position from odometry)
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation (orientation from odometry)
        tf_msg.transform.rotation = msg.pose.pose.orientation

        # Publish the transform
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # Log the transformation
        self.get_logger().info(f"Broadcasting Transform: odom → base_link | "
                               f"x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, z: {msg.pose.pose.position.z}")

def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
