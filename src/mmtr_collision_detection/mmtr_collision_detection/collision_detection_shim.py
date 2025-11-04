#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
# from std_msgs.msg import 

from mmtr_msg.msg import CollisionEvent

directions = ["FRONT", "BACK", "LEFT", "RIGHT"]
class CollisionDetectionShim(Node):
    def __init__(self):
        super().__init__("collision_detection_shim")

        self.collision_sub = self.create_subscription(Contacts, "/contact", self.collision_cb, 10)
        self.stop_pub = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)
        self.contact = 0
        self.probability = 1.0
        self.event_pub = self.create_publisher(CollisionEvent, "/collision/event", 10)

    def collision_cb(self, msg):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.stop_pub.publish(t)

        evt = CollisionEvent()
        evt.header.stamp = self.get_clock().now().to_msg()
        evt.header.frame_id = "base_link"
        evt.hit = True
        evt.side = CollisionEvent.FRONT     # TODO: infer from IMU/contacts later
        evt.confidence = self.probability
        self.event_pub.publish(evt)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionShim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()