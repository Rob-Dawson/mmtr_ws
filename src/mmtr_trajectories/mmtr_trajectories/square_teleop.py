#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SquareTeleop(Node):
    def __init__(self):
        super().__init__('square_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/model/mmtr/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered_ukf', self.odom_callback, 10)

        self.pose_received = False
        self.state = 'forward'
        self.edge_count = 0
        self.start_x = None
        self.start_y = None
        self.target_yaw = None
        self.pause_start_time = None

        self.forward_distance = 4.0
        self.pause_duration = 0.5  # seconds
        self.turn_speed = 1.0
        self.turn_tolerance = 0.01  # radians

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg:Odometry):
        self.pose_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.pose_received:
            return

        twist = Twist()

        if self.state == 'forward':
            if self.start_x is None:
                self.start_x = self.current_x
                self.start_y = self.current_y

            distance = math.sqrt((self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2)

            if distance < self.forward_distance:
                twist.linear.x = 0.5
            else:
                twist = Twist()
                self.state = 'pause'
                self.next_state = 'turn'
                self.pause_start_time = self.get_clock().now()
                self.start_x = None
                self.start_y = None

        elif self.state == 'turn':
            if self.target_yaw is None:
                self.target_yaw = self.normalize_angle(self.current_yaw + math.pi / 2)

            angle_error = self.normalize_angle(self.target_yaw - self.current_yaw)

            if abs(angle_error) > self.turn_tolerance:
                twist.angular.z = self.turn_speed if angle_error > 0 else -self.turn_speed
            else:
                twist = Twist()
                self.edge_count += 1
                if self.edge_count >= 4:
                    self.get_logger().info("Finished square path.")
                    self.destroy_node()
                    return
                self.state = 'pause'
                self.next_state = 'forward'
                self.target_yaw = None
                self.pause_start_time = self.get_clock().now()

        elif self.state == 'pause':
            twist = Twist()
            now = self.get_clock().now()
            if (now - self.pause_start_time).nanoseconds / 1e9 >= self.pause_duration:
                self.state = self.next_state

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = SquareTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
