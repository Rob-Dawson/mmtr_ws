#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from rclpy.clock import Clock, ClockType
# import random
# class AutoTeleop(Node):
#     def __init__(self):
#         super().__init__('AutoTeleop')
#         self.pub = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)
#         self.linear_speed =  0.5
#         self.angular_speed = 1.0
#         self.create_timer(1.0, self.timer_callback)
#         self.states = ["STOPPED", "FORWARDS", "BACKWARDS"]
#         self.state = random.choice(self.states)
#         self.current_speed = 0.0
#         self.target_speed = 0.0
#         self.clock = Clock(clock_type=ClockType.ROS_TIME)
#         self.start_time = self.clock.now()
#         self.target_duration = 2.0

#     def timer_callback(self):
#         now = self.clock.now()
#         elapsed = now - self.start_time
#         elapsedSeconds = elapsed.nanoseconds / 1e9
        
#         if self.state == "STOPPED":
#             if elapsedSeconds >= self.target_duration:
#                 self.state = random.choice(self.states)  # Transition to a random state
#                 self.target_speed = 0.0
#                 self.start_time = self.clock.now()
#         elif self.state == "FORWARDS":
#             if elapsedSeconds >= self.target_duration:
#                 self.state = random.choice(self.states)  # Transition to a random state
#                 self.target_speed = 0.5
#                 self.start_time = self.clock.now()
#         else:
#             if elapsedSeconds >= self.target_duration:
#                 self.state = random.choice(self.states)  # Transition to a random state
#                 self.target_speed = -0.5
#                 self.start_time = self.clock.now()

#         msg = Twist()
#         msg.linear.x = self.target_speed
#         msg.angular.z = 0.0
#         self.pub.publish(msg)


    

# def main(args=None):
#     rclpy.init(args=args)
#     node = AutoTeleop()
#     rclpy.spin(node)
#     rclpy.shutdown()
# if __name__ == '__main__':
#     main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time

# class SquareMove(Node):
#     def __init__(self):
#         super().__init__('square_move_node')
#         self.publisher_ = self.create_publisher(Twist, "/model/mmtr/cmd_vel", 10)
#         self.timer_period = 0.1  # seconds
#         self.timer = self.create_timer(self.timer_period, self.move_square)
#         self.state = 'turn'
#         self.start_time = self.get_clock().now()
#         self.stop_duration = 1.00  # seconds to move straight
#         self.turn_duration = 10.0  # seconds to rotate ~90 degrees
#         self.edge_count = 0

#     def move_square(self):
#         msg = Twist()
#         now = self.get_clock().now()
#         elapsed = (now - self.start_time).nanoseconds / 1e9  # convert to seconds

#         if self.state == 'turn':
#             if elapsed < self.turn_duration:
#                 msg.linear.x = 0.0
#                 msg.angular.z = -0.5
#             else:
#                 msg.linear.x = 0.0
#                 msg.angular.z = 0.0
#                 self.state = "stop"
#                 self.start_time = now  # reset timer AFTER switching state

#         elif self.state == "stop":
#             msg.linear.x = 0.0
#             msg.angular.z = 0.0
#             if elapsed >= self.stop_duration:
#                 self.state = "turn"
#                 self.start_time = now  # reset timer AFTER switching state


#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = SquareMove()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
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
    node = SquareDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
