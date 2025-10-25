#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class DriveNode(Node):
    def __init__(self):
        super().__init__('node1')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Node1 started, publishing on /cmd_vel")

    def publish_twist(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        # stop after move
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def drive_straight(self, distance=0.5, speed=0.2):
        duration = distance / speed
        self.get_logger().info(f"Driving straight {distance} m at {speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def drive_circle(self, radius=1.0, speed=1):
        angular = speed / radius
        duration = (2 * math.pi * radius) / speed
        self.get_logger().info(f"Driving circle radius={radius} m speed={speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=angular, duration=duration/2)

    def drive_letter_p(self):
        """Drive straight, then small circle to form 'P'"""
        self.get_logger().info("Driving letter P...")
        self.drive_straight(distance=0.5, speed=0.2)
        time.sleep(0.5)
        self.publish_twist(linear_x=0.0, angular_z=0.2, duration=5)
        time.sleep(0.5)
        self.drive_circle(radius=0.25, speed=0.1)

    def timer_callback(self):
        if self.counter_ == 0:
            self.drive_letter_p()
            self.get_logger().info("Completed letter P motion")
            self.destroy_node()
            rclpy.shutdown()
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
