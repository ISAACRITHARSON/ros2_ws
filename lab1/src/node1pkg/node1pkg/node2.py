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
        
    def drive_letter_p(self):
        duration = distance / speed
        self.get_logger().info(f"Driving straight {distance} m at {speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=0.0, duration=duration)
        time.sleep(0.5)
        angular = speed / radius
        duration = (2 * math.pi * radius) / speed
        self.get_logger().info(f"Driving circle radius={radius} m speed={speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=angular, duration=duration/2)

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