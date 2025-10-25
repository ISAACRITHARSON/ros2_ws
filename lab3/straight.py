#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class DriveStraightNode(Node):
    def __init__(self):
        super().__init__('drive_straight_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.get_logger().info("Drive Straight Node started")
        
        # Drive straight for 2 meters
        self.drive_straight(distance=2.0, speed=0.2)
        
    def drive_straight(self, distance=2.0, speed=0.2):
        """
        Drive straight for a specified distance
        
        Args:
            distance: Distance to travel in meters
            speed: Linear velocity in m/s
        """
        duration = distance / speed
        self.get_logger().info(f"Driving straight {distance} m at {speed} m/s for {duration} seconds")
        
        # Create velocity command
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        
        # Publish velocity for calculated duration
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        
        # Stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        
        self.get_logger().info("Completed straight line motion")


def main(args=None):
    rclpy.init(args=args)
    
    node = DriveStraightNode()
    
    # Keep node alive briefly to ensure stop command is sent
    time.sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()