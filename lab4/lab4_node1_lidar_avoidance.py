#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')

        # --- Subscriptions ---
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',          # Make sure your LiDAR publishes here
            self.lidar_callback,
            10
        )

        # --- Publisher ---
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',       # Use standard ROS2 velocity topic
            10
        )

        self.cmd = Twist()
        self.get_logger().info("Lidar Avoidance Node Started")

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)

        # Replace inf values for computation
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)

        # FRONT angle = center of the scan array
        num = len(ranges)
        center = num // 2

        # Take ±15° around front
        window = 15
        front_ranges = ranges[center - window : center + window]

        front_distance = np.min(front_ranges)

        # Log front distance occasionally
        self.get_logger().info(f"Front distance: {front_distance:.2f}")

        # --- Simple avoidance ---
        if front_distance < 0.5:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5
        else:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0

        self.pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
