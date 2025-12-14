#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
 
class LidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__("lidar_avoidance")
        self.sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.cmd = Twist()
 
    def lidar_callback(self, msg):
 
        num_readings = len(msg.ranges)

 
        front_index = num_readings // 4  # 90 degrees
        sector_size = 30 
        start_index = front_index - sector_size
        end_index = front_index + sector_size
        front_ranges = list(msg.ranges[start_index:end_index])
        valid_ranges = [r for r in front_ranges if np.isfinite(r) and r > 0.12]
        if len(valid_ranges) == 0:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.0
            self.pub.publish(self.cmd)
            return
        min_front = min(valid_ranges)
        print(f"front: {min_front:.2f}m")
        if min_front < 1.0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5
            print("yo something's there, turning")
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
 
if __name__ == "__main__":
    main()

