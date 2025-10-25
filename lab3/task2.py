#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math

class StraightDriveNode(Node):
    def __init__(self):
        super().__init__('straight_drive_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.initial_yaw = None
        self.current_yaw = 0.0

    def imu_callback(self, msg):
        q = msg.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        if self.initial_yaw is None:
            self.initial_yaw = yaw
        self.current_yaw = yaw

    def control_loop(self):
        if self.initial_yaw is None:
            return
        error = self.initial_yaw - self.current_yaw
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = 1.0 * error  # simple P controller
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = StraightDriveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()