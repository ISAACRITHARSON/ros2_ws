#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # Create a 1 Hz timer
        self.timer_ = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Drive Node started — executing rectangular path...")

        # Parameters (you can change these as needed)
        self.room_length = 6.5   # meters
        self.room_width = 3.0    # meters
        self.speed = 0.3         # m/s (safe constant speed)
        self.angular_speed = 0.5 # rad/s (for smooth 90° turns)
        self.step = 0

    def publish_twist(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Publish Twist messages for a set duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_time = time.time() + duration

        while time.time() < end_time:
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop after each segment
        stop = Twist()
        self.publisher_.publish(stop)
        time.sleep(0.5)

    def drive_forward(self, distance, speed):
        """Drive forward a given distance at specified speed."""
        duration = distance / speed
        self.get_logger().info(f"Driving forward {distance} m at {speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def turn_left_90(self):
        """Turn 90° to the left (counterclockwise)."""
        degrees = 90
        radians = math.radians(degrees)
        duration = abs(radians / self.angular_speed)
        self.get_logger().info(f"Turning left 90° ({duration:.2f}s)")
        self.publish_twist(linear_x=0.0, angular_z=self.angular_speed, duration=duration)

    def execute_rectangle(self):
        """Drive a full rectangular path around the room."""
        self.get_logger().info("Starting rectangular path...")

        # Side 1 – forward 3 m
        self.drive_forward(distance=self.room_width, speed=self.speed)
        self.turn_left_90()

        # Side 2 – forward 6.5 m
        self.drive_forward(distance=self.room_length, speed=self.speed)
        self.turn_left_90()

        # Side 3 – forward 3 m
        self.drive_forward(distance=self.room_width, speed=self.speed)
        self.turn_left_90()

        # Side 4 – forward 6.5 m
        self.drive_forward(distance=self.room_length, speed=self.speed)
        self.turn_left_90()

        self.get_logger().info("Completed full rectangular path.")
        self.stop_robot()

    def stop_robot(self):
        stop = Twist()
        self.publisher_.publish(stop)
        time.sleep(0.5)

    def timer_callback(self):
        """Timer callback executes the motion once."""
        if self.step == 0:
            self.execute_rectangle()
            self.get_logger().info("All motions completed — shutting down.")
            rclpy.shutdown()
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
