#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class DriveRectangle(Node):
    def __init__(self):
        super().__init__('drive_rectangle')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self.get_logger().info("DriveRectangle node started.")
        time.sleep(1.0)
        self.run_path()

    def publish_twist(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Publishes velocity commands for a set duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(msg)
            time.sleep(0.1)
        # stop after each move
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        time.sleep(0.5)

    def drive_straight(self, distance, speed):
        """Drives the TurtleBot straight for a given distance."""
        duration = distance / speed
        self.get_logger().info(f"Driving straight {distance} m at {speed} m/s")
        self.publish_twist(linear_x=speed, angular_z=0.0, duration=duration)

    def turn_left(self, angle_deg, angular_speed):
        """Turns the TurtleBot left by a given angle in degrees."""
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad / angular_speed)
        self.get_logger().info(f"Turning left {angle_deg} degrees at {angular_speed} rad/s")
        self.publish_twist(linear_x=0.0, angular_z=angular_speed, duration=duration)

    def run_path(self):
        """Executes the rectangular path around the room."""
        speed = 1.0  # constant linear speed (m/s)
        angular_speed = 0.5  # rad/s (for ~90° turns)

        # Path sequence: 3 m → left → 6.5 m → left → 3 m → left → 6.5 m → stop
        self.get_logger().info("Starting rectangular path...")

        self.drive_straight(distance=3.0, speed=speed)
        self.turn_left(angle_deg=90, angular_speed=angular_speed)

        self.drive_straight(distance=6.5, speed=speed)
        self.turn_left(angle_deg=90, angular_speed=angular_speed)

        self.drive_straight(distance=3.0, speed=speed)
        self.turn_left(angle_deg=90, angular_speed=angular_speed)

        self.drive_straight(distance=6.5, speed=speed)
        self.turn_left(angle_deg=90, angular_speed=angular_speed)

        self.get_logger().info("Finished rectangular path. Stopping TurtleBot.")
        self.stop_and_exit()

    def stop_and_exit(self):
        """Stops the robot and shuts down."""
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info("TurtleBot stopped.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DriveRectangle()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
