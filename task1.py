#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math


class SensorDataReader(Node):
    def __init__(self):
        super().__init__('sensor_data_reader')
        
        # Subscribe to odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to IMU topic
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('Sensor Data Reader Node Started')
        self.get_logger().info('Subscribing to /odom and /imu/data...')
        
    def quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        Convert quaternion to yaw angle (rotation about z-axis)
        
        Args:
            q: Quaternion from orientation message
            
        Returns:
            yaw: Yaw angle in radians
        """
        # Extract yaw from quaternion using standard conversion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def odom_callback(self, msg: Odometry):
        """
        Callback function for odometry data
        Extracts and displays yaw from odometry orientation
        """
        # Extract quaternion from odometry message
        orientation = msg.pose.pose.orientation
        
        # Convert to yaw
        yaw_odom = self.quaternion_to_yaw(orientation)
        
        # Convert to degrees for readability
        yaw_degrees = math.degrees(yaw_odom)
        
        # Extract position for additional info
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.get_logger().info(
            f'[ODOM] Position: ({x:.3f}, {y:.3f}) | '
            f'Yaw: {yaw_odom:.3f} rad ({yaw_degrees:.1f}°)'
        )
    
    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU data
        Extracts and displays yaw from IMU orientation
        """
        # Extract quaternion from IMU message
        orientation = msg.orientation
        
        # Convert to yaw
        yaw_imu = self.quaternion_to_yaw(orientation)
        
        # Convert to degrees for readability
        yaw_degrees = math.degrees(yaw_imu)
        
        # Extract angular velocity about z-axis
        angular_velocity_z = msg.angular_velocity.z
        
        self.get_logger().info(
            f'[IMU]  Yaw: {yaw_imu:.3f} rad ({yaw_degrees:.1f}°) | '
            f'Angular Velocity (z): {angular_velocity_z:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    
    sensor_reader = SensorDataReader()
    
    try:
        rclpy.spin(sensor_reader)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()