#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('nav2_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Nav2 Goal Sender Node Started")
    
    def send_goal(self, x, y, yaw=0.0):
        """Send navigation goal to Nav2"""
        self._action_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # Convert yaw to quaternion
        q = self.yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        # For 2D navigation, roll=0, pitch=0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        
        # Quaternion: [x, y, z, w]
        return [0.0, 0.0, sy, cy]
    
    def goal_response_callback(self, future):
        """Callback when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback when navigation completes"""
        result = future.result().result
        self.get_logger().info(f'Navigation complete!')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = Nav2GoalSender()
    
    # Send goal: x=-24.18, y=-22.916, yaw=0.0 (or 0.643 radians)
    node.send_goal(-24.18, -22.916, yaw=0.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()