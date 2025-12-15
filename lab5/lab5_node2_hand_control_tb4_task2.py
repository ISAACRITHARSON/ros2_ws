#!/usr/bin/env python3
"""
Lab 5 Task 2: Hand Gesture Control for TurtleBot 4
Adapted from turtlesim with safety limits and no backward motion
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import cv2
import mediapipe as mp
import numpy as np
import threading

class HandGestureTB4Controller(Node):
    def __init__(self):
        super().__init__('hand_gesture_tb4_controller')
        
        # Publisher for TurtleBot 4
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, 
            '/diffdrive_controller/cmd_vel_unstamped', 
            10
        )
        
        # TurtleBot 4 Safety Limits (CRITICAL!)
        self.MAX_LINEAR_VEL = 0.31   # m/s
        self.MAX_ANGULAR_VEL = 1.8   # rad/s
        
        # Control parameters
        self.linear_gain = 0.5
        self.angular_gain = 1.2
        self.deadzone = 0.15
        
        # Smoothing
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.smoothing_factor = 0.3
        
        # Current command
        self.current_twist = TwistStamped()
        
        # Continuous publishing timer (20Hz)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Webcam
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Camera not available!')
            raise RuntimeError('Camera not available')
        
        # Control loop timer
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('🤖 TurtleBot 4 Hand Gesture Controller Started!')
        self.get_logger().info(f'Safety Limits: {self.MAX_LINEAR_VEL}m/s, {self.MAX_ANGULAR_VEL}rad/s')
        self.get_logger().info('⚠️  NO BACKWARD MOTION')
        
    def publish_velocity(self):
        """Continuously publish velocity to avoid timeout"""
        self.current_twist.header.stamp = self.get_clock().now().to_msg()
        self.current_twist.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(self.current_twist)
    
    def smooth_velocity(self, new_value, prev_value):
        """Apply exponential smoothing"""
        return (self.smoothing_factor * new_value + 
                (1 - self.smoothing_factor) * prev_value)
        
    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        
        # Draw reference lines
        cv2.line(frame, (w//2, 0), (w//2, h), (100, 100, 100), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (100, 100, 100), 1)
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                
                # Calculate hand center
                cx = np.mean([lm.x for lm in hand_landmarks.landmark]) * w
                cy = np.mean([lm.y for lm in hand_landmarks.landmark]) * h
                
                # Calculate displacement
                dx = (cx - w/2) / (w/2)
                dy = (h/2 - cy) / (h/2)
                
                # Apply deadzone
                if abs(dx) < self.deadzone:
                    dx = 0.0
                if abs(dy) < self.deadzone:
                    dy = 0.0
                
                # Convert to velocities
                linear_vel = dy * self.linear_gain
                angular_vel = -dx * self.angular_gain
                
                # NO BACKWARD MOTION!
                if linear_vel < 0:
                    linear_vel = 0.0
                
                # Apply smoothing
                linear_vel = self.smooth_velocity(linear_vel, self.prev_linear)
                angular_vel = self.smooth_velocity(angular_vel, self.prev_angular)
                
                # Apply safety limits
                self.current_twist.twist.linear.x = float(np.clip(
                    linear_vel, 0.0, self.MAX_LINEAR_VEL
                ))
                self.current_twist.twist.angular.z = float(np.clip(
                    angular_vel, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL
                ))
                
                self.prev_linear = self.current_twist.twist.linear.x
                self.prev_angular = self.current_twist.twist.angular.z
                
                # Visual feedback
                cv2.circle(frame, (int(cx), int(cy)), 15, (0, 255, 0), -1)
                cv2.putText(frame, f'Lin: {self.current_twist.twist.linear.x:.2f} m/s',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f'Ang: {self.current_twist.twist.angular.z:.2f} rad/s',
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # No hand - stop
            self.current_twist.twist.linear.x = 0.0
            self.current_twist.twist.angular.z = 0.0
            self.prev_linear = 0.0
            self.prev_angular = 0.0
            cv2.putText(frame, 'No hand - STOPPED', (w//2-100, h//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow('TurtleBot 4 Hand Control', frame)
        cv2.waitKey(1)
        
    def destroy_node(self):
        stop_twist = TwistStamped()
        self.cmd_vel_pub.publish(stop_twist)
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandGestureTB4Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n Shutting down...')
    except Exception as e:
        print(f'\n Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()