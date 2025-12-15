#!/usr/bin/env python3
"""
Lab 5 Task 1: Hand Gesture Control for Turtlesim
Controls turtlesim turtle using MediaPipe hand tracking
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import numpy as np

class HandGestureTurtlesimController(Node):
    def __init__(self):
        super().__init__('hand_gesture_turtlesim')
        
        # Publisher to turtlesim
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Control parameters - tuned for drawing
        self.linear_gain = 2.0
        self.angular_gain = 3.0
        self.deadzone = 0.15
        
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
            self.get_logger().info('Falling back to keyboard control')
            raise RuntimeError('Camera not available')
        
        # Timer for control loop
        self.timer = self.create_timer(0.033, self.control_loop)
        
        self.get_logger().info('Hand Gesture Turtlesim Controller Started!')
        self.get_logger().info('Move hand: UP=forward, DOWN=back, LEFT/RIGHT=turn')
        
    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Flip for mirror effect
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        
        # Draw center crosshair
        cv2.line(frame, (w//2, 0), (w//2, h), (100, 100, 100), 1)
        cv2.line(frame, (0, h//2), (w, h//2), (100, 100, 100), 1)
        cv2.circle(frame, (w//2, h//2), 50, (100, 100, 100), 1)
        
        # Convert to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        # Default: stop
        twist = Twist()
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                
                # Calculate hand center
                cx = np.mean([lm.x for lm in hand_landmarks.landmark]) * w
                cy = np.mean([lm.y for lm in hand_landmarks.landmark]) * h
                
                # Calculate displacement from center (-1 to 1)
                dx = (cx - w/2) / (w/2)
                dy = (h/2 - cy) / (h/2)
                
                # Apply deadzone
                if abs(dx) < self.deadzone:
                    dx = 0.0
                if abs(dy) < self.deadzone:
                    dy = 0.0
                
                # Convert to velocities
                twist.linear.x = dy * self.linear_gain
                twist.angular.z = -dx * self.angular_gain
                
                # Visual feedback
                cv2.circle(frame, (int(cx), int(cy)), 15, (0, 255, 0), -1)
                cv2.line(frame, (w//2, h//2), (int(cx), int(cy)), (255, 0, 0), 3)
                
                # Display velocities
                cv2.putText(frame, f'Linear: {twist.linear.x:.2f}',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f'Angular: {twist.angular.z:.2f}',
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(frame, 'No hand detected',
                       (w//2 - 100, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Publish velocity
        self.cmd_vel_pub.publish(twist)
        
        # Show frame
        cv2.imshow('Hand Gesture - Turtlesim', frame)
        cv2.waitKey(1)
        
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HandGestureTurtlesimController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    except Exception as e:
        print(f'\nError: {e}')
        print('Note: Camera not available. Use keyboard control instead.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()