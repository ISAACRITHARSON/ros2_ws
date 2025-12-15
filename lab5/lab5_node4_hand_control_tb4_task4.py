#!/usr/bin/env python3
"""
Lab 5 Task 4: Gesture-Guided Autonomous Navigation
Hand gestures trigger Nav2 navigation goals
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import cv2
import mediapipe as mp
import numpy as np

class GestureGuidedNavigation(Node):
    def __init__(self):
        super().__init__('gesture_guided_navigation')
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publisher for teleop mode
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/diffdrive_controller/cmd_vel',
            10
        )
        
        # Define corner goals (Room 218 - adjust coordinates as needed)
        self.goals = {
            'A': self.create_goal(x=2.0, y=2.0, yaw=0.0),
            'B': self.create_goal(x=2.0, y=-2.0, yaw=1.57),
            'C': self.create_goal(x=-2.0, y=-2.0, yaw=3.14),
            'D': self.create_goal(x=-2.0, y=2.0, yaw=-1.57),
        }
        
        # Current mode
        self.mode = 'STOPPED'  # 'STOPPED', 'TELEOP', 'NAV'
        self.current_goal = None
        self.goal_handle = None
        
        # Control parameters
        self.MAX_LINEAR = 0.31
        self.MAX_ANGULAR = 1.8
        
        # Current command for teleop
        self.current_twist = TwistStamped()
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        # MediaPipe hand tracking
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
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🤖 Gesture-Guided Navigation Started!')
        self.get_logger().info('Open Palm = STOP | Fist = TELEOP')
        self.get_logger().info('Point Up = Goal A | Point Right = Goal B')
        self.get_logger().info('Point Down = Goal C | Point Left = Goal D')
        
    def create_goal(self, x, y, yaw):
        """Create a Nav2 goal pose"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        goal.pose.orientation.z = np.sin(yaw / 2.0)
        goal.pose.orientation.w = np.cos(yaw / 2.0)
        
        return goal
        
    def send_navigation_goal(self, goal_pose):
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server not available!')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f'Sending goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}')
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle Nav2 goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            self.mode = 'STOPPED'
            return
            
        self.get_logger().info('Goal accepted!')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
        
    def goal_result_callback(self, future):
        """Handle Nav2 goal result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached! Waiting 5 seconds...')
            # Could add timer here to wait at corner
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            
        self.mode = 'STOPPED'
        
    def detect_gesture(self, hand_landmarks, frame_width, frame_height):
        """Detect specific hand gestures"""
        if not hand_landmarks:
            return 'NONE'
            
        # Get landmark positions
        landmarks = hand_landmarks.landmark
        
        # Extract key points
        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]
        
        # Calculate if fingers are extended
        fingers_up = []
        
        # Thumb (different logic)
        fingers_up.append(thumb_tip.x < landmarks[3].x if hand_landmarks else False)
        
        # Other fingers (compare tip with PIP joint)
        finger_tips = [8, 12, 16, 20]
        finger_pips = [6, 10, 14, 18]
        
        for tip, pip in zip(finger_tips, finger_pips):
            fingers_up.append(landmarks[tip].y < landmarks[pip].y)
        
        fingers_up_count = sum(fingers_up)
        
        # Gesture recognition
        if fingers_up_count >= 4:
            return 'OPEN_PALM'  # Stop
        elif fingers_up_count == 0:
            return 'FIST'  # Teleop mode
        elif fingers_up[1] and fingers_up_count == 1:  # Only index up
            # Determine direction
            if index_tip.y < wrist.y - 0.2:
                return 'POINT_UP'  # Goal A
            elif index_tip.y > wrist.y + 0.2:
                return 'POINT_DOWN'  # Goal C
            elif index_tip.x > wrist.x + 0.2:
                return 'POINT_RIGHT'  # Goal B
            elif index_tip.x < wrist.x - 0.2:
                return 'POINT_LEFT'  # Goal D
        
        return 'UNKNOWN'
        
    def publish_velocity(self):
        """Continuously publish velocity in teleop mode"""
        if self.mode == 'TELEOP':
            self.current_twist.header.stamp = self.get_clock().now().to_msg()
            self.current_twist.header.frame_id = 'base_link'
            self.cmd_vel_pub.publish(self.current_twist)
        else:
            # Stop
            stop_twist = TwistStamped()
            stop_twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(stop_twist)
            
    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            
            self.mp_draw.draw_landmarks(
                frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
            )
            
            # Detect gesture
            gesture = self.detect_gesture(hand_landmarks, w, h)
            
            # Act on gesture
            if gesture == 'OPEN_PALM':
                self.mode = 'STOPPED'
                self.current_twist.twist.linear.x = 0.0
                self.current_twist.twist.angular.z = 0.0
                cv2.putText(frame, '✋ STOP', (w//2-50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                           
            elif gesture == 'FIST':
                self.mode = 'TELEOP'
                # Calculate velocities from hand position
                cx = np.mean([lm.x for lm in hand_landmarks.landmark]) * w
                cy = np.mean([lm.y for lm in hand_landmarks.landmark]) * h
                
                dx = (cx - w/2) / (w/2)
                dy = (h/2 - cy) / (h/2)
                
                linear = max(0.0, dy * 0.4)
                angular = -dx * 1.0
                
                self.current_twist.twist.linear.x = np.clip(linear, 0.0, self.MAX_LINEAR)
                self.current_twist.twist.angular.z = np.clip(angular, -self.MAX_ANGULAR, self.MAX_ANGULAR)
                
                cv2.putText(frame, '👊 TELEOP MODE', (w//2-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                           
            elif gesture == 'POINT_UP':
                self.mode = 'NAV'
                self.send_navigation_goal(self.goals['A'])
                cv2.putText(frame, '☝ Going to Goal A', (w//2-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                           
            elif gesture == 'POINT_RIGHT':
                self.mode = 'NAV'
                self.send_navigation_goal(self.goals['B'])
                cv2.putText(frame, '👉 Going to Goal B', (w//2-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                           
            elif gesture == 'POINT_DOWN':
                self.mode = 'NAV'
                self.send_navigation_goal(self.goals['C'])
                cv2.putText(frame, '👇 Going to Goal C', (w//2-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                           
            elif gesture == 'POINT_LEFT':
                self.mode = 'NAV'
                self.send_navigation_goal(self.goals['D'])
                cv2.putText(frame, '👈 Going to Goal D', (w//2-100, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        else:
            cv2.putText(frame, 'No hand detected', (w//2-100, h//2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display mode
        cv2.putText(frame, f'Mode: {self.mode}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Gesture-Guided Navigation', frame)
        cv2.waitKey(1)
        
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GestureGuidedNavigation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n Shutting down...')
    except Exception as e:
        print(f'\n Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()