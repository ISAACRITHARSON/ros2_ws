#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class FusionController(Node):
    def __init__(self):
        super().__init__('fusion_controller')
        self.bridge = CvBridge()
        
        # --- Publishers ---
        self.pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        
        # --- Subscribers ---
        # Try multiple possible camera topics
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # --- Control State Variables ---
        self.obstacle_detected = False
        self.target_center = None
        self.frame_width = 320
        self.latest_frame = None
        self.latest_mask = None
        self.frame_count = 0
        self.last_image_time = time.time()
        self.last_lidar_time = time.time()
        
        # Status variables
        self.current_status = "INITIALIZING" 
        self.status_color = (255, 255, 255)
        
        # Red color ranges in HSV
        self.lower1 = np.array([0, 100, 100])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([160, 100, 100])
        self.upper2 = np.array([180, 255, 255])
        
        # Control loop timer (10Hz)
        self.create_timer(0.1, self.control_loop)
        
        # Visualization loop timer (15Hz - reduced for VM stability)
        self.create_timer(1/15.0, self.visualization_loop)

        # Status check timer
        self.create_timer(3.0, self.status_check)
        
        # Create OpenCV windows
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera", 640, 480)
        cv2.resizeWindow("Mask", 640, 480)
        
        self.get_logger().info("=== Fusion Controller Started ===")
        self.get_logger().info("Waiting for camera and LIDAR data...")
    
    def lidar_callback(self, msg):
        """Process LIDAR data to detect obstacles."""
        try:
            self.last_lidar_time = time.time()
            ranges = np.array(msg.ranges)
            
            # Replace inf/NaN with large distance
            ranges = np.where(np.isfinite(ranges), ranges, 10.0)
            
            # Check front sector (±30 degrees)
            num = len(ranges)
            angle_window = 30  # degrees
            window_size = int(num * angle_window / 360)
            
            # Front indices
            front_ranges = np.concatenate((ranges[:window_size], ranges[-window_size:]))
            
            front_distance = np.min(front_ranges)
            
            # Obstacle detection threshold
            self.obstacle_detected = front_distance < 0.5
            
        except Exception as e:
            self.get_logger().error(f"LIDAR error: {e}")
    
    def image_callback(self, msg):
        """Process camera images to detect red objects."""
        try:
            self.last_image_time = time.time()
            self.frame_count += 1
            
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize for consistent processing
            frame = cv2.resize(frame, (320, 240))
            self.frame_width = frame.shape[1]
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Create red mask (two ranges for hue wrap)
            mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
            mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
            mask = mask1 | mask2
            
            # Morphological operations to clean mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            cx = None
            if contours:
                # Find largest contour
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                
                # Filter noise (minimum area threshold)
                if area > 200:
                    M = cv2.moments(largest)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        
                        # Draw tracking visualization
                        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
                        cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                        
                        # Draw bounding box
                        x, y, w, h = cv2.boundingRect(largest)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        
                        cv2.putText(frame, f"Area: {int(area)}", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            self.target_center = cx
            self.latest_frame = frame
            self.latest_mask = mask
            
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
    
    def control_loop(self):
        """Main control logic combining LIDAR and vision."""
        cmd = Twist()
        
        # Check if we have recent sensor data
        time_since_image = time.time() - self.last_image_time
        time_since_lidar = time.time() - self.last_lidar_time
        
        if time_since_image > 2.0 or time_since_lidar > 2.0:
            # No recent data - stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            status = "NO SENSOR DATA"
            color = (128, 128, 128)  # Gray
        
        # PRIORITY 1: Obstacle Avoidance
        elif self.obstacle_detected:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            status = "⚠️ AVOIDING OBSTACLE"
            color = (0, 0, 255)  # Red
        
        # PRIORITY 2: Track Red Object
        elif self.target_center is not None:
            error = self.target_center - (self.frame_width / 2)
            
            # Proportional control
            Kp_angular = 0.005  # Tuning parameter
            Kp_linear = 0.0002
            
            cmd.linear.x = 0.2  # Base forward speed
            cmd.angular.z = -Kp_angular * error
            
            # Clamp angular velocity
            cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
            
            status = f"✓ TRACKING (err: {int(error)}px)"
            color = (0, 255, 0)  # Green
        
        # PRIORITY 3: Search
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
            status = "🔍 SEARCHING"
            color = (255, 165, 0)  # Orange
        
        # Publish command
        self.pub.publish(cmd)
        
        # Update status
        self.current_status = status
        self.status_color = color
        
    def visualization_loop(self):
        """Update OpenCV windows."""
        if self.latest_frame is not None and self.latest_mask is not None:
            frame = self.latest_frame.copy()
            mask = self.latest_mask.copy()
            
            # Add status overlay
            cv2.putText(frame, self.current_status, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.status_color, 2)
            
            # Frame counter
            cv2.putText(frame, f"Frame: {self.frame_count}", (10, 220),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Obstacle warning
            if self.obstacle_detected:
                cv2.putText(frame, "!!! OBSTACLE !!!", (60, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            
            # Center crosshair
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
            
            # Display
            cv2.imshow("Camera", frame)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
        
    def status_check(self):
        """Periodic status logging."""
        img_elapsed = time.time() - self.last_image_time
        lidar_elapsed = time.time() - self.last_lidar_time
        
        if img_elapsed > 2.0:
            self.get_logger().warn(f"⚠️ No camera data for {img_elapsed:.1f}s")
        if lidar_elapsed > 2.0:
            self.get_logger().warn(f"⚠️ No LIDAR data for {lidar_elapsed:.1f}s")
        
        if img_elapsed < 2.0 and lidar_elapsed < 2.0:
            obs = "DETECTED" if self.obstacle_detected else "CLEAR"
            tgt = f"X={self.target_center}" if self.target_center else "NONE"
            self.get_logger().info(f"✓ Obstacle: {obs} | Target: {tgt} | Frames: {self.frame_count}")

def main(args=None):
    rclpy.init(args=args)
    node = FusionController()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        cmd = Twist()
        node.pub.publish(cmd)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()