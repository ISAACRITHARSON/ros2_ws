#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class ColorTracker(Node):
    def __init__(self):
        super().__init__("color_tracker_red_lowres")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_count = 0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        
        # Subscribe to camera
        self.create_subscription(
            Image, 
            "/oakd/rgb/image_raw", 
            self.image_callback, 
            10
        )
        
        # Red color ranges in HSV
        self.lower1 = np.array([0, 120, 70])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 120, 70])
        self.upper2 = np.array([180, 255, 255])
        
        # Timer to process frames at 30Hz
        self.create_timer(0.033, self.process_frame)
        
        # Create windows AFTER node init
        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
        cv2.startWindowThread()  # Start OpenCV event loop
        
        self.get_logger().info("Red Color Tracker started")
    
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = cv2.resize(frame, (320, 240))
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")
    
    def process_frame(self):
        if self.latest_frame is None:
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera...", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("Camera", blank)
            cv2.imshow("Mask", np.zeros((240, 320), dtype=np.uint8))
            cv2.waitKey(1)
            return
        
        # Make a copy to avoid race conditions
        frame = self.latest_frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for red
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = mask1 | mask2
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        h, w, _ = frame.shape
        cx = None
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 100:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        
        # Movement control
        twist = Twist()
        if cx is not None:
            error = cx - w / 2
            twist.angular.z = -float(error) / 200.0
            twist.linear.x = 0.12
        else:
            twist.angular.z = 0.25
            twist.linear.x = 0.0
        
        self.cmd_pub.publish(twist)
        
        # Add frame counter to verify updates
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, 220),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Display
        cv2.imshow("Camera", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.cmd_pub.publish(twist)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()