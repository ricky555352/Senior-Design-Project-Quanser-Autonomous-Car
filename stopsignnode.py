#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class StopSignNode(Node):
    def __init__(self):
        super().__init__('stop_sign_node')

        self.get_logger().info("Stop Sign Node has started and subscribed to the camera feed.")
        
        # Initialize CvBridge for converting ROS images to OpenCV
        self.bridge = CvBridge()

        # Publisher for stop sign detection status
        self.publisher_sign = self.create_publisher(Bool, 'sign_status', 10)
        
        # Subscriber to compressed image topic from the camera
        self.subscription = self.create_subscription(
            CompressedImage,
            '/qcar/csi_front',  # Topic published by CameraNode
            self.listener_callback,
            10
        )
    
        self.last_detection_time = 0  # Timestamp for the last detection
        self.detection_delay = 6.0  # Delay in seconds between detections
        
    def listener_callback(self, msg):
        # Convert the incoming ROS image to OpenCV format
        #self.get_logger().info("Received image for stop sign detection")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        height, width, _ = cv_image.shape
        
        # Define two ROIs: one on the left and one on the right side of the image
        roi_x_start = int(width * 0.0)    # Start at 0% from the left
        roi_x_end = int(width * 1.0)      # End at 100% from the right
        roi_y_start = int(height * 0.3)   # Start at 30% from the top
        roi_y_end = int(height * 0.7)     # End at 70% from the top
        
        # Extract the ROI from the image
        roi = cv_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Draw a yellow rectangle around the ROI for debugging
        cv2.rectangle(cv_image, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 255), 2)

        # Check if a stop sign is detected in the ROI
        detected = self.detect_stop_sign(roi)
        stop_status = Bool()
        
        # Enforce a delay between detections
        current_time = time.time()
        if detected and (current_time - self.last_detection_time >= self.detection_delay):
            stop_status.data = True  # Stop Sign detected
            self.publisher_sign.publish(stop_status)
            self.last_detection_time = current_time
            self.get_logger().info("Stop Sign detected!")
        elif not detected:
            stop_status.data = False  # No stop sign detected
        
        # Publish the stop sign detection result
        self.publisher_sign.publish(stop_status)

        # Display the ROI on the full image with the yellow rectangle
        cv2.imshow("Stop Sign Detection with ROI", cv_image)
        cv2.waitKey(1)

    def detect_stop_sign(self, img):
        # Convert ROI to HSV color space for color filtering
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range for red color in HSV (for stop sign detection)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only red colors
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the red mask
        contours = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # Loop over contours to find octagonal shapes
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)

            # Check if the contour has 8 vertices (octagon shape)
            if len(approx) == 8:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                area = cv2.contourArea(cnt)

                # Ensure the contour is roughly square-shaped, has a reasonable area, and is octagonal
                if 0.8 <= aspect_ratio <= 1.2 and 1000 < area < 10000:  # Adjust size constraints as needed
                    # Further refine the shape detection by using the solidity check (area vs convex hull area)
                    hull = cv2.convexHull(cnt)
                    hull_area = cv2.contourArea(hull)
                    if hull_area > 0:
                        solidity = area / float(hull_area)
                        if solidity > 0.9:  # The object should be solid and convex
                            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.imshow("Detected Stop Sign", img)  # Optional, for debugging
                            cv2.waitKey(1)
                            return True  # Stop sign detected with octagon shape and correct text
        return False  # No stop sign detected

    def stop_node(self):
        self.get_logger().info("Shutting down Stop Sign Node...")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = StopSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stop Sign Node stopped manually.")
    finally:
        node.stop_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()