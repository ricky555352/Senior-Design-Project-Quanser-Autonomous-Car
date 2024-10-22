#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
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
    
    def listener_callback(self, msg):
        # Convert the incoming ROS image to OpenCV format
        self.get_logger().info("Received image for stop sign detection")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Check if a stop sign is detected in the image
        detected = self.detect_stop_sign(cv_image)
        stop_status = Bool()
        
        if detected:
            self.get_logger().info("Stop sign detected!")
            stop_status.data = True
        else:
            self.get_logger().info("No stop sign detected.")
            stop_status.data = False
        
        # Publish the stop sign detection result
        self.publisher_sign.publish(stop_status)

    def detect_stop_sign(self, img):
        height, width, _ = img.shape
        
        # Define two ROIs: one on the left and one on the right side of the image
        roi_left = img[int(height * 0.3):int(height * 0.7), 0:int(width * 0.3)]  # Left 30% of the image
        roi_right = img[int(height * 0.3):int(height * 0.7), int(width * 0.7):width]  # Right 30% of the image

        # Combine both ROIs into one image for easier processing
        roi_combined = np.hstack((roi_left, roi_right))

        # Convert ROI to HSV color space for color filtering
        hsv = cv2.cvtColor(roi_combined, cv2.COLOR_BGR2HSV)

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
                            cv2.rectangle(roi_combined, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.imshow("Detected Stop Sign", roi_combined)  # Optional, for debugging
                            cv2.waitKey(1)
                            return True  # Stop sign detected with octagon shape and correct text
        return False  # No stop sign detected

    def stop_node(self):
        self.get_logger().info("Shutting down Stop Sign Node...")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python library
    node = StopSignNode()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("Stop Sign Node stopped manually.")
    finally:
        node.stop_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
