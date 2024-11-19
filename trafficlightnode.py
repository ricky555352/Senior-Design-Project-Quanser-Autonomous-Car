#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')
        self.bridge = CvBridge()
        self.publisher_sign = self.create_publisher(Bool, 'light_status', 10)
        
        # Subscribe to the camera's image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/qcar/csi_front',
            self.listener_callback,
            10
        )

        self.last_detection_time = 0  # Timestamp for the last detection
        self.detection_delay = 2.0  # Delay in seconds between detections
        
    def listener_callback(self, msg):
        # Convert incoming ROS image to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Define an expanded ROI (Region of Interest) for larger green rectangle
        height, width = cv_image.shape[:2]
        
        # Expand the ROI to cover a larger area
        roi_x_start = int(width * 0.1)  # Start at 10% from the left
        roi_x_end = int(width * 0.9)    # End at 90% from the left
        roi_y_start = int(height * 0.05) # Start at 5% from the top
        roi_y_end = int(height * 0.7)    # End at 70% from the top
        roi = cv_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Detect traffic light color within the ROI
        detected_red = self.detect_red_light(roi)

        # Publish the light status based on detection
        '''
        light_status = Bool()
        light_status.data = detected_red  # True if red light detected, False otherwise
        '''
        light_status = Bool()
        
        # Enforce a delay between detections
        current_time = time.time()
        if detected_red and (current_time - self.last_detection_time >= self.detection_delay):
            light_status.data = True  # Red light detected
            self.publisher_sign.publish(light_status)
            self.last_detection_time = current_time
            self.get_logger().info("Red light detected!")
        elif not detected_red:
            light_status.data = False  # No red light detected
            
        self.publisher_sign.publish(light_status)

        # Show the ROI with the green rectangle for debugging
        cv2.rectangle(cv_image, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 0), 2)
        cv2.imshow("Traffic Light ROI", cv_image)
        cv2.waitKey(1)

    def detect_red_light(self, roi):
        hsv_img = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define a narrow HSV range for detecting the red traffic light
        # This range is fine-tuned to target only the specific red of traffic lights
        hsv_red_lower = (0, 180, 150)
        hsv_red_upper = (10, 255, 255)

        # Create a mask for red light detection within this HSV range
        red_mask = cv2.inRange(hsv_img, hsv_red_lower, hsv_red_upper)

        # Apply Gaussian blur to reduce noise and smooth the edges of the mask
        blurred_mask = cv2.GaussianBlur(red_mask, (9, 9), 2)

        # Define minimum area threshold for red detection
        min_red_area = 300  # Adjust based on camera view for distance #300 default

        # Check if the red mask area is above the threshold
        red_area = cv2.countNonZero(blurred_mask)
        if red_area < min_red_area:
            # If area is too small (not traffic light) will be false
            return False

        # Detect contours in the red mask to check for circular shapes
        contours = cv2.findContours(blurred_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # Initialize red light detection flag
        red_light_detected = False

        # Process each contour and ensure it is circular
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            
            if perimeter == 0:
                continue  # Skip invalid contours that are not round

            circularity = 4 * np.pi * (area / (perimeter ** 2))

            # Check for high circularity: closer to 1 indicates a circular shape
            if 0.85 < circularity < 1.15:  # Strict range for circularity to filter out non-circles
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                if radius > 5 and radius < 50:  # Check radius range to avoid false positives on large objects
                    # Draw a bounding box around the detected red circle
                    top_left = (int(x - radius), int(y - radius))
                    bottom_right = (int(x + radius), int(y + radius))
                    cv2.rectangle(roi, top_left, bottom_right, (0, 255, 0), 2) # Draws a green rectangle
                    red_light_detected = True
                    break  # Exit after detecting the first valid red circular contour

        # Display the ROI for debugging
        cv2.imshow("Red Light Detection within ROI", roi)
        cv2.waitKey(1)

        return red_light_detected

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()