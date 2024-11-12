#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
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

    def listener_callback(self, msg):
        # Convert incoming ROS image to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Define a specific ROI (Region of Interest) for close-range detection
        height, width = cv_image.shape[:2]
        roi_x_start = int(width * 0.4)  # Adjust these values to narrow the region
        roi_x_end = int(width * 0.6)
        roi_y_start = int(height * 0.2)
        roi_y_end = int(height * 0.5)
        roi = cv_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

        # Detect traffic light color within the ROI
        detected_red = self.detect_red_light(roi)

        # Publish the light status based on detection
        light_status = Bool()
        light_status.data = detected_red  # True if red light detected, False otherwise
        self.publisher_sign.publish(light_status)

        # Show the ROI with detected bounding box for debugging
        cv2.rectangle(cv_image, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (0, 255, 0), 2)
        cv2.imshow("Traffic Light ROI", cv_image)
        cv2.waitKey(1)

    def detect_red_light(self, roi):
        hsv_img = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define a narrow HSV range for detecting the red traffic light
        hsv_red_lower = (0, 150, 150)
        hsv_red_upper = (10, 255, 255)

        # Create a mask for red light detection within this HSV range
        red_mask = cv2.inRange(hsv_img, hsv_red_lower, hsv_red_upper)

        # Apply Gaussian blur to reduce noise and smooth the edges of the mask
        blurred_mask = cv2.GaussianBlur(red_mask, (9, 9), 2)

        # Define area threshold for red detection
        red_area = 300  

        # Check if the red mask area is above the threshold
        red_area = cv2.countNonZero(blurred_mask)
        if red_area < red_area:
            # Area too small, likely not the traffic light
            return False

        # Detect circles in the blurred mask
        circles = cv2.HoughCircles(
            blurred_mask, 
            cv2.HOUGH_GRADIENT, 
            dp=1.2, 
            minDist=30, 
            param1=100, 
            param2=15, 
            minRadius=5, 
            maxRadius=50
        )

        # Initialize red light detection flag
        red_light_detected = False

        # Process detected circles and draw bounding box if a red light is detected
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, radius) in circles:
                # Draw a bounding box around the detected red circle
                top_left = (x - radius, y - radius)
                bottom_right = (x + radius, y + radius)
                cv2.rectangle(roi, top_left, bottom_right, (0, 0, 255), 2)
                red_light_detected = True

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
