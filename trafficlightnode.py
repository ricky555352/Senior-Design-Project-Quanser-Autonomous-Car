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

        # Detect traffic light color (check green first, then red)
        detected_green = self.detect_green_light(cv_image)
        #detected_red = self.detect_red_light(cv_image) if not detected_green else False
        detected_red = self.detect_red_light(cv_image)

        # Prepare the message based on detection results
        light_status = Bool()
        if detected_green:
            self.get_logger().info("Green light detected!")
            light_status.data = False  # Go
        elif detected_red:
            self.get_logger().info("Red light detected!")
            light_status.data = True  # Stop
        else:
            self.get_logger().info("No clear traffic light detected.")
            return  # Do not publish if light is unclear

        # Publish the light status
        self.publisher_sign.publish(light_status)

    def detect_red_light(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Adjusted red color range in HSV
        hsv_red_lower1 = (0, 120, 100)
        hsv_red_upper1 = (10, 255, 255)
        hsv_red_lower2 = (160, 120, 100)
        hsv_red_upper2 = (180, 255, 255)

        # Create masks for both red ranges
        mask1 = cv2.inRange(hsv_img, hsv_red_lower1, hsv_red_upper1)
        mask2 = cv2.inRange(hsv_img, hsv_red_lower2, hsv_red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Log the area of red light detection to help adjust threshold
        red_area = np.sum(red_mask)
        self.get_logger().info(f"Red light area: {red_area}")

        # Increase threshold to reduce false positives
        return red_area > 400  # Adjust this threshold to suit your environment

    def detect_green_light(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([55, 100, 100])
        upper_green = np.array([65, 255, 255])
        
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
        
        green_image = cv2.bitwise_and(image, image, mask=green_mask)
        
        #_, thresh = cv2.threshold(green_image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        #contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     if area > 1000 and cv2.arcLength(contour, True) > 20:
        #         x, y, w, h = cv2.boundingRect(contour)
                
    
        # Log the area of green light detection to help adjust threshold
        green_area = np.sum(green_image)
        self.get_logger().info(f"Green light area: {green_area}")

        # Detect green light presence with adjusted threshold
        return green_area > 300  # Adjust this threshold if necessary        
                
                
        # Refine the contour further if needed
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # # Adjusted green color range in HSV based on previous analysis
        # hsv_green_lower = (55, 100, 100) # 108, 99, 62
        # hsv_green_upper = (65, 255, 255)

        # # Create mask for green
        # #green_mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)
        # green_mask = cv2.bitwise_and(hsv_img, hsv_green_lower, hsv_green_upper)

        # # Log the area of green light detection to help adjust threshold
        # green_area = np.sum(green_mask)
        # self.get_logger().info(f"Green light area: {green_area}")

        # # Detect green light presence with adjusted threshold
        # return green_area > 300  # Adjust this threshold if necessary


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
