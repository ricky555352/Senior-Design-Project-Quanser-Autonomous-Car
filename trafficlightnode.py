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
        self.publisher_sign = self.create_publisher(Bool, 'light_status', 1)  
        # The number 1 at the end means number of screen caps it goes through (buffer)
        # The number of images it stores
        
        # Subscribe to the camera's image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/qcar/csi_front',
            self.listener_callback,
            1
        )

    def listener_callback(self, msg):
        # Convert incoming ROS image to OpenCV format
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect traffic light color (check green first, then red)
        #detected_green = self.detect_green_light(cv_image)
        #detected_red = self.detect_red_light(cv_image) if not detected_green else False
        detected_red = self.detect_red_light(cv_image)

        # Display the message based on detection results
        light_status = Bool()
        #light_status.data = False #go by default
        #if detected_green:
        #    self.get_logger().info("Green light detected!")
        light_status.data = False  # Go
        if detected_red:
            #self.get_logger().info("Red light detected!")
            light_status.data = True  # Stop
        else:
            #self.get_logger().info("Green light detected.")
            light_status.data = False  # Go
            #return  # Do not publish if light is unclear

        # Publish the light status
        self.publisher_sign.publish(light_status)

    def detect_red_light(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Adjusted red color range in HSV
        #hsv_red_lower1 = (0, 120, 100)
        #hsv_red_upper1 = (10, 255, 255)
        #hsv_red_lower2 = (160, 120, 100)
        #hsv_red_upper2 = (180, 255, 255)
        #--------------------
        #hsv_red_lower1 = np.array([0, 70, 50])
        #hsv_red_upper1 = np.array([10, 255, 255])
        #hsv_red_lower2 = np.array([170, 70, 50])
        #hsv_red_upper2 = np.array([180, 255, 255])
        # Define a single range for bright red
        hsv_red_lower = (0,203,100)
        hsv_red_upper = (179,255,255)


        # Create masks for both red ranges
        #mask1 = cv2.inRange(hsv_img, hsv_red_lower1, hsv_red_upper1)
        #mask2 = cv2.inRange(hsv_img, hsv_red_lower2, hsv_red_upper2)
        red_mask = cv2.inRange(hsv_img, hsv_red_lower, hsv_red_upper)
        #red_img = cv2.bitwise_and(img, img, mask=red_mask)

        # Log the area of red light detection to help adjust threshold
        red_area = np.sum(red_mask)
        #self.get_logger().info(f"Red light area: {red_area}")

        # Increase threshold to reduce false positives
        return red_area > 1000  # Adjust this threshold to suit your environment

 
def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
