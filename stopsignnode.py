#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import cv2
#from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from pal.products.qcar import QCarCameras, QCar, IS_PHYSICAL_QCAR

class StopSignNode(Node):
    def __init__(self):
        super().__init__('stop_sign_node')
        
        self.publisher_sign = self.create_publisher(Bool, 'sign_status', 10)
    #    timer_period = 0.5
    #    self.timer_sign = self.create_timer(timer_period, self.timer_callback_stop_sign)
                
        self.get_logger().info("Stop Sign Node has started.")
        self.subscription = self.create_subscription(
            Image,
            'Raw_Camera_Image',
            self.listener_callback,
            10
        )
        self.img = []

    #def timer_callback_stop_sign(self):
        #msg = Bool()
        #msg.data = self.detect_stop_sign(self, self.img)
        #self.publisher_sign.publish(msg)
        #self.count += 1
        #self.get_logger().info(f"Publishing {msg.data}")
        
    def listener_callback(self, msg):
        self.get_logger().info("Recieved {msg.data}")
        #self.img = msg.data
        
        msg = Bool()
        msg.data = self.detect_stop_sign(self, msg.data)
        self.publisher_sign.publish(msg)
        
        '''
        # Initialize QCar Cameras
        self.cameras = QCarCameras(
            enableBack=False,
            enableFront=True,
            enableLeft=False,
            enableRight=False,
        )

        # Initialize QCar control for stopping and moving forward
        self.qcar = QCar()

        # Frequency of the camera feed processing (30 FPS)
        self.fps = 30
        self.frame_time = 1.0 / self.fps

        # Create a ROS 2 timer to trigger the camera feed processing
        self.timer = self.create_timer(self.frame_time, self.camera_feed)
        '''
    '''
    def camera_feed(self):
        # Capture RGB Image from the QCar's front camera
        self.cameras.readAll()
        imagedata = self.cameras.csiFront.imageData

        if imagedata is not None and imagedata.size > 0:
            # Process the image to detect stop signs
            processed_frame = self.process_frame(imagedata)
            cv2.imshow("Processed Frame", processed_frame)

            # To quit display, press 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Exiting the display.")
                cv2.destroyAllWindows()
    '''
    def detect_stop_sign(self, img):
        # Convert image to HSV color space
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
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over contours
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

            # Check if the contour has 8 vertices (octagon shape) and is sufficiently large
            if len(approx) == 8 and cv2.contourArea(cnt) > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                if 0.8 <= aspect_ratio <= 1.2:  # Check if the bounding box is roughly square
                    # Draw a bounding box around the detected stop sign
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    return True  # Stop sign detected
        return False  # No stop sign detected

    def process_frame(self, img):
        if self.detect_stop_sign(img):
            self.get_logger().info("Stop sign detected! Stopping...")
            self.stop_car()
            time.sleep(3)  # Stop for 3 seconds
            self.get_logger().info("Resuming movement...")
            self.move_car_forward()
        else:
            self.get_logger().info("No stop sign detected. Moving forward.")
            self.move_car_forward()
        return img
    '''
    def stop_car(self):
        self.get_logger().info("Stopping the QCar.")
        # Issue the stop command to the QCar
        self.qcar.stop()

    def move_car_forward(self):
        self.get_logger().info("Moving the QCar forward.")
        # Issue the forward command to the QCar with speed 0.2 m/s
        self.qcar.forward(0.2)
    '''
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python library
    node = StopSignNode()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info("Stop sign node stopped manually.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
