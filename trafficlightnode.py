#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import signal
import numpy as np
import time
import cv2

from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR

# Used to enable killswitch 
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('green_light_node')
        self.get_logger().info("Green Light Detection Node Started")

        self.declare_parameter('fps', 30)
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.dt = 1.0 / self.fps

        self.cameras = QCarCameras(
            enableBack=True,
            enableFront=True,
            enableLeft=True,
            enableRight=True,
        )

        # Set up a timer to process the camera feed at the desired frame rate
        self.timer = self.create_timer(self.dt, self.process_camera_feed)

    def process_camera_feed(self):
        self.cameras.readAll()
        
        imagedata = self.cameras.csiFront.imageData
        if imagedata is not None and imagedata.size > 0:
            self.process_image(imagedata)
        
        # Stitch images together with black padding
        imageBuffer360 = np.concatenate((self.cameras.csiRight.imageData,
                                         self.cameras.csiBack.imageData,
                                         self.cameras.csiLeft.imageData,
                                         self.cameras.csiFront.imageData),
                                         axis=1)
        
        # Display the stitched image
        imageWidth = 640
        imageHeight = 480
        
        cv2.imshow('Combined View', cv2.resize(imageBuffer360, (int(2*imageWidth), int(imageHeight/2))))
        cv2.imshow("frame", self.cameras.csiFront.imageData)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def process_image(self, img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Lower and upper range of green stoplight color in HSV
        hsv_green_lower = (50, 160, 200)
        hsv_green_upper = (70, 255, 255)

        # Mask with the range of greens for stoplight color
        mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)

        # Only show the pixels that contain the stoplight color
        color_image = cv2.bitwise_and(img, img, mask=mask)

        # Check if the average of the green pixels is above a threshold
        if np.average(color_image) >= 0.0004:
            self.get_logger().info("Light is green, go!")
            self.move_car_forward()
        else:
            self.get_logger().info("Light is red, stop!")
            self.stop_car()

    def move_car_forward(self):
        # Placeholder for car movement logic
        # Replace with actual command to move the QCar forward
        self.get_logger().info("Moving car forward...")

    def stop_car(self):
        # Placeholder for car stop logic
        # Replace with actual command to stop the QCar
        self.get_logger().info("Stopping car...")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
