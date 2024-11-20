#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from pal.products.qcar import QCarCameras

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Create a publisher to publish the front camera image
        self.publisher_image = self.create_publisher(CompressedImage, '/qcar/csi_front', 10)  # Keep this topic consistent

        # Use CvBridge to convert between ROS2 Image messages and OpenCV images
        self.bridge = CvBridge()

        # Set up the QCar cameras
        self.cameras = QCarCameras(
            enableBack=False,  # You can enable other cameras if needed
            enableFront=True,
            enableLeft=False,
            enableRight=False,
        )

        # Get the FPS from parameters or default to 30 FPS
        #self.declare_parameter('fps', 30)
        #self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.fps = 30
        self.dt = 1.0 / self.fps  # Time between frames

        # Set up a timer to process the camera feed at the desired frame rate
        self.timer = self.create_timer(self.dt, self.process_camera_feed)

    def process_camera_feed(self):
        # Read from the front camera
        self.cameras.readAll()
        front_image = self.cameras.csiFront.imageData

        # Check if the front camera image data is valid
        if front_image is not None and front_image.size > 0:
            # Publish the front camera image in compressed format
            compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(front_image, dst_format='jpeg')
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.header.frame_id = 'qcar_front_camera'
            self.publisher_image.publish(compressed_image_msg)

            # Optional: Display the image using OpenCV for testing purposes
            #cv2.imshow("Front Camera", front_image)
            #cv2.waitKey(1)

    def stop_camera(self):
        self.get_logger().info("Shutting down camera...")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera node stopped manually.")
    finally:
        node.stop_camera()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
