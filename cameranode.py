#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        #    self.publisher_lane = self.create_publisher(String, 'lane_status', 10)

        #    self.publisher_light = self.create_publisher(String, 'light_status', 10)
        
        self.publisher_image = self.create_publisher(Image, 'Raw_Camera_Image', 10)
        
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        #timer_period = 0.5
        
        #self.timer_lane = self.create_timer(timer_period, self.timer_callback_detect_lanes)

        #self.timer_light = self.create_timer(timer_period, self.timer_callback_traffic_light)
        
        self.count = 0

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
    
    '''
    def timer_callback_detect_lanes(self, msg):

        self.count += 1
        self.get_logger().info(f"Publishing {msg.data}")

    def timer_callback_traffic_light(self, msg):
        msg = String()
        msg.data = f"Traffic Light Status: {self.count}"
        self.publisher_light.publish(msg)
        self.count += 1
        self.get_logger().info(f"Publishing {msg.data}")
        
    '''
    def process_camera_feed(self):
        self.cameras.readAll()
        
        # Display the stitched image
        imageWidth = 640
        imageHeight = 480
        
        #Image = self.cameras.csiFront.imageData
        #if imagedata is not None and imagedata.size > 0:
        #    self.process_image(imagedata)
        
        # Stitch images together with black padding
        imageBuffer360 = np.concatenate((self.cameras.csiRight.imageData,
                                         self.cameras.csiBack.imageData,
                                         self.cameras.csiLeft.imageData,
                                         self.cameras.csiFront.imageData),
                                         axis=1)
        
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher_image.publish(self.br.cv2_to_imgmsg(frame))
        
        self.cameras.csiFront.imageData = self.br.imgmsg_to_cv2(Image)
        
        cv2.imshow('Combined View', cv2.resize(imageBuffer360, (int(2*imageWidth), int(imageHeight/2))))
        cv2.imshow("Front Camera", self.cameras.csiFront.imagedata)
        
        # Wait for 1 millisecond
        cv2.waitKey(1)

    
        #msg = Image()
        #msg.data = imagedata
        #msg.width = imageWidth
        #msg.height = imageHeight
        #self.publisher_image.publish(msg)
        '''
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publisher_image.publish(self.br.cv2_to_imgmsg(frame))
            
    def img_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        '''
        
def main(args=None): #Main function
    rclpy.init(args=args) #Allows ROS2 to call script
    
    node = CameraNode() #Node shall equal the class name node
    rclpy.spin(node) #Spin function will force the node to run indefinitely until user forcefully kills node
    
    # Stops node
    node.destroy_node()
    rclpy.shutdown() #Allows ^C to kill node if not ran with lidar or engine

if __name__ == '__main__': #Main function that ROS2 will call as the node
    main()
#endregion 