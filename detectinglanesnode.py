#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import signal

from std_msgs.msg import String
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR


class DetectingLanesNode(Node):
    def __init__(self):
        super().__init__('detecting_lanes_node')
        #self.get_logger().info("Camera is live and lane detection is running!")
        self.subscription = self.create_subscription(
            String,
            'Raw_Camera_Image',
            self.listener_callback,
            10
        )
        '''
        # Used to enable killswitch
        global KILL_THREAD
        KILL_THREAD = False

        signal.signal(signal.SIGINT, self.sig_handler)
        '''
        '''
        # Start camera feed in the ROS 2 timer
        self.timer = self.create_timer(1.0 / 30, self.camera_feed)

        self.cameras = QCarCameras(
            enableBack=True,
            enableFront=True,
            enableLeft=True,
            enableRight=True,
        )
        '''
    def listener_callback(self, msg):
        self.get_logger().info("Recieved {msg.data}")

    
    def sig_handler(self, *args):
        global KILL_THREAD
        KILL_THREAD = True

    def roi(self, image, vertices):
        mask = np.zeros_like(image)
        mask_color = 255
        cv2.fillPoly(mask, vertices, mask_color)
        cropped_img = cv2.bitwise_and(image, mask)
        return cropped_img

    def draw_lines(self, image, hough_lines):
        if hough_lines is not None:
            for line in hough_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 5)
        return image

    def process(self, img):
        height = img.shape[0]
        width = img.shape[1]
        
        # Region of Interest (ROI)
        roi_vertices = [
            (0, height),  # bottom left
            (int(1 * width / 6), int(height / 2)),  # top left
            (int(2 * width / 3), int(height / 2)),  # top right
            (width, height)  # bottom right
        ]

        # Convert image to HSV for color masking
        converted = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # White mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        white_mask = cv2.inRange(converted, lower_white, upper_white)

        # Yellow mask
        lower_yellow = np.array([0, 0, 8])
        upper_yellow = np.array([160, 202, 234])
        yellow_mask = cv2.inRange(converted, lower_yellow, upper_yellow)

        # Combine masks for white and yellow lines
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked_img = cv2.bitwise_and(img, img, mask=combined_mask)

        # Convert to grayscale for Canny edge detection
        gray_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.dilate(gray_img, kernel=np.ones((3, 3), np.uint8))
        canny = cv2.Canny(gray_img, 50, 150)

        # Apply region of interest
        roi_img = self.roi(canny, np.array([roi_vertices], np.int32))

        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(roi_img, 1, np.pi / 180, threshold=10, minLineLength=15, maxLineGap=2)
        final_img = self.draw_lines(img, lines)

        # Lane keeping logic
        self.lane_keeping(lines, width)

        return final_img

    def lane_keeping(self, lines, width):
        if lines is not None:
            left_lines = []
            right_lines = []

            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)  # Calculates the slope of the line
                if slope < 0:  # Negative slope gets left lane line
                    left_lines.append(line)
                else:  # Positive slope gets right lane line
                    right_lines.append(line)

            left_x_avg = np.mean([line[0][0] for line in left_lines]) if left_lines else None
            right_x_avg = np.mean([line[0][0] for line in right_lines]) if right_lines else None

            if left_x_avg is not None and right_x_avg is not None:
                lane_center = (left_x_avg + right_x_avg) / 2
                frame_center = width / 2

                if lane_center < frame_center - 10:
                    self.steer_left()
                elif lane_center > frame_center + 10:
                    self.steer_right()
                else:
                    self.go_straight()
            else:
                self.go_straight()
        else:
            self.stop_car()

    def steer_left(self):
        self.get_logger().info("Steering left...")

    def steer_right(self):
        self.get_logger().info("Steering right...")

    def go_straight(self):
        self.get_logger().info("Going straight...")

    def stop_car(self):
        self.get_logger().info("Stopping car...")

    '''
    def camera_feed(self):
        global KILL_THREAD
        if not KILL_THREAD:
            self.cameras.readAll()
            imagedata = self.cameras.csiFront.imageData

            if imagedata is not None and imagedata.size > 0:
                frame = self.process(imagedata)
                cv2.imshow("Processed Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    KILL_THREAD = True

        else:
            cv2.destroyAllWindows()
            self.cameras.close()
        '''

def main(args=None):
    rclpy.init(args=args)
    node = DetectingLanesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
