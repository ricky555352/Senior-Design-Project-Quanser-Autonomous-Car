#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

import rclpy
from rclpy.node import Node
from pal.products.qcar import QCar, QCarLidar

class HardwareStopNode(Node):
    def __init__(self):
        super().__init__('hardwarestop_node')
        self.get_logger().info("The car has stopped!")
        
        # Initializing QCar and Lidar
        myLidar = QCarLidar()
        myCar = QCar()

        # Terminating DAQs if currently running
        myCar.terminate()
        myLidar.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#endregion

