#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

class MainControlNode(Node):

	def __init__(self):
		super().__init__('main_control_node')
		
		self.dt = 0.01  # How fast the car will update the timer
  
		# Publishers for sending speed and steering commands
		self.speed_publisher = self.create_publisher(Vector3Stamped, '/qcar/speed_command', 10)
		self.steering_publisher = self.create_publisher(Vector3Stamped, '/qcar/steering_command', 10)

		# Timers for speed and steering frequencies
		self.speed_timer = self.create_timer(1/100["Speed Controller Frequency"],
												self.speed_publish_commands)
		self.steering_timer = self.create_timer(1/100["Steering Controller Frequency"],
												self.steering_publish_commands)

	def speed_publish_commands(self):
		# Speed command
		speed_msg = Vector3Stamped()
		speed_msg.vector.v_ref = 0.1 # desired speed (in m/s)
		speed_msg.vector.kp = 0
		speed_msg.vector.ki = 0
		self.speed_publisher.publish(speed_msg)

	def steering_publish_commands(self):
		# Steering command
		steering_msg = Vector3Stamped()
		steering_msg.vector.k = 0.1  # Example: desired steering angle (radians)
		self.steering_publisher.publish(steering_msg)

def main(args=None):
	rclpy.init(args=args)
	node = MainControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
