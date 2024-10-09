#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from pal.products.qcar import QCar, QCarGPS
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
from geometry_msgs.msg import Vector3Stamped
import time

class SteeringControllerNode(Node):

	def __init__(self, waypoints, initialPosition, initialOrientation, k=1.5, cyclic=True):
		super().__init__('steering_controller_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

		self.k = k
		self.maxSteeringAngle = np.pi / 6
		self.wp = waypoints
		self.N = len(waypoints[0, :])
		self.wpi = 0
		self.cyclic = cyclic
		self.steering_angle = 0.0  # This will be updated by the master controller
		self.steering_subscriber = self.create_subscription(
      		Vector3Stamped, '/qcar/steering_command', self.steering_callback, 10)

		self.qcar_qos_profile = QoSProfile(reliability=1, history=1, durability=1, depth=1)

		#self.carVelocityPublisher = self.create_publisher(Vector3Stamped, '/qcar/velocity', self.qcar_qos_profile)
		#self.create_timer(1/10, self.velocity_callback)

		self.command = np.array([0, 0])
		self.myCar = QCar(readMode=1, frequency=500)
		self.ekf = QCarEKF(x_0=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
		self.gps = QCarGPS(initialPose=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))

	def update(self, p, th, speed):
		wp_1, wp_2 = self.wp[:, np.mod(self.wpi, self.N-1)], self.wp[:, np.mod(self.wpi+1, self.N-1)]
		v_uv = (wp_2 - wp_1) / np.linalg.norm(wp_2 - wp_1)
		tangent = np.arctan2(v_uv[1], v_uv[0])
		s = np.dot(p - wp_1, v_uv)
		if s >= np.linalg.norm(wp_2 - wp_1):
			if self.cyclic or self.wpi < self.N - 2:
				self.wpi += 1

		ep = wp_1 + v_uv * s
		ect = np.linalg.norm(ep - p) * np.sign(wrap_to_pi(np.arctan2(ep[1] - p[1], ep[0] - p[0]) - tangent))
		psi = wrap_to_pi(tangent - th)

		return np.clip(wrap_to_pi(psi + np.arctan2(self.k * ect, speed)), -self.maxSteeringAngle, self.maxSteeringAngle)

	def steering_callback(self, msg):
		self.steering_angle = msg.vector.y  # Get the desired steering angle from the master controller
	
	def QCar_Taskbase(self):
		self.myCar.read_write_std(throttle=self.command[0], steering=self.command[1], LEDs=[0, 0, 0, 0, 0, 0, 1, 1])

	def stop_qcar(self):
		self.myCar.terminate()

def main(args=None):
	rclpy.init(args=args)

	initialPosition = [0, 0.13, 0]
	initialOrientation = [0, 0, -1.57]
	nodeSequence = [10, 2, 4, 6, 8, 10]

	roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
	waypointSequence = roadmap.generate_path(nodeSequence)

	node = SteeringControllerNode(waypoints=waypointSequence, initialPosition=initialPosition, initialOrientation=initialOrientation)

	try:
		while True:
			rclpy.spin_once(node)
			node.QCar_Taskbase()
	except KeyboardInterrupt:
		node.stop_qcar()
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
