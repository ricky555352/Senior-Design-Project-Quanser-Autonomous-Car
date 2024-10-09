#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3Stamped
import time

from pal.products.qcar import QCar, QCarGPS
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap

#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar Drive Node

class SteeringControllerNode(Node):

	def __init__(self, waypoints, k=0, cyclic=True):
		super().__init__(
			'steering_controller_node',
			allow_undeclared_parameters=True,
			automatically_declare_parameters_from_overrides=True
		)
  
		self.maxSteeringAngle = np.pi / 6
		self.wp = waypoints
		self.N = len(waypoints[0, :])
		self.wpi = 0
		self.k = k
		self.cyclic = cyclic
		self.p_ref = (0, 0)
		self.th_ref = 0
		self.dt = 0.01  # How fast the car will update the timer

		self.steering_subscriber = self.create_subscription(
      		Vector3Stamped, '/qcar/steering_command', self.steering_callback, 10)

	def update(self, p, th, speed):
		wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]  # First waypoint
		wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]  # Second waypoint
		print(wp_1)
		print(wp_2)

		v = wp_2 - wp_1
		v_mag = np.linalg.norm(v)
		try:
			v_uv = v / v_mag
		except ZeroDivisionError:
			return 0

		tangent = np.arctan2(v_uv[1], v_uv[0])
		s = np.dot(p - wp_1, v_uv)

		if s >= v_mag:
			if self.cyclic or self.wpi < self.N - 2:
				self.wpi += 1

		ep = wp_1 + v_uv * s
		ct = ep - p
		dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

		ect = np.linalg.norm(ct) * np.sign(dir)
		psi = wrap_to_pi(tangent - th)

		self.p_ref = ep
		self.th_ref = tangent

		return np.clip(wrap_to_pi(psi + np.arctan2(self.k * ect, speed)), -self.maxSteeringAngle, self.maxSteeringAngle)

	def track(self, initialPosition, initialOrientation, nodeSequence):
		# Waypoints for steering control with nodeSequence
		roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
		waypointSequence = roadmap.generate_path(nodeSequence)

		# Steering Controller - tuned k value for better steering
		self.steeringcontroller = SteeringControllerNode(waypoints=waypointSequence)

		# Configure QCar properties
		self.taskRate = int(500) #Hz
		self.hardware = 1
		self.readMode = 1

		# Initialize QCar using task based I/O
		self.command = np.array([0, 0])
		self.qcar = QCar( readMode=self.readMode,
							frequency=self.taskRate)
		#self.qcar = QCar(readMode=1, frequency=20)
		self.ekf = QCarEKF(x_0=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
		self.gps = QCarGPS(initialPose=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))

		self.t0 = time.time()
		self.timer = self.create_timer(self.dt, self.controlLoop)
	
	def controlLoop(self):
		qcar = self.qcar
		gps = self.gps
		ekf = self.ekf

		u = 0 # Initializing u 
		delta = 0  # Initializing delta 
		t0 = self.t0

		qcar.read()
		if gps.readGPS():
			y_gps = np.array([gps.position[0], gps.position[1], gps.orientation[2]])
			ekf.update([qcar.motorTach, delta], self.dt, y_gps, qcar.gyroscope[2])
		else:
			ekf.update([qcar.motorTach, delta], self.dt, None, qcar.gyroscope[2])

		x = ekf.x_hat[0, 0]
		y = ekf.x_hat[1, 0]
		th = ekf.x_hat[2, 0]
		p = np.array([x, y])
		v = qcar.motorTach
		t = time.time() - t0

		if t >= self.startDelay:
			u = 0 # Placeholder for speed
			delta = self.steeringcontroller.update(p, th, v)

		qcar.write(u, delta)
	 
	def steering_callback(self, msg):
		self.k = msg.vector.k  # Get the desired steering angle from the master controller
	
	def QCar_Taskbase(self):
		self.myCar.read_write_std(throttle=self.command[0], steering=self.command[1], 
                            LEDs=[0, 0, 0, 0, 0, 0, 1, 1])

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : main
def main(args=None):
	rclpy.init(args=args)

	# Set initial position and orientation directly
	initialPosition = [0, 0.13, 0] # X = X position, Y = Y position, Z = radians
	initialOrientation = [0, 0, -1.57] # Z = the angle in radians

	# Define the node sequence based on the map waypoints
	nodeSequence = [10, 2, 4, 6, 8, 10]  # Right-hand side driving with appropriate waypoints
	
	r = SteeringControllerNode(initialPosition=initialPosition, initialOrientation=initialOrientation, nodeSequence=nodeSequence)
	while True:
		try:
			rclpy.spin_once(r)
			r.QCar_Taskbase()
		except KeyboardInterrupt:

			r.stop_qcar()
			r.destroy_node()
			rclpy.shutdown()
			break
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : run
if __name__ == '__main__':
	main()
#endregion