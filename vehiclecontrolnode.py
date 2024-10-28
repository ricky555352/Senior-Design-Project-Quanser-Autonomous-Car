#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import csv  # For saving the data

from pal.products.qcar import QCar, QCarGPS
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
from std_msgs.msg import String, Bool

# To store the x, y, and waypoint data
DATA_TO_SAVE = []

class SpeedController:
	def __init__(self, kp=0, ki=0):
		self.maxThrottle = 0.3
		self.kp = kp
		self.ki = ki
		self.ei = 0

	def update(self, v, v_ref, dt):
		e = v_ref - v
		self.ei += dt * e
		return np.clip(self.kp * e + self.ki * self.ei, -self.maxThrottle, self.maxThrottle)


class SteeringController:
	def __init__(self, waypoints, k=0, cyclic=True):
		self.maxSteeringAngle = np.pi / 6
		self.wp = waypoints
		self.N = len(waypoints[0, :])
		self.wpi = 0
		self.k = k
		self.cyclic = cyclic
		self.p_ref = (0, 0)
		self.th_ref = 0

	def update(self, p, th, speed):
		wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]  # First waypoint
		wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]  # Second waypoint

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


class VehicleControlNode(Node):
	def __init__(self, initialPosition, initialOrientation, nodeSequence):
		super().__init__('vehiclecontrol_node')
		self.get_logger().info("The car is starting!")

		self.dt = 0.01  # How fast the car will update the timer
		self.startDelay = 1.0  # Delay before moving
		self.v_ref = 0.2  # Desired velocity in m/s
		self.steer_command = "go_straight"  # Default steering command
		self.stop_detected = False  # Track if a stop sign is detected
		self.stop_time = None  # Track the time when the stop sign was detected
		self.red_light_detected = False  # Track if red light is detected

		# Speed Controller
		self.speedController = SpeedController(kp=0.3, ki=0.5)
		
		# Waypoints for steering control with nodeSequence
		roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
		self.waypointSequence = roadmap.generate_path(nodeSequence)

		# Steering Controller - tuned k value for better steering
		self.steeringController = SteeringController(waypoints=self.waypointSequence, k=1.5)

		# QCar Interface with initial position and orientation
		self.qcar = QCar(readMode=1, frequency=20)
		self.ekf = QCarEKF(x_0=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
		self.gps = QCarGPS(initialPose=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))

		self.t0 = time.time()
		self.timer = self.create_timer(self.dt, self.controlLoop)

		# Subscribe to lane-keeping commands
		self.lane_steering_subscriber = self.create_subscription(
			String,
			'lane_status',
			self.lane_status_callback,
			10
		)
		
		# Subscriber to stop sign detection status
		self.stop_sign_subscriber = self.create_subscription(
			Bool,
			'sign_status',
			self.stop_sign_callback,
			10
		)

		# Subscriber to traffic light status
		self.red_light_subscriber = self.create_subscription(
			Bool,
			'light_status',
			self.red_light_callback,
			10
		)
		
	def lane_status_callback(self, msg):
		self.steer_command = msg.data

	def stop_sign_callback(self, msg):
		if msg.data and not self.stop_detected:
			self.get_logger().info("Stop sign detected, initiating stop sequence!")
			self.stop_detected = True
			self.stop_time = time.time()  # Record the time when the stop sign was detected
		elif not msg.data and self.stop_detected:
			self.get_logger().info("Stop sign no longer detected.")
			self.stop_detected = False  # Allow the vehicle to resume
			
	def red_light_callback(self, msg):
		if msg.data:
			self.get_logger().info("Red light detected, stopping.")
			self.red_light_detected = True
		else:
			self.get_logger().info("Green light detected, resuming.")
			self.red_light_detected = False
		
	def controlLoop(self):
		qcar = self.qcar
		gps = self.gps
		ekf = self.ekf

		u = 0
		delta = 0  # Initializing delta here
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

		# Retrieve the current waypoint being tracked
		wp_1 = self.steeringController.wp[:, self.steeringController.wpi % len(self.waypointSequence[0, :])]
		wp_1_x, wp_1_y = wp_1[0], wp_1[1]

		# Log QCar position and waypoint position for overlap plotting
		DATA_TO_SAVE.append([x, y, wp_1_x, wp_1_y])

		# Implement delay before starting movement
		if time.time() - self.t0 >= self.startDelay:
			if not self.red_light_detected:
				# Adjust speed and steering towards the next waypoint
				u = self.speedController.update(v, self.v_ref, self.dt)
				delta = self.steeringController.update(p, th, v)
				self.get_logger().info("Moving towards waypoint")
			else:
				# Stop the vehicle when red light is detected
				u = 0
				self.get_logger().info("Vehicle stopped due to red light.")

		# If a stop sign was detected, stop the vehicle for 3 seconds but continue to update steering
		if self.stop_detected:
			u = 0
			self.get_logger().info(f"Stopped. Time since stop: {time.time() - self.stop_time:.2f} seconds.")
			if time.time() - self.stop_time >= 3.0:
				self.get_logger().info("Resuming after stop sign!")
				self.stop_detected = False  # Allow the vehicle to resume
		'''
		# If a red light is detected, stop the vehicle; otherwise, control speed and steering
		if self.red_light_detected:
			u = 0  # Stop the car
			self.get_logger().info("Vehicle stopped due to red light.")
		else:
			self.get_logger().info("Moving towards waypoint")
			self.red_light_detected = False
			#u = self.speedController.update(v, self.v_ref, self.dt)
			#delta = self.steeringController.update(p, th, v)
			'''
		qcar.write(u, delta)

	def stop_vehicle(self):
		self.get_logger().info("Stopping the vehicle...")
		self.qcar.write(0.0, 0.0)

	def on_shutdown(self):
		"""Called when shutting down the node, ensuring the car stops first."""
		self.stop_vehicle()
		self.get_logger().info("Shutting down node...")
		self.save_data()

	# Save the collected x, y, wp_1_x, wp_1_y data to a CSV file
	def save_data(self):
		with open('Data3.csv', mode='w', newline='') as file:
			writer = csv.writer(file)
			writer.writerow(["x", "y", "wp_1_x", "wp_1_y"])
			writer.writerows(DATA_TO_SAVE)
		self.get_logger().info("Saved coordinates to CSV file")

def main(args=None):
	rclpy.init(args=args)

	# Set initial position and orientation directly
	initialPosition = [0, 0.13, 0]  # X = X position, Y = Y position, Z = radians
	initialOrientation = [0, 0, -1.57]  # Z = the angle in radians

	# Define the node sequence based on the map waypoints
	nodeSequence = [0, 2, 4, 6, 8, 10, 1]  # Right-hand side driving with appropriate waypoints

	# Initialize VehicleControlNode with the initial position, orientation, and node sequence
	node = VehicleControlNode(initialPosition=initialPosition, initialOrientation=initialOrientation, nodeSequence=nodeSequence)

	try:
		# Runs the node, allowing spin_once to check for events every update
		while rclpy.ok():
			rclpy.spin_once(node, timeout_sec=0.1)
	except KeyboardInterrupt:
		# "Gracefully" shut down when Ctrl+C is pressed
		node.get_logger().info("Keyboard interrupt detected, shutting down...")
		node.stop_vehicle()
		node.on_shutdown()

	# Stop the node
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
