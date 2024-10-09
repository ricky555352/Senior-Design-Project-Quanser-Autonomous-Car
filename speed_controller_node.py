#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState, Imu
import numpy as np
import time
from pal.products.qcar import QCar

class SpeedControllerNode(Node):

	def __init__(self, kp=0.1, ki=0.5):
		super().__init__('speed_controller_node')

		# Speed Controller Parameters
		self.dt = 0.01  # Update rate
		self.startDelay = 1.0  # Delay before moving
		self.v_ref = 0.2  # Desired velocity in m/s (adjust this if needed)
		self.maxThrottle = 0.2  # Reduce maxThrottle to limit the speed
		self.kp = kp
		self.ki = ki
		self.ei = 0

		self.qcar_qos_profile = QoSProfile(reliability=1, history=1, durability=1, depth=1)
		
		self.speed_subscriber = self.create_subscription(
	  		Vector3Stamped, '/qcar/speed_command', self.speed_callback, 10)

		# Publishers
		self.imuPublisher = self.create_publisher(Imu, '/qcar/imu', self.qcar_qos_profile)
		self.batteryPublisher = self.create_publisher(BatteryState, '/qcar/stateBattery', self.qcar_qos_profile)
		self.carVelocityPublisher = self.create_publisher(Vector3Stamped, '/qcar/velocity', self.qcar_qos_profile)

		# Timers
		self.create_timer(self.dt, self.controlLoop)
		self.create_timer(1/100, self.IMU_callback)
		self.create_timer(1, self.battery_callback)
		self.create_timer(1/10, self.velocity_callback)

		# QCar Initialization
		#self.myCar = QCar(readMode=1)
		self.myCar = QCar(readMode=1, frequency=500)
		self.command = np.array([0, 0])
		self.t0 = time.time()

	def speed_callback(self, msg):
		self.v_ref = msg.vector.x  # Get the desired speed from the master controller
	
	def update(self, v, v_ref, dt):
		e = v_ref - v
		self.ei += dt * e
		throttle = self.kp * e + self.ki * self.ei
		return np.clip(throttle, -0.1, 0.1)  # Tighter limit on throttle to make it slower

	def controlLoop(self):
		v = self.myCar.motorTach
		t = time.time() - self.t0

		if t >= self.startDelay:
			u = self.update(v, self.v_ref, self.dt)
			delta = 0  # Placeholder for steering logic
		else:
			u, delta = 0, 0

		self.myCar.write(u, delta)

	def stop_qcar(self):
		self.myCar.terminate()

def main(args=None):
	rclpy.init(args=args)
	node = SpeedControllerNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.stop_qcar()
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
