#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy as np
from pal.products.qcar import QCar
from geometry_msgs.msg import Vector3Stamped
import time

#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar Drive Node

class SpeedControllerNode(Node):

	def __init__(self, kp=0, ki=0):
		super().__init__(
			'speed_controller_node',
			allow_undeclared_parameters=True,
			automatically_declare_parameters_from_overrides=True
		)

		# test out making custom QoS profile:
		self.qcar_qos_profile = QoSProfile(reliability = 1, history = 1, durability = 1, depth = 1)

		# Initialize publisher data
		self.dataIMU 	 	= np.zeros((6,1))
		self.dataBattery 	= 0
		self.linearVelocity = 0

		# Initialize Publishers
		self.carVelocityPublisher = self.create_publisher(Vector3Stamped,
												'/qcar/velocity', 
												self.qcar_qos_profile)

		self.carVelocityTimer 	  = self.create_timer(1/self.config["velocity_publish_frequency"],
												self.velocity_callback)

		# Initialize QCar Subscribers
		self.command   = np.array([0, 0])
		self.motor_cmd = np.array([0, 0])
		self.LEDs      = np.array([0, 0, 0, 0, 0, 0, 1, 1])

		# Subscriber for user throttle and steering commands
		self.commandSubscriber = self.create_subscription(Vector3Stamped,
														'/qcar/user_command', 
														self.process_cmd, 1)

		# Configure QCar properties
		self.taskRate = int(500) #Hz
		self.hardware = 1
		self.readMode = 1

		# Initialize QCar using task based I/O
		self.myCar = QCar( readMode=self.readMode,
							frequency=self.taskRate)

		# Speed Controller Parameters
		self.dt = 0.01  # Update rate
		self.startDelay = 1.0  # Delay before moving
		self.v_ref = 0  # Desired velocity in m/s (adjust this if needed)
		self.maxThrottle = 0.2  # Reduce maxThrottle to limit the speed
		self.kp = kp
		self.ki = ki
		self.ei = 0
  
	def update(self, v, v_ref, dt):
		e = v_ref - v
		self.ei += dt * e
		return np.clip(self.kp * e + self.ki * self.ei, -self.maxThrottle, self.maxThrottle)

	def controlLoop(self):
		v = self.myCar.motorTach
		t0 = self.t0
		t = time.time() - self.t0
		u = 0 # Initializing u 
		delta = 0  # Initializing delta 
		

		if t >= self.startDelay:
			u = self.update(v, self.v_ref, self.dt)
			delta = 0  # Placeholder for steering logic
		else:
			u, delta = 0, 0

		self.myCar.write(u, delta)
  
	def speed_callback(self, msg):
		self.v_ref = msg.vector.v_ref  # Get the desired speed from the master controller
		self.kp = msg.vector.kp
		self.ki = msg.vector.ki

	def stop_qcar(self):
		self.myCar.terminate()
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : main
def main(args=None):
	rclpy.init(args=args)
	r = SpeedControllerNode()

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