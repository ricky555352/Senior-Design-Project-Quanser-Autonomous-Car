#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
import csv  # For saving the data

from pal.products.qcar import QCar, QCarGPS, QCarLidar
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
from std_msgs.msg import Bool

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
        wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]
        wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]

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

        self.dt = 0.01  # Control loop frequency
        self.startDelay = 1.0  # Initial delay
        self.v_ref = 0.2  # Desired velocity
        self.stop_detected = False  # Track if a stop sign is detected
        self.stop_time = None  # Track the time when the stop sign was detected
        self.red_light_detected = False  # Track if red light is detected
        self.debounce_time = 1.0  # Debounce time for red light detection
        self.last_light_change_time = time.time()  # Last time light state changed

        # Speed Controller
        self.speedController = SpeedController(kp=0.3, ki=0.5)
        
        # Waypoints for steering control
        roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        self.waypointSequence = roadmap.generate_path(nodeSequence)

        # Steering Controller
        self.steeringController = SteeringController(waypoints=self.waypointSequence, k=1.5)

        # QCar Interface with initial position and orientation
        self.qcar = QCar(readMode=1, frequency=20)
        self.ekf = QCarEKF(x_0=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
        self.gps = QCarGPS(initialPose=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))

        self.t0 = time.time()
        self.control_timer = self.create_timer(self.dt, self.controlLoop)  # Main control loop

        # Subscriber to stop sign detection status
        self.stop_sign_subscriber = self.create_subscription(
            Bool,
            'sign_status',
            self.stop_sign_callback,
            10
        )
  
        # Subscribe to traffic light status
        self.red_light_subscriber = self.create_subscription(
            Bool,
            'light_status',
            self.red_light_callback,
            10
        )

    def stop_sign_callback(self, msg):
        if msg.data and not self.stop_detected:
            self.get_logger().info("Stop sign detected, initiating stop sequence!")
            self.stop_detected = msg.data  # true
            self.stop_time = time.time()
        elif not msg.data and self.stop_detected:
            self.get_logger().info("Stop sign no longer detected.")
            self.stop_detected = msg.data # false

    def red_light_callback(self, msg):
        # Debounce red light detection
        current_time = time.time()
        if msg.data != self.red_light_detected and current_time - self.last_light_change_time >= self.debounce_time:
            # Update red light detection state only if it's stable
            self.red_light_detected = msg.data
            self.last_light_change_time = current_time
            status = "red light" if msg.data else "green light"
            self.get_logger().info(f"Traffic light status: {status}")

    def controlLoop(self):
        qcar = self.qcar
        gps = self.gps
        ekf = self.ekf

        qcar.read()
        if gps.readGPS():
            y_gps = np.array([gps.position[0], gps.position[1], gps.orientation[2]])
            ekf.update([qcar.motorTach, 0], self.dt, y_gps, qcar.gyroscope[2])
        else:
            ekf.update([qcar.motorTach, 0], self.dt, None, qcar.gyroscope[2])

        x, y, th = ekf.x_hat[0, 0], ekf.x_hat[1, 0], ekf.x_hat[2, 0]
        p = np.array([x, y])
        v = qcar.motorTach

        # Retrieve the current waypoint being tracked
        wp_1 = self.steeringController.wp[:, self.steeringController.wpi % len(self.waypointSequence[0, :])]
        wp_1_x, wp_1_y = wp_1[0], wp_1[1]

        # Log QCar position and waypoint position
        DATA_TO_SAVE.append([x, y, wp_1_x, wp_1_y])
        
        # Handle stopping for red light
        if self.red_light_detected:
            # Stop the car if red light is detected
            u = 0
            delta = 0
            self.qcar.write(u, delta)
            self.get_logger().info("Red light detected: Stopping the vehicle.")
            return  # Exit control loop to ensure the car stays stopped

        # Handle stopping for 3 seconds at the stop sign
        if self.stop_detected:
            u = 0
            delta = 0
            self.timer_start = time.time()
            self.get_logger().info("Waiting at stop sign...")
            self.qcar.write(u, delta)
        else:
            self.timer_end = time.time()
            if (self.timer_end - self.timer_start) >= 3.0:
                self.get_logger().info("Resuming after stop sign!")
                self.stop_detected = False
            return
            
        # Control QCar to follow the waypoints
        target_speed = self.v_ref
        current_speed = v
        u = self.speedController.update(current_speed, target_speed, self.dt)
        delta = self.steeringController.update(p, th, current_speed)
        self.qcar.write(u, delta)
        self.get_logger().info("Following waypoint path.")

    def stop_vehicle(self):
        self.get_logger().info("Stopping the vehicle...")
        self.qcar.write(0, 0)

    def on_shutdown(self):
        """Stop the vehicle and save data before shutting down."""
        self.stop_vehicle()
        self.get_logger().info("Shutting down node...")
        self.save_data()

    def save_data(self):
        with open('Data3.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "wp_1_x", "wp_1_y"])
            writer.writerows(DATA_TO_SAVE)
        self.get_logger().info("Saved coordinates to CSV file")

def main(args=None):
    rclpy.init(args=args)

    # Set initial position and orientation
    initialPosition = [0, 0.13, 0]
    initialOrientation = [0, 0, -1.57]

    # Define the node sequence for waypoints
    nodeSequence = [0, 2, 4, 6, 8, 10, 1]

    # Initialize and run VehicleControlNode
    node = VehicleControlNode(initialPosition=initialPosition, initialOrientation=initialOrientation, nodeSequence=nodeSequence)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down...")
        node.stop_vehicle()
        node.on_shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
