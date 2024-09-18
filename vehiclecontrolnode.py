#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import numpy
import time

from pal.products.qcar import QCar, QCarGPS
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap

DATA_TO_SAVE = [[]]

class SpeedController:
    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3
        self.kp = kp
        self.ki = ki
        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )


class SteeringController:
    def __init__(self, waypoints, k=1.5, cyclic=True):
        self.maxSteeringAngle = np.pi/6
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        #print("--------------")
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)] # First waypoint
        #print (wp_1)
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)] # Second waypoint
        #print (wp_2)
        
        
        #DATA_TO_SAVE.append([time.time, wp_1, wp_2])
        
        
        # create an array 
        #a = numpy.array([[wp_1], 
        #         [wp_2]]) 
  
        # save array into csv file 
        #numpy.savetxt("data3.csv", a,  
        #      delimiter = ",")
        
        #start = time.time()
        #wp_1()
        #wp_2()
        #print ("it took", time.time() - start, "seconds.")
        
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0

        tangent = np.arctan2(v_uv[1], v_uv[0])

        s = np.dot(p-wp_1, v_uv)

        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1

        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)

        self.p_ref = ep
        self.th_ref = tangent

        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)


class VehicleControlNode(Node):
    def __init__(self, initialPosition, initialOrientation, nodeSequence):
        super().__init__('vehiclecontrol_node')
        self.get_logger().info("The car is starting!")

        global KILL_THREAD
        KILL_THREAD = False

        self.dt = 0.01 # How fast the car will update the timer
        self.startDelay = 1.0  # Delay before moving
        self.v_ref = 0.25  # Desired velocity in m/s

        # Speed Controller
        # "kp" is the proportional gain which determins how fast the Qcar will respond to how far away it is from the target
        # "ki" is the integral gain which determines how aggresive the proportional gain will respond
        self.speedController = SpeedController(kp=0.1, ki=0.05)
        
        # Waypoints for steering control with nodeSequence
        roadmap = SDCSRoadMap(leftHandTraffic=False, useSmallMap=True)
        waypointSequence = roadmap.generate_path(nodeSequence)
        
        # Steering Controller - tuned k value for better steering
        self.steeringController = SteeringController(waypoints=waypointSequence, k=2.0)

        # QCar Interface with initial position and orientation
        self.qcar = QCar(readMode=1, frequency=20)
        self.ekf = QCarEKF(x_0=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
        self.gps = QCarGPS(initialPose=np.array([initialPosition[0], initialPosition[1], initialOrientation[2]]))
        
        self.t0 = time.time()
        self.timer = self.create_timer(self.dt, self.controlLoop)
        
    def controlLoop(self):
        qcar = self.qcar
        gps = self.gps
        ekf = self.ekf

        u = 0
        delta = 0  # Initializing delta here
        t0 = self.t0
        
        #with qcar, gps:
        #    t0 = time.time()
        #    t = 0

        #    while not KILL_THREAD:
        qcar.read()
        if gps.readGPS():
            y_gps = np.array([
                gps.position[0],
                gps.position[1],
                gps.orientation[2]
            ])
            ekf.update(
                [qcar.motorTach, delta],
                self.dt,
                y_gps,
                qcar.gyroscope[2],
            )
        else:
            ekf.update(
                [qcar.motorTach, delta],
                self.dt,
                None,
                qcar.gyroscope[2],
            )

        x = ekf.x_hat[0, 0]
        y = ekf.x_hat[1, 0]
        th = ekf.x_hat[2, 0]
        p = np.array([x, y])
        v = qcar.motorTach
        t = time.time() - t0
        
        #print(str(x) + "," + str(y))

        if t >= self.startDelay:
            u = self.speedController.update(v, self.v_ref, self.dt)
            delta = self.steeringController.update(p, th, v)

        qcar.write(u, delta)
        #   time.sleep(self.dt)
        
        
def main(args=None):
    rclpy.init(args=args)

    # Set initial position and orientation directly
    initialPosition = [-1.28205, -0.45991, -0.7330382858376184]
    initialOrientation = [0, 0, -44.7]

    # Define the node sequence based on the map waypoints
    nodeSequence = [10, 2, 4, 8, 10]  # Right-hand side driving with appropriate waypoints

    # Initialize VehicleControlNode with the initial position, orientation, and node sequence
    node = VehicleControlNode(initialPosition=initialPosition, initialOrientation=initialOrientation, nodeSequence=nodeSequence)
    print(initialPosition)
    print(initialOrientation)
    
    # Runs the node
    rclpy.spin(node)
    # Stops the node 
    node.destroy_node()
    # Shuts down the ROS2 play
    rclpy.shutdown()
    numpy.savetxt("data3.csv", DATA_TO_SAVE ,delimiter = ",")

if __name__ == '__main__':
    main()
