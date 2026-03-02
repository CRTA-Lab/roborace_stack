#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        
        # --- Configuration ---
        self.speed_threshold = 0.6  # Speed (m/s) at which logging starts
        # Updated log directory path as requested
        self.log_dir = '/home/f1tenth/f1tenth_ws/logs'
        
        # --- State Variables ---
        self.file = None
        self.logging_started = False
        
        # Create directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create ROS subscriber
        self.subscription = self.create_subscription(
            Odometry,
            'opti_odom',  # Change topic if necessary
            self.odom_callback,
            10)
        self.get_logger().info(f"Waiting for vehicle to move (speed > {self.speed_threshold} m/s)...")
        self.get_logger().info(f"Waypoints will be saved to: {self.log_dir}")


    def odom_callback(self, data):
        # Calculate speed from the Odometry message
        speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                  data.twist.twist.linear.y, 
                                  data.twist.twist.linear.z]))

        # If logging hasn't started, check if it should
        if not self.logging_started:
            if speed > self.speed_threshold:
                self.logging_started = True
                
                # Open the file only now
                filepath = os.path.join(self.log_dir, strftime('wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv')
                self.file = open(filepath, 'w')
                
                self.get_logger().info(f'Vehicle is moving! Starting to log waypoints to: {filepath}')
            else:
                # If the vehicle is not moving, do nothing
                return

        # If logging is active, save the waypoint
        # Calculate Euler angles from quaternion
        quaternion = np.array([data.pose.pose.orientation.x, 
                               data.pose.pose.orientation.y, 
                               data.pose.pose.orientation.z, 
                               data.pose.pose.orientation.w])
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Saving data in format: x, y, speed, yaw
        line = f"{data.pose.pose.position.x}, {data.pose.pose.position.y}, {speed}, {yaw}\n"
        self.file.write(line)

    def shutdown_hook(self):
        if self.file:
            self.file.close()
            self.get_logger().info('Waypoint file closed. Goodbye!')

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
    """
    x, y, z, w = quaternion
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    
    waypoints_logger = WaypointsLogger()
    
    # Register the shutdown_hook to be called on exit
    atexit.register(waypoints_logger.shutdown_hook)

    try:
        rclpy.spin(waypoints_logger)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroying the node and shutting down rclpy is not strictly necessary
        # as atexit handles cleanup, but it's good practice.
        waypoints_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()