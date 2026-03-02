#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/wall_vel'

        # TODO: create subscribers and publishers
        self.scan_subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.scan_subscription
        self.acker_publisher = self.create_publisher(AckermannDriveStamped, '/wall_vel', 10)
        self.position_publisher = self.create_publisher(Float64, '/y_wall_follow', 10)

        # TODO: set PID gains
        self.Kp = 6.
        self.Kd = 0.1
        self.Ki = 0.01

        # TODO: store history
        self.integral = 0.
        self.prev_error = 0.
        self.error = 0.

        # TODO: store any necessary values you think you'll need
        self.L = 0.5
        self.Ref = 0.5
        self.T = 0.004
        self.angle_min = 0.
        self.angle_increment = 0.
        self.Dt = 0.

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        k = int((angle-self.angle_min)/self.angle_increment)
        range1 = range_data[k]
        while range1 == np.NaN or range1 == np.Inf:
        	k = k+1*np.sign(angle)
        	range1 = range_data[k]
        
        return range1

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        a = self.get_range(range_data, np.pi/4)
        b = self.get_range(range_data, np.pi/2)
        theta = np.pi/4
        Alfa = np.arctan((a*np.cos(theta)-b)/(a*np.sin(theta)))
        Dt = b*np.cos(Alfa)
        self.Dt = Dt
        error = dist-(Dt+self.L*np.sin(Alfa))
        
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # TODO: Use kp, ki & kd to implement a PID controller
        
        angle = -(self.Kp*error + self.Kd/self.T*(error-self.prev_error) + self.Ki*((self.T*self.error)+self.integral))
        if angle < -0.34:
        	angle = -0.34
        elif angle > 0.34:
        	angle = 0.34
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.acker_publisher.publish(drive_msg)
        error_msg = Float64()
        error_msg.data = self.Dt
        self.position_publisher.publish(error_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.error = self.get_error(msg.ranges, self.Ref) # TODO: replace with error calculated by get_error()
        self.integral = self.integral+self.error*self.T
        velocity = 1.8 - 2*np.abs(self.error)# TODO: calculate desired car velocity based on error
        if velocity < 1.0:
        	velocity = 1.0
        self.pid_control(self.error, velocity) # TODO: actuate the car with PID
        self.prev_error = self.error


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
