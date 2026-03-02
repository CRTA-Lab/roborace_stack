#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.
        # TODO: create ROS subscribers and publisher
        self.scan_subscription = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        self.scan_subscription
        self.odom_subscription = self.create_subscription(Odometry,'ego_racecar/odom',self.odom_callback,10)
        self.odom_subscription
        self.acker_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        TTC = []
        scan_list = scan_msg.ranges
        for k in range(len(scan_list)):
            if scan_list[k] < scan_msg.range_min or scan_list[k] > scan_msg.range_max:
                scan_list[k] = scan_msg.range_max
            TTC = TTC + [scan_msg.ranges[k]/max(self.speed*np.cos(scan_msg.angle_min+k*scan_msg.angle_increment),0.0000001)]
        min_TTC = min(TTC)
        if min_TTC <= 4:
            self.speed = 0.
            msg = AckermannDriveStamped()
            msg.drive.speed = self.speed
            # TODO: publish command to brake
            self.acker_publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % TTC)
        
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
