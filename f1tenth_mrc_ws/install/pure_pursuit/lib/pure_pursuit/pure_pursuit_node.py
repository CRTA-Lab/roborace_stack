#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import scipy.interpolate as sp_int
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
import tf_transformations  # Handles quaternion → euler
from visualization_msgs.msg import Marker


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ROS Topics
        odom_topic = '/opti_odom'
        drive_topic = '/disparity_vel'
        self.pose_subscription = self.create_subscription(Odometry, odom_topic, self.pose_callback, 10)
        self.acker_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '/waypoints_marker', 10)
        self.orig_marker_pub = self.create_publisher(Marker, '/original_waypoints', 10)
        

        # Pure Pursuit parameters
        self.L = 1.2  # Lookahead distance
        self.K = 0.5  # Curvature gain

        # Load and interpolate waypoints
        self.waypoints = []
        '''
        with open('/home/mrc/sim_ws/src/f1tenth_lab6_template/waypoints.csv', mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                self.waypoints.append([float(i) for i in row])

        self.original_waypoints = self.waypoints
        self.waypoints = np.transpose(self.waypoints)
        
        tck, _ = sp_int.splprep(self.waypoints, s=0)
        u_fine = np.linspace(0, 1, 100)
        interpolated = sp_int.splev(u_fine, tck)
        self.waypoints = list(zip(interpolated[0], interpolated[1]))  # List of (x, y) tuples
        '''
        #waypoints = np.loadtxt('/home/mrc/sim_ws/src/ppo_racing/ppo_racing/Spielberg_waypoints.csv', delimiter=',')
        #waypoints = np.loadtxt('/home/f1tenth/f1tenth_mrc_ws/src/f1tenth_lab6_template/waypoints_crta.csv', delimiter=',')
        waypoints = np.loadtxt('/home/f1tenth/f1tenth_mrc_ws/raceline_3.csv', delimiter=',')

        # Keep only the first two columns
        waypoints = waypoints[5:, :2]
        #waypoints = waypoints[::5,:]
        self.waypoints = [tuple(row) for row in waypoints]
        self.original_waypoints = []
        
        self.create_timer(1.0, self.publish_all_markers)
        self.get_logger().info("Publishing both interpolated (red) and original (blue) waypoints")

    def pose_callback(self, pose_msg):
        # Get current position
        pos = pose_msg.pose.pose.position
        x, y = pos.x, pos.y

        # Find first waypoint at least L distance away
        

        # Get yaw from quaternion
        ori = pose_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        waypoint = self.find_best_waypoint(x, y, yaw)


        # Transform waypoint into vehicle frame
        dx = waypoint[0] - x
        dy = waypoint[1] - y
        x_car = np.cos(-yaw) * dx - np.sin(-yaw) * dy
        y_car = np.sin(-yaw) * dx + np.cos(-yaw) * dy

        # Compute steering angle
        curvature = 2 * y_car / (self.L ** 2)
        steering_angle = np.clip(self.K * curvature, -0.34, 0.34)
        velocity = 2.0 - 2 * abs(steering_angle)

        # Publish Ackermann drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.acker_publisher.publish(drive_msg)

    def find_best_waypoint(self, x, y, yaw):
        min_diff = float('inf')
        best_wp = self.waypoints[0]
    
        for wp in self.waypoints:
            dx = wp[0] - x
            dy = wp[1] - y
            distance = np.hypot(dx, dy)

            # Check if waypoint is in front of the car
            heading_vec = np.array([np.cos(yaw), np.sin(yaw)])
            wp_vec = np.array([dx, dy])
            dot = np.dot(heading_vec, wp_vec)

            if dot > 0:  # In front
                diff = abs(distance - self.L)
                if diff < min_diff:
                    min_diff = diff
                    best_wp = wp

        return best_wp

        
    def publish_waypoint_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # width of the point
        marker.scale.y = 0.2  # height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for wp in self.waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = 0.0
            marker.points.append(p)


        self.marker_pub.publish(marker)

    def publish_orig_waypoint_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "original_waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # width of the point
        marker.scale.y = 0.2  # height
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        for wp in self.original_waypoints:
            p = Point()
            p.x = wp[0]
            p.y = wp[1]
            p.z = 0.1
            marker.points.append(p)


        self.orig_marker_pub.publish(marker)

    def publish_all_markers(self):
        self.publish_waypoint_markers()         # Interpolated, red
        self.publish_orig_waypoint_markers()

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

