#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from lidar_object_detection_ros2.msg import Pose2D, Object, ObjectsArray, ScanClusters


import numpy as np
from scipy.spatial.transform import Rotation as R
import tf_transformations as tft
from tf_transformations import quaternion_from_matrix
from tf_transformations import quaternion_matrix, quaternion_from_euler, euler_from_quaternion
from pose_estimation_UKF import UKF
import time
from datetime import datetime
import csv

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class UKF_fusion(Node):
    def __init__(self):
        super().__init__('UKF_fusion_node')

        self.publisher = self.create_publisher(Odometry, '/opp_pose', 10)
        self.lidar_sub = self.create_subscription(ObjectsArray, '/lod_objects', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Float64MultiArray, '/camera_measurements', self.camera_callback, 10)
        self.ground_truth_opp_sub = self.create_subscription(PoseStamped, '/optitrack/astro/pose', self.ground_truth_opp_callback, 10)
        self.ground_truth__ego_sub = self.create_subscription(PoseStamped, '/optitrack/f1tenth/pose', self.ground_truth_ego_callback, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.UKF = UKF()
        self.time = time.time()
        
        self.declare_parameter('csv_file', 'logged_data.csv')
        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value

        # Ensure directory exists
        os.makedirs(os.path.dirname(self.csv_file), exist_ok=True) if '/' in self.csv_file else None
        
        self.header = [
            'flag', 'timestamp',
            'UKF_px', 'UKF_py', 'UKF_pz',
            'UKF_qx', 'UKF_qy', 'UKF_qz', 'UKF_qw',
            'UKF_vx', 'UKF_vy', 'UKF_vz',
            'UKF_wx', 'UKF_wy', 'UKF_wz',
            'rgb', 'depth', 'yaw', 'theta',
            'lidar_x', 'lidar_y',
            'gt_x', 'gt_y', 'gt_yaw'
        ]

        # Open CSV file in append mode
        self.csv_fd = open(self.csv_file, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_fd)
        self.csv_writer.writerow(self.header)
        
        self.ground_truth_opp = self.pose_to_matrix(Pose())
        self.ground_truth_ego = self.pose_to_matrix(Pose())
        
        self.get_logger().info('UKF fusion Node started')
        
    def lidar_callback(self, msg):
        
        min_dist = 5000
        min_id = 0
        x = 0
        y = 0
        
        for object in msg.objects:
            dist = np.sqrt((object.pose.x-self.UKF.x[0])**2 + (object.pose.y-self.UKF.x[1])**2)
            if dist < min_dist :
                min_dist = dist
                min_id = object.id
                x = object.pose.x
                y = object.pose.y
        

                
        
        self.UKF.predict(time.time()-self.time)
        self.time = time.time()
        if min_dist < 0.3:
            self.UKF.update_lidar([x, y])
        
        odom = Odometry()
        
        odom.pose.pose.position.x = self.UKF.x[0]
        odom.pose.pose.position.y = self.UKF.x[1]
        odom.pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.UKF.x[2])
        
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.UKF.x[3]
        odom.twist.twist.linear.y = self.UKF.x[4]
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.UKF.x[5]
        
        tf = TransformStamped()

        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'base_link'          # parent frame
        tf.child_frame_id = 'opp_pose'      # child frame

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z

        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)
        
        self.publisher.publish(odom)
        
        g_truth = self.compute_relative_pose_ego_frame()
        
        row = [
            1,  # flag
            self.timestamp(),

            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,

            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,

            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,

            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
            
            0.0,
            0.0,
            0.0,
            0.0,
            
            x,
            y,
            
            g_truth[0],
            g_truth[1],
            g_truth[2],
        ]

        self.csv_writer.writerow(row)
        self.csv_fd.flush()
            
        
    def camera_callback(self, msg):
        
        
        measurement = msg.data  
        
              
        
        self.UKF.predict(time.time()-self.time)
        self.time = time.time()
        
        if self.UKF.x[2] < 0.0 :
            measurement[2] = -1*measurement[2]
            
        if measurement[1] < 0.3 or (measurement[0]*0.5 > measurement[1] or measurement[0]*1.5 < measurement[1]):
            measurement[1] = measurement[0]
            
        self.UKF.update_camera(measurement)
        
        odom = Odometry()
        
        odom.pose.pose.position.x = self.UKF.x[0]
        odom.pose.pose.position.y = self.UKF.x[1]
        odom.pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.UKF.x[2])
        
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.UKF.x[3]
        odom.twist.twist.linear.y = self.UKF.x[4]
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.UKF.x[5]
        
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'          # parent frame
        tf.child_frame_id = 'opp_pose'      # child frame

        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z

        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

        
        self.publisher.publish(odom)
        
        g_truth = self.compute_relative_pose_ego_frame()
        
        row = [
            2,  # flag
            self.timestamp(),

            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,

            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,

            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,

            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z,
            
            measurement[0],
            measurement[1],
            measurement[2],
            measurement[3],
            
            0.0,
            0.0,
            
            g_truth[0],
            g_truth[1],
            g_truth[2],
        ]

        self.csv_writer.writerow(row)
        self.csv_fd.flush()
        
    def timestamp(self):
        return self.get_clock().now().nanoseconds * 1e-9
        
    def ground_truth_opp_callback(self, msg):
        
        #qx = msg.pose.orientation.x
        #qy = msg.pose.orientation.y
        #qz = msg.pose.orientation.z
        #qw = msg.pose.orientation.w
        
        
        #roll, pitch, gt_yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.ground_truth_opp = self.pose_to_matrix(msg.pose)
        #self.get_logger().info(f'x:{msg.pose.position.z}, y:{msg.pose.position.x}, roll:{roll}, pitch:{pitch}, yaw:{gt_yaw}')
        
    def ground_truth_ego_callback(self, msg):
        
        #qx = msg.pose.orientation.x
        #qy = msg.pose.orientation.y
        #qz = msg.pose.orientation.z
        #qw = msg.pose.orientation.w
        
        #roll, pitch, gt_yaw = euler_from_quaternion([qx, qy, qz, qw])
        self.ground_truth_ego = self.pose_to_matrix(msg.pose)
        #self.get_logger().info(f'x:{msg.pose.position.z}, y:{msg.pose.position.x}, roll:{roll}, pitch:{pitch}, yaw:{gt_yaw}')
        
    def compute_relative_pose_ego_frame(self):
        """
        Compute 2D relative pose (x, y, yaw) of opponent in ego frame.

        Frames:
        - World: X left, Y up, Z straight
        - Robot: X forward, Y left, Z up

        Returns:
            x_rel: forward distance (meters)
            y_rel: left distance (meters)
            yaw_rel: relative yaw (radians)
        """

        # 1. Transform opponent into ego frame
        T_ego_opp = np.dot(
            tft.inverse_matrix(self.ground_truth_ego),
            self.ground_truth_opp
        )

        # 2. Translation: robot moves in X–Z plane
        # Ego frame:
        #   X = forward
        #   Y = left
        #   Z = up
        x_rel = T_ego_opp[0, 3]   # forward
        y_rel = T_ego_opp[1, 3]   # left

        # 3. Extract yaw (rotation about robot Z)
        roll, pitch, yaw = tft.euler_from_matrix(T_ego_opp, axes='sxyz')
        self.get_logger().info(f'x:{x_rel}, y:{y_rel}, roll:{roll}, pitch:{pitch}, yaw:{yaw}')

        return [x_rel, y_rel, yaw]
        
    def pose_to_matrix(self, pose):
        """
        Convert geometry_msgs/Pose to 4x4 homogeneous transform.
        """

        # Quaternion (x, y, z, w)
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

        # Build rotation matrix
        T = tft.quaternion_matrix(q)

        # Insert translation
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z

        return T



def main(args=None):
    rclpy.init(args=args)
    node = UKF_fusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_fd.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
