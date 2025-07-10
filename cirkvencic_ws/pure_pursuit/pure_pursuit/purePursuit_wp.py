import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PointStamped
from math import sqrt
import tf2_geometry_msgs
import math


from tf2_ros import TransformException, Buffer, TransformListener




class PurePursuit(Node):
   
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.path_sub_ = self.create_subscription(Path, "driverPath", self.path_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, "/pf/pose/odom", self.pose_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "wall_vel", 10)
        self.path = []
        self.path_array = None
        self.vehicle_speed_subscriber = self.create_subscription(AckermannDrive, "vehicle_speed", self.vehicle_speed_callback, 10)


        #Parameters
        self.declare_parameter('lookahead_distance', 0.9)
        self.declare_parameter('Kpp', 0.15)
        self.declare_parameter('vehicle_speed', 1.0)
        self.declare_parameter('max_steering_angle', 0.35)

        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        self.steering_gain = self.get_parameter('Kpp').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.vehicle_speed = 1.0

        self.point_in_map = PointStamped()
        self.point_in_map.header.frame_id = 'map'
        self.target_frame = 'laser' 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
    def vehicle_speed_callback(self,msg):
        self.vehicle_speed = msg.speed
        if self.vehicle_speed > 1.4:
            self.steering_gain = 0.25
        elif self.vehicle_speed > 1.8:
            self.steering_gain = 0.4
        elif self.vehicle_speed > 2.0:
            self.steering_gain = 0.5
    
    def path_callback(self,path_msg):
        if not self.path: 
            self.path = [[pose_stamped.pose.position.x, pose_stamped.pose.position.y] for pose_stamped in path_msg.poses]
                
            self.path_array = np.array(self.path) #
            

        else:
            pass




    def pose_callback(self, pose_msg):
        robot_x = pose_msg.pose.pose.position.x
        robot_y = pose_msg.pose.pose.position.y
        
        robot_position = np.array([robot_x, robot_y])

        if self.path_array is not None:
            vectors_robot_to_point = self.path_array - robot_position
            distances_to_point = np.linalg.norm(vectors_robot_to_point, axis=1)
            closest_point_index = np.argmin(distances_to_point)


            total_points = len(self.path_array)
            target_point = None
            for counter in range(total_points):
                current_index = (closest_point_index + counter) % total_points 

                point = self.path_array[current_index]
                x = point[0]
                y = point[1]

                distance_to_target = sqrt((x - robot_x)**2 + (y - robot_y)**2)

                if distance_to_target >= self.lookahead_distance:
                    target_point = self.path_array[current_index]
                    break



            self.point_in_map.point.x = target_point[0] 
            self.point_in_map.point.y = target_point[1]  

            try:
                point_in_base_link = self.tf_buffer.transform(
                    self.point_in_map,       
                    self.target_frame,      
                    rclpy.duration.Duration(seconds=1.0) 
                )

            except TransformException as ex:
                self.get_logger().warn(f"Error transforming the point: {ex}")
            



            self.steering_angle = self.steering_gain * ((2 * point_in_base_link.point.y) / np.power(self.lookahead_distance, 2))
            self.steering_angle = max(min(self.steering_angle, self.max_steering_angle), -self.max_steering_angle)


            msg_drive_stamped = AckermannDriveStamped()
            msg_drive_stamped.header = Header()
            msg_drive_stamped.header.stamp = self.get_clock().now().to_msg()  
            msg_drive_stamped.header.frame_id = 'base_link'

            drive_msg = AckermannDrive()
            drive_msg.steering_angle = self.steering_angle
            drive_msg.speed = self.vehicle_speed 
            msg_drive_stamped.drive = drive_msg

            self.publisher_.publish(msg_drive_stamped)
        

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

