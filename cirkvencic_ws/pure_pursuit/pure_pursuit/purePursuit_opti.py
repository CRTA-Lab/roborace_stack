import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from math import sqrt
import tf2_geometry_msgs
import math
from tf2_ros import TransformException, Buffer, TransformListener
import csv

class PurePursuit(Node):
   
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Subscribers and Publishers
        self.pose_sub_ = self.create_subscription(PoseStamped, "/optitrack/f1tenth/pose", self.pose_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/wall_vel", 10)
        
        self.path = []
        self.path_array = None

        # Load path from CSV
        self.load_waypoints_from_csv('/home/f1tenth/recorded_waypoints/raceline_1.csv')

        # Parameters
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('Kpp', 0.25)
        self.declare_parameter('vehicle_speed', 1.3)
        self.declare_parameter('max_steering_angle', 0.44)

        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.steering_gain = self.get_parameter('Kpp').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.vehicle_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        
        # For the tf2 listener
        self.point_in_world = PointStamped()
        self.point_in_world.header.frame_id = 'world'
        self.target_frame = 'f1tenth' # Goal frame for robot's reference
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def load_waypoints_from_csv(self, filename):
        try:
            with open(filename, 'r') as csvfile:
                reader = csv.reader(csvfile)
                self.path = []
                for row in reader:
                    if len(row) >= 2: 
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            self.path.append([x, y])
                        except ValueError:
                            self.get_logger().warn(f"Skipping row due to non-numeric data: {row}")
            self.path_array = np.array(self.path)
            if self.path_array.size == 0:
                self.get_logger().error(f"No valid waypoints loaded from {filename}")
            else:
                self.get_logger().info(f"Loaded {len(self.path)} waypoints from {filename}")
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found: {filename}")
            self.path_array = None
        except Exception as e:
            self.get_logger().error(f"Error loading waypoints: {e}")
            self.path_array = None

    def pose_callback(self, pose_msg):
        robot_x = pose_msg.pose.position.x
        robot_y = pose_msg.pose.position.y
        
        robot_position = np.array([robot_x, robot_y])

        if self.path_array is not None and self.path_array.size > 0:
            vectors_robot_to_point = self.path_array - robot_position
            distances_to_point = np.linalg.norm(vectors_robot_to_point, axis=1)
            closest_point_index = np.argmin(distances_to_point)

            total_points = len(self.path_array)
            target_point = None
            
            # Find the target point that is one lookahead distance from the vehicle
            for counter in range(total_points):
                current_index = (closest_point_index + counter) % total_points 

                point = self.path_array[current_index]
                x = point[0]
                y = point[1]

                distance_to_target = sqrt((x - robot_x)**2 + (y - robot_y)**2)

                if distance_to_target >= self.lookahead_distance:
                    target_point = self.path_array[current_index]
                    break
            
            if target_point is None:
                # If no point is found beyond lookahead, use the furthest point
                target_point = self.path_array[closest_point_index]

            # Transform goal point from world frame to vehicle frame (f1tenth)
            self.point_in_world.point.x = target_point[0]
            self.point_in_world.point.y = target_point[1]
            self.point_in_world.point.z = 0.0 

            try:
                point_in_f1tenth = self.tf_buffer.transform(
                    self.point_in_world,     
                    self.target_frame,       
                    rclpy.duration.Duration(seconds=1.0)
                )
            except TransformException as ex:
                self.get_logger().warn(f"Error transforming the point from 'world' to '{self.target_frame}': {ex}")
                return 


            
            y_target_in_f1tenth = point_in_f1tenth.point.y

            actual_lookahead_distance = sqrt(point_in_f1tenth.point.x**2 + point_in_f1tenth.point.y**2)

            if actual_lookahead_distance < 0.1:
                self.steering_angle = 0.0
            else:
                self.steering_angle = self.steering_gain * ((2 * y_target_in_f1tenth) / (actual_lookahead_distance**2))
            
            self.steering_angle = max(min(self.steering_angle, self.max_steering_angle), -self.max_steering_angle)

     
            msg_drive_stamped = AckermannDriveStamped()
            msg_drive_stamped.header.stamp = self.get_clock().now().to_msg()
            msg_drive_stamped.header.frame_id = 'f1tenth' 

            drive_msg = AckermannDrive()
            drive_msg.steering_angle = self.steering_angle
            drive_msg.speed = self.vehicle_speed
            msg_drive_stamped.drive = drive_msg

            self.publisher_.publish(msg_drive_stamped)
        else:
            self.get_logger().warn("Path not loaded or is empty. Cannot perform Pure Pursuit.")
        

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()