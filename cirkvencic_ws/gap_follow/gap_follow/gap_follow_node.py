import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header
import numpy as np
from rclpy.node import Node
import math


class GapFollow(Node):
    def __init__(self):
        super().__init__("gap_follow")
        self.get_logger().info("This node just says 'Hello'")
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.preprocess_lidar, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "disparity_vel", 10)
        self.vehicle_speed_subscriber = self.create_subscription(AckermannDrive, "vehicle_speed", self.vehicle_speed_callback, 10)

        self.declare_parameter('range_cap', 1.6)
        self.declare_parameter('vehicle_speed', 1.0)
        self.width_safety = 8.0
        self.vehicle_speed = 1.0 

        self.range_cap = self.get_parameter('range_cap').get_parameter_value().double_value

    def vehicle_speed_callback(self,msg):
        self.vehicle_speed = msg.speed

    def preprocess_lidar(self, data):
        self.scanovi = data.ranges
        max_range = data.ranges[180:900]
        self.ranges = [20.0 if np.isinf(x) else x for x in max_range]

        threshold = 0.1
       


        for i in range(len(self.ranges) - 1):
            diff = self.ranges[i + 1] - self.ranges[i]

            if np.abs(diff) > threshold:
                if diff > 0:
                    vrij = self.ranges[i]
                    width_sin = np.sin(0.25) * vrij + self.width_safety
                    width = math.ceil(0.1/width_sin) 
                    for v in range(i, min(i + width, 1080-i)): 
                        self.ranges[v] = vrij
                    
                    i=i+width
                    continue
                    
                

  
                elif diff < 0:
                    vrij = self.ranges[i]
                    width_sin = np.sin(0.25) * vrij + self.width_safety
                    width = math.ceil(0.1/width_sin)
                    for v in range(i, max(i - width, -1), -1):
                        self.ranges[v] = vrij
                     
        
        self.ranges_ = np.array(self.ranges)
        self.get_logger().info(f"type ranges je: {type(self.ranges_)}")
        self.ranges_[self.ranges_ > self.range_cap] = self.range_cap

        

        index_list = np.where(self.ranges_ == self.range_cap)[0]


        if index_list.size == 0:
            index_list = np.where(self.ranges_ < self.range_cap)[0]
        

        len_index_list = len(index_list) 
        x= len_index_list // 2


        middle_value_index_list = index_list[x] 

       
        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = 'base_link'

       
        drive_msg = AckermannDrive()
        steer = np.clip(data.angle_increment * (((np.array(middle_value_index_list) + 180) - len(self.scanovi)//2)), -0.34, 0.34)
        
        drive_msg.steering_angle = steer 
        drive_msg.speed = self.vehicle_speed
        msg.drive = drive_msg

       
        self.publisher_.publish(msg)


        
     
    



    
def main(args=None):
    rclpy.init(args=args)
    node = GapFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
