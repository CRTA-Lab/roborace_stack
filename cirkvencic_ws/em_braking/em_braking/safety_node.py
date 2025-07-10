import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header
import math






class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")
        self.odom_sub = self.create_subscription(Odometry,"odom", self.callback_odom, 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.callback_scan, 10)
        self.vel_pub = self.create_publisher(AckermannDriveStamped, "aem_vel", 10)
        self.speed = 0
        self.prev_timestamp = None
        self.prev_ranges = None
        
        
        

    def callback_odom(self, msg):
        self.speed = msg.twist.twist.linear.x
        self.get_logger().debug('"speed is: "%f" ' %self.speed)
        

    def callback_scan(self,msg):
        
        self.ranges = np.array(msg.ranges)
        k=0
        min_ttc = 40
        TTC = []
        
        if self.speed > 0.1:
            for i in self.ranges:
                if math.isinf(i) :
                    i = msg.range_max
                
                range_rate =  self.speed * math.cos(msg.angle_min + (k * msg.angle_increment))
                iTTC = i / range_rate
                TTC.append(iTTC)

    
                if iTTC > 0 and iTTC < min_ttc:
                    min_ttc = iTTC
                k=k+1
            self.get_logger().info('"TTC is: "%f" ' %min_ttc)
            

            if min_ttc < 0.4:
            	while(True):
            	    self.publish_drive()
            	    self.get_logger().info('"PREBLIZU SMO: "%f" ' %min_ttc)
            	    
            	   
            else: 
            	self.get_logger().debug('"speed is: "%f" ' %self.speed)

  


    def publish_drive(self):
        msg = AckermannDriveStamped()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  
        msg.header.frame_id = 'base_link'

        drive_msg = AckermannDrive()
        drive_msg.speed = 0.0

        msg.drive = drive_msg

        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
