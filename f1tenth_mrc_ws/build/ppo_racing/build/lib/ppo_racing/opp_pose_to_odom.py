import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import tf_transformations
import math
import time

class PoseToOdom(Node):
    def __init__(self):
        super().__init__('opp_pose_to_odom')

        qos = QoSProfile(depth=10)
        self.pose_sub = self.create_subscription(PoseStamped, '/optitrack/astro/pose', self.pose_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/opp_odom', qos)

        self.last_pose = None
        self.last_time = None

    def pose_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now().to_msg()

        if self.last_pose is not None and self.last_time is not None:
            dt = self.time_diff(self.last_time, current_time)
            if dt == 0:
                return  # avoid division by zero

            dx = msg.pose.position.x - self.last_pose.pose.position.x
            dy = msg.pose.position.y - self.last_pose.pose.position.y
            dz = msg.pose.position.z - self.last_pose.pose.position.z

            vx = dx / dt
            vy = dy / dt
            vz = dz / dt

            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odom.header.frame_id = 'opp_odom'
            odom.child_frame_id = 'astro_base_link'

            odom.pose.pose = msg.pose
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.linear.z = vz

            self.odom_pub.publish(odom)

        self.last_pose = msg
        self.last_time = current_time

    def time_diff(self, t1, t2):
        """Return time difference in seconds between two builtin_interfaces/Time"""
        return (t2.sec - t1.sec) + (t2.nanosec - t1.nanosec) * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
