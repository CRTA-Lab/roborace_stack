import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations as tf
from os.path import expanduser, join, exists
from time import strftime, gmtime
import os
import time

# Setup file path
home = expanduser('~')
folder = join(home, 'recorded_waypoints')
os.makedirs(folder, exist_ok=True)

base_name = 'wp-' + strftime('%Y-%m-%d', gmtime())
filename = base_name + '.csv'
filepath = join(folder, filename)

version = 1
while exists(filepath):
    filename = f"{base_name}-v{version}.csv"
    filepath = join(folder, filename)
    version += 1

file = open(filepath, 'w')

class Logger(Node):
    def __init__(self):
        super().__init__("waypoint_logger")
        self.odom_sub = self.create_subscription(Odometry, "/pf/pose/odom", self.waypoint_saver, 10) #define topic
        self.last_saved_time = 0.0  # time in seconds
        self.save_interval = 0.15
        self.get_logger().info("Starting")

    def waypoint_saver(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time < self.save_interval:
            return  # Not enough time has passed; skip saving

        quaternion = np.array([
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        euler = tf.euler_from_quaternion(quaternion)

        if msg.twist.twist.linear.x > 0.2:
            self.save_interval = 1/ (msg.twist.twist.linear.x *4)
            file.write('%f, %f, %f\n' % (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                euler[2]
            ))
        self.last_saved_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = Logger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down waypoint logger...")
    finally:
        file.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f"File saved to: {filepath}")

if __name__ == "__main__":
    main()
