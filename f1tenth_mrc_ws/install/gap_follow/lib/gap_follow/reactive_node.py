#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/disparity_vel'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.scan_subscription = self.create_subscription(LaserScan,lidarscan_topic,self.lidar_callback,10)
        self.scan_subscription
        self.acker_publisher = self.create_publisher(AckermannDriveStamped,drive_topic, 10)
        
        self.angle_min = 0.
        self.angle_increment = 0.

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = ranges[:1080]
        k_min = 180 #int((-np.pi/2-self.angle_min)/self.angle_increment)
        k_max = 900 #int((np.pi/2-self.angle_min)/self.angle_increment)
        k = k_min
        proc_ranges = ranges
        while k < (k_max-1):
            disp = ranges[k+1]-ranges[k]
            if disp > 0.15:
                p = int((np.arctan(0.28/ranges[k]))/self.angle_increment)
                for i in range(p+1):
                    if (k+i) < len(proc_ranges):
                        if ranges[k+i] > ranges[k]:
                            proc_ranges[k+i] = ranges[k]
                k = k+p
            k = k+1
        for n in range(k_min,k_max-1):
            disp = ranges[n+1]-ranges[n]
            if disp < -0.15:
                p = int((np.arctan(0.28/ranges[n+1]))/self.angle_increment)
                for j in range(p+1):
                    if ranges[n-j] > ranges[n+1] and (n-j) >= 0:
                        proc_ranges[n-j] = ranges[n+1]
        		
        for i in range(len(ranges)):
            if ranges[i] > 5:
                ranges[i] = 5
        	
        		
        proc_ranges = proc_ranges[k_min:k_max]
        return proc_ranges

    def detect_corner(self, ranges):
        k_D = 180 #int((-np.pi/2-self.angle_min)/self.angle_increment)
        k_L = 900 #int((np.pi/2-self.angle_min)/self.angle_increment)
        obstacle = [1,1]
        for k in range(k_D-30,k_D+2):
            if ranges[k] < 0.25:
                obstacle[1] = 0
        for k in range(k_L-2,k_L+30):
            if ranges[k] < 0.25:
                obstacle[0] = 0
        return obstacle
        
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_index = 0
        end_index = 0
        max_start = 0
        max_end = 0
        prolaz = True
        found_gap = False
        for k in range(len(free_space_ranges)-1):
            disp = free_space_ranges[k] - free_space_ranges[k+1]
            if disp > 0.15 and prolaz:
                end_index = k
                prolaz = False
                found_gap = True
        		#if (max_end-max_start) < (end_index-start_index):
        			#max_start = start_index
        			#max_end = end_index
            elif disp < -0.15:
                start_index = k+1
                prolaz = True
                found_gap = True
            if (max_end-max_start) < (end_index-start_index):
                max_start = start_index
                max_end = end_index	
        if not found_gap:
            max_end = k
            max_start = start_index
        if found_gap and prolaz:
            end_index = len(free_space_ranges)-1
            if (max_end-max_start) < (end_index-start_index):
                max_start = start_index
                max_end = end_index
        
        return max_start, max_end
    
    def find_best_point(self, start_i, end_i, ranges, obstacle):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        start_i = 0
        end_i = len(ranges)-1
        if end_i-start_i != 0:
        	#suma = 0.
        	#nazivnik = 0.
            max_value = 0.
            max_i = 0
        	
            for k in range(start_i, end_i+1):
                if ranges[k] > max_value:
                    max_i = k
                    max_value = ranges[k]
        		#suma = suma + k * pow(ranges[k],5)
        		#nazivnik = nazivnik + pow(ranges[k],5)
        	#best_point = suma/nazivnik
            best_point = max_i
            #best_point = (end_i+start_i)/2
            best_angle = best_point * self.angle_increment - np.pi/2
            if best_angle <= -0.34*obstacle[1]:
                best_angle = -0.34*obstacle[1]
            if best_angle >= 0.34*obstacle[0]:
                best_angle = 0.34*obstacle[0]
        else:
            best_angle = 0.
        
        return best_angle

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        #self.get_logger().info('Publishing_ranges: "%s"' % [ranges[180:720]])
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        proc_ranges = self.preprocess_lidar(ranges)
        max_start, max_end = self.find_max_gap(proc_ranges)
        #self.get_logger().info('Publishing_proc_ranges: "%s"' % [proc_ranges])
        #self.get_logger().info('Publishing: "%s"' % [max_start,max_end])
        obstacle = self.detect_corner(ranges)
        #self.get_logger().info('Publishing: "%s"' % [obstacle])
        steering_angle = self.find_best_point(max_start, max_end, proc_ranges, obstacle)
        #self.get_logger().info('Publishing: "%s"' % [steering_angle])
        velocity = -1 * abs(steering_angle) + 2.0
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.acker_publisher.publish(drive_msg)
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

