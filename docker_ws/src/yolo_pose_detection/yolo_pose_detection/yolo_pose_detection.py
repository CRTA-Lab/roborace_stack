import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray


class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')

       # Subscribers
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        # Synchronize the topics outputs
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.process_frames)
        

        # Load the YOLOv8 Pose model
        
        self.model = YOLO('astro_detection.pt')

        # Set up the publisher for the output values
        self.pub_ = self.create_publisher(Float64MultiArray, '/camera_measurements', 10)
        self.img_pub = self.create_publisher(Image, '/slika', 10)

        # Initialize CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        
        self.target_class_id = 0
        
        self.real_width = 36.0
        self.real_height = 15.5
        self.real_length = 32.0
        
        
        self.r_min = self.real_width/self.real_height
        self.r_max = 39.0/self.real_height
        self.last_pos = None

    

    def process_frames(self,rgb_msg, depth_msg):

        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8') 
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')   
        
        if rgb_msg.height == depth_msg.height and rgb_msg.width == depth_msg.width:
            sizes_same = 1
        else:
            sizes_same = 0

        # Run the YOLOv11 pose model
        results = self.model(rgb_image, verbose=False)

        # Annotate the frame with results
        annotated_frame = results[0].plot()

        # Optionally resize the annotated frame
        annotated_frame = cv2.resize(annotated_frame, (annotated_frame.shape[1]*1, annotated_frame.shape[0]*1))

        # Convert the annotated frame to a ROS image message
        ros_image = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        # Publish the image
        self.img_pub.publish(ros_image)

        # Display the image (optional, for local debugging)
        #cv2.imshow("YOLOv11 Pose + RealSense", annotated_frame)
        #cv2.waitKey(1)
        
        cx, cy, w, h, conf = self.get_best_bbox(results)
        
        if conf > 0.1 and sizes_same:
            yaw_bbox = self.get_orientation_from_bbox(w,h)
            rgb_dist = self.estimate_rgb_dist_from_bbox(h)
            depth_dist = self.get_depth_distance(depth_image, cx, cy, sizes_same)
            theta = self.get_theta(cx, depth_msg.width)
            #dist = self.dist_fusion(rgb_dist, depth_dist)
            #pos = self.get_pos_from_dist(img, dist, cx, cy)
            #yaw_vel = self.get_orientation_from_movement_direction(pos)
            #yaw = self.yaw_fusion(yaw_bbox, yaw_vel)
            self.get_logger().info(f"yaw:{yaw_bbox},rgb:{rgb_dist},depth:{depth_dist},theta:{theta}")
            #self.get_logger().info(f"w,h = {w} | {h}")
            self.publish_data(yaw_bbox, rgb_dist, depth_dist, theta)
            
	    
    def get_best_bbox(self, results):
        """
        Extract the highest-confidence bounding box for a specific class.

        Args:
        results (list): Ultralytics YOLO results list (from model(...))
        target_class (int or str): Class ID (int) or class name (str)

        Returns:
        tuple or None:
            (cx, cy, w, h, confidence) in pixel coordinates,
            or None if no detection of the class exists.
        """
        best_conf = 1.0
        cx, cy, w, h = 0.0, 0.0, 0.0, 0.0
        try:
            boxes=results[0].boxes.xywh.cpu()
            #track_ids=results.boxes.id.int().cpu().tolist()
        
            cx, cy, w, h = boxes[0]
            
        except:
            best_conf = 0.0

        '''for result in results:
            boxes = result.boxes

            for box in boxes:
                cls_id = int(box.cls.item())

                # Match by class ID or class name
                if cls_id == self.target_class_id:
                    conf = box.conf.item()

                    if conf > best_conf:
                        cx, cy, w, h = box.xywh
                        self.get_logger().info(f"yaw:{yaw_bbox},rgb:{rgb_dist},depth:{depth_dist}")
                        #x1, y1, x2, y2 = box.xyxy[0].tolist()
                        #w = abs(x2 - x1)
                        #h = abs(y2 - y1)
                        #cx = x1 + w / 2.0
                        #cy = y1 + h / 2.0

                        best_conf = conf'''
                    

        return cx, cy, w, h, best_conf

        
    def get_orientation_from_bbox(self, w, h):
    
        """
        Calculating yaw angle from the bounding box width.
        Using w/h ratio because of distance invariance.
        Needs left/right checking because the output is symmetrical 
        so the actual direction is not known just the value is known.
        """
        
        ratio = w/h
        self.get_logger().info(f'yaw_ratio:{ratio}, r_max:{self.r_max}, r_min:{self.r_min}')
        ratio_n = (ratio-self.r_min)/(self.r_max-self.r_min)
        yaw = ratio_n * np.arctan(self.real_width/self.real_height)
        
        self.r_min = min(self.r_min, w/h)
        self.r_max = max(self.r_max, w/h)

        
        return yaw
        
    def estimate_rgb_dist_from_bbox(self, h):
        """
        calculating distance from the bbox, using only 
        hight because width changes with rotation. Assuming height stays
        the same no matter the rotation, just changes based on distance
        from the car.
        Potential improvement is to use both width amd height and
        compensate for the rotation.
        """
        
        dist = (6000/(h-38.5)) + 55
        
        return dist/100
        
    def get_depth_distance(self, img, cx, cy, sizes_same):
        """
        calculating distance from depth data at the center of the bbox,
        potential improvement to use more points for better estimation.
        """
        if sizes_same  == 1:
            depth_dist = img[int(cy), int(cx)]
        else:
            #depth_dist = float('nan')
            depth_dist = 0.0
        
        return depth_dist/1000
        
    def get_pos_from_dist(self, img, dist, cx, cy):
        """
        converting distance in the image to the real world coordinates
        """
        
        a = size(img,0)/2 - cx
        alpha = a/(size(img,0)/2) * 0.602
        x = np.cos(alpha) * dist
        y = np.sin(alpha) * dist
        z = 0
        
        return [x, y, z]
        
    def get_orientation_from_movement_direction(self, pos):
        """
        since the car is ackermann kinematics we can estimate yaw from
        the direction of travel
        """
        
        yaw = np.arctan((pos[1]-self.last_pos[1])/(pos[0] - self.last_pos[0]))
        
        self.last_pos = pos
        
        return yaw
        
    def get_theta(self, cx, img_width):
        
        cxw = img_width/2
    
        theta = ((cx-cxw)/img_width) * 1.2042
        
        return -theta
        
    def publish_data(self, yaw_bbox, rgb_dist, depth_dist, theta):
        """
        publish the data 
        """
        data = Float64MultiArray()
        
        data.data = [
            float(rgb_dist),
            float(depth_dist),
            float(yaw_bbox),
            float(theta)
        ]

        self.pub_.publish(data)
        
        
    def dist_fusion(self, rgb_dist, depth_dist):
        
        dist = (rgb_dist + depth_dist)/2
        
        return dist
        
    def yaw_fusion(self, yaw_bbox, yaw_vel):
        
        yaw = (yaw_bbox + yaw_vel)/2
        
        return yaw
        

def main(args=None):
    #ROS initialization
    rclpy.init(args=args)
    #Creation of a node
    realsense_node = PoseDetectionNode()
    rclpy.spin(realsense_node)
    #Destory the node
    realsense_node.destroy_node()
    #ROS shutdown
    rclpy.shutdown()

  
if __name__ == '__main__':
    main()
