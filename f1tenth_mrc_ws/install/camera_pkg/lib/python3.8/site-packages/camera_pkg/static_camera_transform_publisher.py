
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
import numpy as np


class StaticCameraTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_camera_transform_publisher')

        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 50 mm in Z

        # Identity quaternion (no rotation)
        q = tf_transformations.quaternion_from_euler(-np.pi/2, 0.0, -np.pi/2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform from base_link → camera_link")


def main(args=None):
    rclpy.init(args=args)
    node = StaticCameraTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
