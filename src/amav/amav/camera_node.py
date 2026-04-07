"""Camera node: bridges Gazebo sim camera to ROS2 Image topic.

Subscribes to Gazebo camera sensor via ros_gz_bridge and republishes
as a standard sensor_msgs/Image for downstream nodes.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('input_topic', '/camera')
        self.declare_parameter('output_topic', 'camera/image')
        self.declare_parameter('resolution_scale', 1.0)

        agent_id = self.get_parameter('agent_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.resolution_scale = self.get_parameter('resolution_scale').value

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            10,
        )

        self.publisher = self.create_publisher(
            Image,
            output_topic,
            10,
        )

        self.cvb = CvBridge()
        self.prev_time = self.get_clock().now()
        self.frame_count = 0

        self.get_logger().info(
            f'[{agent_id}] Camera node started: {input_topic} -> {output_topic}'
        )

    def _image_callback(self, msg: Image):
        # Convert ros2 image message to opencv image (numpy array of pixels)
        img = self.cvb.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        # If the scale is less than 1.o, resize the image
        if self.resolution_scale < 1.0:
            img = cv2.resize(img, None, fx=self.resolution_scale, fy=self.resolution_scale, interpolation=cv2.INTER_AREA)

            # Fill the parameters of message to publish
            msg_pub = self.cvb.cv2_to_imgmsg(cvim=img, encoding="bgr8")
            msg_pub.header = msg.header
        else:
            msg_pub = msg

        # Calculate the fps and log it
        current_time = self.get_clock().now()
        self.frame_count += 1

        # Calculate the time difference in seconds (will be converted from nanoseconds)
        time_diff = (current_time - self.prev_time).nanoseconds / 1e9
        
        # Log the FPS every second
        if time_diff >= 1.0:
            fps = self.frame_count / time_diff
            self.get_logger().info(f'FPS: {fps:.2f}')
            
            # Reset the counters
            self.prev_time = current_time
            self.frame_count = 0

        # Finally, publish the message
        self.publisher.publish(msg_pub)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
