"""Camera node: bridges Gazebo sim camera to ROS2 Image topic.

Subscribes to Gazebo camera sensor via ros_gz_bridge and republishes
as a standard sensor_msgs/Image for downstream nodes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', 'camera/image')

        agent_id = self.get_parameter('agent_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

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

        self.get_logger().info(
            f'[{agent_id}] Camera node started: {input_topic} -> {output_topic}'
        )

    def _image_callback(self, msg: Image):
        # For now, passthrough. Later: resize for latency-aware pipeline.
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
