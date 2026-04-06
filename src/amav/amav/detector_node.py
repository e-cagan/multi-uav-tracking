"""Detector node: runs YOLO inference on incoming images.

Subscribes to camera/image, publishes DetectionArray.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from amav_interfaces.msg import DetectionArray


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('input_topic', 'camera/image')
        self.declare_parameter('output_topic', 'detections')

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
            DetectionArray,
            output_topic,
            10,
        )

        self.get_logger().info(f'[{agent_id}] Detector node started')
        # TODO: Load YOLO model here

    def _image_callback(self, msg: Image):
        # TODO: cv_bridge -> numpy -> YOLO inference -> publish DetectionArray
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
