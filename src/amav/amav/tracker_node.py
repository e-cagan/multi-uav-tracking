"""Tracker node: runs ByteTrack on detections for temporal consistency.

Subscribes to DetectionArray, publishes TrackedObjectArray.
"""

import rclpy
from rclpy.node import Node
from amav_interfaces.msg import DetectionArray, TrackedObjectArray


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('input_topic', 'detections')
        self.declare_parameter('output_topic', 'tracked_objects')
        self.declare_parameter('max_lost_frames', 30)

        agent_id = self.get_parameter('agent_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            DetectionArray,
            input_topic,
            self._detection_callback,
            10,
        )

        self.publisher = self.create_publisher(
            TrackedObjectArray,
            output_topic,
            10,
        )

        self.get_logger().info(f'[{agent_id}] Tracker node started')
        # TODO: Initialize ByteTrack

    def _detection_callback(self, msg: DetectionArray):
        # TODO: Feed detections to ByteTrack, publish TrackedObjectArray
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
