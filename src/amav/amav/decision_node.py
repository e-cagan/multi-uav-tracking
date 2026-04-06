"""Decision node: per-agent behavior logic based on tracked objects.

Confidence-aware: if confidence drops, command drone to approach.
Publishes AgentStatus for coordinator awareness.
"""

import rclpy
from rclpy.node import Node
from amav_interfaces.msg import TrackedObjectArray, AgentStatus
from amav_interfaces.srv import Handoff


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('confidence_low_threshold', 0.4)
        self.declare_parameter('confidence_critical_threshold', 0.2)
        self.declare_parameter('lost_frames_handoff_trigger', 15)
        self.declare_parameter('input_topic', 'tracked_objects')
        self.declare_parameter('status_topic', 'agent_status')

        self.agent_id = self.get_parameter('agent_id').value

        self.subscription = self.create_subscription(
            TrackedObjectArray,
            self.get_parameter('input_topic').value,
            self._tracked_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            AgentStatus,
            self.get_parameter('status_topic').value,
            10,
        )

        # Service server: coordinator can request this agent to accept a handoff
        self.handoff_srv = self.create_service(
            Handoff,
            f'{self.agent_id}/handoff',
            self._handoff_callback,
        )

        self.get_logger().info(f'[{self.agent_id}] Decision node started')

    def _tracked_callback(self, msg: TrackedObjectArray):
        # TODO: Evaluate tracked objects, decide behavior, publish status
        pass

    def _handoff_callback(self, request, response):
        # TODO: Decide whether to accept handoff based on current state
        response.accepted = False
        response.accepting_agent_id = self.agent_id
        response.reject_reason = 'Not implemented yet'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
