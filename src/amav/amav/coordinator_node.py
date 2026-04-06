"""Coordinator node: manages multi-agent task allocation and handoffs.

Subscribes to all agents' AgentStatus, triggers Handoff service
when an agent loses its target or drops below thresholds.
"""

import rclpy
from rclpy.node import Node
from amav_interfaces.msg import AgentStatus
from amav_interfaces.srv import Handoff


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        self.declare_parameter('agent_ids', ['drone_0', 'drone_1'])
        self.declare_parameter('handoff_confidence_threshold', 0.3)
        self.declare_parameter('handoff_lost_frames_threshold', 10)

        self.agent_ids = self.get_parameter('agent_ids').value
        self.agent_states: dict[str, AgentStatus] = {}

        # Subscribe to each agent's status
        self.status_subs = []
        for agent_id in self.agent_ids:
            sub = self.create_subscription(
                AgentStatus,
                f'{agent_id}/agent_status',
                self._status_callback,
                10,
            )
            self.status_subs.append(sub)

        # Handoff service clients (one per agent)
        self.handoff_clients: dict[str, rclpy.client.Client] = {}
        for agent_id in self.agent_ids:
            client = self.create_client(Handoff, f'{agent_id}/handoff')
            self.handoff_clients[agent_id] = client

        self.get_logger().info(
            f'Coordinator started, managing agents: {self.agent_ids}'
        )

    def _status_callback(self, msg: AgentStatus):
        self.agent_states[msg.agent_id] = msg
        # TODO: Evaluate if handoff is needed
        self._evaluate_handoffs()

    def _evaluate_handoffs(self):
        # TODO: Check if any agent needs to hand off its target
        pass


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
