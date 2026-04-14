"""Coordinator node: manages multi-agent task allocation and handoffs.

Subscribes to all agents' AgentStatus, triggers Handoff service
when an agent loses its target or drops below thresholds.
"""

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from amav_interfaces.msg import AgentStatus
from amav_interfaces.srv import Handoff
from geometry_msgs.msg import Point

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator_node')

        self.declare_parameter('agent_ids', ['drone_0', 'drone_1'])
        self.declare_parameter('handoff_confidence_threshold', 0.3)
        self.declare_parameter('handoff_lost_frames_threshold', 10)

        self.agent_ids = self.get_parameter('agent_ids').value
        self.agent_states: dict[str, AgentStatus] = {}
        self.last_handoff_time: dict[str, float] = {}  # For rate limiting

        # Subscribe to each agent's status topic
        self.status_subs = []
        for agent_id in self.agent_ids:
            sub = self.create_subscription(
                AgentStatus,
                f'/{agent_id}/agent_status',
                self._status_callback,
                10,
            )
            self.status_subs.append(sub)

        # Create handoff service clients for each agent
        self.handoff_clients: dict[str, Client] = {}
        for agent_id in self.agent_ids:
            client = self.create_client(Handoff, f'/{agent_id}/handoff')
            self.handoff_clients[agent_id] = client

        self.get_logger().info(f'Coordinator started, managing agents: {self.agent_ids}')

    def _status_callback(self, msg: AgentStatus):
        self.agent_states[msg.agent_id] = msg
        self._evaluate_handoffs()

    def _evaluate_handoffs(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        for failing_id, status in self.agent_states.items():
            # Rate limiting check: Allow 1 handoff attempt per 5 seconds per agent
            if failing_id in self.last_handoff_time and (current_time - self.last_handoff_time[failing_id]) < 5.0:
                continue

            # Check if handoff is required due to low confidence
            low_conf = status.state in [AgentStatus.STATE_TRACKING, AgentStatus.STATE_APPROACHING] and \
                       status.track_confidence < self.get_parameter('handoff_confidence_threshold').value
            
            if low_conf:
                self.get_logger().warn(f'[{failing_id}] needs handoff! Confidence: {status.track_confidence:.2f}')
                self._trigger_handoff(failing_id, status)
                self.last_handoff_time[failing_id] = current_time

    def _trigger_handoff(self, failing_agent: str, status: AgentStatus):
        # Find an available receiver (an agent that is SEARCHING or IDLE)
        best_receiver = None
        for agent_id in self.agent_ids:
            if agent_id == failing_agent: 
                continue
            
            if agent_id in self.agent_states and \
               self.agent_states[agent_id].state in [AgentStatus.STATE_IDLE, AgentStatus.STATE_SEARCHING]:
                best_receiver = agent_id
                break

        if best_receiver:
            self.get_logger().info(f'Requesting handoff: {failing_agent} -> {best_receiver}')
            
            req = Handoff.Request()
            req.requesting_agent_id = failing_agent
            req.target_track_id = status.tracked_object_id
            
            # TODO: Currently, this sends the failing drone's position as the target's position.
            # This is acceptable for now since the drone should be close to the target,
            # but in future milestones, this should be updated to send the actual 
            # estimated 3D world coordinates of the target.
            req.target_last_position = Point(x=status.pose.position.x, 
                                            y=status.pose.position.y, 
                                            z=status.pose.position.z)
            req.reason = Handoff.Request.REASON_LOW_CONFIDENCE

            # Call the service asynchronously
            self.handoff_clients[best_receiver].call_async(req).add_done_callback(self._handle_response)

    def _handle_response(self, future):
        try:
            res = future.result()
            if res.accepted:
                self.get_logger().info(f'Handoff successful! Target taken by {res.accepting_agent_id}')
            else:
                self.get_logger().info(f'Handoff rejected: {res.reject_reason}')
        except Exception as e:
            self.get_logger().error(f'Handoff service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
