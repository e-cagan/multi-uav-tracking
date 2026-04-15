"""Coordinator node: manages multi-agent task allocation and handoffs.

Subscribes to all agents' AgentStatus, triggers Handoff service
when an agent loses its target or drops below thresholds.
Logs system-wide metrics.
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

        # Declare thresholds and agent configuration parameters
        self.declare_parameter('agent_ids', ['drone_0', 'drone_1'])
        self.declare_parameter('handoff_confidence_threshold', 0.3)
        self.declare_parameter('handoff_lost_frames_threshold', 10)

        self.agent_ids = self.get_parameter('agent_ids').value
        
        # Dictionaries to keep track of current states and prevent handoff spamming
        self.agent_states: dict[str, AgentStatus] = {}
        self.last_handoff_time: dict[str, float] = {}

        self.metrics = {
            'total_handoff_attempts': 0,
            'successful_handoffs': 0,
            'total_handoff_latency': 0.0,
            'tracking_time': 0.0,
            'searching_time': 0.0
        }
        
        # Track when a handoff starts to calculate latency later
        self.active_handoff_starts = {}
        self.last_metrics_print_time = self.get_clock().now().nanoseconds / 1e9

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
        # Update the state of the agent that published this message
        self.agent_states[msg.agent_id] = msg
        
        # Check if any handoffs are required based on the new status
        self._evaluate_handoffs()
        
        # Update and potentially print the system metrics
        self._update_and_print_metrics()

    def _evaluate_handoffs(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        for failing_id, status in self.agent_states.items():
            # Rate limiting check: Allow 1 handoff attempt per 5 seconds per agent
            if failing_id in self.last_handoff_time and (current_time - self.last_handoff_time[failing_id]) < 5.0:
                continue

            # Check if handoff is required due to critically low confidence
            low_conf = status.state in [AgentStatus.STATE_TRACKING, AgentStatus.STATE_APPROACHING] and \
                       status.track_confidence < self.get_parameter('handoff_confidence_threshold').value
            
            # If the condition is met, trigger the handoff protocol
            if low_conf:
                self.get_logger().warn(f'[{failing_id}] needs handoff! Confidence: {status.track_confidence:.2f}')
                self._trigger_handoff(failing_id, status)
                
                # Update the rate limiting timer
                self.last_handoff_time[failing_id] = current_time

    def _trigger_handoff(self, failing_agent: str, status: AgentStatus):
        # Find an available receiver (an agent that is currently SEARCHING or IDLE)
        best_receiver = None
        for agent_id in self.agent_ids:
            if agent_id == failing_agent: 
                continue
            
            if agent_id in self.agent_states and \
               self.agent_states[agent_id].state in [AgentStatus.STATE_IDLE, AgentStatus.STATE_SEARCHING]:
                best_receiver = agent_id
                break

        # If we found an available agent, initiate the request
        if best_receiver:
            self.get_logger().info(f'Requesting handoff: {failing_agent} -> {best_receiver}')
            
            # Record start time for latency calculation
            self.active_handoff_starts[failing_agent] = self.get_clock().now().nanoseconds / 1e9
            self.metrics['total_handoff_attempts'] += 1
            
            # Prepare the Handoff Service Request
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

            # Call async and pass the failing_agent info to the callback via lambda function
            future = self.handoff_clients[best_receiver].call_async(req)
            future.add_done_callback(lambda fut, fa=failing_agent: self._handle_response(fut, fa))

    def _handle_response(self, future, failing_agent):
        try:
            res = future.result()
            if res.accepted:
                # Calculate the time it took to complete the handoff (latency)
                current_time = self.get_clock().now().nanoseconds / 1e9
                if failing_agent in self.active_handoff_starts:  # Fixed typo here
                    latency = current_time - self.active_handoff_starts[failing_agent]
                    self.metrics['total_handoff_latency'] += latency
                    
                    # Remove from active tracking list
                    del self.active_handoff_starts[failing_agent]
                
                # Increment success metric
                self.metrics['successful_handoffs'] += 1
                self.get_logger().info(f'Handoff successful! Target taken by {res.accepting_agent_id}')
            else:
                self.get_logger().info(f'Handoff rejected: {res.reject_reason}')
        except Exception as e:
            self.get_logger().error(f'Handoff service call failed: {e}')

    def _update_and_print_metrics(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Accumulate time spent in different mission states (assuming ~10Hz callback rate -> 0.1s)
        for status in self.agent_states.values():
            if status.state in [AgentStatus.STATE_TRACKING, AgentStatus.STATE_APPROACHING]:
                self.metrics['tracking_time'] += 0.1 
            elif status.state == AgentStatus.STATE_SEARCHING:
                self.metrics['searching_time'] += 0.1

        # Print metrics report every 15 seconds
        if current_time - self.last_metrics_print_time > 15.0:
            avg_handoff = 0.0
            
            # Avoid division by zero
            if self.metrics['successful_handoffs'] > 0:
                avg_handoff = self.metrics['total_handoff_latency'] / self.metrics['successful_handoffs']
            
            # Calculate what percentage of the mission was spent tracking a target
            total_mission_time = self.metrics['tracking_time'] + self.metrics['searching_time']
            track_ratio = (self.metrics['tracking_time'] / total_mission_time * 100) if total_mission_time > 0 else 0

            # Render the report to the console
            print("\n" + "="*40)
            print("=== AMAV SYSTEM METRICS REPORT ===")
            print("="*40)
            print(f"Handoff Attempts   : {self.metrics['total_handoff_attempts']}")
            print(f"Successful Handoffs: {self.metrics['successful_handoffs']}")
            print(f"Avg Handoff Latency: {avg_handoff:.2f} seconds")
            print(f"Tracking Time Ratio: %{track_ratio:.1f} (of mission)")
            print("="*40 + "\n")

            # Reset the timer
            self.last_metrics_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
