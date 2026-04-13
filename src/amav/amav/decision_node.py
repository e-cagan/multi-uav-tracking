"""Decision node: per-agent behavior logic based on tracked objects.

Confidence-aware: if confidence drops, command drone to approach.
Publishes AgentStatus for coordinator awareness.
"""

import math
import rclpy
from rclpy.node import Node
from amav_interfaces.msg import TrackedObjectArray, AgentStatus, TrackedObject
from amav_interfaces.srv import Handoff
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Twist


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('agent_id', 'drone_0')
        self.declare_parameter('confidence_low_threshold', 0.4)
        self.declare_parameter('confidence_critical_threshold', 0.2)
        self.declare_parameter('lost_frames_handoff_trigger', 15)
        self.declare_parameter('approach_speed', 2.0)
        self.declare_parameter('search_altitude', 10.0)
        self.declare_parameter('waypoint_reached_threshold', 2.0)
        self.declare_parameter('input_topic', 'tracked_objects')
        self.declare_parameter('status_topic', 'agent_status')

        self.agent_id = self.get_parameter('agent_id').value

        # Subscribers
        self.tracked_sub = self.create_subscription(
            TrackedObjectArray,
            self.get_parameter('input_topic').value,
            self._tracked_callback,
            10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10,
        )

        # Publishers
        self.status_pub = self.create_publisher(
            AgentStatus,
            self.get_parameter('status_topic').value,
            10,
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Service server: coordinator can request this agent to accept a handoff
        self.handoff_srv = self.create_service(
            Handoff,
            f'{self.agent_id}/handoff',
            self._handoff_callback,
        )

        # Service clients
        self.set_mode_cli = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        self.set_mode_req = SetMode().Request()

        self.cmd_bool_cli = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        self.cmd_bool_req = CommandBool().Request()

        # Timer
        self.timer = self.create_timer(timer_period_sec=0.05, callback=self._setpoint_timer_callback)

        # Waypoints
        self.search_waypoints = [
            (0, 0, 10),
            (20, 0, 10),
            (20, 20, 10),
            (0, 20, 10),
            (-20, 20, 10),
            (-20, 0, 10),
        ]
        
        # State variables
        self.state = AgentStatus.STATE_IDLE
        self.current_target_id = -1
        self.current_target_confidence = 0.0
        self.current_pose = None               # Updates from _pose_callback
        self.current_setpoint = PoseStamped()  # Timer publish message
        self.current_waypoint_index = 0
        self.latest_tracker_fps = 0.0
        self.armed = False
        self.offboard_set = False
        self.startup_complete = False
        self.start_time = self.get_clock().now()

        self.get_logger().info(f'[{self.agent_id}] Decision node started')

    # Helper methods
    def _make_pose(self, x, y, z) -> PoseStamped:
        """PoseStamped oluştur"""
        msg = PoseStamped()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        return msg

    def _distance_to_waypoint(self, wp) -> float:
        """Calculates the distance between current pose and waypoint."""

        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - wp[0]
        dy = self.current_pose.position.y - wp[1]
        dz = self.current_pose.position.z - wp[2]
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _publish_agent_status(self, msg_header):
        """AgentStatus doldur ve publish et"""

        msg = AgentStatus()
        msg.header = msg_header
        msg.agent_id = self.agent_id
        msg.pose = self.current_pose
        msg.state = self.state
        msg.tracked_object_id = self.current_target_id
        msg.track_confidence = self.current_target_confidence

        self.status_pub.publish(msg=msg)

    def _startup_sequence(self):
        """OFFBOARD + arm (async service call)"""

        # SetMode - OFFBOARD
        self.set_mode_req.custom_mode = 'OFFBOARD'
        self.set_mode_cli.call_async(self.set_mode_req)

        # Arming
        self.cmd_bool_req.value = True
        self.cmd_bool_cli.call_async(self.cmd_bool_req)

    def _pose_callback(self, msg: PoseStamped):
        # Set current pose using received PoseStamped message
        self.current_pose = msg.pose

    def _setpoint_timer_callback(self):
        # Check the startup is complete or not
        setpoint = PoseStamped()
        if not self.startup_complete:
            # Create a setpoint to publish
            setpoint = self._make_pose(x=0, y=0, z=self.get_parameter('search_altitude').value)
            self.setpoint_pub.publish(setpoint)

            # Track the time and set/arm the drone if desired time passes
            elapsed = self.get_clock().now() - self.start_time
            seconds = elapsed.nanoseconds / 1e9

            if seconds >= 2.0:
                # Call the startup sequence
                self._startup_sequence()
                self.startup_complete = True
            return

        # Publish the current setpoint
        self.setpoint_pub.publish(self.current_setpoint)

    def _tracked_callback(self, msg: TrackedObjectArray):
        # Take tracker FPS
        tracker_fps = msg.tracker_fps

        # Look for the target
        target = None
        for obj in msg.tracked_objects:
            if obj.track_id == self.current_target_id:
                target = obj
                break
        
        # Check for the found target if any
        if target is not None:
            # Take the confidence
            conf = target.confidence

            # Manage the state machine
            # Tracked state
            if target.state == TrackedObject.STATE_TRACKED:
                # State = TRACKING, so we have to hover at current position
                if conf > self.get_parameter('confidence_low_threshold').value:
                    # Hover
                    if self.current_pose:
                        self.current_setpoint = self._make_pose(
                            self.current_pose.position.x, 
                            self.current_pose.position.y, 
                            self.current_pose.position.z
                        )
                # State = APPROACHING, so we have to go forward
                if conf < self.get_parameter('confidence_low_threshold').value:
                    # Forward
                    if self.current_pose:
                        self.current_setpoint = self._make_pose(
                            self.current_pose.position.x + self.get_parameter('approach_speed').value, 
                            self.current_pose.position.y, 
                            self.current_pose.position.z
                        )
            # Lost state
            if target.state == TrackedObject.STATE_LOST:
                # State = SEARCHING, so we drop the target
                if target.frames_since_seen > self.get_parameter('lost_frames_handoff_trigger').value:
                    wp = self.search_waypoints[self.current_waypoint_index]
                    self.current_setpoint = self._make_pose(wp[0], wp[1], wp[2])

                    # Check if we reached the waypoint
                    if self._distance_to_waypoint(wp) < self.get_parameter('waypoint_reached_threshold').value:
                        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.search_waypoints)

            else:
                # State = SEARCHING
                wp = self.search_waypoints[self.current_waypoint_index]
                self.current_setpoint = self._make_pose(wp[0], wp[1], wp[2])

                # Check if we reached the waypoint
                if self._distance_to_waypoint(wp) < self.get_parameter('waypoint_reached_threshold').value:
                    self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.search_waypoints)

            

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
