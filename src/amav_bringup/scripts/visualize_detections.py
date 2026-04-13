"""
Module for visualizing detections in YOLO inference and ByteTrack tracking.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from amav_interfaces.msg import DetectionArray, TrackedObjectArray, TrackedObject, AgentStatus
from cv_bridge import CvBridge


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('detection_topic', 'detections')
        self.declare_parameter('tracking_topic', 'tracked_objects')
        self.declare_parameter('status_topic', 'agent_status')

        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        tracking_topic = self.get_parameter('tracking_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.cvb = CvBridge()
        self.latest_image = None
        self.current_agent_status = None # Store the latest status

        # Define a color palette for different track IDs (BGR format for OpenCV)
        self.COLORS = [
            (255, 0, 0),    # Blue
            (0, 255, 0),    # Green
            (0, 255, 255),  # Yellow
            (255, 0, 255),  # Magenta
            (255, 255, 0),  # Cyan
            (128, 0, 0),    # Dark Blue
            (0, 128, 0),    # Dark Green
            (0, 0, 128),    # Maroon
            (128, 128, 0),  # Teal
            (128, 0, 128),  # Purple
            (0, 128, 128),  # Olive
            (255, 128, 0),  # Light Blue
            (255, 0, 128),  # Purple/Pink
            (0, 255, 128),  # Light Green
            (128, 128, 128) # Gray
        ]

        # State mapping dictionary to convert uint8 to string for visualizer
        self.STATE_NAMES = {
            AgentStatus.STATE_IDLE: "IDLE",
            AgentStatus.STATE_SEARCHING: "SEARCHING",
            AgentStatus.STATE_TRACKING: "TRACKING",
            AgentStatus.STATE_APPROACHING: "APPROACHING",
            AgentStatus.STATE_HANDOFF: "HANDOFF"
        }

        self.cam_sub = self.create_subscription(
            Image,
            camera_topic,
            self.cam_callback,
            10
        )

        self.det_sub = self.create_subscription(
            DetectionArray,
            detection_topic,
            self.det_callback,
            10
        )

        self.track_sub = self.create_subscription(
            TrackedObjectArray,
            tracking_topic,
            self.track_callback,
            10
        )

        # Subscription for AgentStatus
        self.status_sub = self.create_subscription(
            AgentStatus,
            status_topic,
            self.status_callback,
            10
        )

        self.get_logger().info(f"Visualizer node started. Listening to '{camera_topic}', '{detection_topic}', '{tracking_topic}' and '{status_topic}'")

    def cam_callback(self, msg: Image):
        # Save the latest OpenCV image
        self.latest_image = self.cvb.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        
    def status_callback(self, msg: AgentStatus):
        # Update the latest agent status
        self.current_agent_status = msg

    def det_callback(self, msg: DetectionArray):
        # Wait if no camera image exists
        if self.latest_image is None:
            return

        # Take the copy of image to protect the original
        display_img = self.latest_image.copy()

        # Iterate through detections
        for det in msg.detections:
            # Top-left corner
            x1 = int(det.bbox.x)
            y1 = int(det.bbox.y)
            # Right-bottom corner
            x2 = int(det.bbox.x + det.bbox.width)
            y2 = int(det.bbox.y + det.bbox.height)

            # Unify label and confidence for simplicity
            label = f"{det.class_name} {det.confidence:.2f}"

            # Draw bbox
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Display label on the frame
            cv2.putText(display_img, label, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Write inference time on the frame
        info_text = f"Inference Time: {msg.inference_time_ms:.1f} ms"
        cv2.putText(display_img, info_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Display the raw detection image on its own OpenCV window
        cv2.imshow("YOLO Detections (Debug)", display_img)
        cv2.waitKey(1)

    def track_callback(self, msg: TrackedObjectArray):
        # Wait if no camera image exists
        if self.latest_image is None:
            return

        # Take the copy of image to protect the original
        display_img = self.latest_image.copy()

        # Process each tracked object
        for tr_obj in msg.tracked_objects:
            x1 = int(tr_obj.bbox.x)
            y1 = int(tr_obj.bbox.y)
            x2 = int(tr_obj.bbox.x + tr_obj.bbox.width)
            y2 = int(tr_obj.bbox.y + tr_obj.bbox.height)

            # Check if the track is LOST
            is_lost = (tr_obj.state == TrackedObject.STATE_LOST)

            # Assign color: Red if lost, otherwise from palette based on track_id
            if is_lost:
                color = (0, 0, 255)  # Red
                thickness = 2
            else:
                color = self.COLORS[tr_obj.track_id % len(self.COLORS)]
                thickness = 2

            # Draw bbox
            cv2.rectangle(display_img, (x1, y1), (x2, y2), color, thickness)

            # Format label: "person #3 0.87"
            label = f"{tr_obj.class_name} #{tr_obj.track_id} {tr_obj.confidence:.2f}"
            if is_lost:
                label += " (LOST)"

            # Display label above the bbox
            cv2.putText(display_img, label, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Format extra temporal/spatial info
            info_vel = f"Vel: ({tr_obj.velocity_x:.1f}, {tr_obj.velocity_y:.1f})"
            info_age = f"Age: {tr_obj.age_frames} | Lost: {tr_obj.frames_since_seen}"

            # Display extra info below the bbox
            cv2.putText(display_img, info_vel, (x1, y2 + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            cv2.putText(display_img, info_age, (x1, y2 + 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        # Write tracker FPS on the frame
        fps_text = f"Tracker FPS: {msg.tracker_fps:.1f}"
        cv2.putText(display_img, fps_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    
        # Display Agent Status
        if self.current_agent_status:
            status = self.current_agent_status
            state_str = self.STATE_NAMES.get(status.state, "UNKNOWN")
            
            # Format: State: TRACKING | Target: #2193 | Conf: 0.82
            if status.tracked_object_id != -1:
                agent_info = f"State: {state_str} | Target: #{status.tracked_object_id} | Conf: {status.track_confidence:.2f}"
            else:
                agent_info = f"State: {state_str} | Target: None | Conf: 0.00"
                
            # Draw a slight background for readability
            text_size = cv2.getTextSize(agent_info, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(display_img, (10, 45), (10 + text_size[0], 45 - text_size[1] - 5), (0, 0, 0), -1)
            
            cv2.putText(display_img, agent_info, (10, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2) # Yellow color

        # Display the tracking image on a separate OpenCV window
        cv2.imshow("ByteTrack Tracking", display_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy opencv windows while closing
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
