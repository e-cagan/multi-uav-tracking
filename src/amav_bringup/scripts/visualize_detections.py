"""
Module for visualizing detections in YOLO inference.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from amav_interfaces.msg import DetectionArray
from cv_bridge import CvBridge


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('camera_topic', 'camera/image')
        self.declare_parameter('detection_topic', 'detections')

        camera_topic = self.get_parameter('camera_topic').value
        detection_topic = self.get_parameter('detection_topic').value

        self.cvb = CvBridge()
        self.latest_image = None

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

        self.get_logger().info(f"Visualizer node started. Listening to '{camera_topic}' and '{detection_topic}'")

    def cam_callback(self, msg: Image):
        # Save the latest OpenCV image
        self.latest_image = self.cvb.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

    def det_callback(self, msg: DetectionArray):
        # Wait if no camera image exists
        if self.latest_image is None:
            return

        # Take the copy of image to protect the original
        display_img = self.latest_image.copy()

        # Iterate trough detections
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

        # Display the image on OpenCV frame
        cv2.imshow("YOLO Detections (Debug)", display_img)
        
        # Wait for 1 ms to update frame (OpenCV rule)
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
