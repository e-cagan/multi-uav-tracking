"""Detector node: runs YOLO inference on incoming images.

Subscribes to camera/image, publishes DetectionArray.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from amav_interfaces.msg import Detection, DetectionArray
from cv_bridge import CvBridge
from ultralytics import YOLO


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
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            DetectionArray,
            output_topic,
            10,
        )

        self.get_logger().info(f'[{agent_id}] Detector node started')
        
        self.model = YOLO(model=self.get_parameter("model_path").value)
        self.cvb = CvBridge()

    def _image_callback(self, msg: Image):
        """
        cv_bridge -> numpy -> YOLO inference -> publish DetectionArray
        """

        det_arr = DetectionArray()

        # Convert ros2 image message to opencv image
        img = self.cvb.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        # Start time for inference diff calc.
        start_time = time.monotonic()
        
        # Take the detection results every frame
        # İpucu: conf parametresini modele doğrudan vermek performansı artırır, 
        # YOLO düşük güvenli kutuları en baştan hesaplamaz/döndürmez.
        results = self.model(img, conf=self.confidence_threshold)
        
        # End time for inference diff calc.
        end_time = time.monotonic()
        
        # Convert the time difference from seconds to miliseconds
        inference_time_ms = (end_time - start_time) * 1000.0

        # Take the boxes
        boxes = results[0].boxes
        
        # Take class names dict (e.g.: {0: 'person', 1: 'bicycle'})
        names_dict = results[0].names

        # Iterate trough bounding boxes
        for box in boxes:
            det_msg = Detection()

            # box.xywh (1, 4) boyutunda bir tensördür. [0] ile içindeki listeye erişiyoruz.
            # center x/y, width, height
            cx, cy, w, h = box.xywh[0].tolist()

            # Convert center x/y to actual x/y (top-left)
            x = cx - w / 2
            y = cy - h / 2

            # Take the box data on that moment
            cls = int(box.cls.item())
            cls_name = names_dict[cls]
            conf = float(box.conf.item())

            # Filter out the confidences based on confidence threshold
            if conf < self.confidence_threshold:
                continue

            # Fill out the detection message fields
            det_msg.header = msg.header
            det_msg.bbox.x = float(x)
            det_msg.bbox.y = float(y)
            det_msg.bbox.width = float(w)
            det_msg.bbox.height = float(h)
            det_msg.class_name = cls_name
            det_msg.class_id = cls
            det_msg.confidence = conf

            det_arr.detections.append(det_msg)

        # Fill out the other fields of detection array
        det_arr.header = msg.header
        det_arr.image_width = img.shape[1]
        det_arr.image_height = img.shape[0]
        det_arr.inference_time_ms = float(inference_time_ms)

        # Finally, publish the message
        self.publisher.publish(det_arr)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
