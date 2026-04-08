"""Tracker node: runs ByteTrack on detections for temporal consistency.

Subscribes to DetectionArray, publishes TrackedObjectArray.
"""

import time
import numpy as np
import supervision as sv
import rclpy
from rclpy.node import Node
from amav_interfaces.msg import DetectionArray, TrackedObjectArray, TrackedObject


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

        self.tracker = sv.ByteTrack()
        self.track_history = dict()
        self.frames_since_seen = 0
        self.max_lost_frames = 0
        self.frame_count = 0

        self.get_logger().info(f'[{agent_id}] Tracker node started')

    def _detection_callback(self, msg: DetectionArray):
        # Feed detections to ByteTrack, publish TrackedObjectArray
        tracked_array = TrackedObjectArray()
        xyxy_list = []
        conf_list = []
        class_ids = []
        class_names = []

        # Add fields of detections
        for det in msg.detections:
            x1 = det.bbox.x
            y1 = det.bbox.y
            x2 = det.bbox.x + det.bbox.width
            y2 = det.bbox.y + det.bbox.height
            xyxy_list.append([x1, y1, x2, y2])
            conf_list.append(det.confidence)
            class_ids.append(det.class_id)
            class_names.append(det.class_name)
        
        
        # Convert them to numpy array
        xyxy = np.array(xyxy_list, dtype=np.float32)        # (N, 4)
        confidence = np.array(conf_list, dtype=np.float32)  # (N,)

        # Adjust tracker with handling no detections edge case
        if len(msg.detections) == 0:
            sv_dets = sv.Detections().empty()
        else:    
            sv_dets = sv.Detections(xyxy=xyxy, confidence=confidence)
            sv_dets.class_id = np.array(class_ids, dtype=int)
            tracked = self.tracker.update_with_detections(detections=sv_dets)

            # Seen track ids in frame
            seen_ids = set()

            # Iterate trough tracked object and unpack the fields of it
            for i in range(len(tracked)):
                track_id = tracked.tracker_id[i]
                tx1, ty1, tx2, ty2 = tracked.xyxy[i]
                tconf = tracked.confidence[i]
                class_id = tracked.class_id[i]
                class_name = class_names[class_id]

                # Add id to seen ids set
                seen_ids.add(track_id)
                
                # Calculate center of bbox (for velocity)
                cx = (tx1 + tx2) / 2
                cy = (ty1 + ty2) / 2
                
                # track_history'de varsa → velocity hesapla
                # yoksa → yeni track, velocity = 0
                
                # track_history güncelle
                
                # TrackedObject oluştur, STATE_TRACKED, frames_since_seen=0
                # tracked_array'e append et

            # track_history'deki AMA seen_ids'de OLMAYAN track'ler → LOST
            for tid in list(self.track_history.keys()):
                if tid not in seen_ids:
                    self.track_history[tid]['frames_since_seen'] += 1
                    
                    if self.frames_since_seen > self.max_lost_frames:
                        # Delete the track id from history
                        del self.track_history[tid]
                    else:
                        # TrackedObject oluştur, STATE_LOST
                        # Son bilinen bbox ve bilgileri track_history'den al
                        # tracked_array'e append et
                        pass



        # Finally, publish the TrackedObjectArray message
        self.publisher.publish(msg=tracked_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
