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
        self.max_lost_frames = self.get_parameter('max_lost_frames').value

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
        self.frame_count = 0
        self.last_callback_time = None 

        self.get_logger().info(f'[{agent_id}] Tracker node started')

    def _detection_callback(self, msg: DetectionArray):
        # Calculate time difference (dt) for velocity estimation
        current_time = time.monotonic()
        if self.last_callback_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_callback_time
        
        self.last_callback_time = current_time

        # Feed detections to ByteTrack, publish TrackedObjectArray
        tracked_array = TrackedObjectArray()
        tracked_array.header = msg.header

        xyxy_list = []
        conf_list = []
        class_ids = []
        
        # Build a dictionary to map class_id to class_name directly from this frame's detections
        id_to_name = {}

        # Add fields of detections
        for det in msg.detections:
            x1 = det.bbox.x
            y1 = det.bbox.y
            x2 = det.bbox.x + det.bbox.width
            y2 = det.bbox.y + det.bbox.height
            xyxy_list.append([x1, y1, x2, y2])
            conf_list.append(det.confidence)
            class_ids.append(det.class_id)
            
            # Map the YOLO class_id to its string name for safe lookup later
            id_to_name[det.class_id] = det.class_name
        
        # Seen track ids in the current frame
        seen_ids = set()

        # Empty detection edge case handling
        if len(msg.detections) == 0:
                sv_dets = sv.Detections.empty()
                self.tracker.update_with_detections(sv_dets)

        # Process detections if there are any
        else:
            # Convert them to numpy array
            xyxy = np.array(xyxy_list, dtype=np.float32)        # (N, 4)
            confidence = np.array(conf_list, dtype=np.float32)  # (N,)

            sv_dets = sv.Detections(xyxy=xyxy, confidence=confidence)
            sv_dets.class_id = np.array(class_ids, dtype=int)
            
            # Update tracker
            tracked = self.tracker.update_with_detections(detections=sv_dets)

            # Iterate through tracked objects and unpack the fields
            for i in range(len(tracked)):
                track_id = int(tracked.tracker_id[i])
                tx1, ty1, tx2, ty2 = tracked.xyxy[i]
                tconf = float(tracked.confidence[i])
                class_id = int(tracked.class_id[i])
                
                # Look up the class name safely using our dictionary
                class_name = id_to_name.get(class_id, "unknown")

                # Add id to seen ids set
                seen_ids.add(track_id)
                
                # Calculate center of bbox (for velocity)
                cx = (tx1 + tx2) / 2.0
                cy = (ty1 + ty2) / 2.0
                
                # If it exists in track_history -> calculate velocity
                if track_id in self.track_history.keys():
                    prev_cx, prev_cy = self.track_history[track_id]['last_center']
                    
                    if dt > 0.0:
                        velocity_x = (cx - prev_cx) / dt
                        velocity_y = (cy - prev_cy) / dt
                    else:
                        velocity_x = 0.0
                        velocity_y = 0.0
                        
                    # Keep the old first_frame value
                    first_frame = self.track_history[track_id]['first_frame']
                    
                # If not -> new track, velocity = 0.0
                else:
                    velocity_x = 0.0
                    velocity_y = 0.0
                    first_frame = self.frame_count

                # Update track_history dictionary
                self.track_history[track_id] = {
                    'first_frame': first_frame,
                    'last_center': (cx, cy),
                    'last_frame': self.frame_count,
                    'frames_since_seen': 0,
                    'class_id': class_id,
                    'class_name': class_name,
                    'bbox': (tx1, ty1, tx2-tx1, ty2-ty1),  # x, y, w, h
                    'confidence': tconf,
                }

                # Create TrackedObject, fill all fields, and append it to TrackObjectArray
                tr_obj = TrackedObject()
                tr_obj.header = msg.header
                tr_obj.track_id = track_id
                tr_obj.class_name = class_name
                tr_obj.class_id = class_id
                
                tr_obj.bbox.x = float(tx1)
                tr_obj.bbox.y = float(ty1)
                tr_obj.bbox.width = float(tx2 - tx1)
                tr_obj.bbox.height = float(ty2 - ty1)
                
                tr_obj.confidence = tconf
                tr_obj.velocity_x = float(velocity_x)
                tr_obj.velocity_y = float(velocity_y)
                tr_obj.age_frames = self.frame_count - first_frame
                tr_obj.frames_since_seen = 0
                tr_obj.state = TrackedObject.STATE_TRACKED

                # Append to tracked_array
                tracked_array.tracked_objects.append(tr_obj)
 
        # This ensures that if 0 detections are received, all active tracks correctly become LOST.
        for tid in list(self.track_history.keys()):
            if tid not in seen_ids:
                # Increment the specific track's missing frame counter
                self.track_history[tid]['frames_since_seen'] += 1
                
                # Check using the specific history counter, NOT the deleted instance variable
                if self.track_history[tid]['frames_since_seen'] > self.max_lost_frames:
                    # Delete the track id from history
                    del self.track_history[tid]
                else:
                    # Create TrackedObject with STATE_LOST
                    hist = self.track_history[tid]
                    tr_obj = TrackedObject()
                    tr_obj.header = msg.header
                    tr_obj.track_id = tid
                    tr_obj.class_name = hist['class_name']
                    tr_obj.class_id = hist['class_id']
                    
                    # Fetch last known bounding box
                    bx, by, bw, bh = hist['bbox']
                    tr_obj.bbox.x = float(bx)
                    tr_obj.bbox.y = float(by)
                    tr_obj.bbox.width = float(bw)
                    tr_obj.bbox.height = float(bh)
                    
                    tr_obj.confidence = hist['confidence']
                    tr_obj.velocity_x = 0.0  # No movement info while lost
                    tr_obj.velocity_y = 0.0
                    tr_obj.age_frames = self.frame_count - hist['first_frame']
                    tr_obj.frames_since_seen = hist['frames_since_seen']
                    tr_obj.state = TrackedObject.STATE_LOST
                    
                    # Append lost track to array
                    tracked_array.tracked_objects.append(tr_obj)

        # Calculate tracker FPS
        if dt > 0:
            tracked_array.tracker_fps = 1.0 / dt
        else:
            tracked_array.tracker_fps = 0.0

        # Finally, publish the TrackedObjectArray message then, increment frame count at the end of the callback
        self.publisher.publish(msg=tracked_array)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
