# All tracked objects in current frame
# Published by tracker_node each frame

std_msgs/Header header

amav_interfaces/TrackedObject[] tracked_objects
float32 tracker_fps          # Actual processing FPS for latency monitoring
