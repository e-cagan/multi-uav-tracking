# All detections from a single frame
# YOLO outputs N detections per image, this groups them

std_msgs/Header header

amav_interfaces/Detection[] detections
uint32 image_width
uint32 image_height
float32 inference_time_ms    # Track for latency-aware pipeline
