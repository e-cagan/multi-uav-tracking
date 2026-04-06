# Single object detection from YOLO inference
# Published by detector_node per detected object

std_msgs/Header header

# Detection result
amav_interfaces/BoundingBox2D bbox
string class_name
uint16 class_id
float32 confidence           # YOLO confidence [0.0, 1.0]
