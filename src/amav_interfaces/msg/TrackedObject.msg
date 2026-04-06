# Single tracked object from ByteTrack
# Extends detection with tracking state and temporal info

std_msgs/Header header

# Identity
int32 track_id               # ByteTrack assigned ID (-1 = untracked)
string class_name
uint16 class_id

# Spatial
amav_interfaces/BoundingBox2D bbox
float32 confidence

# Temporal tracking state
float32 velocity_x           # Estimated px/s in image frame
float32 velocity_y           # Estimated px/s in image frame
uint32 age_frames            # How many frames this track has existed
uint32 frames_since_seen     # 0 = visible now, >0 = occluded/lost

# Status
uint8 STATE_TRACKED=0
uint8 STATE_LOST=1
uint8 STATE_REMOVED=2
uint8 state
