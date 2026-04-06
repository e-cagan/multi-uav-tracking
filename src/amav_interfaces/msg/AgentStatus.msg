# Status of a single agent (drone) in the system
# Each drone's decision_node publishes this periodically
# coordinator_node subscribes to all agents

std_msgs/Header header

# Agent identity
string agent_id              # e.g. "drone_0", "drone_1"

# Agent pose in world frame
geometry_msgs/Pose pose

# Mission state
uint8 STATE_IDLE=0
uint8 STATE_SEARCHING=1
uint8 STATE_TRACKING=2
uint8 STATE_APPROACHING=3    # Confidence-aware: moving closer
uint8 STATE_HANDOFF=4        # Transferring target to another agent
uint8 state

# What this agent is tracking (if any)
int32 tracked_object_id      # -1 = nothing
float32 track_confidence     # Current avg confidence of tracked target

# Pipeline health
float32 detector_fps
float32 tracker_fps
float32 current_resolution_scale  # 1.0 = full, 0.5 = half (latency-aware)
