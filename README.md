# AMAV — Adaptive Multi-Agent Vision System

A ROS2-based multi-agent drone system for collaborative search and tracking of ground targets. Two autonomous drones coordinate to search an area, detect and track a person using real-time computer vision, and hand off tracking responsibility when occlusion or low confidence occurs.

Built from scratch with zero vibe-coding — every line written with full understanding.

## Demo

**Scenario:** A walking person moves through an urban area with buildings. drone_0 searches the area via waypoint patrol, detects and tracks the person using YOLOv8 + ByteTrack. When the person moves behind a building (occlusion) or detection confidence drops critically, the coordinator node triggers a handoff to drone_1, which flies to the last known position to re-acquire the target.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo Garden                            │
│  search_area.sdf (100x100m, 7 buildings, walking person)       │
│  x500_mono_cam (drone_0) + x500 (drone_1)                     │
└──────────┬──────────────────────────────────────────────────────┘
           │ /camera (gz.msgs.Image)
           ▼
┌──────────────────┐
│  ros_gz_bridge    │  Gazebo → ROS2 image bridge
└──────────┬───────┘
           │ /camera (sensor_msgs/Image)
           ▼
┌──────────────────┐
│  camera_node      │  Optional resize for latency-aware pipeline
└──────────┬───────┘
           │ camera/image
           ▼
┌──────────────────┐
│  detector_node    │  YOLOv8 inference → DetectionArray
└──────────┬───────┘
           │ detections (amav_interfaces/DetectionArray)
           ▼
┌──────────────────┐
│  tracker_node     │  ByteTrack → TrackedObjectArray
└──────────┬───────┘
           │ tracked_objects (amav_interfaces/TrackedObjectArray)
           ▼
┌──────────────────┐
│  decision_node    │  State machine + MAVROS2 flight control
│  (per drone)      │  Publishes AgentStatus
└──────────┬───────┘
           │ agent_status (amav_interfaces/AgentStatus)
           ▼
┌──────────────────┐
│ coordinator_node  │  Multi-agent handoff management
│  (global)         │  Calls Handoff.srv on receiving agents
└──────────────────┘
```

## Key Features

### 1. Confidence-Aware Behavior

The decision node monitors YOLO detection confidence in real-time. When confidence drops below a configurable threshold (default 0.4), the drone automatically transitions from TRACKING to APPROACHING state and moves closer to the target. This is not a simple threshold check — it drives physical drone behavior.

### 2. Occlusion Handling via Multi-Agent Handoff

When a tracked target moves behind a building and is lost for more than N frames, the coordinator node identifies an available drone and triggers a handoff via ROS2 service call. The receiving drone navigates to the target’s last known position and re-acquires tracking. This is a real-world SAR (Search and Rescue) pattern.

### 3. Temporal Tracking with ByteTrack

ByteTrack provides frame-to-frame identity consistency — the same person keeps the same track ID across frames. The tracker node extends this with velocity estimation (px/s), track age, and frames-since-seen counters. ByteTrack’s dual-threshold matching means even low-confidence detections (e.g., partially occluded targets) maintain their tracks.

### 4. Full State Machine

Each drone runs an independent state machine:

```
IDLE → SEARCHING → TRACKING → APPROACHING
                      ↑            │
                      └────────────┘
```

State transitions are logged, published as AgentStatus messages, and visualized in real-time.

## Tech Stack

|Component        |Technology                         |
|-----------------|-----------------------------------|
|Middleware       |ROS2 Humble                        |
|Simulation       |Gazebo Garden (gz-sim 7.x)         |
|Flight Controller|PX4 Autopilot v1.15.x SITL         |
|Flight Comms     |MAVROS2 (MAVLink → ROS2)           |
|Detection        |YOLOv8n (ultralytics)              |
|Tracking         |ByteTrack (via supervision)        |
|Language         |Python 3.10                        |
|Drone Model      |PX4 x500 quadrotor with mono camera|

## Repository Structure

```
amav_ws/
├── src/
│   ├── amav_interfaces/         # Custom ROS2 message/service definitions
│   │   ├── msg/
│   │   │   ├── BoundingBox2D.msg        # Reusable 2D bbox (x, y, w, h)
│   │   │   ├── Detection.msg            # Single YOLO detection
│   │   │   ├── DetectionArray.msg       # All detections in one frame
│   │   │   ├── TrackedObject.msg        # Tracked object with velocity, age, state
│   │   │   ├── TrackedObjectArray.msg   # All tracks in one frame
│   │   │   └── AgentStatus.msg          # Drone state, pose, target info
│   │   ├── srv/
│   │   │   └── Handoff.srv              # Request/response for target handoff
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── amav/                    # Core ROS2 Python nodes
│   │   ├── amav/
│   │   │   ├── camera_node.py           # Gazebo camera → ROS2 with optional resize
│   │   │   ├── detector_node.py         # YOLOv8 inference, publishes DetectionArray
│   │   │   ├── tracker_node.py          # ByteTrack, publishes TrackedObjectArray
│   │   │   ├── decision_node.py         # State machine + MAVROS2 flight commands
│   │   │   ├── coordinator_node.py      # Multi-agent handoff management
│   │   │   └── utils/
│   │   ├── config/
│   │   │   └── agent_params.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── amav_bringup/            # Launch files, world, scripts
│       ├── launch/
│       │   ├── dev.launch.py             # Single-agent development
│       │   └── constrainted.launch.py    # Multi-agent (constrained)
│       ├── worlds/
│       │   └── search_area.sdf          # 100x100m area with 7 buildings + walking person
│       ├── scripts/
│       │   └── visualize_detections.py  # OpenCV debug visualizer
│       ├── config/
│       ├── CMakeLists.txt
│       └── package.xml
│
└── PX4-Autopilot/               # (external, not in workspace)
```

## Custom Interfaces

### Messages

**BoundingBox2D.msg** — Pixel-coordinate bounding box, reused across Detection, TrackedObject, and Handoff. OpenCV convention: (x, y) is top-left corner.

**Detection.msg** — Single YOLO detection output: bbox, class_name, class_id, confidence. Published per-object, grouped in DetectionArray.

**DetectionArray.msg** — All detections from one frame, plus image dimensions and inference_time_ms for latency monitoring.

**TrackedObject.msg** — Extends Detection with temporal information: track_id (ByteTrack assigned), velocity_x/y (px/s estimated from frame-to-frame center displacement), age_frames, frames_since_seen, and state (TRACKED/LOST/REMOVED).

**TrackedObjectArray.msg** — All tracked objects in current frame plus tracker_fps.

**AgentStatus.msg** — Per-drone status report: agent_id, 3D pose, state machine state (IDLE/SEARCHING/TRACKING/APPROACHING/HANDOFF), currently tracked object ID, track confidence, and pipeline health metrics (detector_fps, tracker_fps, resolution_scale).

### Services

**Handoff.srv** — Coordinator requests a drone to take over tracking. Request includes: requesting_agent_id, target_track_id, target_last_position (3D), target_last_bbox, and reason enum (LOW_CONFIDENCE/OCCLUSION/OUT_OF_RANGE/LOW_FPS). Response: accepted bool, accepting_agent_id, reject_reason.

## Node Details

### camera_node

Subscribes to raw Gazebo camera images via ros_gz_bridge. Optionally resizes images based on `resolution_scale` parameter (foundation for latency-aware pipeline). Logs FPS every second. Republishes as standard sensor_msgs/Image.

### detector_node

Loads YOLOv8 model at startup. Runs inference on every incoming frame. Converts YOLO output (center_x, center_y, w, h) to top-left (x, y, w, h) for BoundingBox2D compatibility. Filters detections by confidence_threshold. Measures inference_time_ms with time.monotonic(). Confidence threshold is passed directly to YOLO for GPU-side filtering.

### tracker_node

Initializes supervision ByteTrack instance. Converts DetectionArray to numpy arrays (N x 4 xyxy + N confidence), feeds to tracker. Maintains per-track history dictionary with: first seen frame, last center position, frames since seen. Computes velocity as (center_delta / dt) between consecutive callbacks. Publishes both active (STATE_TRACKED) and lost (STATE_LOST) tracks. Removes tracks after max_lost_frames. Handles empty detection frames by calling tracker update with empty detections to advance internal state.

### decision_node

Implements a 4-state finite state machine:

- **IDLE** — Startup state. Transitions to SEARCHING after arm + takeoff sequence.
- **SEARCHING** — Patrols predefined waypoints in sequence. Selects highest-confidence tracked person as target. Uses Euclidean distance check for waypoint arrival.
- **TRACKING** — Hovers at current position while target is tracked with sufficient confidence.
- **APPROACHING** — Moves forward when confidence drops below threshold. Returns to TRACKING when confidence recovers.

Startup sequence: publishes setpoint for 2 seconds, then calls MAVROS SetMode (OFFBOARD) and CommandBool (arm) asynchronously. 20Hz timer continuously publishes current_setpoint to satisfy PX4’s setpoint stream requirement.

Handoff callback: accepts handoff only when IDLE or SEARCHING. Routes to target’s last known position while staying in SEARCHING state (because local tracker will assign a new track ID to the same person).

### coordinator_node

Subscribes to all agents’ AgentStatus. Evaluates handoff conditions every status callback: if an agent is TRACKING/APPROACHING with confidence below critical threshold, finds an available receiver (IDLE or SEARCHING), calls Handoff service asynchronously. Rate-limited to one handoff attempt per 5 seconds per agent to prevent spam. Logs all handoff attempts and results.

### visualize_detections.py (debug tool)

Dual-window OpenCV visualizer. Detection window: raw YOLO bboxes with class + confidence. Tracking window: ByteTrack bboxes with track ID (color-coded), velocity, age, frames_since_seen, and agent state overlay. Displays inference time and tracker FPS. LOST tracks shown in red.

## Prerequisites

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Gazebo Garden
sudo apt install gz-garden
sudo apt install libgz-sim7-dev libgz-transport12-dev libgz-sensors7-dev \
  libgz-math7-dev libgz-msgs9-dev libgz-rendering7-dev libgz-physics6-dev \
  libgz-common5-dev libgz-plugin2-dev libgz-fuel-tools8-dev libgz-gui7-dev \
  libgz-cmake3-dev

# Gazebo Garden ROS bridge
sudo apt install ros-humble-ros-gzgarden-bridge

# MAVROS2
sudo apt install ros-humble-mavros ros-humble-mavros-extras
ros2 run mavros install_geographiclib_datasets.sh

# Python dependencies
pip install ultralytics supervision opencv-python-headless numpy

# PX4 Autopilot (outside workspace)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout v1.15.4
git submodule update --init --recursive
pip install kconfiglib
make px4_sitl gz_x500  # Initial build
```

## Build

```bash
cd ~/amav_ws

# Build interfaces first (amav depends on them)
colcon build --packages-select amav_interfaces
source install/setup.bash

# Build everything
colcon build
source install/setup.bash
```

## Running

### Single Agent (Development)

```bash
# Terminal 1: PX4 SITL
cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
GZ_VERSION=garden PX4_GZ_WORLD=search_area PX4_GZ_MODEL=x500_mono_cam make px4_sitl gz_x500_mono_cam

# Terminal 2: MAVROS
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580

# Terminal 3: AMAV stack
cd ~/amav_ws && source install/setup.bash
ros2 launch amav_bringup dev.launch.py model_path:=/home/cagan/amav_ws/yolov8n.pt

# Terminal 4 (optional): Visualizer
cd ~/amav_ws && source install/setup.bash
python3 src/amav_bringup/scripts/visualize_detections.py
```

### Multi-Agent

```bash
# Terminal 1: PX4 SITL drone_0 (with camera)
cd ~/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
GZ_VERSION=garden PX4_GZ_WORLD=search_area PX4_GZ_MODEL=x500_mono_cam make px4_sitl gz_x500_mono_cam

# Terminal 2: PX4 SITL drone_1 (no camera — Gazebo single-camera limitation)
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="20,0,0,0,0,0" \
  PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 3: MAVROS drone_0
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580

# Terminal 4: MAVROS drone_1
ros2 launch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14581 \
  namespace:=drone_1 tgt_system:=2

# Terminal 5: AMAV multi-agent stack
cd ~/amav_ws && source install/setup.bash
ros2 launch amav_bringup multi_agent.launch.py
```

## Gazebo World

`search_area.sdf` — a 100x100m area designed for the search and track scenario:

- **7 buildings** of varying sizes arranged to create occlusion zones (west side has dense buildings, east side is open)
- **Walking person actor** that follows a looping trajectory through the building area, creating natural occlusion events
- **PX4-required plugins**: Physics, IMU, AirPressure, NavSat, Contact, Sensors (ogre2), ApplyLinkWrench
- **PX4-required parameters**: gravity, magnetic_field (Zurich default), spherical_coordinates (GPS datum)
- **Physics**: max_step_size=0.004 (PX4 lockstep requirement)

## Design Decisions

**MAVROS2 over Micro-XRCE-DDS:** MAVROS2 provides a familiar ROS2 interface with well-documented arming, mode-setting, and setpoint services. While Micro-XRCE-DDS is PX4’s newer native approach, MAVROS2 offered faster development with proven reliability.

**ByteTrack over DeepSORT:** ByteTrack’s dual-threshold association uses low-confidence detections as a “second chance” for matching — critical for our scenario where targets at distance or partially occluded produce low confidence scores. DeepSORT would require a separate ReID model adding complexity without clear benefit in a single-class (person) scenario.

**Python over C++:** All nodes are in Python. Detection (YOLO), tracking (ByteTrack/supervision), and image processing (OpenCV) are all Python-first libraries. The inference bottleneck runs on CUDA regardless of the wrapper language. Decision logic and ROS2 communication overhead are negligible compared to inference time.

**2 packages + bringup:** `amav_interfaces` (ament_cmake, required for msg/srv generation), `amav` (ament_python, all nodes), `amav_bringup` (ament_cmake, launch/world/config). This separates interface definitions from logic from deployment configuration.

**Coordinator uses service calls, not topics:** Handoff requires acknowledgment — “did you accept?” Topics are fire-and-forget. ROS2 services provide the request-response pattern needed for reliable handoff.

## Docker Deployment (Recommended)

To avoid dependency issues and run the perception stack in an isolated environment, you can use the provided Docker container. 
*Note: PX4 SITL and Gazebo must still run on the host machine.*

1. **Build the image:**
   ```bash
   cd ~/amav_ws
   docker build -t amav_system:latest .
  
2. **Run the container:**
   ```bash
   docker run -it --rm --net=host amav_system:latest ros2 launch amav_bringup multi_agent.launch.py

This will start the AMAV multi-agent system inside the container. Make sure to run the PX4 SITL and Gazebo on the host before launching the container.

## Known Limitations

- **Gazebo Garden single camera:** The ogre2 render engine in Gazebo Garden cannot reliably initialize multiple camera sensors in a single simulation (libEGL context limitation on some GPU/driver combinations). drone_1 runs without a camera and operates on coordinate-based handoff only. In a real deployment or with Gazebo Harmonic, each drone would have its own camera and full perception pipeline.
- **2D tracking only:** ByteTrack operates in image pixel coordinates. There is no 3D world-frame target position estimation. The handoff sends the failing drone’s position as an approximation of the target’s location.
- **No re-identification across drones:** Each drone’s ByteTrack instance assigns independent track IDs. After handoff, the receiving drone’s tracker assigns a new ID to the same person. Cross-drone ReID would require a visual embedding model.
- **Simplified approach behavior:** APPROACHING state moves the drone forward along the x-axis rather than toward the target in image space. A proper visual servoing controller would use bbox center offset to generate velocity commands.

## Future Work (Milestone 6)

- Latency-aware pipeline: dynamically reduce resolution when FPS drops
- Metric collection: handoff latency, detection-to-track delay, search coverage
- Docker containerization for reproducible deployment
- Architecture diagram generation from running system (rqt_graph)
- Demo video with picture-in-picture (Gazebo + visualizer + terminal)

## Author

Emin Çağan Apaydın

## License

MIT