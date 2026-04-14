"""
AMAV Multi-Agent Launch File

Starts full perception + decision pipeline for two drones + coordinator.
PX4 SITL instances and MAVROS must be started separately.

Usage:
  Terminal 1: PX4 SITL instance 0 (first drone)
    cd ~/PX4-Autopilot
    export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
    GZ_VERSION=garden PX4_GZ_WORLD=search_area PX4_GZ_MODEL=x500_mono_cam make px4_sitl gz_x500_mono_cam

  Terminal 2: PX4 SITL instance 1 (second drone)
    cd ~/PX4-Autopilot
    PX4_SYS_AUTOSTART=4010 PX4_GZ_MODEL=x500_mono_cam PX4_GZ_MODEL_POSE="20,0,0,0,0,0" PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4 -i 1

  Terminal 3: MAVROS for drone_0
    ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580

  Terminal 4: MAVROS for drone_1
    ros2 launch mavros px4.launch fcu_url:=udp://:14541@127.0.0.1:14581 namespace:=drone_1 tgt_system:=2

  Terminal 5: This launch file
    ros2 launch amav_bringup multi_agent.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # === Launch Arguments ===
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/cagan/amav_ws/yolov8n.pt',
        description='Path to YOLO model weights'
    )

    conf_thresh_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='YOLO confidence threshold'
    )

    # Camera topics — update these after confirming gz topic -l output
    gz_cam_0_arg = DeclareLaunchArgument(
        'gz_camera_topic_0', default_value='/camera',
        description='Gazebo camera topic for drone 0'
    )

    gz_cam_1_arg = DeclareLaunchArgument(
        'gz_camera_topic_1', default_value='/camera_1',
        description='Gazebo camera topic for drone 1 (check gz topic -l)'
    )

    # === Helper: create a full agent pipeline within a namespace ===
    def make_agent_group(agent_id, namespace, gz_camera_topic, mavros_prefix, waypoints_str):
        """
        Creates all nodes for one agent inside a namespace.
        waypoints_str: semicolon-separated waypoints e.g. "0,0,10;20,0,10;20,20,10"
        """
        return GroupAction([
            PushRosNamespace(namespace),

            # camera_node
            Node(
                package='amav',
                executable='camera_node',
                name='camera_node',
                parameters=[{
                    'agent_id': agent_id,
                    'input_topic': gz_camera_topic,
                    'output_topic': 'camera/image',
                    'resolution_scale': 1.0,
                }],
                output='screen',
            ),

            # detector_node
            Node(
                package='amav',
                executable='detector_node',
                name='detector_node',
                parameters=[{
                    'agent_id': agent_id,
                    'model_path': LaunchConfiguration('model_path'),
                    'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                    'input_topic': 'camera/image',
                    'output_topic': 'detections',
                }],
                output='screen',
            ),

            # tracker_node
            Node(
                package='amav',
                executable='tracker_node',
                name='tracker_node',
                parameters=[{
                    'agent_id': agent_id,
                    'input_topic': 'detections',
                    'output_topic': 'tracked_objects',
                    'max_lost_frames': 30,
                }],
                output='screen',
            ),

            # decision_node
            Node(
                package='amav',
                executable='decision_node',
                name='decision_node',
                parameters=[{
                    'agent_id': agent_id,
                    'input_topic': 'tracked_objects',
                    'status_topic': 'agent_status',
                    'mavros_prefix': mavros_prefix,
                    'confidence_low_threshold': 0.4,
                    'confidence_critical_threshold': 0.2,
                    'lost_frames_handoff_trigger': 15,
                    'approach_speed': 2.0,
                    'search_altitude': 10.0,
                    'waypoint_reached_threshold': 2.0,
                }],
                output='screen',
            ),
        ])

    # === Gazebo camera bridges ===
    gz_bridge_0 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen',
    )

    # Second camera bridge — topic name TBD, update after gz topic -l
    gz_bridge_1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera_1@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen',
    )

    # === Agent pipelines ===

    # drone_0: north/east search pattern
    drone_0 = make_agent_group(
        agent_id='drone_0',
        namespace='drone_0',
        gz_camera_topic='/camera',
        mavros_prefix='/mavros',
        waypoints_str='0,0,10;20,0,10;20,20,10;0,20,10',
    )

    # drone_1: south/west search pattern
    drone_1 = make_agent_group(
        agent_id='drone_1',
        namespace='drone_1',
        gz_camera_topic='/camera_1',
        mavros_prefix='/drone_1',
        waypoints_str='0,0,10;-20,0,10;-20,-20,10;0,-20,10',
    )

    # === Coordinator (global, no namespace) ===
    coordinator_node = Node(
        package='amav',
        executable='coordinator_node',
        name='coordinator_node',
        parameters=[{
            'agent_ids': ['drone_0', 'drone_1'],
            'handoff_confidence_threshold': 0.3,
            'handoff_lost_frames_threshold': 10,
        }],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        model_path_arg,
        conf_thresh_arg,
        gz_cam_0_arg,
        gz_cam_1_arg,

        # Camera bridges
        gz_bridge_0,
        gz_bridge_1,

        # Agent pipelines
        drone_0,
        drone_1,

        # Coordinator
        coordinator_node,
    ])