"""
AMAV Development Launch File

Starts all perception nodes for single-agent testing.
PX4 SITL and MAVROS must be started separately.

Usage:
  Terminal 1: PX4 SITL (see README)
  Terminal 2: ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
  Terminal 3: ros2 launch amav_bringup dev.launch.py

This launch file starts:
  - ros_gz_bridge (Gazebo camera -> ROS2)
  - camera_node
  - detector_node
  - tracker_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # === Launch Arguments ===
    agent_id_arg = DeclareLaunchArgument(
        'agent_id', default_value='drone_0',
        description='Unique identifier for this agent'
    )

    ns_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS2 namespace (empty for single agent dev)'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='yolov8n.pt',
        description='Path to YOLO model weights'
    )

    conf_thresh_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='YOLO confidence threshold'
    )

    gz_camera_topic_arg = DeclareLaunchArgument(
        'gz_camera_topic', default_value='/camera',
        description='Gazebo camera topic name'
    )

    # === Nodes ===

    # ros_gz_bridge: bridges Gazebo camera topic to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_camera_bridge',
        arguments=[
            LaunchConfiguration('gz_camera_topic').perform(None) +
            '@sensor_msgs/msg/Image@gz.msgs.Image'
        ] if False else [],  # See below for workaround
        output='screen',
    )

    # Workaround: parameter_bridge needs the mapping as a single argument
    # Using ExecuteProcess since Node() doesn't handle the special syntax well
    gz_bridge_proc = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen',
    )

    # camera_node: republishes camera with optional resize
    camera_node = Node(
        package='amav',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'agent_id': LaunchConfiguration('agent_id'),
            'input_topic': LaunchConfiguration('gz_camera_topic'),
            'output_topic': 'camera/image',
            'resolution_scale': 1.0,
        }],
        output='screen',
    )

    # detector_node: YOLO inference
    detector_node = Node(
        package='amav',
        executable='detector_node',
        name='detector_node',
        parameters=[{
            'agent_id': LaunchConfiguration('agent_id'),
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'input_topic': 'camera/image',
            'output_topic': 'detections',
        }],
        output='screen',
    )

    # tracker_node: ByteTrack
    tracker_node = Node(
        package='amav',
        executable='tracker_node',
        name='tracker_node',
        parameters=[{
            'agent_id': LaunchConfiguration('agent_id'),
            'input_topic': 'detections',
            'output_topic': 'tracked_objects',
            'max_lost_frames': 30,
        }],
        output='screen',
    )

    # decision_node: Decisions based on state machine
    decision_node = Node(
        package='amav',
        executable='decision_node',
        name='decision_node',
        parameters=[{
            'agent_id': LaunchConfiguration('agent_id'),
            'input_topic': 'tracked_objects',
            'status_topic': 'agent_status',
            'confidence_low_threshold': 0.4,
        }],
        output='screen',
    )

    # visualizer_node: Visualizes detections, tracking and agent status
    visualizer_node = Node(
        package='amav_bringup',
        executable='visualize_detections.py',
        name='visualizer_node',
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        agent_id_arg,
        ns_arg,
        model_path_arg,
        conf_thresh_arg,
        gz_camera_topic_arg,

        # Processes
        gz_bridge_proc,
        camera_node,
        detector_node,
        tracker_node,
        decision_node,
        visualizer_node,
    ])