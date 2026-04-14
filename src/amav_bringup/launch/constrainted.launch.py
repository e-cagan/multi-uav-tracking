"""
AMAV Multi-Agent Launch File (Constrained)

Due to Gazebo Garden ogre2 render engine limitation, only one camera sensor
can be active per simulation. This launch runs:

- drone_0: Full perception pipeline (camera -> detector -> tracker -> decision)
- drone_1: Decision-only (no camera, navigates by coordinator commands)
- coordinator: Manages handoffs between agents

PX4 SITL and MAVROS must be started separately — see README.md for full instructions.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='/home/cagan/amav_ws/yolov8n.pt',
        description='Path to YOLO model weights'
    )

    conf_thresh_arg = DeclareLaunchArgument(
        'confidence_threshold', default_value='0.25',
        description='YOLO confidence threshold'
    )

    gz_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen',
    )

    drone_0 = GroupAction([
        PushRosNamespace('drone_0'),
        Node(
            package='amav', executable='camera_node', name='camera_node',
            parameters=[{
                'agent_id': 'drone_0', 'input_topic': '/camera',
                'output_topic': 'camera/image', 'resolution_scale': 1.0,
            }], output='screen',
        ),
        Node(
            package='amav', executable='detector_node', name='detector_node',
            parameters=[{
                'agent_id': 'drone_0',
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'input_topic': 'camera/image', 'output_topic': 'detections',
            }], output='screen',
        ),
        Node(
            package='amav', executable='tracker_node', name='tracker_node',
            parameters=[{
                'agent_id': 'drone_0', 'input_topic': 'detections',
                'output_topic': 'tracked_objects', 'max_lost_frames': 30,
            }], output='screen',
        ),
        Node(
            package='amav', executable='decision_node', name='decision_node',
            parameters=[{
                'agent_id': 'drone_0', 'input_topic': 'tracked_objects',
                'status_topic': 'agent_status', 'mavros_prefix': '/mavros',
                'confidence_low_threshold': 0.4, 'confidence_critical_threshold': 0.2,
                'lost_frames_handoff_trigger': 15, 'approach_speed': 2.0,
                'search_altitude': 10.0, 'waypoint_reached_threshold': 2.0,
            }], output='screen',
        ),
    ])

    drone_1 = GroupAction([
        PushRosNamespace('drone_1'),
        Node(
            package='amav', executable='decision_node', name='decision_node',
            parameters=[{
                'agent_id': 'drone_1', 'input_topic': 'tracked_objects',
                'status_topic': 'agent_status', 'mavros_prefix': '/drone_1',
                'confidence_low_threshold': 0.4, 'confidence_critical_threshold': 0.2,
                'lost_frames_handoff_trigger': 15, 'approach_speed': 2.0,
                'search_altitude': 10.0, 'waypoint_reached_threshold': 2.0,
            }], output='screen',
        ),
    ])

    coordinator_node = Node(
        package='amav', executable='coordinator_node', name='coordinator_node',
        parameters=[{
            'agent_ids': ['drone_0', 'drone_1'],
            'handoff_confidence_threshold': 0.3,
            'handoff_lost_frames_threshold': 10,
        }], output='screen',
    )

    return LaunchDescription([
        model_path_arg, conf_thresh_arg,
        LogInfo(msg='=== AMAV Multi-Agent System (Constrained Mode) ==='),
        LogInfo(msg='drone_0: Full perception (camera + YOLO + ByteTrack + decision)'),
        LogInfo(msg='drone_1: Decision-only (coordinate-based handoff)'),
        gz_bridge, drone_0, drone_1, coordinator_node,
    ])