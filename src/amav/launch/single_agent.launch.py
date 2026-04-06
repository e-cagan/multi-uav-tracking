"""Single agent launch: all perception + decision nodes for one drone."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('amav')
    default_params = os.path.join(pkg_share, 'config', 'agent_params.yaml')

    agent_id_arg = DeclareLaunchArgument(
        'agent_id', default_value='drone_0',
        description='Unique identifier for this agent'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='drone_0',
        description='ROS2 namespace for this agent'
    )

    ns = LaunchConfiguration('namespace')

    camera_node = Node(
        package='amav',
        executable='camera_node',
        name='camera_node',
        namespace=ns,
        parameters=[default_params],
        output='screen',
    )

    detector_node = Node(
        package='amav',
        executable='detector_node',
        name='detector_node',
        namespace=ns,
        parameters=[default_params],
        output='screen',
    )

    tracker_node = Node(
        package='amav',
        executable='tracker_node',
        name='tracker_node',
        namespace=ns,
        parameters=[default_params],
        output='screen',
    )

    decision_node = Node(
        package='amav',
        executable='decision_node',
        name='decision_node',
        namespace=ns,
        parameters=[default_params],
        output='screen',
    )

    return LaunchDescription([
        agent_id_arg,
        namespace_arg,
        camera_node,
        detector_node,
        tracker_node,
        decision_node,
    ])
