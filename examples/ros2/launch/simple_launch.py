"""
Simple launch file example for ROS2.
This launch file starts a publisher and subscriber node together.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='publisher',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('chatter', 'topic')
            ]
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='subscriber',
            remappings=[
                ('chatter', 'topic')
            ]
        )
    ])