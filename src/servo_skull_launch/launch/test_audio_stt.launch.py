from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="microphone_node",
                executable="microphone_node",
                name="microphone_node",
                output="screen",
            ),
            Node(
                package="stt_node",
                executable="stt_node",
                name="stt_node",
                output="screen",
                # arguments=['--ros-args', '--log-level', 'stt_node:=debug'],
            ),
        ]
    )
