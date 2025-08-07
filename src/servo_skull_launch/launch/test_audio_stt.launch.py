from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'device_index',
                default_value='-1',
                description='Audio input device index for microphone_node'
            ),
            Node(
                package="microphone_node",
                executable="microphone_node",
                name="microphone_node",
                output="screen",
                parameters=[{'device_index': LaunchConfiguration('device_index')}],
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
