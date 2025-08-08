from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    skull_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('skull_control_node'),
                'launch',
                'skull_control.launch.py'
            )
        ),
        launch_arguments={}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'mic_device',
            default_value='0',
            description='Audio input device index or name for microphone_node'
        ),
        DeclareLaunchArgument(
            'speaker_device',
            default_value='1',
            description='Audio output device index or name for speaker_node'
        ),
        Node(
            package="microphone_node",
            executable="microphone_node",
            name="microphone_node",
            output="screen",
            parameters=[{'device_index': LaunchConfiguration('mic_device')}],
        ),
        Node(
            package="stt_node",
            executable="stt_node",
            name="stt_node",
            output="screen",
        ),
        skull_control_launch,
        Node(
            package="tts_node",
            executable="tts_node",
            name="tts_node",
            output="screen",
        ),
        Node(
            package="speaker_node",
            executable="speaker_node",
            name="speaker_node",
            output="screen",
            parameters=[{'device': LaunchConfiguration('speaker_device')}],
        ),
    ])
