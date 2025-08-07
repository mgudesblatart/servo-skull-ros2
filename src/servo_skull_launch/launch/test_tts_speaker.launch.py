from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Audio output device index or name for speaker_node'
        ),
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
            parameters=[{'device': LaunchConfiguration('device')}],
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub',
                '--once',
                '/text_to_speech/text_input',
                'std_msgs/msg/String',
                '"data: Hello, servo skull! This is a test of the TTS and speaker nodes."'
            ],
            output='screen',
            shell=True
        ),
    ])
