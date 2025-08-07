from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
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
