from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Audio output device index or name for speaker_node'
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='-1',
            description='Audio input device index for microphone_node'
        ),
        Node(
            package='person_sensor_node',
            executable='person_sensor_node',
            name='person_sensor_node',
            output='screen'
        ),
        Node(
            package='person_tracking_node',
            executable='person_tracking_node',
            name='person_tracking_node',
            output='screen'
        ),
        Node(
            package='servo_skull_esp32',
            executable='interface_node',
            name='esp32_interface_node',
            output='screen'
        ),
        Node(
            package='skull_control_node',
            executable='skull_control_node',
            name='skull_control_node',
            output='screen'
        ),
        # Example: add speaker_node and microphone_node with device params if needed
        # Node(
        #     package='speaker_node',
        #     executable='speaker_node',
        #     name='speaker_node',
        #     output='screen',
        #     parameters=[{'device': LaunchConfiguration('device')}],
        # ),
        # Node(
        #     package='microphone_node',
        #     executable='microphone_node',
        #     name='microphone_node',
        #     output='screen',
        #     parameters=[{'device_index': LaunchConfiguration('device_index')}],
        # ),
    ])
