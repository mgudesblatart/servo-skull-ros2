from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
    ])
