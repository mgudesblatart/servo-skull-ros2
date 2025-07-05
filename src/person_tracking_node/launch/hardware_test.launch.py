import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='person_sensor_node',
            executable='person_sensor_node',
            name='person_sensor_node',
            output='screen',
        ),
        Node(
            package='person_tracking_node',
            executable='person_tracking_node_exe',
            name='person_tracking_node',
            output='screen',
        ),
    ])
