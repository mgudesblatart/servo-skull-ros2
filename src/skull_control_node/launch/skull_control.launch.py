from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='skull_control_node',
            executable='skull_control_bt_node.py',
            name='skull_control_bt_node',
            output='screen',
        ),
        Node(
            package='skull_control_node',
            executable='llm_agent_node.py',
            name='llm_agent_node',
            output='screen',
        ),
        Node(
            package='llama_ros',
            executable='llama_ros_node',
            name='llama_ros_node',
            output='screen',
            parameters=[
                {'model_path': '/home/$USER/projects/models/tinyllama-1.1b-chat-v1.0.Q4_0.gguf'}
            ]
        ),
    ])
