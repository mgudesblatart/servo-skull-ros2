from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.parameter_descriptions import ParameterValue
    return LaunchDescription([
        DeclareLaunchArgument(
            "prompt",
            default_value="Hello, Llama!",
            description="Prompt for the llama node"
        ),
        Node(
            package='skull_control_node',
            executable='test_llama',
            name='test_llama',
            output='screen',
            parameters=[{"prompt": ParameterValue(LaunchConfiguration("prompt"), value_type=str)}]
        ),
    ])
