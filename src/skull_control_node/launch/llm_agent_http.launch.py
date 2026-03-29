from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


VENV_SITE_PACKAGES = '/home/murray/projects/venv-servo-skull/lib/python3.12/site-packages'


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=[VENV_SITE_PACKAGES + ':', EnvironmentVariable('PYTHONPATH', default_value='')]
        ),
        DeclareLaunchArgument(
            'axllm_base_url',
            default_value='http://127.0.0.1:8081',
            description='Base URL of the running axllm HTTP server.',
        ),
        DeclareLaunchArgument(
            'axllm_model',
            default_value='AXERA-TECH/Qwen3-1.7B',
            description='Model name passed to /v1/chat/completions.',
        ),
        DeclareLaunchArgument(
            'system_prompt',
            default_value='',
            description='Optional system prompt injected as the first message.',
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='Path to a YAML config file loaded by llm_agent_http_node (system_prompt, model, timeouts, etc.).',
        ),
        DeclareLaunchArgument(
            'startup_timeout_sec',
            default_value='30.0',
            description='Seconds to wait for the axllm server to become ready.',
        ),
        DeclareLaunchArgument(
            'request_timeout_sec',
            default_value='60.0',
            description='Per-request HTTP timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'max_history_turns',
            default_value='8',
            description='Number of user/assistant exchange pairs to keep in rolling context.',
        ),
        Node(
            package='skull_control_node',
            executable='llm_agent_http_node',
            name='llm_agent_http_node',
            output='screen',
            parameters=[{
                'axllm_base_url': LaunchConfiguration('axllm_base_url'),
                'axllm_model': LaunchConfiguration('axllm_model'),
                'system_prompt': LaunchConfiguration('system_prompt'),
                'config_path': LaunchConfiguration('config_path'),
                'startup_timeout_sec': LaunchConfiguration('startup_timeout_sec'),
                'request_timeout_sec': LaunchConfiguration('request_timeout_sec'),
                'max_history_turns': LaunchConfiguration('max_history_turns'),
            }],
        ),
    ])
