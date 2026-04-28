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
            default_value='90.0',
            description='Per-request HTTP timeout in seconds.',
        ),
        DeclareLaunchArgument(
            'summary_refinement_request_timeout_sec',
            default_value='180.0',
            description='HTTP timeout for summary-refinement LLM calls in seconds (allows longer thinking).',
        ),
        DeclareLaunchArgument(
            'max_output_tokens',
            default_value='128',
            description='Maximum assistant output tokens per response.',
        ),
        DeclareLaunchArgument(
            'disable_thinking',
            default_value='true',
            description='If true, appends a no-think instruction to each LLM request.',
        ),
        DeclareLaunchArgument(
            'summary_refinement_disable_thinking',
            default_value='false',
            description='If true, disable thinking for summary-refinement LLM calls.',
        ),
        DeclareLaunchArgument(
            'summary_refinement_max_output_tokens',
            default_value='384',
            description='Max output tokens for summary-refinement LLM calls.',
        ),
        DeclareLaunchArgument(
            'max_history_turns',
            default_value='8',
            description='Number of user/assistant exchange pairs to keep in rolling context.',
        ),
        DeclareLaunchArgument(
            'max_window_tokens',
            default_value='1600',
            description='Approximate token budget for system prompt + summary + recent turns.',
        ),
        DeclareLaunchArgument(
            'enable_llm_summary_refinement',
            default_value='true',
            description='If true, attempt one extra LLM call during context reset to refine the structured summary.',
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
                'max_output_tokens': LaunchConfiguration('max_output_tokens'),
                'disable_thinking': LaunchConfiguration('disable_thinking'),
                'summary_refinement_disable_thinking': LaunchConfiguration('summary_refinement_disable_thinking'),
                'summary_refinement_request_timeout_sec': LaunchConfiguration('summary_refinement_request_timeout_sec'),
                'summary_refinement_max_output_tokens': LaunchConfiguration('summary_refinement_max_output_tokens'),
                'max_history_turns': LaunchConfiguration('max_history_turns'),
                'max_window_tokens': LaunchConfiguration('max_window_tokens'),
                'enable_llm_summary_refinement': LaunchConfiguration('enable_llm_summary_refinement'),
            }],
        ),
    ])
