from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Default skull_control launch: starts HTTP LLM agent with the default config."""
    default_http_config = os.path.join(
        get_package_share_directory("skull_control_node"),
        "configs",
        "http_servo_skull.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'disable_thinking',
            default_value='true',
            description='If true, appends a no-think instruction to each llm_agent_http request.',
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("skull_control_node"),
                    "launch",
                    "llm_agent_http.launch.py",
                )
            ),
            launch_arguments={
                'config_path': default_http_config,
                'disable_thinking': LaunchConfiguration('disable_thinking'),
                'summary_refinement_disable_thinking': LaunchConfiguration('summary_refinement_disable_thinking'),
                'summary_refinement_max_output_tokens': LaunchConfiguration('summary_refinement_max_output_tokens'),
                'max_history_turns': LaunchConfiguration('max_history_turns'),
                'max_window_tokens': LaunchConfiguration('max_window_tokens'),
                'enable_llm_summary_refinement': LaunchConfiguration('enable_llm_summary_refinement'),
            }.items(),
        )
    ])


