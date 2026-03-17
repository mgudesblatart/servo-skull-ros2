from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


VENV_SITE_PACKAGES = '/home/murray/projects/venv-servo-skull/lib/python3.12/site-packages'
def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("skull_control_node"),
        "configs",
        "axcl_servo_skull.yaml",
    )

    return LaunchDescription([
         SetEnvironmentVariable(
            name='PYTHONPATH',
            value=[VENV_SITE_PACKAGES + ':', EnvironmentVariable('PYTHONPATH', default_value='')]
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=default_config,
            description="Path to the AXCL agent config YAML.",
        ),
        DeclareLaunchArgument(
            "runtime_command",
            default_value="",
            description="Optional override for the interactive model-pack runtime command.",
        ),
        DeclareLaunchArgument(
            "runtime_cwd",
            default_value="",
            description="Optional override for the runtime working directory.",
        ),
        DeclareLaunchArgument(
            "tokenizer_script",
            default_value="/home/murray/models/Qwen2.5-1.5B-Instruct/qwen2.5_tokenizer_uid.py",
            description="Path to the tokenizer service script.",
        ),
        DeclareLaunchArgument(
            "tokenizer_cwd",
            default_value="/home/murray/models/Qwen2.5-1.5B-Instruct",
            description="Working directory for tokenizer service (must contain qwen2.5_tokenizer folder).",
        ),
        DeclareLaunchArgument(
            "start_tokenizer",
            default_value="false",
            description="Whether to start the tokenizer service from this launch file.",
        ),
        DeclareLaunchArgument(
            "inline_system_prompt",
            default_value="",
            description="Optional override: true/false. Empty uses config YAML value.",
        ),
        ExecuteProcess(
            cmd=["python3", LaunchConfiguration("tokenizer_script")],
            cwd=LaunchConfiguration("tokenizer_cwd"),
            output="log",
            condition=IfCondition(LaunchConfiguration("start_tokenizer")),
        ),
        Node(
            package="skull_control_node",
            executable="llm_agent_axcl_node",
            name="llm_agent_axcl_node",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "runtime_command": LaunchConfiguration("runtime_command"),
                "runtime_cwd": LaunchConfiguration("runtime_cwd"),
                "inline_system_prompt": LaunchConfiguration("inline_system_prompt"),
            }],
        ),
    ])