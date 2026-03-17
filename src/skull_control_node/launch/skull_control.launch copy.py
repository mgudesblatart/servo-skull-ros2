from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("skull_control_node"),
        "configs",
    )
    axcl_launch = os.path.join(
        get_package_share_directory("skull_control_node"),
        "launch",
        "llm_agent_axcl.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "llm_backend",
            default_value="llama_ros",
            description="LLM backend to use: llama_ros or axcl",
        ),
        DeclareLaunchArgument(
            "axcl_config_path",
            default_value=os.path.join(config_dir, "axcl_servo_skull.yaml"),
            description="Config file for the AXCL-backed LLM agent node.",
        ),
        DeclareLaunchArgument(
            "axcl_runtime_command",
            default_value="",
            description="Optional override for the AXCL runtime command.",
        ),
        DeclareLaunchArgument(
            "axcl_runtime_cwd",
            default_value="",
            description="Optional override for the AXCL runtime working directory.",
        ),
        DeclareLaunchArgument(
            "start_tokenizer",
            default_value="false",
            description="Whether to start the AXCL tokenizer service when llm_backend=axcl.",
        ),
        create_llama_launch_from_yaml(
            os.path.join(config_dir, "smollmv2_servo_skull.yaml"),
            condition=UnlessCondition(
                PythonExpression(["'", LaunchConfiguration("llm_backend"), "' == 'axcl'"])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(axcl_launch),
            launch_arguments={
                "config_path": LaunchConfiguration("axcl_config_path"),
                "runtime_command": LaunchConfiguration("axcl_runtime_command"),
                "runtime_cwd": LaunchConfiguration("axcl_runtime_cwd"),
                "start_tokenizer": LaunchConfiguration("start_tokenizer"),
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("llm_backend"), "' == 'axcl'"])
            ),
        ),
    ])


