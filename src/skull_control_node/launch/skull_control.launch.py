from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("skull_control_node"),
                    "launch",
                    "llm_agent_http.launch.py",
                )
            ),
            launch_arguments={'config_path': default_http_config}.items(),
        )
    ])


