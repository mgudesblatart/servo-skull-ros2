from launch import LaunchDescription
from launch_ros.actions import Node
from llama_bringup.utils import create_llama_launch_from_yaml
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        create_llama_launch_from_yaml(os.path.join(
            get_package_share_directory("skull_control_node"), "configs", "smollmv2_servo_skull.yaml"))
    ])


