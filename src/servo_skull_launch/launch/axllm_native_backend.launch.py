from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_dir",
            default_value="/home/murray/models/Qwen2.5-1.5B-Instruct",
            description="Model directory containing config.json and tokenizer.txt",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="8011",
            description="Port for axllm OpenAI-compatible server",
        ),
        DeclareLaunchArgument(
            "axllm_bin",
            default_value="/home/murray/projects/ax-llm/build_native/axllm",
            description="Path to native axllm binary",
        ),
        DeclareLaunchArgument(
            "startup_script",
            default_value="/home/murray/projects/servo-skull/scripts/start_axllm_native_serve.sh",
            description="Startup wrapper that runs preflight then starts axllm serve",
        ),
        ExecuteProcess(
            cmd=[
                "bash",
                LaunchConfiguration("startup_script"),
                LaunchConfiguration("model_dir"),
                LaunchConfiguration("port"),
                LaunchConfiguration("axllm_bin"),
            ],
            output="screen",
            emulate_tty=True,
        ),
    ])
