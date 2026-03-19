from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    ExecuteProcess,
    LogInfo,
    EmitEvent,
    GroupAction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


VENV_SITE_PACKAGES = '/home/murray/projects/venv-servo-skull/lib/python3.12/site-packages'
WAIT_FOR_READY_SCRIPT = r"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

topic = sys.argv[1]
timeout = float(sys.argv[2])

rclpy.init()

class ReadyWaiter(Node):
    def __init__(self):
        super().__init__('ready_waiter_' + topic.strip('/').replace('/', '_'))
        self.ready = False

node = ReadyWaiter()
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def cb(msg):
    if bool(msg.data):
        node.ready = True

sub = node.create_subscription(Bool, topic, cb, qos)
deadline = time.time() + timeout

while rclpy.ok() and not node.ready and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

ok = node.ready
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
"""


def generate_launch_description():
    default_axcl_config = os.path.join(
        get_package_share_directory('skull_control_node'),
        'configs',
        'axcl_servo_skull.yaml',
    )

    axcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('skull_control_node'),
                'launch',
                'llm_agent_axcl.launch.py'
            )
        ),
        launch_arguments={
            'config_path': LaunchConfiguration('axcl_config_path'),
            'runtime_command': LaunchConfiguration('runtime_command'),
            'runtime_cwd': LaunchConfiguration('runtime_cwd'),
            'tokenizer_script': LaunchConfiguration('tokenizer_script'),
            'tokenizer_cwd': LaunchConfiguration('tokenizer_cwd'),
            'start_tokenizer': LaunchConfiguration('start_tokenizer'),
            'inline_system_prompt': LaunchConfiguration('inline_system_prompt'),
        }.items(),
    )

    microphone_node = Node(
        package="microphone_node",
        executable="microphone_node",
        name="microphone_node",
        output="screen",
        parameters=[{'device_index': LaunchConfiguration('mic_device')}],
    )

    speaker_node = Node(
        package="speaker_node",
        executable="speaker_node",
        name="speaker_node",
        output="screen",
        parameters=[{'device': LaunchConfiguration('speaker_device')}],
    )

    tts_node = Node(
        package="tts_node",
        executable="tts_node",
        name="tts_node",
        output="screen",
    )

    stt_node = Node(
        package="stt_node",
        executable="stt_node",
        name="stt_node",
        output="screen",
    )

    skull_control_node = Node(
        package="skull_control_node",
        executable="skull_control_node",
        name="skull_control_node",
        output="screen",
        parameters=[{'enable_test_events': True}],
    )

    stt_stage = GroupAction(
        actions=[
            skull_control_node,
            stt_node,
        ]
    )

    wait_mic_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/microphone_node/ready', '30.0'],
        output='log',
    )
    wait_speaker_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/speaker_node/ready', '30.0'],
        output='log',
    )
    wait_tts_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/tts_node/ready', '45.0'],
        output='log',
    )
    wait_stt_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/stt_node/ready', '45.0'],
        output='log',
    )

    def start_next_on_success(next_action, stage_name: str):
        def _handler(event, _context):
            if event.returncode == 0:
                return [LogInfo(msg=f"{stage_name} ready; starting next stage."), next_action]
            return [
                LogInfo(msg=f"{stage_name} readiness check failed; shutting down launch."),
                EmitEvent(event=Shutdown(reason=f"{stage_name} did not become ready")),
            ]
        return _handler

    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=[VENV_SITE_PACKAGES + ':', EnvironmentVariable('PYTHONPATH', default_value='')]
        ),
        SetEnvironmentVariable(
            name='JACK_NO_START_SERVER',
            value='1'
        ),
        DeclareLaunchArgument(
            'mic_device',
            default_value='-1',
            description='Audio input device index or name for microphone_node'
        ),
        DeclareLaunchArgument(
            'speaker_device',
            default_value='',
            description='Audio output device index or name for speaker_node'
        ),
        DeclareLaunchArgument(
            'axcl_config_path',
            default_value=default_axcl_config,
            description='AXCL config path override.'
        ),
        DeclareLaunchArgument(
            'runtime_command',
            default_value='',
            description='Optional AXCL runtime command override.'
        ),
        DeclareLaunchArgument(
            'runtime_cwd',
            default_value='',
            description='Optional AXCL runtime working directory override.'
        ),
        DeclareLaunchArgument(
            'tokenizer_script',
            default_value='/home/murray/models/Qwen2.5-1.5B-Instruct/qwen2.5_tokenizer_uid.py',
            description='Tokenizer service script path used by AXCL launch.'
        ),
        DeclareLaunchArgument(
            'tokenizer_cwd',
            default_value='/home/murray/models/Qwen2.5-1.5B-Instruct',
            description='Tokenizer service working directory (must contain qwen2.5_tokenizer folder).'
        ),
        DeclareLaunchArgument(
            'start_tokenizer',
            default_value='true',
            description='Whether to start tokenizer from AXCL launch. Defaults to true for convenience.'
        ),
        DeclareLaunchArgument(
            'inline_system_prompt',
            default_value='',
            description='Optional AXCL inline_system_prompt override. Empty uses AXCL config value.'
        ),

        # Ordered startup chain:
        # 1) microphone -> wait mic ready
        # 2) speaker -> wait speaker ready
        # 3) tts -> wait tts ready
        # 4) stt -> wait stt ready
        # 5) AXCL LLM agent
        microphone_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=microphone_node,
                on_start=[wait_mic_ready],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_mic_ready,
                on_exit=start_next_on_success(speaker_node, 'microphone_node'),
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=speaker_node,
                on_start=[wait_speaker_ready],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_speaker_ready,
                on_exit=start_next_on_success(tts_node, 'speaker_node'),
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=tts_node,
                on_start=[wait_tts_ready],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_tts_ready,
                on_exit=start_next_on_success(stt_stage, 'tts_node'),
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=stt_node,
                on_start=[wait_stt_ready],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_stt_ready,
                on_exit=start_next_on_success(axcl_launch, 'stt_node'),
            )
        ),
    ])
