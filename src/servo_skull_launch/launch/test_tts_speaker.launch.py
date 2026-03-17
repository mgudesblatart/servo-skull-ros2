from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
    EmitEvent,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

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
    speaker_node = Node(
        package="speaker_node",
        executable="speaker_node",
        name="speaker_node",
        output="screen",
        parameters=[{'device': LaunchConfiguration('device')}],
    )

    tts_node = Node(
        package="tts_node",
        executable="tts_node",
        name="tts_node",
        output="screen",
    )

    publish_text = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--once',
            '--wait-matching-subscriptions', '1',
            '/text_to_speech/text_input',
            'std_msgs/msg/String',
            '"data: Hello, servo skull! This is a test of the TTS and speaker nodes."'
        ],
        output='screen',
        shell=True
    )

    wait_speaker_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/speaker_node/ready', '30.0'],
        output='log',
    )
    wait_tts_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/tts_node/ready', '45.0'],
        output='log',
    )

    def start_tts_on_speaker_ready(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg='speaker_node ready; starting tts_node.'), tts_node]
        return [
            LogInfo(msg='speaker_node readiness check failed; shutting down launch.'),
            EmitEvent(event=Shutdown(reason='speaker_node did not become ready')),
        ]

    def publish_on_tts_ready(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg='tts_node ready; publishing test text.'), publish_text]
        return [
            LogInfo(msg='tts_node readiness check failed; shutting down launch.'),
            EmitEvent(event=Shutdown(reason='tts_node did not become ready')),
        ]

    return LaunchDescription([
        SetEnvironmentVariable(
            name='PYTHONPATH',
            value=[VENV_SITE_PACKAGES + ':', EnvironmentVariable('PYTHONPATH', default_value='')]
        ),
        DeclareLaunchArgument(
            'device',
            default_value='',
            description='Audio output device index or name for speaker_node'
        ),
        speaker_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=speaker_node,
                on_start=[wait_speaker_ready],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_speaker_ready,
                on_exit=start_tts_on_speaker_ready,
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
                on_exit=publish_on_tts_ready,
            )
        ),
    ])
