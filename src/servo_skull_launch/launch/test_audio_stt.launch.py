from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
    ExecuteProcess,
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
    microphone_node = Node(
        package="microphone_node",
        executable="microphone_node",
        name="microphone_node",
        output="screen",
        parameters=[{'device_index': LaunchConfiguration('mic_device')}],
    )

    stt_node = Node(
        package="stt_node",
        executable="stt_node",
        name="stt_node",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'stt_node:=debug'],
    )

    wait_mic_ready = ExecuteProcess(
        cmd=['python3', '-c', WAIT_FOR_READY_SCRIPT, '/microphone_node/ready', '30.0'],
        output='log',
    )

    def start_stt_on_mic_ready(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg='microphone_node ready; starting stt_node.'), stt_node]
        return [
            LogInfo(msg='microphone_node readiness check failed; shutting down launch.'),
            EmitEvent(event=Shutdown(reason='microphone_node did not become ready')),
        ]

    return LaunchDescription(
        [
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
                    on_exit=start_stt_on_mic_ready,
                )
            ),
        ]
    )
