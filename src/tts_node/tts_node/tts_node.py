import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_msgs.msg import Bool
from servo_skull_msgs.msg import AudioData  # Updated import path
from ament_index_python.packages import get_package_share_directory
from piper import PiperVoice
import os
import array
import threading
import queue
import soundfile as sf
import time
import numpy as np

MODEL_DIR = os.path.join(get_package_share_directory("tts_node"), "models")
MODEL_PATH =  os.path.join(
            MODEL_DIR, os.environ.get('PIPER_MODEL_PATH', 'en_GB-alan-medium.onnx'))

DEFAULT_SAMPLE_RATE = 22050
DEFAULT_CHANNELS = 1
QUEUE_SIZE = 10
DEBUG_WRITE_WAV = False

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.stop_requested = threading.Event()
        self.subscription = self.create_subscription(
            String,
            '/text_to_speech/text_input',
            self.tts_callback,
            QUEUE_SIZE
        )
        self.control_sub = self.create_subscription(
            String,
            '/tts_node/control',
            self.control_callback,
            QUEUE_SIZE
        )
        self.audio_pub = self.create_publisher(AudioData, '/speaker/audio_output', QUEUE_SIZE)
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, '/tts_node/ready', ready_qos)
        self.tts_queue = queue.Queue(maxsize=100)
        self.worker_thread = threading.Thread(target=self.tts_worker, daemon=True)
        self.worker_thread.start()
        try:
            self.voice = PiperVoice.load(MODEL_PATH)
        except Exception as e:
            self.get_logger().error(f'Failed to load Piper model: {e}')
            self.voice = None

        if self.voice is not None:
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
            self.get_logger().info('Published readiness: /tts_node/ready = true')
        self.get_logger().info('TTS Node initialized and waiting for requests.')

    def _drain_tts_queue(self):
        dropped = 0
        while True:
            try:
                self.tts_queue.get_nowait()
                self.tts_queue.task_done()
                dropped += 1
            except queue.Empty:
                break
        return dropped

    def control_callback(self, msg: String):
        command = msg.data.strip().upper()
        if command not in {'STOP', 'CANCEL', 'HALT'}:
            return
        self.stop_requested.set()
        dropped = self._drain_tts_queue()
        self.get_logger().warning(f'Received TTS control command {command}; cleared {dropped} queued request(s).')

    def tts_callback(self, msg):
        text = msg.data
        if not text:
            self.get_logger().warning('Received empty TTS request. Skipping synthesis.')
            return
        if self.stop_requested.is_set():
            self.stop_requested.clear()
        try:
            self.tts_queue.put(text, block=False)
            self.get_logger().info(f'Enqueued TTS request: {text}')
        except queue.Full:
            self.get_logger().error('TTS queue is full. Dropping request.')

    def tts_worker(self):
        while True:
            text = self.tts_queue.get()
            if not self.voice:
                self.get_logger().error('Piper model not loaded. Cannot synthesize.')
                self.tts_queue.task_done()
                continue
            if self.stop_requested.is_set():
                self.get_logger().warning('Stop requested before synthesis; skipping queued text.')
                self.tts_queue.task_done()
                self.stop_requested.clear()
                continue
            self.get_logger().info(f'Processing TTS request: {text}')
            try:
                chunk_count = 0
                all_samples = []
                sample_rate = DEFAULT_SAMPLE_RATE
                channels = DEFAULT_CHANNELS
                chunks = self.voice.synthesize(text)
                interrupted = False
                chunks_buffer = []
                for chunk in chunks:
                    if self.stop_requested.is_set():
                        interrupted = True
                        break
                    chunks_buffer.append(chunk)
                if interrupted:
                    self.get_logger().warning('TTS synthesis interrupted by control command.')
                    self.stop_requested.clear()
                    self.tts_queue.task_done()
                    continue

                for i, chunk in enumerate(chunks_buffer):
                    if self.stop_requested.is_set():
                        interrupted = True
                        break
                    audio_msg = AudioData()
                    samples = list(array.array('h', chunk.audio_int16_bytes))
                    audio_msg.data = samples
                    audio_msg.sample_rate = getattr(chunk, 'sample_rate', getattr(self.voice, 'sample_rate', DEFAULT_SAMPLE_RATE))
                    audio_msg.channels = getattr(chunk, 'channels', getattr(self.voice, 'channels', DEFAULT_CHANNELS))
                    audio_msg.is_last_chunk = (i == len(chunks_buffer) - 1)
                    self.audio_pub.publish(audio_msg)
                    all_samples.extend(samples)
                    sample_rate = audio_msg.sample_rate
                    channels = audio_msg.channels
                    chunk_count += 1
                if interrupted:
                    self.get_logger().warning('TTS publish interrupted by control command.')
                    self.stop_requested.clear()
                    self.tts_queue.task_done()
                    continue
                if chunk_count > 0:
                    if DEBUG_WRITE_WAV:
                        wav_data = np.array(all_samples, dtype=np.int16)
                        if channels > 1:
                            wav_data = wav_data.reshape(-1, channels)
                        filename = f'tts_debug_{int(time.time())}.wav'
                        sf.write(filename, wav_data, sample_rate)
                        self.get_logger().info(f'Wrote utterance to {filename}')
                    self.get_logger().info(f'TTS processing complete. Chunks published: {chunk_count}')
                else:
                    self.get_logger().warning('Piper returned no audio chunks.')
            except Exception as e:
                self.get_logger().error(f'Error during TTS synthesis: {e}')
            self.tts_queue.task_done()

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
