import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        self.subscription = self.create_subscription(
            String,
            '/text_to_speech/text_input',
            self.tts_callback,
            QUEUE_SIZE
        )
        self.audio_pub = self.create_publisher(AudioData, '/speaker/audio_output', QUEUE_SIZE)
        self.tts_queue = queue.Queue(maxsize=100)
        self.worker_thread = threading.Thread(target=self.tts_worker, daemon=True)
        self.worker_thread.start()
        try:
            self.voice = PiperVoice.load(MODEL_PATH)
        except Exception as e:
            self.get_logger().error(f'Failed to load Piper model: {e}')
            self.voice = None
        self.get_logger().info('TTS Node initialized and waiting for requests.')

    def tts_callback(self, msg):
        text = msg.data
        if not text:
            self.get_logger().warning('Received empty TTS request. Skipping synthesis.')
            return
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
            self.get_logger().info(f'Processing TTS request: {text}')
            try:
                chunk_count = 0
                all_samples = []
                sample_rate = DEFAULT_SAMPLE_RATE
                channels = DEFAULT_CHANNELS
                chunks = list(self.voice.synthesize(text))
                for i, chunk in enumerate(chunks):
                    audio_msg = AudioData()
                    samples = list(array.array('h', chunk.audio_int16_bytes))
                    audio_msg.data = samples
                    audio_msg.sample_rate = getattr(chunk, 'sample_rate', getattr(self.voice, 'sample_rate', DEFAULT_SAMPLE_RATE))
                    audio_msg.channels = getattr(chunk, 'channels', getattr(self.voice, 'channels', DEFAULT_CHANNELS))
                    audio_msg.is_last_chunk = (i == len(chunks) - 1)
                    self.audio_pub.publish(audio_msg)
                    all_samples.extend(samples)
                    sample_rate = audio_msg.sample_rate
                    channels = audio_msg.channels
                    chunk_count += 1
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
