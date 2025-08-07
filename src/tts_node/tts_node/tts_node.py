import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from servo_skull_msgs.msg import AudioData  # Updated import path
from ament_index_python.packages import get_package_share_directory
from piper import PiperVoice
import os
import array

MODEL_DIR = os.path.join(get_package_share_directory("tts_node"), "models")
MODEL_PATH =  os.path.join(
            MODEL_DIR, os.environ.get('PIPER_MODEL_PATH', 'en_GB-alan-medium.onnx'))

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/text_to_speech/text_input',
            self.tts_callback,
            10
        )
        self.audio_pub = self.create_publisher(AudioData, '/speaker/audio_output', 10)
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
        if not self.voice:
            self.get_logger().error('Piper model not loaded. Cannot synthesize.')
            return
        self.get_logger().info(f'Received TTS request: {text}')
        try:
            chunk_count = 0
            for chunk in self.voice.synthesize(text):
                audio_msg = AudioData()
                audio_msg.data = list(array.array('h', chunk.audio_int16_bytes))
                self.audio_pub.publish(audio_msg)
                chunk_count += 1
            if chunk_count == 0:
                self.get_logger().warning('Piper returned no audio chunks.')
            else:
                self.get_logger().info(f'TTS processing complete. Chunks published: {chunk_count}')
        except Exception as e:
            self.get_logger().error(f'Error during TTS synthesis: {e}')

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
