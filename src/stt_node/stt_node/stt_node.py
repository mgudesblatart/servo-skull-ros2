import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import numpy as np
import sherpa_onnx
import os
from ament_index_python.packages import get_package_share_directory
import queue
import threading
from scipy.signal import resample_poly

MODEL_DIR = os.path.join(get_package_share_directory("stt_node"), "models")
BUFFER_SECONDS = 2
BUFFER_SIZE = 48000 * BUFFER_SECONDS


class STTNode(Node):
    def __init__(self):
        super().__init__("stt_node", enable_logger_service=True)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("Initializing STTNode...")

        self.subscription = self.create_subscription(
            UInt8MultiArray, "audio/raw", self.audio_callback, 10
        )
        self.get_logger().info("Subscribed to audio/raw topic")
        self.transcript_publisher = self.create_publisher(
            String, "/speech_to_text/transcript", 10
        )
        encoder = os.path.join(
            MODEL_DIR,"encoder-epoch-99-avg-1.int8.onnx"
        )
        decoder = os.path.join(
            MODEL_DIR, "decoder-epoch-99-avg-1.onnx"
        )
        joiner = os.path.join(
            MODEL_DIR, "joiner-epoch-99-avg-1.int8.onnx"
        )
        tokens = os.path.join(MODEL_DIR, "tokens.txt")
        self.get_logger().info(f"Loading Sherpa-ONNX model files from {MODEL_DIR}")
        self.get_logger().debug(f"Encoder: {encoder}")
        self.get_logger().debug(f"Decoder: {decoder}")
        self.get_logger().debug(f"Joiner: {joiner}")
        self.get_logger().debug(f"Tokens: {tokens}")
        self.recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=tokens,
            encoder=encoder,
            decoder=decoder,
            joiner=joiner,
            num_threads=2,
            provider="cpu",
            sample_rate=16000,
            feature_dim=80,
            decoding_method="modified_beam_search",
            max_active_paths=6,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=2.4,  # seconds
            rule2_min_trailing_silence=1.8,
            rule3_min_utterance_length=300
        )
        self.stream = self.recognizer.create_stream()
        self.get_logger().info("Sherpa-ONNX recognizer initialized")
        self.pcm_buffer = np.array([], dtype=np.float32)
        self.last_result = ""
        self.output = []
        self.audio_queue = queue.Queue()
        self.worker_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.worker_thread.start()

    def audio_callback(self, msg):
        self.get_logger().debug(f"Received audio buffer of size {len(msg.data)} bytes (queued)")
        self.audio_queue.put(bytes(msg.data))

    def _audio_worker(self):
        self.get_logger().info("Audio worker thread started")
        while True:
            try:
                chunk = self.audio_queue.get()
                self.get_logger().debug("Audio worker pulled chunk from queue")
                pcm = np.frombuffer(chunk, dtype=np.int16)
                pcm_float = pcm.astype(np.float32) / 32768.0
                self.pcm_buffer = np.concatenate((self.pcm_buffer, pcm_float))
                pcm_16k = resample_poly(self.pcm_buffer, up=1, down=3)
                self.stream.accept_waveform(16000, pcm_16k)
                self.pcm_buffer = np.array([], dtype=np.float32)  # Clear buffer after feeding
                while self.recognizer.is_ready(self.stream):
                    self.recognizer.decode_stream(self.stream)
                result = self.recognizer.get_result(self.stream)
                self.get_logger().debug(f"Result content: {result}")
                if self.recognizer.is_endpoint(self.stream):
                    self.get_logger().debug(f"Transcription complete")
                    final_result = self.recognizer.get_result(self.stream)
                    self.get_logger().debug(f"Final result: {final_result}")
                    # Publish transcript to /speech_to_text/transcript
                    msg = String()
                    msg.data = final_result if isinstance(final_result, str) else str(final_result)
                    self.transcript_publisher.publish(msg)
                    self.recognizer.reset(self.stream)
            except Exception as e:
                self.get_logger().error(f"Error in audio worker: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    node.get_logger().info('STTNode main loop starting')
    rclpy.spin(node)
    node.get_logger().info('Shutting down STTNode')
    node.destroy_node()
    rclpy.shutdown()
