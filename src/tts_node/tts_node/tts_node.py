"""
tts_node.py

Piper TTS synthesis node. Receives text strings on /text_to_speech/text_input,
synthesises them using a Piper voice model, and publishes AudioData chunks to
/speaker/audio_output for playback by speaker_node.

Topic wiring:
  Subscriptions:
    /text_to_speech/text_input  (std_msgs/String)  — text to synthesise
    /tts_node/control           (std_msgs/String)  — STOP / CANCEL / HALT
  Publications:
    /speaker/audio_output  (servo_skull_msgs/AudioData)  — int16 PCM chunks
    /tts_node/ready        (std_msgs/Bool, latched)      — readiness flag

Synthesis is done on a background worker thread so the ROS executor is never
blocked. Interrupts are checked at two points:
  1. During Piper's synthesis loop (before any chunks are published). This
     ensures that a HALT received mid-synthesis doesn't result in a partial
     utterance being played.
  2. During the publish loop over the synthesised buffer. Redundant but
     cheap — catches any interrupt that arrives in the window between the two.
"""

import array
import os
import queue
import threading
import time

import numpy as np
import rclpy
import soundfile as sf
from ament_index_python.packages import get_package_share_directory
from piper import PiperVoice
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from servo_skull_msgs.msg import AudioData
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Absolute path to the model directory (populated by colcon install)
MODEL_DIR = os.path.join(get_package_share_directory("tts_node"), "models")

# Allow the model file to be overridden at runtime via environment variable
MODEL_PATH = os.path.join(
    MODEL_DIR, os.environ.get("PIPER_MODEL_PATH", "en_GB-alan-medium.onnx")
)

DEFAULT_SAMPLE_RATE = 22050  # Hz — standard Piper output rate
DEFAULT_CHANNELS = 1         # mono
QUEUE_SIZE = 10              # ROS topic and worker queue depth

# Set to True to write each synthesised utterance as a .wav file (debug only)
DEBUG_WRITE_WAV = False


# ---------------------------------------------------------------------------
# TTSNode
# ---------------------------------------------------------------------------

class TTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")

        # Threading.Event set by control_callback when a STOP/CANCEL/HALT arrives.
        # The worker thread checks this flag during synthesis and publishing.
        self.stop_requested = threading.Event()

        self._setup_subscriptions()
        self._setup_publishers()

        # Background worker processes the queue serially (one utterance at a time)
        self.tts_queue: queue.Queue[str] = queue.Queue(maxsize=100)
        self.worker_thread = threading.Thread(target=self.tts_worker, daemon=True)
        self.worker_thread.start()

        # Load Piper voice model; node still starts if loading fails
        try:
            self.voice = PiperVoice.load(MODEL_PATH)
        except Exception as e:
            self.get_logger().error(f"Failed to load Piper model: {e}")
            self.voice = None

        if self.voice is not None:
            ready_msg = Bool()
            ready_msg.data = True
            self.ready_pub.publish(ready_msg)
            self.get_logger().info("Published readiness: /tts_node/ready = true")

        self.get_logger().info("TTS Node initialized and waiting for requests.")

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _setup_subscriptions(self):
        self.subscription = self.create_subscription(
            String, "/text_to_speech/text_input", self.tts_callback, QUEUE_SIZE
        )
        self.control_sub = self.create_subscription(
            String, "/tts_node/control", self.control_callback, QUEUE_SIZE
        )

    def _setup_publishers(self):
        self.audio_pub = self.create_publisher(AudioData, "/speaker/audio_output", QUEUE_SIZE)

        # Latched (TRANSIENT_LOCAL) so late-joining subscribers still receive readiness
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, "/tts_node/ready", ready_qos)

    # -----------------------------------------------------------------------
    # Queue helpers
    # -----------------------------------------------------------------------

    def _drain_tts_queue(self) -> int:
        """Discard all pending synthesis requests. Returns the count dropped."""
        dropped = 0
        while True:
            try:
                self.tts_queue.get_nowait()
                self.tts_queue.task_done()
                dropped += 1
            except queue.Empty:
                break
        return dropped

    # -----------------------------------------------------------------------
    # Topic callbacks (run on the ROS executor thread)
    # -----------------------------------------------------------------------

    def control_callback(self, msg: String):
        """Handle STOP / CANCEL / HALT — interrupt the current synthesis and clear pending work."""
        command = msg.data.strip().upper()
        if command not in {"STOP", "CANCEL", "HALT"}:
            return
        self.stop_requested.set()
        dropped = self._drain_tts_queue()
        self.get_logger().warning(
            f"Received TTS control command {command}; cleared {dropped} queued request(s)."
        )

    def tts_callback(self, msg: String):
        """Enqueue a text string for synthesis. Clears a stale stop flag if present."""
        text = msg.data
        if not text:
            self.get_logger().warning("Received empty TTS request. Skipping synthesis.")
            return
        # A stop flag left over from a previous interrupt would block the new request
        if self.stop_requested.is_set():
            self.stop_requested.clear()
        try:
            self.tts_queue.put(text, block=False)
            self.get_logger().info(f"Enqueued TTS request: {text}")
        except queue.Full:
            self.get_logger().error("TTS queue is full. Dropping request.")

    # -----------------------------------------------------------------------
    # Worker thread — synthesis and publishing
    # -----------------------------------------------------------------------

    def tts_worker(self):
        """
        Background thread: dequeues text, synthesises with Piper, and publishes
        AudioData chunks. Checks stop_requested at two points to support fast
        interrupt handling (see module docstring for the two-phase rationale).
        """
        while True:
            text = self.tts_queue.get()

            if not self.voice:
                self.get_logger().error("Piper model not loaded. Cannot synthesize.")
                self.tts_queue.task_done()
                continue

            if self.stop_requested.is_set():
                self.get_logger().warning("Stop requested before synthesis; skipping queued text.")
                self.tts_queue.task_done()
                self.stop_requested.clear()
                continue

            self.get_logger().info(f"Processing TTS request: {text}")
            try:
                self._synthesise_and_publish(text)
            except Exception as e:
                self.get_logger().error(f"Error during TTS synthesis: {e}")

            self.tts_queue.task_done()

    def _synthesise_and_publish(self, text: str):
        """
        Phase 1 — collect all Piper chunks into a local buffer (checking for
        interrupts as we go). Phase 2 — publish them one by one (checking again).
        This two-pass design prevents a partial utterance reaching the speaker.
        """
        all_samples: list[int] = []
        sample_rate = DEFAULT_SAMPLE_RATE
        channels = DEFAULT_CHANNELS

        # Phase 1: drain the Piper generator into a buffer
        chunks_buffer = []
        for chunk in self.voice.synthesize(text):
            if self.stop_requested.is_set():
                self.get_logger().warning("TTS synthesis interrupted by control command.")
                self.stop_requested.clear()
                return
            chunks_buffer.append(chunk)

        # Phase 2: publish buffered chunks as AudioData messages
        chunk_count = 0
        for i, chunk in enumerate(chunks_buffer):
            if self.stop_requested.is_set():
                self.get_logger().warning("TTS publish interrupted by control command.")
                self.stop_requested.clear()
                return

            audio_msg = AudioData()
            samples = list(array.array("h", chunk.audio_int16_bytes))
            audio_msg.data = samples
            audio_msg.sample_rate = getattr(
                chunk, "sample_rate", getattr(self.voice, "sample_rate", DEFAULT_SAMPLE_RATE)
            )
            audio_msg.channels = getattr(
                chunk, "channels", getattr(self.voice, "channels", DEFAULT_CHANNELS)
            )
            # speaker_node uses is_last_chunk to know when to start playback
            audio_msg.is_last_chunk = i == len(chunks_buffer) - 1
            self.audio_pub.publish(audio_msg)
            all_samples.extend(samples)
            sample_rate = audio_msg.sample_rate
            channels = audio_msg.channels
            chunk_count += 1

        if chunk_count == 0:
            self.get_logger().warning("Piper returned no audio chunks.")
            return

        self.get_logger().info(f"TTS processing complete. Chunks published: {chunk_count}")

        if DEBUG_WRITE_WAV:
            wav_data = np.array(all_samples, dtype=np.int16)
            if channels > 1:
                wav_data = wav_data.reshape(-1, channels)
            filename = f"tts_debug_{int(time.time())}.wav"
            sf.write(filename, wav_data, sample_rate)
            self.get_logger().info(f"Wrote utterance to {filename}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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


if __name__ == "__main__":
    main()
