"""
stt_node.py

Sherpa-ONNX online transducer speech-to-text node. Receives raw 48 kHz uint8
PCM audio from the microphone node, downsamples to 16 kHz, feeds it to a
Sherpa-ONNX streaming recognizer, and publishes completed transcripts.

Topic wiring:
  Subscriptions:
    audio/raw  (std_msgs/UInt8MultiArray)  — raw int16 PCM at 48 kHz, mono
        /skull_control/state_transition (servo_skull_msgs/StateTransition) — speech-state mute gate
  Publications:
    /speech_to_text/transcript  (std_msgs/String)  — completed utterance text
    /stt_node/ready             (std_msgs/Bool, latched)  — readiness flag

The recognizer uses endpoint detection to decide when an utterance is complete
(rules 1–3 in __init__ control the silence/length thresholds). The audio worker
thread is started explicitly via start_worker() rather than in __init__ so that
callers can inject audio before the worker begins processing (e.g. in tests).
"""

import os
import queue
import threading
import traceback
import time

import numpy as np
import rclpy
import sherpa_onnx
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.signal import resample_poly
from servo_skull_msgs.msg import StateTransition
from std_msgs.msg import Bool, String, UInt8MultiArray


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Directory containing the Sherpa-ONNX model files (populated by colcon install)
MODEL_DIR = os.path.join(get_package_share_directory("stt_node"), "models")

# Audio accumulation buffer: 2 seconds of 48 kHz audio.
# We don't actually use a circular buffer here — we feed each received chunk
# directly — but this documents the expected chunk cadence.
BUFFER_SECONDS = 2
BUFFER_SIZE = 48000 * BUFFER_SECONDS  # samples

# Delay before STT resumes after speech returns to IDLE.
# This absorbs residual output tail/room echo from the speaker path.
STT_RESUME_DELAY_SEC = 0.8


# ---------------------------------------------------------------------------
# STTNode
# ---------------------------------------------------------------------------

class STTNode(Node):
    def __init__(self):
        super().__init__("stt_node", enable_logger_service=True)
        self.get_logger().info("Initializing STTNode...")

        self._setup_subscriptions()
        self._setup_publishers()
        self._init_recognizer()

        # Raw PCM buffer; chunks are appended here then immediately consumed
        self.pcm_buffer = np.array([], dtype=np.float32)

        self.audio_queue: queue.Queue[bytes] = queue.Queue()
        self.worker_thread: threading.Thread | None = None  # started via start_worker()
        self.running = True
        self.transcription_enabled = True
        self.transcription_resume_at_ts = 0.0
        self._transcription_gate_lock = threading.Lock()

        self.get_logger().info("STT Node started.")

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _setup_subscriptions(self):
        self.subscription = self.create_subscription(
            UInt8MultiArray, "audio/raw", self.audio_callback, 10
        )
        self.state_transition_subscription = self.create_subscription(
            StateTransition,
            "/skull_control/state_transition",
            self.state_transition_callback,
            10,
        )

    def _setup_publishers(self):
        self.transcript_publisher = self.create_publisher(
            String, "/speech_to_text/transcript", 10
        )
        # Latched so late-joining subscribers still see the readiness signal
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_publisher = self.create_publisher(Bool, "/stt_node/ready", ready_qos)

    def _init_recognizer(self):
        """
        Load the Sherpa-ONNX int8-quantised transducer model files and configure
        the online recogniser.

        Endpoint detection rules (in priority order):
          rule1: declare endpoint after 2.4 s trailing silence (mid-speech pause)
          rule2: declare endpoint after 1.8 s trailing silence (shorter pause ok
                 if we've accumulated enough audio)
          rule3: force endpoint after 300 frames regardless (runaway utterance guard)
        """
        encoder = os.path.join(MODEL_DIR, "encoder-epoch-99-avg-1.int8.onnx")
        decoder = os.path.join(MODEL_DIR, "decoder-epoch-99-avg-1.onnx")
        joiner  = os.path.join(MODEL_DIR, "joiner-epoch-99-avg-1.int8.onnx")
        tokens  = os.path.join(MODEL_DIR, "tokens.txt")

        self.get_logger().info(f"Loading Sherpa-ONNX model files from {MODEL_DIR}")

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
            rule1_min_trailing_silence=2.4,   # seconds
            rule2_min_trailing_silence=1.8,   # seconds
            rule3_min_utterance_length=300,   # feature frames
        )
        self.stream = self.recognizer.create_stream()
        self.get_logger().info("Sherpa-ONNX recognizer initialized.")

    # -----------------------------------------------------------------------
    # Worker lifecycle
    # -----------------------------------------------------------------------

    def start_worker(self):
        """
        Start the audio processing thread. Called explicitly from main() (not
        from __init__) so the node is fully spun up before audio processing begins.
        """
        if self.worker_thread is not None:
            self.get_logger().warning("Worker thread already started.")
            return
        self.worker_thread = threading.Thread(target=self._audio_worker, daemon=True)
        self.worker_thread.start()
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_publisher.publish(ready_msg)
        self.get_logger().info("Published readiness: /stt_node/ready = true")

    def stop_worker(self):
        """Signal the worker thread to stop and wait for it to exit."""
        self.running = False
        if self.worker_thread is not None:
            self.worker_thread.join(timeout=2)

    # -----------------------------------------------------------------------
    # Topic callback (ROS executor thread)
    # -----------------------------------------------------------------------

    def audio_callback(self, msg: UInt8MultiArray):
        """Forward raw audio bytes to the worker queue without processing."""
        if not self._is_transcription_enabled():
            return
        self.audio_queue.put(bytes(msg.data))

    def state_transition_callback(self, msg: StateTransition):
        """Mute STT while speech subsystem is in SPEAKING to suppress speaker loopback."""
        if str(msg.subsystem).lower() != "speech":
            return

        to_state = str(msg.to_state).strip().upper()
        if to_state == "SPEAKING":
            self._set_transcription_enabled(False, reason="speech_state_speaking")
        elif to_state == "IDLE":
            self._set_transcription_enabled(True, reason="speech_state_idle")

    def _is_transcription_enabled(self) -> bool:
        with self._transcription_gate_lock:
            if not self.transcription_enabled:
                return False
            return time.monotonic() >= self.transcription_resume_at_ts

    def _set_transcription_enabled(self, enabled: bool, reason: str):
        with self._transcription_gate_lock:
            if self.transcription_enabled == enabled:
                return
            self.transcription_enabled = enabled
            self.transcription_resume_at_ts = (
                time.monotonic() + STT_RESUME_DELAY_SEC if enabled else 0.0
            )

        if enabled:
            self.get_logger().info(
                f"STT transcription scheduled to resume in {STT_RESUME_DELAY_SEC:.1f}s ({reason})."
            )
            return

        self._clear_pending_audio()
        self._reset_recognizer_stream()
        self.get_logger().info(f"STT transcription muted ({reason}).")

    def _clear_pending_audio(self):
        """Drop queued mic frames so no stale speaker bleed is processed after unmute."""
        while True:
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break

    def _reset_recognizer_stream(self):
        """Reset recognizer stream and local buffer to clear partial decode context."""
        self.pcm_buffer = np.array([], dtype=np.float32)
        try:
            self.recognizer.reset(self.stream)
        except Exception as e:
            self.get_logger().warning(f"Failed to reset STT recognizer stream while muting: {e}")

    # -----------------------------------------------------------------------
    # Audio processing thread
    # -----------------------------------------------------------------------

    def _audio_worker(self, once: bool = False):
        """
        Dequeue raw audio bytes, downsample from 48 kHz → 16 kHz, feed the
        Sherpa-ONNX stream, and publish a transcript on each detected endpoint.

        resample_poly(up=1, down=3): exact rational resampling, 48kHz / 3 = 16kHz.
        Using resample_poly rather than resample because the poly variant is
        significantly faster for integer ratios and introduces less ringing.

        The 'once' parameter is provided for testing purposes.
        """
        self.get_logger().info("Audio worker thread started.")
        while self.running:
            try:
                chunk = self.audio_queue.get(timeout=0.1)
            except queue.Empty:
                if once:
                    break
                continue

            try:
                # Interpret raw bytes as int16 PCM, convert to float32 [-1, 1]
                pcm = np.frombuffer(chunk, dtype=np.int16)
                pcm_float = pcm.astype(np.float32) / 32768.0

                if not self._is_transcription_enabled():
                    self.pcm_buffer = np.array([], dtype=np.float32)
                    if once:
                        break
                    continue

                self.pcm_buffer = np.concatenate((self.pcm_buffer, pcm_float))

                # Downsample: 48 kHz → 16 kHz (factor of 3)
                pcm_16k = resample_poly(self.pcm_buffer, up=1, down=3)
                self.stream.accept_waveform(16000, pcm_16k)
                self.pcm_buffer = np.array([], dtype=np.float32)

                # Drain the recognizer's decode queue
                while self.recognizer.is_ready(self.stream):
                    self.recognizer.decode_stream(self.stream)

                if self.recognizer.is_endpoint(self.stream):
                    final_result = self.recognizer.get_result(self.stream)
                    transcript = (
                        final_result if isinstance(final_result, str) else str(final_result)
                    ).strip()

                    if transcript:
                        msg = String()
                        msg.data = transcript
                        self.transcript_publisher.publish(msg)
                        self.get_logger().info(f"Transcript: '{transcript}'")

                    # Reset so the recognizer is ready for the next utterance
                    self.recognizer.reset(self.stream)

            except Exception as e:
                self.get_logger().error(f"Error in audio worker: {e}")
                self.get_logger().error(traceback.format_exc())

            if once:
                break


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    node.start_worker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.stop_worker()
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
