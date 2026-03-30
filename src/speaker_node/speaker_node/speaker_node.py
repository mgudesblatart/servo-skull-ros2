"""
speaker_node.py

Receives AudioData chunks from tts_node and plays them through a sounddevice
OutputStream. Each utterance is accumulated from multiple chunks (tts_node
publishes several frames per synthesis call) and only played once the final
chunk arrives, identified by the is_last_chunk flag.

Topic wiring:
  Subscriptions:
    /speaker/audio_output   (servo_skull_msgs/AudioData)  — int16 PCM chunks
        /speaker_node/control   (std_msgs/String)             — STOP / CANCEL / HALT / RESET
  Publications:
    /speaker_node/ready  (std_msgs/Bool, latched)  — readiness flag

Device selection priority:
  1. ROS parameter 'device' (integer index or string name) if provided
  2. First USB audio device found by sounddevice
  3. First device with 'speaker' or 'output' in its name
  4. sounddevice default output device

The OutputStream is opened once at startup and kept alive for the lifetime of
the node. This avoids the perceptible latency of open/close per utterance and
prevents underrun artefacts when the TTS pipeline fires in quick succession.
"""

import queue
import threading
import time

import numpy as np
import rclpy
import scipy.signal
import sounddevice as sd
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from servo_skull_msgs.msg import AudioData, PlaybackTiming
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

QUEUE_SIZE = 10

# The OutputStream always runs at this rate; incoming audio is resampled if needed.
PLAYBACK_SAMPLE_RATE = 48000   # Hz

DEFAULT_SAMPLE_RATE = 22050    # Hz — Piper's default output rate
DEFAULT_CHANNELS = 1           # mono

# Divisor for converting int16 PCM to float32 in [-1.0, 1.0]
NORMALIZATION_INT16 = 32768.0


# ---------------------------------------------------------------------------
# Device selection
# ---------------------------------------------------------------------------

def find_speaker_device() -> int | str:
    """
    Probe sounddevice for a suitable output device.
    USB audio is preferred (for the skull's external speaker),
    then any device with 'speaker' / 'output' in the name,
    then sounddevice's default output.
    """
    devices = sd.query_devices()

    # Prefer USB audio (USB speaker dongle or similar)
    for idx, dev in enumerate(devices):
        name = dev.get("name", "").lower()
        if dev["max_output_channels"] > 0 and "usb" in name:
            return idx

    # Fallback: anything that sounds like a speaker
    for idx, dev in enumerate(devices):
        name = dev.get("name", "").lower()
        if dev["max_output_channels"] > 0 and ("speaker" in name or "output" in name):
            return idx

    # Final fallback: whatever sounddevice considers default
    return sd.default.device[1]


# ---------------------------------------------------------------------------
# SpeakerNode
# ---------------------------------------------------------------------------

class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker_node")

        # Set by control_callback; triggers drain + stream abort in playback_worker
        self.stop_requested = threading.Event()

        self._resolve_device()
        self._setup_subscriptions()
        self._setup_publishers()
        self._start_stream()

        # Background thread owns all sounddevice interaction
        self.audio_queue: queue.Queue[AudioData] = queue.Queue(maxsize=100)
        self.playback_thread = threading.Thread(
            target=self.playback_worker, daemon=True
        )
        self.playback_thread.start()

        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)
        self.get_logger().info("Published readiness: /speaker_node/ready = true")
        self.get_logger().info("Speaker Node initialized and listening for audio.")

    # -----------------------------------------------------------------------
    # Initialisation helpers
    # -----------------------------------------------------------------------

    def _resolve_device(self):
        """Pick output device from ROS param or auto-detect."""
        self.declare_parameter("device", "")
        param = self.get_parameter("device").get_parameter_value()
        # Parameter type 2 = integer, type 4 = string
        if param.type == 2 and param.integer_value >= 0:
            self.device = param.integer_value
        elif param.type == 4 and param.string_value:
            self.device = param.string_value
        else:
            self.device = find_speaker_device()
        self.get_logger().info(f"Using audio output device: {self.device}")

    def _setup_subscriptions(self):
        self.subscription = self.create_subscription(
            AudioData, "/speaker/audio_output", self.audio_callback, QUEUE_SIZE
        )
        self.control_sub = self.create_subscription(
            String, "/speaker_node/control", self.control_callback, QUEUE_SIZE
        )

    def _setup_publishers(self):
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, "/speaker_node/ready", ready_qos)
        self.playback_timing_pub = self.create_publisher(PlaybackTiming, "/speaker_node/playback_timing", 10)

    def _publish_playback_timing(self, duration_sec: float, channels: int):
        """Publish playback timing so skull_control can adapt STT echo suppression."""
        if duration_sec <= 0.0:
            return
        now_mono = time.monotonic()
        msg = PlaybackTiming()
        msg.duration_sec = float(duration_sec)
        msg.expected_end_mono = float(now_mono + duration_sec)
        msg.sample_rate = int(PLAYBACK_SAMPLE_RATE)
        msg.channels = int(channels)
        self.playback_timing_pub.publish(msg)

    def _start_stream(self):
        """Open the persistent OutputStream. Called once at __init__ time."""
        self.stream = sd.OutputStream(
            samplerate=PLAYBACK_SAMPLE_RATE,
            channels=DEFAULT_CHANNELS,
            dtype="float32",
            device=self.device,
            blocksize=0,  # let sounddevice pick block size
        )
        self.stream.start()
        self.get_logger().info(
            f"Persistent OutputStream started at {PLAYBACK_SAMPLE_RATE} Hz, "
            f"{DEFAULT_CHANNELS} channel(s)"
        )

    # -----------------------------------------------------------------------
    # Queue helpers
    # -----------------------------------------------------------------------

    def _drain_audio_queue(self) -> int:
        """Discard all pending audio chunks. Returns the count dropped."""
        dropped = 0
        while True:
            try:
                self.audio_queue.get_nowait()
                self.audio_queue.task_done()
                dropped += 1
            except queue.Empty:
                break
        return dropped

    def _write_silence(self, duration_sec: float = 0.1):
        """Write a short silence buffer to the stream. Used to prime the stream and pad between utterances."""
        num_samples = int(duration_sec * PLAYBACK_SAMPLE_RATE)
        silence = np.zeros((num_samples, DEFAULT_CHANNELS), dtype=np.float32)
        try:
            self.stream.write(silence)
        except Exception as e:
            self.get_logger().warning(f"Failed to write silence: {e}")

    # -----------------------------------------------------------------------
    # Topic callbacks (run on the ROS executor thread)
    # -----------------------------------------------------------------------

    def control_callback(self, msg: String):
        """
        Handle STOP / CANCEL / HALT / RESET. Drains the queue, then aborts and
        restarts the OutputStream to cut off mid-utterance playback immediately.
        """
        command = msg.data.strip().upper()
        if command not in {"STOP", "CANCEL", "HALT", "RESET"}:
            return

        self.stop_requested.set()
        dropped = self._drain_audio_queue()
        try:
            self.stream.abort()
            self.stream.start()
        except Exception as e:
            self.get_logger().warning(f"Failed to abort/restart stream on {command}: {e}")

        # Do not clear here; playback_worker must observe this and drop any
        # partially accumulated in-flight utterance before continuing.
        self.get_logger().warning(
            f"Received speaker control {command}; dropped {dropped} queued chunk(s)."
        )

    def audio_callback(self, msg: AudioData):
        """Forward incoming AudioData chunk to the playback worker queue."""
        if not msg.data:
            self.get_logger().warning("Received empty audio data.")
            return
        try:
            self.audio_queue.put(msg, block=False)
        except queue.Full:
            self.get_logger().error("Audio queue is full. Dropping audio chunk.")

    # -----------------------------------------------------------------------
    # Playback worker thread
    # -----------------------------------------------------------------------

    def playback_worker(self):
        """
        Background thread: accumulates AudioData chunks per utterance into a
        local list until is_last_chunk is True, then concatenates, optionally
        resamples, and writes the full utterance to the OutputStream in one call.

        Accumulating before playing means we resample exactly once (rather than
        per-chunk) and we get a clean gap-free write to the audio device.
        """
        utterance_chunks: list[np.ndarray] = []
        utterance_sample_rate: int | None = None
        utterance_channels: int | None = None

        # Prime the stream so the first utterance has no underrun at the start
        self._write_silence(0.2)

        while True:
            msg = self.audio_queue.get()

            if self.stop_requested.is_set():
                # Interrupt path: drop any partially built utterance and this chunk.
                utterance_chunks = []
                utterance_sample_rate = None
                utterance_channels = None
                self.audio_queue.task_done()
                self.stop_requested.clear()
                continue

            audio_np = np.array(msg.data, dtype=np.int16)
            if audio_np.size == 0:
                self.audio_queue.task_done()
                continue

            # Convert int16 → float32 for sounddevice
            audio_float = audio_np.astype(np.float32) / NORMALIZATION_INT16

            sample_rate = getattr(msg, "sample_rate", DEFAULT_SAMPLE_RATE)
            channels = getattr(msg, "channels", DEFAULT_CHANNELS)
            is_last_chunk = getattr(msg, "is_last_chunk", False)

            if channels > 1:
                audio_float = audio_float.reshape(-1, channels)

            # Latch sample rate and channel count from the first chunk of this utterance
            if utterance_sample_rate is None:
                utterance_sample_rate = sample_rate
            if utterance_channels is None:
                utterance_channels = channels

            utterance_chunks.append(audio_float)

            if is_last_chunk:
                if self.stop_requested.is_set():
                    utterance_chunks = []
                    utterance_sample_rate = None
                    utterance_channels = None
                    self.audio_queue.task_done()
                    self.stop_requested.clear()
                    continue
                self._play_utterance(utterance_chunks, utterance_sample_rate, utterance_channels)
                # Reset accumulators for the next utterance
                utterance_chunks = []
                utterance_sample_rate = None
                utterance_channels = None

            self.audio_queue.task_done()

    def _play_utterance(
        self,
        chunks: list[np.ndarray],
        sample_rate: int,
        channels: int,
    ):
        """
        Concatenate the accumulated chunks, resample to PLAYBACK_SAMPLE_RATE if
        the source rate differs, then write to the persistent OutputStream.
        scipy.signal.resample is used over resample_poly here because we're
        resampling the whole utterance at once and quality > speed.
        """
        full_audio = np.concatenate(chunks, axis=0)

        if sample_rate != PLAYBACK_SAMPLE_RATE:
            num_samples = int(len(full_audio) * PLAYBACK_SAMPLE_RATE / sample_rate)
            if channels > 1:
                full_audio = scipy.signal.resample(full_audio, num_samples, axis=0)
            else:
                full_audio = scipy.signal.resample(full_audio, num_samples)
            self.get_logger().info(
                f"Resampled utterance from {sample_rate} Hz to {PLAYBACK_SAMPLE_RATE} Hz"
            )

        # sounddevice requires a C-contiguous float32 array
        full_audio = np.ascontiguousarray(full_audio, dtype=np.float32)

        if self.stop_requested.is_set():
            self.get_logger().warning("Playback interrupted before stream write; dropping utterance.")
            self.stop_requested.clear()
            return

        try:
            duration_sec = float(len(full_audio)) / float(PLAYBACK_SAMPLE_RATE)
            self._publish_playback_timing(duration_sec, channels)
            self.stream.write(full_audio)
            self.get_logger().info(
                f"Played utterance at {PLAYBACK_SAMPLE_RATE} Hz, "
                f"{channels} channel(s) via persistent stream"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to play utterance via stream: {e}")

        # Brief post-utterance silence prevents the next utterance from sounding clipped
        self._write_silence(0.1)

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def destroy_node(self):
        """Close the persistent OutputStream on shutdown."""
        try:
            if hasattr(self, "stream") and self.stream:
                self.stream.stop()
                self.stream.close()
                self.get_logger().info("Persistent OutputStream closed.")
        except Exception as e:
            self.get_logger().error(f"Error closing OutputStream: {e}")
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
