import rclpy
from rclpy.node import Node
from servo_skull_msgs.msg import AudioData
import numpy as np
import sounddevice as sd
import threading
import queue
import scipy.signal

# Default audio parameters (can be extended to read from AudioData if needed)
CHANNELS = 1
QUEUE_SIZE = 10
NORMALIZATION_INT16 = 32768.0
DEFAULT_SAMPLE_RATE = 22050
DEFAULT_CHANNELS = 1
PLAYBACK_SAMPLE_RATE = 48000  # Set to 48000 if you want to try that instead

def find_speaker_device():
    devices = sd.query_devices()
    print("[SpeakerNode] Detected audio output devices:")
    for idx, dev in enumerate(devices):
        print(f"  {idx}: {dev['name']} ({dev['max_output_channels']} channels)")
    # Prefer USB audio devices
    for idx, dev in enumerate(devices):
        name = dev.get('name', '').lower()
        if dev['max_output_channels'] > 0 and 'usb' in name:
            print(f"[SpeakerNode] Selected USB audio device: {idx} - {dev['name']}")
            return idx
    # Fallback: look for 'speaker' or 'output' in the name
    for idx, dev in enumerate(devices):
        name = dev.get('name', '').lower()
        if dev['max_output_channels'] > 0 and ('speaker' in name or 'output' in name):
            print(f"[SpeakerNode] Selected fallback device: {idx} - {dev['name']}")
            return idx
    # Fallback: return default output device
    print(f"[SpeakerNode] Using default output device: {sd.default.device[1]}")
    return sd.default.device[1]

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        # Add device parameter (can be int index or string name)
        self.declare_parameter('device', 1)  # Default to 1 (user's speaker)
        param_device = self.get_parameter('device').get_parameter_value()
        device = param_device.integer_value if param_device.type == 2 else param_device.string_value if param_device.type == 4 else None
        if device is not None and device != '':
            self.device = device
            self.get_logger().info(f"Using audio output device from ROS param: {self.device}")
        else:
            self.device = find_speaker_device()
            self.get_logger().info(f'Using audio output device: {self.device}')
        self.subscription = self.create_subscription(
            AudioData,
            '/speaker/audio_output',
            self.audio_callback,
            QUEUE_SIZE
        )
        # Start persistent OutputStream on init
        self.stream = sd.OutputStream(
            samplerate=PLAYBACK_SAMPLE_RATE,
            channels=DEFAULT_CHANNELS,
            dtype='float32',
            device=self.device,
            blocksize=0,
        )
        self.stream.start()
        self.get_logger().info(f"Persistent OutputStream started at {PLAYBACK_SAMPLE_RATE} Hz, {DEFAULT_CHANNELS} channel(s)")
        self.get_logger().info('Speaker Node initialized and listening for audio.')
        self.audio_queue = queue.Queue(maxsize=100)
        self.playback_thread = threading.Thread(target=self.playback_worker, daemon=True)
        self.playback_thread.start()

    def destroy_node(self):
        # Clean up persistent stream
        try:
            if hasattr(self, 'stream') and self.stream:
                self.stream.stop()
                self.stream.close()
                self.get_logger().info('Persistent OutputStream closed')
        except Exception as e:
            self.get_logger().error(f'Error closing OutputStream: {e}')
        super().destroy_node()

    def audio_callback(self, msg):
        if not msg.data:
            self.get_logger().warning('Received empty audio data.')
            return
        try:
            self.audio_queue.put(msg, block=False)
            self.get_logger().info(f'Enqueued audio chunk of length {len(msg.data)} at {getattr(msg, "sample_rate", DEFAULT_SAMPLE_RATE)} Hz, {getattr(msg, "channels", DEFAULT_CHANNELS)} channel(s)')
        except queue.Full:
            self.get_logger().error('Audio queue is full. Dropping audio chunk.')

    def _write_silence(self, duration_sec=0.1):
        # Write a short buffer of silence to the stream
        num_samples = int(duration_sec * PLAYBACK_SAMPLE_RATE)
        silence = np.zeros((num_samples, DEFAULT_CHANNELS), dtype=np.float32)
        try:
            self.stream.write(silence)
            self.get_logger().debug(f'Wrote {duration_sec:.3f}s of silence to stream')
        except Exception as e:
            self.get_logger().warning(f'Failed to write silence: {e}')

    def playback_worker(self):
        utterance_chunks = []
        utterance_sample_rate = None
        utterance_channels = None
        # Prime the stream with silence on startup
        self._write_silence(0.2)
        while True:
            msg = self.audio_queue.get()
            audio_np = np.array(msg.data, dtype=np.int16)
            print(f"[SpeakerNode] Got chunk: shape={audio_np.shape}, min={audio_np.min() if audio_np.size else 'n/a'}, max={audio_np.max() if audio_np.size else 'n/a'}, is_last_chunk={getattr(msg, 'is_last_chunk', False)}")
            if audio_np.size == 0:
                self.audio_queue.task_done()
                continue
            audio_float = audio_np.astype(np.float32) / NORMALIZATION_INT16
            sample_rate = getattr(msg, 'sample_rate', DEFAULT_SAMPLE_RATE)
            channels = getattr(msg, 'channels', DEFAULT_CHANNELS)
            is_last_chunk = getattr(msg, 'is_last_chunk', False)
            if channels > 1:
                audio_float = audio_float.reshape(-1, channels)
            if utterance_sample_rate is None:
                utterance_sample_rate = sample_rate
            if utterance_channels is None:
                utterance_channels = channels
            utterance_chunks.append(audio_float)
            if is_last_chunk:
                full_audio = np.concatenate(utterance_chunks, axis=0)
                if utterance_sample_rate != PLAYBACK_SAMPLE_RATE:
                    num_samples = int(len(full_audio) * PLAYBACK_SAMPLE_RATE / utterance_sample_rate)
                    self.get_logger().info(f'utterance channels: {utterance_channels}')
                    if utterance_channels > 1:
                        full_audio = scipy.signal.resample(full_audio, num_samples, axis=0)
                    else:
                        full_audio = scipy.signal.resample(full_audio, num_samples)
                    self.get_logger().info(f'Resampled utterance from {utterance_sample_rate} Hz to {PLAYBACK_SAMPLE_RATE} Hz')
                    utterance_sample_rate = PLAYBACK_SAMPLE_RATE
                print(f"[SpeakerNode] Playing utterance: shape={full_audio.shape}, min={full_audio.min()}, max={full_audio.max()}, sample_rate={utterance_sample_rate}")
                full_audio = np.ascontiguousarray(full_audio, dtype=np.float32)
                print(f"[SpeakerNode] Playback array dtype: {full_audio.dtype}, contiguous: {full_audio.flags['C_CONTIGUOUS']}, shape: {full_audio.shape}")
                try:
                    self.stream.write(full_audio)
                    self.get_logger().info(f'Played utterance at {utterance_sample_rate} Hz, {utterance_channels} channel(s) via persistent stream')
                except Exception as e:
                    self.get_logger().error(f'Failed to play utterance via stream: {e}')
                # Write a short silence after each utterance
                self._write_silence(0.1)
                utterance_chunks = []
                utterance_sample_rate = None
                utterance_channels = None
            self.audio_queue.task_done()

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

if __name__ == '__main__':
    main()
