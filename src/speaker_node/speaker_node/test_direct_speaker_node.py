import rclpy
from rclpy.node import Node
from servo_skull_msgs.msg import AudioData
import numpy as np
import time
from speaker_node import SpeakerNode, DEFAULT_SAMPLE_RATE, DEFAULT_CHANNELS
import threading

def make_sine_wave(duration_sec=1.0, freq=440, sample_rate=DEFAULT_SAMPLE_RATE, channels=1):
    t = np.linspace(0, duration_sec, int(sample_rate * duration_sec), endpoint=False)
    wave = 0.2 * np.sin(2 * np.pi * freq * t)
    wave = (wave * 32767).astype(np.int16)
    if channels > 1:
        wave = np.tile(wave[:, None], (1, channels))
    return wave

def main():
    rclpy.init()
    node = SpeakerNode()
    pub = node.create_publisher(AudioData, '/speaker/audio_output', 10)
    # Start spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1)  # Let node initialize
    # Generate a 1-second 440Hz sine wave
    wave = make_sine_wave()
    msg = AudioData()
    msg.data = wave.flatten().tolist()
    msg.sample_rate = DEFAULT_SAMPLE_RATE
    msg.channels = DEFAULT_CHANNELS
    msg.is_last_chunk = True
    print("Publishing test sine wave...")
    pub.publish(msg)
    # Wait for the message to be processed
    print("Waiting for speaker node to process audio queue...")
    node.audio_queue.join()
    print("Audio queue processed. Shutting down.")
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=1)

if __name__ == '__main__':
    main()
