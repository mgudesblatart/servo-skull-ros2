# This is a minimal ROS2 message definition for raw audio data.
# You can expand this as needed for your use case.

# The audio data as raw int16 (e.g., PCM 16-bit LE)
int16[] data

# The sample rate of the audio data in Hz
uint32 sample_rate

# The number of channels in the audio data (e.g., 1 for mono, 2 for stereo)
uint8 channels

# Flag to indicate if this is the last chunk of the audio data
bool is_last_chunk

# Optionally, you could add sample width, etc. for richer messages.
# uint8 sample_width
