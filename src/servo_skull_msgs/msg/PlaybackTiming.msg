# Speaker playback timing metadata for adaptive STT echo suppression.

# Duration of the utterance written to the output stream, in seconds.
float32 duration_sec

# Monotonic-clock estimate for when playback should end (time.monotonic()).
float64 expected_end_mono

# Playback format metadata.
uint32 sample_rate
uint8 channels
